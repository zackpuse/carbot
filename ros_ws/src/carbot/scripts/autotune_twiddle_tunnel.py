#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
autotune_twiddle_tunnel.py
==========================
Auto-tune PID (kp, kd, kp_wall) untuk tunnel_nav.py guna algoritma Twiddle.
Node BERASINGAN - subscribe /tunnel_nav/debug untuk mengira ralat
(error = |L - R|). Kawalan melalui Web Dashboard (tab "Auto-Tune TUNNEL").

REKA BENTUK KESELAMATAN (baca sebelum jalankan - lebih kritikal daripada
versi lane sebab melibatkan dinding fizikal sebenar):
    - TIDAK dimasukkan dalam carbot_full.launch - WAJIB dijalankan manual:
          python autotune_twiddle_tunnel.py
    - LISTEN-ONLY terhadap /autotune_tunnel/cmd (NEXT / ABORT).
    - Gain PID calon SETERUSNYA hanya di-push SELEPAS operator confirm
      (tekan NEXT) robot dah berhenti & diletak semula di start terowong -
      BUKAN serta-merta lepas trial sebelumnya tamat. Ini elak gain
      melampau tiba-tiba aktif semasa robot masih coasting rapat dengan
      dinding.
    - Hormat Emergency Stop sedia ada tunnel_nav.py: jika mode debug
      mengandungi "STOP", atau L/R < MIN_WALL_DIST_M, trial terus di-abort
      (bukan cuba override/bypass logik emergency stop tunnel_nav).
    - dynamic_reconfigure Client guna nama node SEBENAR "tunnel_nav"
      (disahkan via `rosnode list` - BUKAN "tunnel_nav_dashboard", nama
      yang tidak wujud sebagai node aktif).

ALIRAN SATU TRIAL (1 klik NEXT setiap trial):
    1. Skrip pilih calon parameter (kp/kd/kp_wall) seterusnya, TAPI BELUM
       push - robot kekal guna gain SEBELUM ini.
    2. Publish status "menunggu" (trial_active=False) - dashboard papar
       calon yang bakal diuji dengan label "BELUM aktif".
    3. Operator pastikan robot BERHENTI & diletak semula di start
       terowong, hidupkan semula robot, tekan "Mula/Teruskan" ->
       hantar "NEXT" ke /autotune_tunnel/cmd.
    4. HANYA SELEPAS NEXT diterima, gain calon di-push, tunggu 0.3s
       settle, baru mula rekod |L-R| selama TRIAL_DURATION_SEC saat.
    5. Auto-abort awal jika terlanggar dinding (L/R < MIN_WALL_DIST_M),
       emergency-stop tunnel_nav sendiri triggered (mode mengandungi
       "STOP"), error melampau, ATAU operator tekan "Batal" (ABORT).
    6. Cost dikira, bandingkan dengan terbaik setakat ini, param
       seterusnya ditentukan ikut logik Twiddle. Kembali ke langkah 1.

OUTPUT:
    - tuning_log_tunnel.csv
    - best_pid_tunnel.json
"""
from __future__ import print_function
import rospy
import csv
import json
import os
import time
import threading
from std_msgs.msg import String
from dynamic_reconfigure.client import Client

# ==================== SETTING TUNING ====================
TRIAL_DURATION_SEC = 15.0
MAX_ABS_ERROR_M     = 0.40   # 40cm perbezaan L dan R = dianggap terbabas
MIN_WALL_DIST_M     = 0.15   # rapat sangat dengan dinding (padan Emergency Stop tunnel_nav) -> abort
CRASH_PENALTY       = 1e6
TWIDDLE_TOL         = 0.05
STATUS_PUBLISH_HZ   = 5.0
LOG_FILE            = "tuning_log_tunnel.csv"
BEST_FILE           = "best_pid_tunnel.json"
PARAM_NAMES         = ["kp", "kd", "kp_wall"]  # NOTA: 'ki' sengaja tidak ditala setakat ini


class TwiddleTunerTunnel(object):
    def __init__(self, initial_params, initial_step_ratio=0.5):
        self.params = dict(initial_params)
        self.dp = {k: max(v * initial_step_ratio, 1e-5) for k, v in initial_params.items()}
        self.best_cost = 0.0
        self.best_params = dict(initial_params)
        self.iteration = 0
        self.status_msg = "Menunggu permulaan Twiddle Tunnel..."

        self.errors = []
        self.trial_active = False
        self.trial_aborted = False

        self.cmd_event = threading.Event()
        self.abort_requested = False

        # Nama node disahkan via `rosnode list` di Jetson - JANGAN tukar
        # tanpa sahkan semula, node "tunnel_nav_dashboard" TIDAK wujud.
        try:
            self.reconf_client = Client("tunnel_nav", timeout=10)
        except rospy.ROSException:
            rospy.logerr("[TWIDDLE-TUNNEL] GAGAL: Tiada service /tunnel_nav/set_parameters.")
            rospy.logerr("[TWIDDLE-TUNNEL] Pastikan node tunnel_nav berjalan dan dynamic_reconfigure tidak ralat.")
            os._exit(1)

        rospy.Subscriber("/tunnel_nav/debug", String, self._debug_cb, queue_size=1)
        rospy.Subscriber("/autotune_tunnel/cmd", String, self._cmd_cb, queue_size=1)
        self.pub_status = rospy.Publisher("/autotune_tunnel/status", String, queue_size=1)

        self._log_init()
        threading.Thread(target=self._status_loop).start()

    # -------------------- dashboard command (listen-only) --------------------
    def _cmd_cb(self, msg):
        cmd = (msg.data or "").strip().upper()
        if cmd == "NEXT":
            self.cmd_event.set()
        elif cmd == "ABORT":
            self.abort_requested = True
            if self.trial_active:
                self.trial_aborted = True
            self.cmd_event.set()

    def _status_loop(self):
        rate = rospy.Rate(STATUS_PUBLISH_HZ)
        while not rospy.is_shutdown():
            payload = {
                "iteration": self.iteration,
                "msg": self.status_msg,
                "testing_params": self.params,
                "best_params": self.best_params,
                "best_cost": self.best_cost,
                "trial_active": self.trial_active,
            }
            try:
                self.pub_status.publish(json.dumps(payload))
            except Exception:
                pass
            rate.sleep()

    def _wait_for_next(self, text):
        self.status_msg = text
        print("[TWIDDLE-TUNNEL] " + text)
        self.cmd_event.clear()
        self.cmd_event.wait()  # blok sehingga "NEXT" diterima dari dashboard

    # -------------------- logging --------------------
    def _log_init(self):
        new_file = not os.path.exists(LOG_FILE)
        self.log_fh = open(LOG_FILE, "a")
        self.log_writer = csv.writer(self.log_fh)
        if new_file:
            self.log_writer.writerow(["kp", "kd", "kp_wall", "cost", "n_samples", "aborted"])

    def _log_trial(self, params, cost, n_samples, aborted):
        self.log_writer.writerow([params["kp"], params["kd"], params["kp_wall"],
                                   cost, n_samples, aborted])
        self.log_fh.flush()

    # -------------------- /tunnel_nav/debug -> error --------------------
    def _debug_cb(self, msg):
        if not self.trial_active:
            return

        # Contoh string: "mode:CENTER L:0.350 R:0.410 F:1.500 steer:0.120 spd:0.15 TUN:1"
        parts = msg.data.split()
        data = {}
        for p in parts:
            if ':' in p:
                k, v = p.split(':', 1)
                data[k] = v

        try:
            mode = data.get('mode', 'IDLE')
            if 'STOP' in mode:
                # Hormat emergency-stop tunnel_nav sendiri - jangan override
                self.trial_aborted = True
                return

            left = float(data.get('L', 999.0))
            right = float(data.get('R', 999.0))

            if left < MIN_WALL_DIST_M or right < MIN_WALL_DIST_M:
                self.trial_aborted = True
                return

            if left < 10.0 and right < 10.0:  # buang bacaan sentinel/tak sah
                err = abs(left - right)
                self.errors.append(err)
                if err > MAX_ABS_ERROR_M:
                    self.trial_aborted = True
        except Exception:
            pass

    def _push_params(self, params):
        self.reconf_client.update_configuration(params)

    def _run_trial(self, params):
        # PENTING: gain calon BELUM di-push di sini - robot kekal guna
        # gain lama sehingga operator confirm sedia. Lihat nota
        # keselamatan di docstring atas fail ini.
        self.params = dict(params)
        print("\n[TWIDDLE-TUNNEL] Calon param (belum diaktifkan): %s" % params)

        self._wait_for_next(
            "Calon seterusnya: KP=%.5f KD=%.5f KP_WALL=%.5f (BELUM aktif). "
            "Pastikan robot BERHENTI & diletak semula di START TEROWONG, "
            "hidupkan robot, kemudian tekan Mula/Teruskan." % (
                params["kp"], params["kd"], params["kp_wall"]))

        if self.abort_requested:
            return CRASH_PENALTY

        # Operator dah confirm sedia - baru sekarang gain diaktifkan.
        self._push_params(params)
        time.sleep(0.3)

        self.errors = []
        self.trial_aborted = False
        self.trial_active = True
        self.status_msg = "Sedang merekod |L-R|... (%.0fs) - pantau robot" % TRIAL_DURATION_SEC
        print("[TWIDDLE-TUNNEL] Merekod selama %.0fs..." % TRIAL_DURATION_SEC)

        start = time.time()
        rate = rospy.Rate(20)
        while (time.time() - start) < TRIAL_DURATION_SEC and not rospy.is_shutdown():
            if self.trial_aborted:
                print("[TWIDDLE-TUNNEL] Trial di-abort (terlanggar dinding / emergency-stop / operator ABORT)")
                break
            rate.sleep()

        self.trial_active = False

        n = len(self.errors)
        if self.trial_aborted or n == 0:
            cost = CRASH_PENALTY
        else:
            cost = sum(e * e for e in self.errors) / float(n)

        self._log_trial(params, cost, n, self.trial_aborted)
        self.status_msg = "Trial selesai. Cost=%.5f (n=%d, aborted=%s). Letak robot semula di START." % (
            cost, n, self.trial_aborted)
        print("[TWIDDLE-TUNNEL] " + self.status_msg)
        return cost

    # -------------------- twiddle main loop --------------------
    def run(self):
        self.best_cost = self._run_trial(self.params)
        self.best_params = dict(self.params)
        self._save_best()

        while sum(self.dp.values()) > TWIDDLE_TOL and not rospy.is_shutdown() and not self.abort_requested:
            self.iteration += 1
            print("\n===== Iterasi Twiddle Tunnel #%d (jumlah dp=%.5f) =====" %
                  (self.iteration, sum(self.dp.values())))

            for name in PARAM_NAMES:
                original = self.best_params[name]
                candidate = dict(self.best_params)

                candidate[name] = max(0.0, original + self.dp[name])
                cost = self._run_trial(candidate)

                if cost < self.best_cost:
                    self.best_cost = cost
                    self.best_params = dict(candidate)
                    self.dp[name] *= 1.1
                    self._save_best()
                    continue

                candidate[name] = max(0.0, original - self.dp[name])
                cost = self._run_trial(candidate)

                if cost < self.best_cost:
                    self.best_cost = cost
                    self.best_params = dict(candidate)
                    self.dp[name] *= 1.1
                    self._save_best()
                else:
                    self.dp[name] *= 0.9

        if self.abort_requested:
            self.status_msg = "DIBATALKAN OLEH OPERATOR. Param terbaik setakat ini: %s (cost=%.5f)" % (self.best_params, self.best_cost)
        else:
            self.status_msg = "SELESAI. Param terbaik: %s (cost=%.5f)" % (self.best_params, self.best_cost)
        print("\n[TWIDDLE-TUNNEL] " + self.status_msg)
        self._push_params(self.best_params)
        self.log_fh.close()

    def _save_best(self):
        with open(BEST_FILE, "w") as f:
            json.dump({"params": self.best_params, "cost": self.best_cost}, f, indent=2)


def main():
    rospy.init_node("autotune_twiddle_tunnel", anonymous=True)

    initial = {"kp": 1.4, "kd": 0.8, "kp_wall": 6.0}
    print("[TWIDDLE-TUNNEL] Node autotune_twiddle_tunnel bermula. Nilai awal: %s" % initial)
    print("[TWIDDLE-TUNNEL] Buka tab 'Auto-Tune TUNNEL' pada dashboard untuk kawal proses ini.")
    print("[TWIDDLE-TUNNEL] Node ini TIDAK akan start/stop robot secara automatik.\n")

    tuner = TwiddleTunerTunnel(initial)
    tuner.run()


if __name__ == "__main__":
    main()