#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
autotune_twiddle.py
====================
Auto-tune PID (kp, ki, kd) untuk lane_follow.py guna algoritma Twiddle
(coordinate ascent).

REKA BENTUK KESELAMATAN (PENTING - baca sebelum jalankan):
    - Skrip ini TIDAK dimasukkan dalam carbot_full.launch. Ia WAJIB
      dijalankan secara manual di terminal Jetson dahulu:
          python autotune_twiddle.py
      Selagi tidak dijalankan, butang di web dashboard ("Auto-Tune PID"
      tab) tiada kesan langsung - ia cuma publish ke /autotune/cmd yang
      tiada sesiapa dengar.
    - Skrip ini LISTEN-ONLY terhadap /autotune/cmd (NEXT / ABORT) -
      ia tidak initiate apa-apa sendiri, cuma bertindak balas kepada
      arahan yang diterima.
    - Skrip TIDAK publish/subscribe /carbot/status - mission_control.py
      turut subscribe topic itu; autotune elak sentuh terus supaya tidak
      trigger logik state-machine mission_control secara tidak sengaja.
      START/STOP fizikal robot kekal tanggungjawab operator (butang
      fizikal / dashboard sedia ada), BUKAN skrip ini.
    - error_px dikira SENDIRI oleh skrip ini daripada /lane_follow/mask_image
      (topic sedia ada, read-only) - lane_follow.py tidak diubah langsung.

ALIRAN SATU TRIAL (1 klik NEXT setiap trial):
    1. Skrip pilih calon parameter (kp/ki/kd) seterusnya, TAPI BELUM push
       ke dynamic_reconfigure - robot kekal guna gain SEBELUM ini.
    2. Publish status "menunggu" (trial_active=False) - dashboard papar
       calon yang bakal diuji berserta amaran "BELUM aktif".
    3. Operator pastikan robot BERHENTI & diletak semula di titik start,
       hidupkan semula robot, kemudian tekan "Mula/Teruskan" pada
       dashboard -> hantar "NEXT" ke /autotune/cmd.
    4. HANYA SELEPAS NEXT diterima, skrip push gain calon ke
       dynamic_reconfigure, tunggu 0.3s settle, baru mula rekod error_px
       selama TRIAL_DURATION_SEC saat (trial_active=True, butang
       dashboard auto-disable).
    5. Auto-abort awal jika error melampau / garisan hilang berturutan,
       ATAU jika operator tekan "Batal" (ABORT) semasa trial berjalan -
       tak perlu tunggu genap TRIAL_DURATION_SEC.
    6. Cost dikira, bandingkan dengan terbaik setakat ini, param
       seterusnya ditentukan ikut logik Twiddle. Kembali ke langkah 1.

OUTPUT:
    - tuning_log.csv   : setiap trial (kp, ki, kd, cost, n_samples, aborted)
    - best_pid.json    : parameter terbaik yang dijumpai setakat ini
"""
from __future__ import print_function
import rospy
import numpy as np
import csv
import json
import os
import time
import threading
from sensor_msgs.msg import Image
from std_msgs.msg import String
from dynamic_reconfigure.client import Client

# ==================== SETTING TUNING ====================
TRIAL_DURATION_SEC  = 15.0   # panjang setiap segmen rekod (selepas NEXT diterima)
MAX_ABS_ERROR_PX     = 220.0  # error melebihi ini -> auto-abort trial (penalty)
NO_LINE_FRAMES_ABORT = 10     # bilangan frame mask kosong berturutan -> abort
NO_LINE_PENALTY      = 1e6    # penalty bila garisan hilang / trial di-abort
TWIDDLE_TOL          = 0.02   # syarat berhenti (jumlah dp merentasi 3 param)
STATUS_PUBLISH_HZ    = 5.0
LOG_FILE             = "tuning_log.csv"
BEST_FILE            = "best_pid.json"
PARAM_NAMES          = ["kp", "ki", "kd"]


class TwiddleTuner(object):
    def __init__(self, initial_params, initial_step_ratio=0.5):
        self.params = dict(initial_params)          # calon sedang/akan diuji
        self.dp = {k: max(v * initial_step_ratio, 1e-5) for k, v in initial_params.items()}
        self.best_cost = 0.0
        self.best_params = dict(initial_params)
        self.iteration = 0
        self.status_msg = "Menunggu permulaan Twiddle..."

        self.errors = []
        self.trial_active = False
        self.trial_aborted = False
        self.no_line_count = 0

        # NEXT/ABORT daripada dashboard - listen only, tidak initiate apa-apa
        self.cmd_event = threading.Event()
        self.abort_requested = False

        self.reconf_client = Client("lane_follow", timeout=10)
        rospy.Subscriber("/lane_follow/mask_image", Image, self._mask_cb, queue_size=1)
        rospy.Subscriber("/autotune/cmd", String, self._cmd_cb, queue_size=1)
        self.pub_status = rospy.Publisher("/autotune/status", String, queue_size=1)

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

    def _wait_for_next(self, msg_text):
        self.status_msg = msg_text
        print("[TWIDDLE] " + msg_text)
        self.cmd_event.clear()
        self.cmd_event.wait()  # blok sehingga "NEXT" diterima dari dashboard

    # -------------------- logging --------------------
    def _log_init(self):
        new_file = not os.path.exists(LOG_FILE)
        self.log_fh = open(LOG_FILE, "a")
        self.log_writer = csv.writer(self.log_fh)
        if new_file:
            self.log_writer.writerow(["kp", "ki", "kd", "cost", "n_samples", "aborted"])

    def _log_trial(self, params, cost, n_samples, aborted):
        self.log_writer.writerow([params["kp"], params["ki"], params["kd"],
                                   cost, n_samples, aborted])
        self.log_fh.flush()

    # -------------------- mask_image -> error_px --------------------
    def _mask_cb(self, msg):
        if not self.trial_active:
            return
        try:
            mask = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width)
        except Exception:
            return

        w = msg.width
        lower_half = mask[msg.height // 2:, :]  # bahagian bawah frame lebih stabil
        col_sums = lower_half.sum(axis=0).astype(np.float64)
        total = col_sums.sum()

        if total < 1.0:
            self.no_line_count += 1
            if self.no_line_count >= NO_LINE_FRAMES_ABORT:
                self.trial_aborted = True
            return

        self.no_line_count = 0
        xs = np.arange(w, dtype=np.float64)
        centroid_x = float((col_sums * xs).sum() / total)
        err = centroid_x - (w / 2.0)

        self.errors.append(err)
        if abs(err) > MAX_ABS_ERROR_PX:
            self.trial_aborted = True

    def _push_params(self, params):
        self.reconf_client.update_configuration(params)

    def _run_trial(self, params):
        # PENTING: gain PID calon ini SENGAJA belum di-push lagi di sini.
        # self.params ditetapkan dulu (untuk paparan "testing_params" pada
        # dashboard), tapi robot masih guna gain LAMA sehingga operator
        # confirm (tekan NEXT) yang robot dah berhenti & sedia semula.
        # Ini elak gain baru tiba-tiba aktif pada robot yang mungkin masih
        # bergerak/coasting daripada trial sebelumnya.
        self.params = dict(params)
        print("\n[TWIDDLE] Calon param (belum diaktifkan): %s" % params)

        self._wait_for_next(
            "Calon seterusnya: KP=%.5f KI=%.5f KD=%.5f (BELUM aktif). Pastikan "
            "robot BERHENTI & diletak semula di START, hidupkan robot, kemudian "
            "tekan Mula/Teruskan untuk aktifkan gain ini & mula rekod." % (
                params["kp"], params["ki"], params["kd"]))

        # Operator dah confirm sedia - baru sekarang gain diaktifkan.
        self._push_params(params)
        time.sleep(0.3)  # bagi dynamic_reconfigure settle

        self.errors = []
        self.trial_aborted = False
        self.no_line_count = 0
        self.trial_active = True
        self.status_msg = "Sedang merekod error... (%.0fs) - pantau robot" % TRIAL_DURATION_SEC
        print("[TWIDDLE] Merekod selama %.0fs..." % TRIAL_DURATION_SEC)

        start = time.time()
        rate = rospy.Rate(20)
        while (time.time() - start) < TRIAL_DURATION_SEC and not rospy.is_shutdown():
            if self.trial_aborted:
                print("[TWIDDLE] Trial di-abort (error melampau / garisan hilang / operator ABORT)")
                break
            rate.sleep()

        self.trial_active = False

        n = len(self.errors)
        if self.trial_aborted or n == 0:
            cost = NO_LINE_PENALTY
        else:
            cost = sum(e * e for e in self.errors) / float(n)

        self._log_trial(params, cost, n, self.trial_aborted)
        self.status_msg = "Trial selesai. Cost=%.2f (n=%d, aborted=%s). Letak robot semula di START." % (
            cost, n, self.trial_aborted)
        print("[TWIDDLE] " + self.status_msg)
        return cost

    # -------------------- twiddle main loop --------------------
    def run(self):
        self.best_cost = self._run_trial(self.params)
        self.best_params = dict(self.params)
        self._save_best()

        while sum(self.dp.values()) > TWIDDLE_TOL and not rospy.is_shutdown():
            self.iteration += 1
            print("\n===== Iterasi Twiddle #%d (jumlah dp=%.5f) =====" %
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

        self.status_msg = "SELESAI. Param terbaik: %s (cost=%.2f)" % (self.best_params, self.best_cost)
        print("\n[TWIDDLE] " + self.status_msg)
        self._push_params(self.best_params)
        self.log_fh.close()

    def _save_best(self):
        with open(BEST_FILE, "w") as f:
            json.dump({"params": self.best_params, "cost": self.best_cost}, f, indent=2)


def main():
    rospy.init_node("autotune_twiddle", anonymous=True)

    initial = {"kp": 0.008, "ki": 0.0001, "kd": 0.004}  # nilai live sebenar (dynparam get)
    print("[TWIDDLE] Node autotune_twiddle bermula. Nilai awal: %s" % initial)
    print("[TWIDDLE] Buka tab 'Auto-Tune PID' pada dashboard untuk kawal proses ini.")
    print("[TWIDDLE] Node ini TIDAK akan start/stop robot secara automatik.\n")

    tuner = TwiddleTuner(initial)
    tuner.run()


if __name__ == "__main__":
    main()
