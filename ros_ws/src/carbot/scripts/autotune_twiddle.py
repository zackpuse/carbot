#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
autotune_twiddle.py
====================
Auto-tune PID (kp, ki, kd) untuk lane_follow.py guna algoritma Twiddle
(coordinate ascent). Node BERASINGAN - tidak mengubah lane_follow.py,
tidak publish ke /carbot/status (mission_control.py subscribe topic itu
juga dan mungkin bertindak atasnya - autotune elak sentuh terus).

Cara error_px dikira: skrip ini subscribe /lane_follow/mask_image
(topic sedia ada, read-only) dan kira centroid pixel putih pada
separuh bawah mask sendiri - tak perlu ubah lane_follow.py.

CARA GUNA (semua start/stop robot dibuat MANUAL oleh operator):
    1. Pastikan lane_follow.py sudah jalan (rosnode list ada /lane_follow).
    2. Jalankan skrip ini: python autotune_twiddle.py
    3. Letak CarBot di titik start litar, tekan ENTER bila skrip minta.
    4. Skrip akan push parameter kp/ki/kd baru via dynamic_reconfigure,
       kemudian minta awak START robot (guna kaedah biasa - button/dashboard).
    5. Skrip rekod error selama TRIAL_DURATION_SEC, kemudian minta awak STOP
       robot dan letak semula di titik start sebelum trial seterusnya.

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
TRIAL_DURATION_SEC = 15.0   # panjang setiap segmen ujian (selepas operator START)
MAX_ABS_ERROR_PX    = 220.0  # error melebihi ini -> auto-abort trial (penalty)
NO_LINE_FRAMES_ABORT = 10    # bilangan frame mask kosong berturutan -> abort
NO_LINE_PENALTY     = 1e6    # penalty bila garisan hilang / trial di-abort
TWIDDLE_TOL         = 0.02   # syarat berhenti (jumlah dp merentasi 3 param)
LOG_FILE            = "tuning_log.csv"
BEST_FILE           = "best_pid.json"
PARAM_NAMES         = ["kp", "ki", "kd"]


class TwiddleTuner(object):
    def __init__(self, initial_params, initial_step_ratio=0.5):
        self.params = dict(initial_params)
        self.dp = {k: max(v * initial_step_ratio, 1e-5) for k, v in initial_params.items()}
        self.best_cost = None
        self.best_params = dict(initial_params)

        self.errors = []
        self.trial_active = False
        self.trial_aborted = False
        self.no_line_count = 0

        self.cmd_event = threading.Event()
        self.ui_cmd = ""
        
        self.status_msg = "Menunggu permulaan Twiddle..."
        self.iteration = 0

        self.reconf_client = Client("lane_follow", timeout=10)
        rospy.Subscriber("/lane_follow/mask_image", Image, self._mask_cb, queue_size=1)
        rospy.Subscriber("/autotune/cmd", String, self._cmd_cb, queue_size=1)
        self.pub_status = rospy.Publisher("/autotune/status", String, queue_size=1)

        threading.Thread(target=self._publish_status_loop).start()

        self._log_init()

    # -------------------- Web UI Comm --------------------
    def _cmd_cb(self, msg):
        cmd = msg.data.strip()
        if cmd == "NEXT":
            self.cmd_event.set()
        elif cmd == "ABORT":
            self.trial_aborted = True

    def _publish_status_loop(self):
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            status_data = {
                "iteration": self.iteration,
                "msg": self.status_msg,
                "testing_params": self.params,
                "best_params": self.best_params,
                "best_cost": self.best_cost if self.best_cost else 0.0,
                "trial_active": self.trial_active
            }
            self.pub_status.publish(json.dumps(status_data))
            rate.sleep()

    def _wait_for_user(self, text):
        self.status_msg = text
        print("[TWIDDLE] " + text)
        self.cmd_event.clear()
        self.cmd_event.wait()

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
        print("\n[TWIDDLE] Uji param: %s" % params)
        self._push_params(params)
        time.sleep(0.3)  # bagi dynamic_reconfigure settle

        self._wait_for_user("Pastikan CarBot di titik START litar. Sedia untuk trial...")

        self.errors = []
        self.trial_aborted = False
        self.no_line_count = 0
        self.trial_active = True

        self.status_msg = "Sedang Merekod Error... (Sila pantau robot)"
        print("[TWIDDLE] >>> START CarBot SEKARANG (guna kaedah biasa). Merekod selama %.0fs..." %
              TRIAL_DURATION_SEC)

        start = time.time()
        rate = rospy.Rate(20)
        while (time.time() - start) < TRIAL_DURATION_SEC and not rospy.is_shutdown():
            if self.trial_aborted:
                print("[TWIDDLE] Trial di-abort awal (error melampau / garisan hilang / User Abort)")
                break
            rate.sleep()

        self.trial_active = False
        self._wait_for_user("Sila STOP robot! Angkat letak semula di START untuk teruskan.")

        n = len(self.errors)
        if self.trial_aborted or n == 0:
            cost = NO_LINE_PENALTY
        else:
            cost = sum(e * e for e in self.errors) / float(n)

        self._log_trial(params, cost, n, self.trial_aborted)
        print("[TWIDDLE] Cost = %.2f (n=%d, aborted=%s)" % (cost, n, self.trial_aborted))
        return cost

    # -------------------- twiddle main loop --------------------
    def run(self):
        self.iteration = 0
        self.best_cost = self._run_trial(self.params)
        self.best_params = dict(self.params)
        self._save_best()

        while sum(self.dp.values()) > TWIDDLE_TOL and not rospy.is_shutdown():
            self.iteration += 1
            print("\n===== Iterasi Twiddle #%d (jumlah dp=%.5f) =====" %
                  (self.iteration, sum(self.dp.values())))

            for name in PARAM_NAMES:
                original = self.params[name]

                self.params[name] = max(0.0, original + self.dp[name])
                cost = self._run_trial(self.params)

                if cost < self.best_cost:
                    self.best_cost = cost
                    self.best_params = dict(self.params)
                    self.dp[name] *= 1.1
                    self._save_best()
                else:
                    self.params[name] = max(0.0, original - self.dp[name])
                    cost = self._run_trial(self.params)

                    if cost < self.best_cost:
                        self.best_cost = cost
                        self.best_params = dict(self.params)
                        self.dp[name] *= 1.1
                        self._save_best()
                    else:
                        self.params[name] = original
                        self.dp[name] *= 0.9

        print("\n[TWIDDLE] SELESAI. Param terbaik: %s (cost=%.2f)" %
              (self.best_params, self.best_cost))
        self._push_params(self.best_params)
        self.log_fh.close()

    def _save_best(self):
        with open(BEST_FILE, "w") as f:
            json.dump({"params": self.best_params, "cost": self.best_cost}, f, indent=2)


def main():
    rospy.init_node("autotune_twiddle", anonymous=True)

    initial = {"kp": 0.008, "ki": 0.0001, "kd": 0.004}  # nilai live sebenar (dynparam get)
    print("[TWIDDLE] Mula dengan param: %s" % initial)
    print("[TWIDDLE] Buka Web Dashboard tab 'Auto-Tune PID' untuk mengawal skrip ini.")

    tuner = TwiddleTuner(initial)
    tuner.run()


if __name__ == "__main__":
    main()
