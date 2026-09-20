#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
autotune_twiddle_tunnel.py
==========================
Auto-tune PID (kp, kd, kp_wall) untuk tunnel_nav_dashboard.py guna algoritma Twiddle.
Node BERASINGAN - subscribe kepada /tunnel_nav/debug untuk mengira ralat (error = |L - R|).
Berkomunikasi secara terus dengan Web Dashboard melalui ROS topics.

OUTPUT:
    - tuning_log_tunnel.csv
    - best_pid_tunnel.json
"""
from __future__ import print_function
import rospy
import numpy as np
import csv
import json
import os
import time
import threading
from std_msgs.msg import String
from dynamic_reconfigure.client import Client

# ==================== SETTING TUNING ====================
TRIAL_DURATION_SEC  = 15.0
MAX_ABS_ERROR_M     = 0.40   # 40cm perbezaan L dan R = terbabas
MIN_WALL_DIST_M     = 0.15   # Kalau rapat sangat dinding = abort
CRASH_PENALTY       = 1e6
TWIDDLE_TOL         = 0.05
LOG_FILE            = "tuning_log_tunnel.csv"
BEST_FILE           = "best_pid_tunnel.json"
PARAM_NAMES         = ["kp", "kd", "kp_wall"]

class TwiddleTunerTunnel(object):
    def __init__(self, initial_params, initial_step_ratio=0.5):
        self.params = dict(initial_params)
        self.dp = {k: max(v * initial_step_ratio, 1e-5) for k, v in initial_params.items()}
        self.best_cost = None
        self.best_params = dict(initial_params)

        self.errors = []
        self.trial_active = False
        self.trial_aborted = False

        self.cmd_event = threading.Event()
        self.ui_cmd = ""
        
        self.status_msg = "Menunggu permulaan Twiddle Tunnel..."
        self.iteration = 0

        self.reconf_client = Client("tunnel_nav_dashboard", timeout=10)
        rospy.Subscriber("/tunnel_nav/debug", String, self._debug_cb, queue_size=1)
        rospy.Subscriber("/autotune_tunnel/cmd", String, self._cmd_cb, queue_size=1)
        self.pub_status = rospy.Publisher("/autotune_tunnel/status", String, queue_size=1)

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
        print("[TWIDDLE-TUNNEL] " + text)
        self.cmd_event.clear()
        self.cmd_event.wait()

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

    # -------------------- debug_cb -> error --------------------
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
                self.trial_aborted = True
                return
                
            left = float(data.get('L', 999.0))
            right = float(data.get('R', 999.0))

            if left < MIN_WALL_DIST_M or right < MIN_WALL_DIST_M:
                self.trial_aborted = True
                return

            if left < 10.0 and right < 10.0:
                err = abs(left - right)
                self.errors.append(err)

                if err > MAX_ABS_ERROR_M:
                    self.trial_aborted = True
                    
        except Exception as e:
            pass

    def _push_params(self, params):
        self.reconf_client.update_configuration(params)

    def _run_trial(self, params):
        print("\n[TWIDDLE-TUNNEL] Uji param: %s" % params)
        self._push_params(params)
        time.sleep(0.3)

        self._wait_for_user("Pastikan CarBot di titik START TEROWONG. Sedia untuk trial...")

        self.errors = []
        self.trial_aborted = False
        self.trial_active = True

        self.status_msg = "Sedang Merekod Error... (Sila pantau robot)"
        print("[TWIDDLE-TUNNEL] >>> START CarBot SEKARANG. Merekod %.0fs..." % TRIAL_DURATION_SEC)

        start = time.time()
        rate = rospy.Rate(20)
        while (time.time() - start) < TRIAL_DURATION_SEC and not rospy.is_shutdown():
            if self.trial_aborted:
                print("[TWIDDLE-TUNNEL] Trial di-abort (terlanggar dinding / manual abort)")
                break
            rate.sleep()

        self.trial_active = False
        self._wait_for_user("Sila STOP robot! Angkat letak semula di START untuk teruskan.")

        n = len(self.errors)
        if self.trial_aborted or n == 0:
            cost = CRASH_PENALTY
        else:
            cost = sum(e * e for e in self.errors) / float(n)

        self._log_trial(params, cost, n, self.trial_aborted)
        print("[TWIDDLE-TUNNEL] Cost = %.5f (n=%d, aborted=%s)" % (cost, n, self.trial_aborted))
        return cost

    # -------------------- twiddle main loop --------------------
    def run(self):
        self.iteration = 0
        self.best_cost = self._run_trial(self.params)
        self.best_params = dict(self.params)
        self._save_best()

        while sum(self.dp.values()) > TWIDDLE_TOL and not rospy.is_shutdown():
            self.iteration += 1
            print("\n===== Iterasi Twiddle Tunnel #%d (jumlah dp=%.5f) =====" %
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

        print("\n[TWIDDLE-TUNNEL] SELESAI. Param terbaik: %s (cost=%.5f)" %
              (self.best_params, self.best_cost))
        self._push_params(self.best_params)
        self.log_fh.close()

    def _save_best(self):
        with open(BEST_FILE, "w") as f:
            json.dump({"params": self.best_params, "cost": self.best_cost}, f, indent=2)


def main():
    rospy.init_node("autotune_twiddle_tunnel", anonymous=True)

    initial = {"kp": 1.4, "kd": 0.8, "kp_wall": 6.0} 
    print("[TWIDDLE-TUNNEL] Mula dengan param: %s" % initial)
    print("[TWIDDLE-TUNNEL] Buka Web Dashboard tab 'Auto-Tune TUNNEL' untuk mengawal skrip ini.")

    tuner = TwiddleTunerTunnel(initial)
    tuner.run()


if __name__ == "__main__":
    main()
