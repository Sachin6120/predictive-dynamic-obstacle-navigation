#!/usr/bin/env python3
"""A launch leader exiting must not hide its still-running child process."""
import os
import signal
import subprocess
import sys
import unittest

from run import group_alive, stop


class CleanupTests(unittest.TestCase):
    def test_exited_leader_child_is_stopped(self):
        child = 'import time; time.sleep(30)'
        leader = ('import subprocess,sys; '
                  f'subprocess.Popen([sys.executable,"-c",{child!r}])')
        process = subprocess.Popen([sys.executable,'-c',leader],start_new_session=True,
                                   stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
        try:
            process.wait(timeout=5)
            self.assertTrue(group_alive(process.pid))
            stop(process)
            self.assertFalse(group_alive(process.pid))
        finally:
            try:
                os.killpg(process.pid,signal.SIGKILL)
            except ProcessLookupError:
                pass


if __name__=='__main__':
    unittest.main()
