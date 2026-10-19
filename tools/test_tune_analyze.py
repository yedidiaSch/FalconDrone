#!/usr/bin/env python3
"""Tests for tune_analyze.py, using tune_sim.py as the rig.

    python tools/test_tune_analyze.py
"""
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import tune_analyze as ta  # noqa: E402
import tune_sim as sim     # noqa: E402


def advise(**kw):
    return ta.analyze(ta.parse_log(sim.run_test(**kw))[0])['advice']


class ParseLog(unittest.TestCase):
    def test_splits_tests_and_ignores_noise(self):
        lines = [
            '12,0.1,0.2,0,0,0,1000,1000,1000,1000,0,1000,0,0,0',     # normal CSV
            'T,5,1,60,60,1,0,1,0,0,1,1300,0',                        # before any start
            '[TUNE] start axis=p mode=RATE amp=40.0 step=300 reps=1 kp=0.600 ki=0.000 '
            'kd=0.0200 lim=300 akp=2.000',
            'T,10,1,40,40,3.0,0,22,0,0,22,1300,0',
            'T,20,1,40,40,garbage,0,22,0,0,22,1300,0',                # bad number
            'T,30,1,40,40',                                           # short line
            '[ARM] something else',
            '[TUNE] aborted: pilot moved the stick',
            '[TUNE] start axis=r mode=ANGLE amp=10.0 step=800 reps=2 kp=1 ki=0 kd=0 lim=300 akp=4',
        ]
        tests = ta.parse_log(lines)
        self.assertEqual(len(tests), 2)
        self.assertEqual(tests[0].params['axis'], 'p')
        self.assertEqual(tests[0].params['kd'], 0.02)
        self.assertEqual(len(tests[0].rows), 1)
        self.assertEqual(tests[0].end, 'aborted: pilot moved the stick')
        self.assertEqual(tests[1].params['mode'], 'ANGLE')
        self.assertEqual(tests[1].end, '')

    def test_no_data(self):
        t = ta.parse_log(['[TUNE] start axis=r mode=RATE amp=60 kp=1 ki=0 kd=0 lim=300 akp=2',
                          '[TUNE] done'])[0]
        self.assertEqual(ta.analyze(t)['advice']['verdict'], 'NO DATA')


class RateAdvice(unittest.TestCase):
    def test_sluggish_raises_p(self):
        a = advise(kp=0.2, ki=0.0, kd=0.0)
        self.assertEqual(a['verdict'], 'ADJUST')
        self.assertGreater(a['gains']['kp'], 0.2)
        self.assertTrue(a['command'].startswith('g rr '))

    def test_oscillation_lowers_p(self):
        a = advise(kp=8.0, ki=0.0, kd=0.02)
        self.assertEqual(a['verdict'], 'ADJUST')
        self.assertLess(a['gains']['kp'], 8.0)

    def test_overshoot_without_d_adds_d(self):
        a = advise(kp=2.1, ki=0.0, kd=0.0)
        self.assertEqual(a['verdict'], 'ADJUST')
        self.assertGreater(a['gains']['kd'], 0.0)
        self.assertEqual(a['gains']['kp'], 2.1)

    def test_offset_adds_i(self):
        a = advise(kp=0.9, ki=0.0, kd=0.02, disturbance=600)
        self.assertEqual(a['verdict'], 'ADJUST')
        self.assertGreater(a['gains']['ki'], 0.0)

    def test_offset_below_liftoff_asks_for_throttle(self):
        a = advise(kp=0.9, ki=0.0, kd=0.02, disturbance=600, thr=1100)
        self.assertEqual(a['verdict'], 'CHECK')
        self.assertIn('LIFTOFF', a['why'])

    def test_saturation_invalidates(self):
        log = sim.run_test(kp=1.0, ki=0.0, kd=0.02)
        log = [l[:-1] + '1' if l.startswith('T,') else l for l in log]   # all saturated
        res = ta.analyze(ta.parse_log(log)[0])
        self.assertEqual(res['advice']['verdict'], 'INVALID')
        self.assertIsNone(res['advice']['command'])

    def test_good_gains(self):
        self.assertEqual(advise(kp=2.17, ki=0.0, kd=0.0264)['verdict'], 'GOOD')


class AngleAdvice(unittest.TestCase):
    def test_slow_raises_angle_p(self):
        a = advise(kp=2.17, ki=0.0, kd=0.0264, mode='ANGLE', amp=10.0, akp=2.0)
        self.assertEqual(a['verdict'], 'ADJUST')
        self.assertTrue(a['command'].startswith('g ar '))
        self.assertGreater(a['gains']['akp'], 2.0)

    def test_good_angle(self):
        a = advise(kp=2.17, ki=0.0, kd=0.0264, mode='ANGLE', amp=10.0, akp=16.0)
        self.assertEqual(a['verdict'], 'GOOD')


class Autotune(unittest.TestCase):
    """Following the advice converges from bad starting points (on the model)."""

    def check(self, kp, kd, disturbance=0.0):
        import contextlib
        import io
        with contextlib.redirect_stdout(io.StringIO()):
            kp, ki, kd, verdict = sim.autotune(kp, 0.0, kd, 40, disturbance, 1350)
        self.assertTrue(verdict.startswith('GOOD'), verdict)
        final = advise(kp=kp, ki=ki, kd=kd, disturbance=disturbance, seed=99)
        self.assertEqual(final['verdict'], 'GOOD', final['why'])

    def test_from_too_low(self):
        self.check(0.2, 0.0)

    def test_from_too_high(self):
        self.check(8.0, 0.02)

    def test_with_offset(self):
        self.check(0.6, 0.02, disturbance=400)


if __name__ == '__main__':
    unittest.main(verbosity=1)
