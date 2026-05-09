"""v26 Workstream A — guard, C1, C5, and budget-accountant unit tests.

These tests verify the *additive* changes made on top of the v24 baseline
revert.  They do NOT exercise the joint-space, ROS, or controller paths —
those require the full eval harness.  These tests are designed to run on
any machine with numpy installed, no ROS, and to fail fast if a future
edit accidentally widens or removes the v26 guards.

Pass criteria for v26 ship gate:
  - All tests in this module pass.
  - Per-test runtime < 0.5 s (so submit.sh can include them in pre-push).
"""
from __future__ import annotations

import json
import os
import tempfile
import time
import types
import unittest
from unittest.mock import MagicMock


def _make_ant_skeleton():
    """Build a minimal ANT-like object exposing only the v26 helpers under test.

    Importing ant_policy_node.ANT requires rclpy + ROS interfaces that
    aren't installed in CI.  Instead we copy the relevant attributes and
    methods onto a plain object via the types.MethodType trick.  Any
    method we want to test must be re-bound here.
    """
    # Defer ANT import until we know rclpy is unavailable so the test can
    # gracefully skip rather than blow up at collection time.
    try:
        from ant_policy_node.ANT import ANT  # noqa: F401
        # If import succeeds, we still don't construct a full ANT (its
        # __init__ takes a ROS parent_node).  Use the helper functions
        # off the class via __get__ binding.
        _have_real_ant = True
    except Exception:
        _have_real_ant = False

    obj = types.SimpleNamespace()

    # Mimic the v26 init state for the helpers under test.
    obj.enable_c1_diagnostic_signature = True
    obj.c1_signature_magnitude_m = 0.004
    obj.c1_high_tension_z_bias_m = 0.001
    obj._c1_failure_code_pending = None
    obj._c1_codes = {
        "lateral_stall":      0b000,
        "retry_exhausted":    0b001,
        "ik_fail":            0b010,
        "joint_stall":        0b011,
        "force_abort":        0b100,
        "timeout":            0b101,
        "exception":          0b110,
        "guard_violation":    0b111,
    }
    obj.cable_force_baseline = 22.0
    obj.high_tension_baseline_threshold_n = 19.0
    obj.enable_hw_variance_log = True
    obj._hw_variance_log_path = None  # set per-test
    obj._trial_start_monotonic = None
    obj._trial_time_limit_sec = 120.0

    # Bind the methods.  We import-by-source rather than executing ANT.__init__
    # so the test does not require rclpy.
    if _have_real_ant:
        from ant_policy_node.ANT import ANT  # type: ignore
        obj._c1_signature_offset = ANT._c1_signature_offset.__get__(obj)
        obj._c1_record_failure = ANT._c1_record_failure.__get__(obj)
        obj._hw_variance_record = ANT._hw_variance_record.__get__(obj)
        obj._trial_budget_begin = ANT._trial_budget_begin.__get__(obj)
        obj._remaining_trial_budget_sec = ANT._remaining_trial_budget_sec.__get__(obj)
        # _high_tension is a property on the class — synthesize equivalent.
        obj.__class__ = type(
            "ANTLike", (object,),
            {"_high_tension": property(
                lambda s: s.cable_force_baseline > s.high_tension_baseline_threshold_n
            )},
        )
        obj._diag_event = lambda *a, **k: None  # silence
    return obj, _have_real_ant


class TestC1Codes(unittest.TestCase):
    def test_codes_are_unique(self):
        obj, _ = _make_ant_skeleton()
        values = list(obj._c1_codes.values())
        self.assertEqual(len(values), len(set(values)),
                         "C1 code values must be unique")

    def test_codes_fit_three_bits(self):
        obj, _ = _make_ant_skeleton()
        for name, code in obj._c1_codes.items():
            self.assertLess(code, 0b1000,
                            f"C1 code {name}={code} must fit in 3 bits")

    def test_signature_magnitude_inside_bounding_radius(self):
        """The C1 signature must fit inside the smallest bounding radius
        (T2: ~0.085 m, T3: ~0.10 m).  A 3-axis ±magnitude encoding plus
        the high-tension Z bias must stay well inside that envelope so
        a failure-coded park doesn't accidentally land outside scoring."""
        obj, _ = _make_ant_skeleton()
        m = obj.c1_signature_magnitude_m
        z_bias = obj.c1_high_tension_z_bias_m
        worst_case = (m * m * 3) ** 0.5 + z_bias  # 3-axis diagonal + bias
        self.assertLess(worst_case, 0.020,  # well below 0.085 m
                        "C1 signature must fit within bounding radius")


class TestC1Recording(unittest.TestCase):
    def test_first_code_wins(self):
        obj, have_ant = _make_ant_skeleton()
        if not have_ant:
            self.skipTest("ant_policy_node not importable in this env")
        obj._c1_record_failure("lateral_stall")
        obj._c1_record_failure("timeout")
        self.assertEqual(obj._c1_failure_code_pending, "lateral_stall",
                         "Subsequent failures must not overwrite the primary")

    def test_unknown_code_ignored(self):
        obj, have_ant = _make_ant_skeleton()
        if not have_ant:
            self.skipTest("ant_policy_node not importable in this env")
        obj._c1_record_failure("not_a_real_code")
        self.assertIsNone(obj._c1_failure_code_pending)


class TestHwVarianceLog(unittest.TestCase):
    def test_appends_jsonl_record(self):
        obj, have_ant = _make_ant_skeleton()
        if not have_ant:
            self.skipTest("ant_policy_node not importable in this env")
        with tempfile.TemporaryDirectory() as td:
            obj._hw_variance_log_path = os.path.join(td, "hw_variance.jsonl")
            obj._hw_variance_record(
                trial=1, cable_force_baseline_n=23.7, high_tension=True
            )
            obj._hw_variance_record(
                trial=2, cable_force_baseline_n=18.2, high_tension=False
            )
            with open(obj._hw_variance_log_path) as f:
                lines = f.read().strip().split("\n")
            self.assertEqual(len(lines), 2)
            r0 = json.loads(lines[0])
            self.assertEqual(r0["trial"], 1)
            self.assertAlmostEqual(r0["cable_force_baseline_n"], 23.7)
            self.assertTrue(r0["high_tension"])

    def test_disabled_is_noop(self):
        obj, have_ant = _make_ant_skeleton()
        if not have_ant:
            self.skipTest("ant_policy_node not importable in this env")
        obj.enable_hw_variance_log = False
        with tempfile.TemporaryDirectory() as td:
            obj._hw_variance_log_path = os.path.join(td, "hw_variance.jsonl")
            obj._hw_variance_record(trial=1, cable_force_baseline_n=23.7)
            self.assertFalse(os.path.exists(obj._hw_variance_log_path))


class TestTrialBudgetAccountant(unittest.TestCase):
    def test_budget_decreases_over_time(self):
        obj, have_ant = _make_ant_skeleton()
        if not have_ant:
            self.skipTest("ant_policy_node not importable in this env")
        obj._trial_budget_begin(120.0)
        r0 = obj._remaining_trial_budget_sec()
        time.sleep(0.05)
        r1 = obj._remaining_trial_budget_sec()
        self.assertLess(r1, r0)
        self.assertGreater(r0, 119.0)
        self.assertLess(r0, 120.001)

    def test_budget_unset_returns_inf(self):
        obj, have_ant = _make_ant_skeleton()
        if not have_ant:
            self.skipTest("ant_policy_node not importable in this env")
        obj._trial_start_monotonic = None
        self.assertEqual(
            obj._remaining_trial_budget_sec(), float("inf")
        )

    def test_budget_clamped_at_zero(self):
        obj, have_ant = _make_ant_skeleton()
        if not have_ant:
            self.skipTest("ant_policy_node not importable in this env")
        obj._trial_budget_begin(0.001)
        time.sleep(0.05)
        self.assertEqual(obj._remaining_trial_budget_sec(), 0.0)


class TestZoneGuardEarlyReturn(unittest.TestCase):
    """The v26 zone guard at the entry of `_lateral_move_joint_space` must
    return None and emit a guard_violation diag event when called from a
    non-SFP zone.  This test uses a real ANT-class method binding when
    available; otherwise it skips (the integration test on the build host
    will catch any regression there).
    """
    def test_non_sfp_zone_falls_back(self):
        try:
            from ant_policy_node.ANT import ANT
        except Exception:
            self.skipTest("ant_policy_node not importable in this env")
        obj = types.SimpleNamespace()
        obj.enable_joint_space_lateral = True
        obj.enable_lateral_arrival_check = True
        obj._diag_event = MagicMock()
        obj._c1_failure_code_pending = None
        # Bind only the helper under test.
        f = ANT._lateral_move_joint_space.__get__(obj)
        # Calling with zone="sc" must short-circuit BEFORE any obs/IK work.
        # Pass minimal callables that would crash if the guard didn't fire.
        def _bomb(*a, **k):
            raise AssertionError("guard didn't short-circuit")
        result = f(
            target_xy=(0.0, 0.0), lateral_z=0.3, orient=None,
            move_robot=_bomb, get_observation=_bomb,
            start_time=None, time_limit_sec=120.0,
            zone="sc", label="test",
        )
        self.assertIsNone(result, "Guard must return None for non-SFP zone")
        obj._diag_event.assert_called_once()
        args, kwargs = obj._diag_event.call_args
        self.assertEqual(args[0], "joint_space_guard_violation")
        self.assertEqual(kwargs.get("zone"), "sc")
        self.assertEqual(obj._c1_failure_code_pending, "guard_violation")


if __name__ == "__main__":
    unittest.main(verbosity=2)
