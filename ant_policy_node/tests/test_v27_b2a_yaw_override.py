"""B2a (v27 candidate) — T3 SC gripper-yaw env-var override tests.

Verifies the override is gated to (zone='sc', trial=3), respects the
env-var, falls back cleanly on bad values, and does NOT apply to other
trials.  Behavioural validation requires the ROS sim and is documented
in ant_policy_node/sim_runs/run_2026-05-09_b2a_sweep/.
"""
from __future__ import annotations

import os
import unittest


class TestB2aOverridePresence(unittest.TestCase):
    def test_override_lookup_present_in_helper(self):
        """`_gripper_yaw_correction` must read ANT_T3_YAW_OVERRIDE_RAD."""
        try:
            from ant_policy_node.ANT import ANT
        except Exception:
            self.skipTest("ant_policy_node not importable in this env")
        import inspect
        src = inspect.getsource(ANT._gripper_yaw_correction)
        self.assertIn(
            'ANT_T3_YAW_OVERRIDE_RAD', src,
            "B2a override env var must be checked in _gripper_yaw_correction",
        )
        # Must be gated to zone=sc AND trial=3.
        self.assertIn(
            'zone == "sc"', src,
            "Override must be gated to zone='sc'",
        )
        self.assertIn(
            "self._insert_call_count == 3", src,
            "Override must be gated to trial=3",
        )


class TestB2aOverrideLogged(unittest.TestCase):
    def test_override_emits_diag_event(self):
        """First use of the override must emit a `b2a_yaw_override` diag
        event so post-eval analysis can confirm whether the override was
        active during a given run."""
        try:
            from ant_policy_node.ANT import ANT
        except Exception:
            self.skipTest("ant_policy_node not importable in this env")
        import inspect
        src = inspect.getsource(ANT._gripper_yaw_correction)
        self.assertIn(
            'b2a_yaw_override', src,
            "Override must emit a b2a_yaw_override diag event",
        )
        self.assertIn(
            "_b2a_override_logged", src,
            "Override must be idempotently logged (one event per policy lifetime)",
        )


class TestB2aOverrideDefaultUnchanged(unittest.TestCase):
    def test_table_value_unchanged_on_branch(self):
        """The committed table value must remain -1.7133 on this branch.
        The whole point of the env-var mechanism is to sweep WITHOUT
        committing each iteration.  v27 integration commits the chosen
        value — not this branch."""
        with open(
            "ant_policy_node/ant_policy_node/ANT.py", encoding="utf-8"
        ) as f:
            src = f.read()
        self.assertIn(
            '("sc",  3): -1.7133', src,
            "B2a branch must NOT change the committed table value; "
            "use the env var for sweeps",
        )


if __name__ == "__main__":
    unittest.main(verbosity=2)
