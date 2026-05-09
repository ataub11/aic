"""B1 (v27 candidate) — T1 joint-space escalation flag tests.

Verifies the flag exists, defaults to False (v26 compatibility), and the
residual threshold is sane.  The actual T1 escalation behavior requires
the full ROS sim and is validated by the Eng-3 sim runs documented in
ant_policy_node/sim_runs/run_2026-05-09_b1_sweep/.
"""
from __future__ import annotations

import unittest


class TestB1FlagDefaults(unittest.TestCase):
    def test_flag_exists_and_defaults_off(self):
        try:
            from ant_policy_node.ANT import ANT  # noqa: F401
        except Exception:
            self.skipTest("ant_policy_node not importable in this env")
        # Inspect __init__ source-level for the flag default.  This avoids
        # constructing an ANT instance (which requires a ROS parent_node).
        import inspect
        src = inspect.getsource(ANT.__init__)
        self.assertIn("self.joint_space_t1_sfp", src,
                      "B1 flag must exist on ANT instance")
        # Default must be False (v26 compatibility — does not enable T1
        # joint-space until v27 bake-off explicitly flips it on).
        self.assertRegex(
            src,
            r"self\.joint_space_t1_sfp\s*=\s*False",
            "B1 flag MUST default to False so v26-compatible builds do not "
            "accidentally enable T1 joint-space",
        )

    def test_residual_threshold_sane(self):
        try:
            from ant_policy_node.ANT import ANT  # noqa: F401
        except Exception:
            self.skipTest("ant_policy_node not importable in this env")
        import inspect, re
        src = inspect.getsource(ANT.__init__)
        m = re.search(
            r"self\.joint_space_t1_residual_threshold_m\s*=\s*([0-9.]+)", src
        )
        self.assertIsNotNone(
            m, "B1 residual threshold must be defined in __init__"
        )
        threshold = float(m.group(1))
        # Sanity bounds: tighter than the Bug 106 arrival tolerance (2.5 cm
        # is the same value Bug 106 uses) but not so tight that T1 sim
        # noise triggers escalation on every run.
        self.assertGreaterEqual(threshold, 0.020, "threshold must be ≥ 2 cm")
        self.assertLessEqual(threshold, 0.050, "threshold must be ≤ 5 cm")


class TestB1CallSitePresent(unittest.TestCase):
    def test_b1_escalation_branch_in_t1_sfp_path(self):
        """The B1 escalation must live INSIDE the T1 SFP branch and ONLY
        run after `_lateral_arrival_check_and_retry`.  This test reads the
        source to verify the call ordering — the actual behaviour is sim-
        validated, but we want to catch a refactor that accidentally moves
        the escalation before the arrival check (which would fire it on
        every T1 trial)."""
        import re
        with open(
            "ant_policy_node/ant_policy_node/ANT.py", encoding="utf-8"
        ) as f:
            src = f.read()
        # Find the T1 SFP arrival-check call and the closing of the
        # SFP-fast-path block.  B1 escalation must sit between them.
        idx_arrival = src.find(
            'label="Stage 1 SFP T1 WP2",'
        )
        self.assertGreater(
            idx_arrival, 0, "Could not locate T1 arrival-check call in ANT.py"
        )
        # Walk backwards from this label to confirm it's the arrival-check
        # invocation (not the earlier _move_to_pose_and_wait with a similar
        # label).  The arrival-check helper is _lateral_arrival_check_and_retry.
        preceding_window = src[max(0, idx_arrival - 600):idx_arrival]
        self.assertIn(
            "_lateral_arrival_check_and_retry", preceding_window,
            "B1 must NOT remove or relocate the existing Bug 106 arrival check"
            " (label kwarg should be on the arrival-check call site)",
        )
        # Locate the B1 comment block marker AFTER the arrival check label.
        # We use the unique "B1 (v27 candidate)" comment marker rather than
        # the bare "B1 escalation" string (the latter also appears as a
        # debug-label suffix INSIDE the call site, which is structurally
        # later than the comment marker).
        idx_b1 = src.find("B1 (v27 candidate)", idx_arrival)
        self.assertGreater(
            idx_b1, idx_arrival,
            "B1 escalation comment marker must appear AFTER the T1 arrival-check call",
        )
        # And the B1 block must invoke _lateral_move_joint_space.
        b1_block = src[idx_b1:idx_b1 + 2500]
        self.assertIn(
            "_lateral_move_joint_space", b1_block,
            "B1 escalation must call _lateral_move_joint_space",
        )
        # And the B1 block must be flag-gated so v26-compat is preserved.
        self.assertIn(
            "if self.joint_space_t1_sfp", b1_block,
            "B1 escalation must be gated by self.joint_space_t1_sfp flag",
        )


if __name__ == "__main__":
    unittest.main(verbosity=2)
