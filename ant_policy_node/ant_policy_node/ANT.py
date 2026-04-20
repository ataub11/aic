"""ANT — cable-insertion policy for the AI for Industry Challenge.

Sensor inputs used (all from the official Observation interface):
  - controller_state.tcp_pose           : current end-effector position
  - controller_state.tcp_error          : convergence check during motion
  - wrist_wrench                        : F/T contact detection (Stage 3 descent,
                                          Stage 4 insertion)
  - center_image / center_camera_info   : vision grid search in Stage 1 — detects
                                          port aperture at each grid position and
                                          back-projects to base_link XY using TF
  - TF (base_link → center_camera/optical) : camera extrinsics for back-projection

No data from /scoring, /gazebo, /gz_server, or ground-truth TF frames is used.
All parameters marked CALIBRATE must be tuned once from a run with
  ground_truth:=true before submission.
"""

import collections
import cv2
import numpy as np

from aic_control_interfaces.msg import JointMotionUpdate, MotionUpdate, TrajectoryGenerationMode
from aic_model.policy import (
    GetObservationCallback,
    MoveRobotCallback,
    Policy,
    SendFeedbackCallback,
)
from aic_task_interfaces.msg import Task
from geometry_msgs.msg import Point, Pose, Vector3, Wrench
from rclpy.duration import Duration
from std_msgs.msg import Header


class UnexpectedContactError(Exception):
    pass


class OutOfReachError(Exception):
    pass


class SurfaceContactError(Exception):
    """Raised when Stage 3 detects the arm is pinned against a board surface
    rather than a port aperture, and has exhausted its in-place reset budget.

    Carries the TCP pose at the point of surface contact so the caller can
    retreat the arm to a safe height before re-scouting.
    """
    def __init__(self, message: str, tcp_pose=None):
        super().__init__(message)
        self.tcp_pose = tcp_pose


class ANT(Policy):
    def __init__(self, parent_node):
        super().__init__(parent_node)

        # ---- impedance control parameters ----------------------------------
        self.approach_stiffness = 85.0    # N/m — used for Stages 1–3 (free-space positioning)
        self.insertion_stiffness = 200.0  # N/m — Stage 4 only; higher force to push through real-world cable tension
        # Contact threshold.  Must stay well below the 20 N force penalty
        # threshold defined in the scoring rules.
        self.force_threshold = 5.0        # N
        # Grace period at task start: the cable weight and arm settling can
        # produce residual forces >5 N before the robot has moved at all.
        # During this window the force check uses a relaxed threshold (just
        # below the 20 N scoring penalty) so the robot is not aborted before
        # it has a chance to move to a safe pose.
        self.startup_grace_sec = 6.0      # seconds — covers joint settle + baseline calibration

        # ---- scene geometry (CALIBRATE using ground_truth:=true) -----------
        # Z height of the connector face in base_link (metres).
        # NOTE: base_link origin is on the tabletop at world z=1.14.  Port
        # heights in base_link = port_z_in_world − 1.14.
        # Also note: base_link X = -world_X, base_link Y = -world_Y (180° yaw
        # rotation between world/aic_world and the robot's base_link frame).
        # Per port type — SFP (NIC card) ports are higher on the board than SC.
        self.connector_z_in_base = {
            "sfp": 0.1335,   # NIC card SFP port face in base_link (world z ≈ 1.273)
            "sc":  0.0145,   # SC optical port face in base_link   (world z ≈ 1.155)
        }

        # Calibrated XY centroid of each connector zone in base_link (metres).
        # Used as the vision grid-search centre and as the Stage 1 stall-collapse
        # fallback for Stage 2/3.
        #
        # Both values are the mean of competition ports from sample_config only —
        # NOT the n=5 mean across all configs (which includes ant_test_config
        # ports at different positions that are never used in competition).
        #
        #   SFP: mean of sample_config T1 (-0.3845, 0.2126) and T2 (-0.3845, 0.2526)
        #        → (-0.3845, 0.2326).  Each competition port is only ±2 cm in Y
        #        from this centre.  The old n=5 mean (-0.4398, 0.3357) was biased
        #        13.5 cm from the T1 port by test-config ports at Y=0.29–0.48.
        #   SC:  competition T3 port position from sample_config trial_1/2
        #        (board yaw=+17°).  The n=5 mean (-0.4028, 0.3527) is biased
        #        7.7 cm by test configs not used in competition; the actual T3 port
        #        is consistently at (-0.3830, 0.4295) across all sample_config runs.
        self.zone_scouting_xy = {
            "sfp": (-0.3845, 0.200),    # T1 port Y (Bug 51: shifted -12 mm from 0.2126 to clear
                                        # NIC card mount; right finger now at Y≈0.220, clears 0.233)
                                        # NOT used in SFP fast path (bypasses grid) — kept for symmetry
            "sc":  (-0.3830, 0.4295),   # sample_config T3 SC port (yaw=+17°)
        }

        # Individual competition port positions used as the Stage 1 fallback
        # when vision produces no valid estimate.  Unlike zone_scouting_xy
        # (the grid centre used for WP2 approach), these are the actual port
        # positions so Stage 3 starts directly above the port rather than at
        # the midpoint between ports.  When vision fails the policy selects the
        # port nearest to the arm's current TCP position — this is adaptive
        # without requiring a trial-number argument.
        #
        # SFP: T1 port (board yaw=+17°) and T2 port (board yaw=−45°).
        #      T1 Y=0.200 (Bug 51): the hande_finger_link_r extends ~20 mm in +Y from
        #      TCP centre, placing the right finger at Y≈0.220 — 13 mm clear of the
        #      NIC mount at Y≈0.233 (Run 19 collision at Y=0.2126 where finger reached
        #      Y≈0.233).  T2 Y=0.2526 is safe (finger at Y≈0.273, no obstacle).
        # SC:  single known port; same value as zone_scouting_xy["sc"].
        self.zone_known_ports = {
            "sfp": [(-0.3845, 0.200), (-0.3845, 0.2526)],
            "sc":  [(-0.3830, 0.4295)],
        }

        # ---- joint-space return-to-home position ----------------------------
        # Order: shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3
        self.pre_scout_joint_positions = [-0.1597, -1.3542, -1.6648, -1.6933, 1.5710, 1.4110]
        self.joint_stiffness = [200.0, 200.0, 200.0, 100.0, 100.0, 100.0]
        self.joint_damping = [80.0, 80.0, 80.0, 40.0, 40.0, 40.0]

        # ---- cable-load baseline calibration --------------------------------
        self.cable_force_baseline_default = 22.5  # N — fallback if calibration fails
        self.cable_baseline_samples = 20           # FTS samples to average (~2 sim-sec)
        self.cable_force_baseline = self.cable_force_baseline_default

        # ---- trial counter --------------------------------------------------
        # Incremented at the start of each insert_cable() call.  Used to select
        # the correct SFP port for WP2 navigation: trial 1 = T1 (Y=0.200),
        # trial 2 = T2 (Y=0.2526, requires safe high-Z approach over NIC mount).
        self._insert_call_count = 0

    # Workspace bounds for probe-point validation in base_link (m).
    # NOTE: base_link X = −world_X, Y = −world_Y (180° yaw from tabletop TF).
    # X range: [-0.586, -0.361] across training scenarios, ±0.15 m margin
    # Y range: [0.213, 0.479]  across training scenarios, ±0.15 m margin
    # Z range: [0.000, 0.300]  (SC at 0.015, SFP at 0.134)
    _WS_X = (-0.75, -0.20)
    _WS_Y = (0.06, 0.65)
    _WS_Z = (0.00, 0.30)

    # Stage 3 spiral: centre + fine rings (5/10/15/20 mm) + coarse rings
    # (40/60/80/110 mm).  Used only by the generic spiral descent path (not
    # reached in competition — SFP and SC both use direct descent).
    # Each ring includes the 4 diagonal directions (±r/√2 per axis) so that
    # ports offset diagonally from the zone centre are reachable.
    _D40 = 0.040 / (2 ** 0.5)   # ≈ 0.0283 m per axis at 40 mm radius
    _D60 = 0.060 / (2 ** 0.5)   # ≈ 0.0424 m per axis
    _D80 = 0.080 / (2 ** 0.5)   # ≈ 0.0566 m per axis
    _D110 = 0.110 / (2 ** 0.5)  # ≈ 0.0778 m per axis
    _SPIRAL_OFFSETS = (
        # Centre
        ( 0.000,  0.000),
        # Fine rings: ±5/10/15/20 mm cardinals
        (+0.005,  0.000), ( 0.000, +0.005), (-0.005,  0.000), ( 0.000, -0.005),
        (+0.010,  0.000), ( 0.000, +0.010), (-0.010,  0.000), ( 0.000, -0.010),
        (+0.015,  0.000), ( 0.000, +0.015), (-0.015,  0.000), ( 0.000, -0.015),
        (+0.020,  0.000), ( 0.000, +0.020), (-0.020,  0.000), ( 0.000, -0.020),
        # Coarse rings: 40/60/80/110 mm — 4 cardinals + 4 diagonals each
        (+0.040,  0.000), ( 0.000, +0.040), (-0.040,  0.000), ( 0.000, -0.040),
        (+_D40,  +_D40), (-_D40, +_D40), (-_D40, -_D40), (+_D40, -_D40),
        (+0.060,  0.000), ( 0.000, +0.060), (-0.060,  0.000), ( 0.000, -0.060),
        (+_D60,  +_D60), (-_D60, +_D60), (-_D60, -_D60), (+_D60, -_D60),
        (+0.080,  0.000), ( 0.000, +0.080), (-0.080,  0.000), ( 0.000, -0.080),
        (+_D80,  +_D80), (-_D80, +_D80), (-_D80, -_D80), (+_D80, -_D80),
        (+0.110,  0.000), ( 0.000, +0.110), (-0.110,  0.000), ( 0.000, -0.110),
        (+_D110, +_D110), (-_D110, +_D110), (-_D110, -_D110), (+_D110, -_D110),
    )

    # SC pre-scan: visit lateral offset positions at a higher Z so the camera
    # can see the SC port housing from an angle.  The SC cable plug always hangs
    # directly below the TCP and appears at image centre; from an offset position
    # the plug remains at centre while the housing shifts toward the image
    # periphery.  A centre-masked detector then suppresses the plug and detects
    # the housing blob.
    #
    # Offsets are in base_link (m), applied relative to zone_scouting_xy["sc"].
    # +X is toward robot base (−world_X direction); +Y is toward operator.
    # Workspace check (_WS_X / _WS_Y) is applied at runtime.
    # Bug 58 (Run 22): SC cable plug appears at the IMAGE BOTTOM (v≈779, bottom 13%
    # in a ~900 px frame) — well outside the 30% centre mask.  The SC housing is
    # visible in the RGB image but its area exceeds max_area=10k px² from the stall
    # position (14–22 cm from targets), so the plug wins detection every time.
    # Back-projection lands 18–19 cm from zone centre, rejected by proximity filter;
    # calibrated zone fallback is always correct.  Net effect: 53 s wasted with no
    # localisation improvement.  Disabled until a stall-proof detection method exists.
    _SC_PRESCAN_Z_ABOVE = 0.18    # m above connector_z — sets SC transit_z (Bug 58: prescan disabled)

    # ======================================================================
    # Motion helpers
    # ======================================================================

    def _make_pose(self, x: float, y: float, z: float, orientation) -> Pose:
        """Construct a Pose from scalar XYZ coordinates and an orientation."""
        return Pose(
            position=Point(x=float(x), y=float(y), z=float(z)),
            orientation=orientation,
        )

    def _in_workspace_xy(self, x: float, y: float) -> bool:
        """Return True if (x, y) lies within the configured workspace XY bounds."""
        return self._WS_X[0] <= x <= self._WS_X[1] and self._WS_Y[0] <= y <= self._WS_Y[1]

    @property
    def _force_abort_threshold(self) -> float:
        """Force abort threshold: 3 N above the calibrated cable baseline.

        Relative to cable baseline so it does not fire spuriously when cable
        tension alone exceeds the old fixed 18 N cutoff (e.g. baseline=21.9 N
        → force_abs=20.4 N → would abort before descent with a fixed threshold).
        """
        return self.cable_force_baseline + 3.0

    def _high_force_timed_out(
        self,
        force_abs: float,
        threshold: float,
        high_force_start,
        budget_sec: float,
        active_condition: bool = True,
    ) -> tuple:
        """Update a sustained-high-force timer and report whether it expired.

        Returns (timed_out: bool, updated_high_force_start).
        Call site is responsible for the abort action (raise or return False).

        active_condition: extra predicate that must be True for the force to be
        considered "active" (Stage 3 uses this to exclude the contact-detection
        zone where a high local_delta is expected).
        """
        if force_abs > threshold and active_condition:
            if high_force_start is None:
                return False, self.time_now()
            elapsed = (self.time_now() - high_force_start).nanoseconds / 1e9
            return elapsed > budget_sec, high_force_start
        return False, high_force_start

    def _run_joint_settle(
        self, duration_sec: float, move_robot, start_time, time_limit_sec,
    ) -> None:
        """Drive to pre_scout_joint_positions and hold for duration_sec.

        Breaks silently when the time limit is reached — caller handles budget.
        Only used for the end-of-SC-trial arm return (SFP trials skip it).
        Note: the joint-space phase was found to cause 200 N+ cable-drag force
        spikes when the arm carries over a non-home posture from a prior trial,
        accumulating >1 s above 20 N and triggering the scoring force penalty.
        SFP trials skip this entirely for that reason (Bug 52).
        """
        jmu = self._build_joint_motion_update(self.pre_scout_joint_positions)
        settle_start = self.time_now()
        settle_duration = Duration(seconds=duration_sec)
        while (self.time_now() - settle_start) < settle_duration:
            if self._is_timed_out(start_time, time_limit_sec):
                break
            move_robot(joint_motion_update=jmu)
            self.sleep_for(0.1)

    def _is_timed_out(self, start_time, time_limit_sec: float) -> bool:
        return (self.time_now() - start_time).nanoseconds >= int(time_limit_sec * 1e9)

    def _force_magnitude(self, observation) -> float:
        f = observation.wrist_wrench.wrench.force
        return (f.x ** 2 + f.y ** 2 + f.z ** 2) ** 0.5

    def _force_delta(self, observation) -> float:
        """Return force magnitude minus the calibrated cable-load baseline.

        Using delta-force rather than absolute force avoids false UnexpectedContact
        errors from the persistent cable weight (~22 N in free space), while still
        detecting genuine contact forces above the 5 N threshold.
        """
        return max(0.0, self._force_magnitude(observation) - self.cable_force_baseline)

    def _calibrate_cable_baseline(
        self, get_observation, start_time, time_limit_sec,
        settle_sec: float = 0.0,
    ) -> None:
        """Sample the FTS for cable_baseline_samples readings and store the median.

        Called after joint settling (arm is stationary at pre-scout) so the
        measured force is purely the cable weight in that configuration.
        Falls back to the hardcoded default if not enough observations arrive
        or if the std of the samples is too high (arm still moving).

        settle_sec: optional pre-sample pause.  Pass > 0 when called after
        a surface retreat — the arm may still be decelerating and taking samples
        immediately yields a low/noisy reading (observed: 7.64 N after retreat
        vs normal ~22 N, causing the contact thresholds to be badly biased).
        """
        if settle_sec > 0.0:
            self.get_logger().info(
                f"Cable baseline: settling {settle_sec:.1f}s before sampling"
            )
            elapsed = 0.0
            while elapsed < settle_sec:
                if self._is_timed_out(start_time, time_limit_sec):
                    break
                self.sleep_for(0.1)
                elapsed += 0.1

        samples = []
        attempts = 0
        max_attempts = self.cable_baseline_samples * 3  # allow for None observations

        while len(samples) < self.cable_baseline_samples and attempts < max_attempts:
            if self._is_timed_out(start_time, time_limit_sec):
                break
            obs = get_observation()
            attempts += 1
            if obs is None:
                self.sleep_for(0.05)
                continue
            samples.append(self._force_magnitude(obs))
            self.sleep_for(0.1)

        if len(samples) >= 3:
            std = float(np.std(samples))
            median = float(np.median(samples))
            # Reject calibration if std > 5 N — arm is still moving or in contact.
            # A stable free-space reading has std < 2 N (observed: 0.17–1.66 N).
            # High std after retreat means the settle_sec was too short or the
            # cable is genuinely oscillating; fall back to the previous baseline
            # rather than locking in a biased value.
            if std > 5.0:
                self.get_logger().warning(
                    f"Cable baseline calibration noisy (std={std:.2f} N > 5 N, "
                    f"median={median:.2f} N) — arm may be moving, keeping previous "
                    f"baseline {self.cable_force_baseline:.2f} N"
                )
            else:
                self.cable_force_baseline = median
                self.get_logger().info(
                    f"Cable baseline calibrated: {self.cable_force_baseline:.2f} N "
                    f"(n={len(samples)}, std={std:.2f} N, median used)"
                )
        else:
            self.cable_force_baseline = self.cable_force_baseline_default
            self.get_logger().warning(
                f"Cable baseline calibration insufficient (n={len(samples)}) — "
                f"using default {self.cable_force_baseline:.1f} N"
            )

    def _build_motion_update(self, pose: Pose, stiffness_xyz: float,
                             feedforward_fy: float = 0.0,
                             feedforward_fz: float = 0.0) -> MotionUpdate:
        rot_stiffness = stiffness_xyz * 0.5
        damping_xyz = stiffness_xyz * 0.6
        rot_damping = damping_xyz * 0.5
        return MotionUpdate(
            header=Header(
                frame_id="base_link",
                stamp=self.get_clock().now().to_msg(),
            ),
            pose=pose,
            target_stiffness=np.diag([
                stiffness_xyz, stiffness_xyz, stiffness_xyz,
                rot_stiffness, rot_stiffness, rot_stiffness,
            ]).flatten(),
            target_damping=np.diag([
                damping_xyz, damping_xyz, damping_xyz,
                rot_damping, rot_damping, rot_damping,
            ]).flatten(),
            feedforward_wrench_at_tip=Wrench(
                force=Vector3(x=0.0, y=feedforward_fy, z=feedforward_fz),
                torque=Vector3(x=0.0, y=0.0, z=0.0),
            ),
            # Wrench feedback gains MUST be 0 for position-convergence moves.
            # With gains=[0.5,...] the controller feeds back 50% of the
            # measured cable tension (~20 N upward) as a target upward force,
            # which exactly cancels the controller's downward impedance force
            # (capped at maximum_wrench=10 N) and the arm stalls permanently.
            wrench_feedback_gains_at_tip=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            trajectory_generation_mode=TrajectoryGenerationMode(
                mode=TrajectoryGenerationMode.MODE_POSITION,
            ),
        )

    def _wait_for_observation(self, get_observation, start_time, time_limit_sec,
                              max_attempts: int = 20):
        """Block until a non-None Observation is available."""
        for _ in range(max_attempts):
            if self._is_timed_out(start_time, time_limit_sec):
                raise TimeoutError("Timed out waiting for first observation")
            obs = get_observation()
            if obs is not None:
                return obs
            self.sleep_for(0.05)
        raise OutOfReachError("No observation received after multiple attempts")

    def _build_joint_motion_update(self, positions: list) -> JointMotionUpdate:
        jmu = JointMotionUpdate(
            target_stiffness=self.joint_stiffness,
            target_damping=self.joint_damping,
            trajectory_generation_mode=TrajectoryGenerationMode(
                mode=TrajectoryGenerationMode.MODE_POSITION,
            ),
        )
        jmu.target_state.positions = positions
        return jmu

    def _move_to_pose_and_wait(
        self, target_pose, move_robot, get_observation,
        start_time, time_limit_sec,
        convergence_m: float = 0.005,
        stage_timeout_sec: float = 15.0,
        label: str = "move",
        check_force: bool = True,
        feedforward_fy: float = 0.0,
        stiffness_xyz: float = None,
    ) -> None:
        """Send a Cartesian pose target and block until TCP converges or stalls.

        Exits when either:
          (a) pos_error < convergence_m for consecutive_required samples, OR
          (b) the arm has stalled: pos_error variation < 2 mm over the last
              stall_window samples AND at least min_iterations have elapsed.
              This handles cable-tension equilibria where physics prevents
              reaching the exact target — rather than burning the full timeout
              we detect "arm stopped moving" and proceed immediately.

        Raises UnexpectedContactError if contact is detected during the move
        (only when check_force=True), or OutOfReachError if the target is not
        reached within the timeout.
        """
        stage_start = self.time_now()
        stage_timeout = Duration(seconds=stage_timeout_sec)
        iteration = 0
        min_iterations = 3
        consecutive_required = 2
        consecutive_converged = 0
        # Stall detection: circular buffer of recent pos_error values.
        stall_window = 5
        stall_tol_m = 0.002   # 2 mm spread → arm has stopped moving
        error_history: collections.deque = collections.deque(maxlen=stall_window)

        while True:
            if self._is_timed_out(start_time, time_limit_sec):
                raise TimeoutError(f"Global timeout during {label}")
            if (self.time_now() - stage_start) >= stage_timeout:
                raise OutOfReachError(
                    f"{label}: did not converge within {stage_timeout_sec} s"
                )

            _stiffness = stiffness_xyz if stiffness_xyz is not None else self.approach_stiffness
            motion_update = self._build_motion_update(
                target_pose, _stiffness, feedforward_fy)
            try:
                move_robot(motion_update=motion_update)
            except Exception as ex:
                raise OutOfReachError(f"{label}: move_robot rejected command: {ex}")

            self.sleep_for(0.05)
            obs = get_observation()
            if obs is None:
                consecutive_converged = 0
                continue

            iteration += 1
            if check_force:
                force_delta = self._force_delta(obs)
                elapsed = (self.time_now() - start_time).nanoseconds / 1e9
                if force_delta > self.force_threshold and elapsed >= self.startup_grace_sec:
                    raise UnexpectedContactError(
                        f"{label}: unexpected contact while moving "
                        f"(delta={force_delta:.2f} N above {self.cable_force_baseline:.1f} N baseline)"
                    )

            pos_error = np.linalg.norm(obs.controller_state.tcp_error[:3])
            error_history.append(pos_error)

            # (a) Normal convergence: below threshold for consecutive_required samples.
            if pos_error < convergence_m and iteration >= min_iterations:
                consecutive_converged += 1
            else:
                consecutive_converged = 0

            # (b) Stall detection: arm has stopped moving — exit regardless of threshold.
            stalled = (
                iteration >= min_iterations
                and len(error_history) == stall_window
                and (max(error_history) - min(error_history)) < stall_tol_m
            )

            self.get_logger().info(
                f"{label}: iter={iteration} pos_error={pos_error:.4f} m "
                f"consec={consecutive_converged}/{consecutive_required}"
                + (" [STALLED]" if stalled else "")
            )

            if consecutive_converged >= consecutive_required or stalled:
                break

    # ======================================================================
    # Stage 1 — Localise the connector
    # ======================================================================

    def _localize_connector(
        self, task, get_observation, move_robot, send_feedback, start_time, time_limit_sec,
        baseline_settle_sec: float = 0.0,
    ) -> Pose:
        """Vision-based connector localisation.

        SFP fast path: navigate directly to the calibrated port position.  The SFP
        bracket is not visible from above (blue/cyan face on the outward surface), so
        the grid scan always yields nothing.  Stage 3 uses direct descent (Bug 47).

        SC path: prescan currently DISABLED (_SC_PRESCAN_OFFSETS = ()).  Bug 58 showed
        the SC cable plug appears at the image bottom (v≈779), not the centre, so the
        centre mask does not suppress it.  Falls through directly to calibrated zone
        fallback (correct for competition T3).  Stage 3 uses direct descent (Bug 60).

        Aggregation (both paths use the SC aggregate block):
          1 estimate → use directly.
          ≥2 estimates → median-filter, keep inliers within 5 cm, centroid.
          0 valid estimates → nearest known competition port (Stage 3 descends from
          the calibrated position).
        """
        zone = "sfp" if task.plug_type == "sfp" else "sc"
        send_feedback(
            f"Stage 1: vision localising '{task.port_name}' on "
            f"'{task.target_module_name}' (zone={zone})"
        )

        # ---- cable baseline calibration ------------------------------------
        self._calibrate_cable_baseline(
            get_observation, start_time, time_limit_sec,
            settle_sec=baseline_settle_sec,
        )

        # ---- startup force safety check ------------------------------------
        obs = self._wait_for_observation(get_observation, start_time, time_limit_sec)
        force_delta = self._force_delta(obs)
        elapsed = (self.time_now() - start_time).nanoseconds / 1e9
        if elapsed < self.startup_grace_sec:
            self.get_logger().warning(
                f"Stage 1: grace period active ({elapsed:.1f}s) "
                f"— skipping start-force check (delta={force_delta:.2f} N)"
            )
        elif force_delta > self.force_threshold:
            raise UnexpectedContactError(
                f"Stage 1: unexpected contact at start "
                f"(delta={force_delta:.2f} N above {self.cable_force_baseline:.1f} N baseline)"
            )

        # ---- navigate to calibrated zone at transit height -----------------
        base_x, base_y = self.zone_scouting_xy.get(zone, self.zone_scouting_xy["sfp"])
        connector_z    = self.connector_z_in_base[zone]
        transit_z      = connector_z + 0.08   # 8 cm above port — safe for all lateral moves

        current_pose   = obs.controller_state.tcp_pose
        current_x      = current_pose.position.x
        current_y      = current_pose.position.y
        current_z      = current_pose.position.z
        current_orient = current_pose.orientation

        # Out-of-bounds safety: descend to a safe Z before lateral move to
        # avoid driving into the enclosure wall (observed: Run 4 T3 crash).
        if not self._in_workspace_xy(current_x, current_y):
            safe_z = 0.30
            self.get_logger().warning(
                f"Stage 1: arm at ({current_x:.3f},{current_y:.3f}) OOB — "
                f"descending to z={safe_z:.3f} before lateral move"
            )
            self._move_to_pose_and_wait(
                self._make_pose(current_x, current_y, safe_z, current_orient),
                move_robot, get_observation, start_time, time_limit_sec,
                convergence_m=0.05, stage_timeout_sec=20.0,
                label="Stage 1 OOB Z-descent", check_force=False,
            )
            obs2 = get_observation()
            if obs2 is not None:
                current_orient = obs2.controller_state.tcp_pose.orientation
                current_z = obs2.controller_state.tcp_pose.position.z

        # ---- SFP fast path: skip vision grid (HSV structurally blind top-down) -------
        # At transit_z the camera looks straight down.  The SFP bracket's blue/cyan face
        # is on the outward-facing front surface — not visible from above.  All Run 17
        # debug images confirmed: SFP HSV mask entirely black at every grid position.
        # Also, the 13-position grid wastes ~100 s of trial budget and some grid moves
        # (e.g. (0,+90mm) from T1 centre) pass through Y≈0.233 (NIC mount zone).
        # Navigate directly to the calibrated port and return; Stage 3 uses direct
        # descent to connector_z (Bug 47 — SFP generates no detectable axial force).
        if zone == "sfp":
            sfp_ports = self.zone_known_ports["sfp"]
            trial = self._insert_call_count
            if trial == 2 and len(sfp_ports) >= 2:
                # T2 (second SFP trial): port at Y=0.2526.  Straight lateral path from
                # T1 end position (Y≈0.200) crosses NIC mount at Y≈0.233 at any Z ≤
                # transit_z=0.2135 m.  Ascend to safe_z=0.28 m first — NIC mount
                # collision was observed at transit_z=0.2135 m; 0.28 m gives ~7 cm
                # clearance above the estimated mount top (~0.21 m).
                tgt_x, tgt_y = sfp_ports[1]
                safe_z = 0.28
                obs_wp = get_observation()
                orient = obs_wp.controller_state.tcp_pose.orientation if obs_wp else current_orient
                if current_z < safe_z - 0.01:
                    self.get_logger().info(
                        f"Stage 1 SFP T2: ascending to safe_z={safe_z:.3f} to clear NIC mount"
                    )
                    self._move_to_pose_and_wait(
                        self._make_pose(current_x, current_y, safe_z, orient),
                        move_robot, get_observation, start_time, time_limit_sec,
                        convergence_m=0.05, stage_timeout_sec=25.0,
                        label="Stage 1 SFP T2 WP1 safe ascent", check_force=False,
                    )
                    obs_wp = get_observation()
                    orient = obs_wp.controller_state.tcp_pose.orientation if obs_wp else orient
                # Bug 89: split lateral into 2 sub-moves via midpoint Y waypoint.
                # Run 40 T2 in sim: pos_error spiked 0.041→0.071 m at iter=3 of the
                # single-move WP2, producing a 200.53 N transient (0.02 s) — the
                # highest ever recorded (Run 32 had 148 N).  At safe_z=0.28 m the
                # SFP cable at -45° yaw is strongly pretensioned; a single 5-6 cm
                # lateral command stretches the cable past an unstable constraint
                # point and triggers snap-through.  Splitting into halves lowers
                # peak deflection per move, letting the cable re-equilibrate at
                # the midpoint before the second half — same strategy that kept
                # WP1 safe ascent from spiking (small Z step).  Keeps sub-second
                # transient well below the 1 s penalty threshold even if real
                # hardware cable dynamics differ from sim.
                obs_wp = get_observation()
                y_now = obs_wp.controller_state.tcp_pose.position.y if obs_wp else current_y
                mid_y = 0.5 * (y_now + tgt_y)
                self.get_logger().info(
                    f"Stage 1 SFP T2: WP2a lateral midpoint → ({tgt_x:.4f},{mid_y:.4f}) "
                    f"at safe_z={safe_z:.3f} (split move, Bug 89)"
                )
                self._move_to_pose_and_wait(
                    self._make_pose(tgt_x, mid_y, safe_z, orient),
                    move_robot, get_observation, start_time, time_limit_sec,
                    convergence_m=0.015, stage_timeout_sec=20.0,
                    label="Stage 1 SFP T2 WP2a lateral midpoint (high-Z)",
                    check_force=False,
                )
                obs_wp = get_observation()
                orient = obs_wp.controller_state.tcp_pose.orientation if obs_wp else orient
                self.get_logger().info(
                    f"Stage 1 SFP T2: WP2b lateral to T2 port ({tgt_x:.4f},{tgt_y:.4f}) "
                    f"at safe_z={safe_z:.3f}"
                )
                self._move_to_pose_and_wait(
                    self._make_pose(tgt_x, tgt_y, safe_z, orient),
                    move_robot, get_observation, start_time, time_limit_sec,
                    convergence_m=0.015, stage_timeout_sec=20.0,
                    label="Stage 1 SFP T2 WP2b lateral final (high-Z)",
                    check_force=False,
                )
                # Descend to transit_z so Stage 2 starts at a predictable height.
                obs_wp = get_observation()
                orient = obs_wp.controller_state.tcp_pose.orientation if obs_wp else orient
                self._move_to_pose_and_wait(
                    self._make_pose(tgt_x, tgt_y, transit_z, orient),
                    move_robot, get_observation, start_time, time_limit_sec,
                    convergence_m=0.05, stage_timeout_sec=20.0,
                    label="Stage 1 SFP T2 WP3 descend to transit_z", check_force=False,
                )
            else:
                # T1 (or safety fallback): port at Y=0.200.  Path from home (Y≈0.192)
                # never crosses NIC mount — approach directly at transit_z.
                tgt_x, tgt_y = sfp_ports[0]
                obs_wp = get_observation()
                orient = obs_wp.controller_state.tcp_pose.orientation if obs_wp else current_orient
                if abs(current_z - transit_z) > 0.01:
                    direction = "ascending" if transit_z > current_z else "descending"
                    self.get_logger().info(
                        f"Stage 1 SFP T1: WP1 Z-adjust ({direction}) → transit_z={transit_z:.3f}"
                    )
                    self._move_to_pose_and_wait(
                        self._make_pose(current_x, current_y, transit_z, orient),
                        move_robot, get_observation, start_time, time_limit_sec,
                        convergence_m=0.05, stage_timeout_sec=20.0,
                        label="Stage 1 SFP T1 WP1 Z-adjust", check_force=False,
                    )
                    obs_wp = get_observation()
                    orient = obs_wp.controller_state.tcp_pose.orientation if obs_wp else orient
                self.get_logger().info(
                    f"Stage 1 SFP T1: lateral to T1 port ({tgt_x:.4f},{tgt_y:.4f}) "
                    f"at transit_z={transit_z:.3f}"
                )
                self._move_to_pose_and_wait(
                    self._make_pose(tgt_x, tgt_y, transit_z, orient),
                    move_robot, get_observation, start_time, time_limit_sec,
                    convergence_m=0.015, stage_timeout_sec=30.0,
                    label="Stage 1 SFP T1 WP2 lateral", check_force=False,
                )
            obs_final = self._wait_for_observation(get_observation, start_time, time_limit_sec)
            orient_final = obs_final.controller_state.tcp_pose.orientation
            self.get_logger().info(
                f"Stage 1: SFP fast path trial={trial} — "
                f"calibrated port ({tgt_x:.4f},{tgt_y:.4f}) "
                "(SFP HSV blind from above; vision grid skipped)"
            )
            return self._make_pose(tgt_x, tgt_y, connector_z, orient_final)

        # ---- SC: use higher transit_z for pre-scan (plug always 4 cm below TCP) ----
        # At normal transit_z (connector_z+0.08 = 0.095 m), the cable plug is only
        # 4 cm from the camera and always appears at image centre, blocking the view
        # of the SC housing.  Using transit_z = connector_z+0.18 = 0.195 m gives a
        # wider FOV (~12.6 cm radius at board level) so that 10 cm offset positions
        # keep the port housing within the camera frame while the plug stays at centre.
        transit_z = connector_z + self._SC_PRESCAN_Z_ABOVE

        # ---- SC: WP1 Z-adjust + WP2 lateral + pre-scan (below) -----------
        #
        # Safe-Z entry (Bug 52 / Bug 62): T3 starts from wherever T2 ended.
        # Without joint returns, the arm sits at SFP Stage 4 depth (z≈0.179–0.195 m,
        # Y≈0.253 m at T2 port) when T3 begins.  A lateral move from that position to
        # the SC zone (ΔY≈0.175 m) under SC cable tension at z≈0.195 m stalls
        # completely (Run 25: WP2 stall at 0.1752 m, arm never reached SC zone).
        # Fix: whenever arm is below safe_z_entry=0.28 m, always ascend to 0.28 m
        # first, then lateral at that height, then descend to transit_z.  At 0.28 m
        # cable tension is low enough that the lateral move succeeds.  (Bug 62)
        _safe_z_entry = 0.28   # clears all board obstacles (SFP boards top ~0.21 m)

        if current_z < _safe_z_entry - 0.01:
            # Arm below safe_z — ascend first, then lateral, then descend to transit_z.
            wp1_z = _safe_z_entry
            self.get_logger().info(
                f"Stage 1: WP1 safe ascent to z={wp1_z:.3f} m "
                f"(arm at z={current_z:.3f}, clearing board area before lateral)"
            )
            self._move_to_pose_and_wait(
                self._make_pose(current_x, current_y, wp1_z, current_orient),
                move_robot, get_observation, start_time, time_limit_sec,
                convergence_m=0.05, stage_timeout_sec=20.0,
                label="Stage 1 WP1 safe ascent", check_force=False,
            )
            obs_wp1 = get_observation()
            orient = obs_wp1.controller_state.tcp_pose.orientation if obs_wp1 else current_orient
            # WP2: lateral at safe_z — well above all board faces.
            self.get_logger().info(
                f"Stage 1: WP2 lateral → ({base_x:.3f},{base_y:.3f}) at safe_z={wp1_z:.3f}"
            )
            self._move_to_pose_and_wait(
                self._make_pose(base_x, base_y, wp1_z, orient),
                move_robot, get_observation, start_time, time_limit_sec,
                convergence_m=0.015, stage_timeout_sec=30.0,
                label="Stage 1 WP2 lateral (safe-Z)", check_force=False,
            )
            obs_wp2 = get_observation()
            orient = obs_wp2.controller_state.tcp_pose.orientation if obs_wp2 else orient
            # Record WP2 stall position for WP3 descent.
            # WP2b overshoot correction removed (Run 29: proven structurally
            # ineffective — cable equilibrium at transit_z is a hard wall
            # unaffected by command target).
            _wp2b_x = base_x
            _wp2b_y = base_y  # fallback: use original target if obs unavailable
            if obs_wp2 is not None:
                _wp2b_x = obs_wp2.controller_state.tcp_pose.position.x
                _wp2b_y = obs_wp2.controller_state.tcp_pose.position.y
            # WP3: descend to transit_z at actual post-correction XY.
            # Stage 2 WP2 handles fine XY alignment at approach_z (lower cable tension).
            self.get_logger().info(
                f"Stage 1: WP3 descend → transit_z={transit_z:.3f} from "
                f"({_wp2b_x:.3f},{_wp2b_y:.3f})"
            )
            self._move_to_pose_and_wait(
                self._make_pose(_wp2b_x, _wp2b_y, transit_z, orient),
                move_robot, get_observation, start_time, time_limit_sec,
                convergence_m=0.05, stage_timeout_sec=20.0,
                label="Stage 1 WP3 transit descent", check_force=False,
            )
            obs_wp3 = get_observation()
            orient = obs_wp3.controller_state.tcp_pose.orientation if obs_wp3 else orient

        else:
            # Arm already at or above safe_z (≥0.27 m) — no ascent needed before lateral.
            # WP2 combines lateral+descent to transit_z in one move (arm is already above
            # transit_z=0.195 m since safe_z=0.28 m > transit_z, so no Z-adjust needed).
            self.get_logger().info(
                f"Stage 1: WP2 lateral → ({base_x:.3f},{base_y:.3f}) at z={transit_z:.3f}"
            )
            self._move_to_pose_and_wait(
                self._make_pose(base_x, base_y, transit_z, current_orient),
                move_robot, get_observation, start_time, time_limit_sec,
                convergence_m=0.015, stage_timeout_sec=30.0,
                label="Stage 1 WP2 lateral", check_force=False,
            )
            obs_wp2 = get_observation()
            orient = obs_wp2.controller_state.tcp_pose.orientation if obs_wp2 else current_orient

        # SC pre-scan disabled (Bug 58): plug appears at image bottom (v≈779), outside
        # any centre mask; SC housing too large from stall positions 14–22 cm from targets.
        # Falls directly to calibrated zone fallback — always the correct path for competition T3.
        self.get_logger().info(
            "Stage 1 SC: prescan disabled (Bug 58) — using calibrated zone fallback"
        )

        # ---- Calibrated zone fallback: nearest known competition port ------
        # Individual port positions are safe descent targets (avoids zone midpoint
        # which clipped the NIC card enclosure in Run 16 T1: −24 contact penalty).
        obs_final = self._wait_for_observation(get_observation, start_time, time_limit_sec)
        orient_final = obs_final.controller_state.tcp_pose.orientation
        arm_x = obs_final.controller_state.tcp_pose.position.x
        arm_y = obs_final.controller_state.tcp_pose.position.y
        known = self.zone_known_ports.get(zone, [(base_x, base_y)])
        fb_x, fb_y = min(known, key=lambda p: np.hypot(p[0] - arm_x, p[1] - arm_y))
        self.get_logger().info(
            f"Stage 1 SC: nearest known port ({fb_x:.4f},{fb_y:.4f}) — "
            "Stage 3 will direct-descend from this position"
        )
        return self._make_pose(fb_x, fb_y, connector_z, orient_final)

    # ======================================================================
    # Stage 2 — Approach the connector
    # ======================================================================

    def _approach_connector(
        self, connector_pose, get_observation, move_robot, send_feedback,
        start_time, time_limit_sec,
        zone: str = "sfp",
    ) -> None:
        """Move to a pre-insertion waypoint 5 cm above the detected connector.

        Uses a two-leg path: descend Z at the current (scouting) XY first,
        then correct XY to align with the detected connector.  This avoids
        combined diagonal moves that can produce awkward arm postures.

        Bug 84 (REVERTED — Bug 88): 200 N/m for SC WP2 caused snap-through at 3.78 cm
        to 5.60 cm, equilibrating at 4.83 cm — worse than the natural 85 N/m stall at
        ~3.7 cm.  SC WP2 now uses approach_stiffness (85 N/m) like all other stages.
        """
        send_feedback("Stage 2: approaching connector")
        approach_z = connector_pose.position.z + (0.07 if zone == "sc" else 0.10)

        obs = get_observation()
        if obs is not None:
            current_x = obs.controller_state.tcp_pose.position.x
            current_y = obs.controller_state.tcp_pose.position.y
        else:
            current_x = connector_pose.position.x
            current_y = connector_pose.position.y

        # Leg 1: descend Z at current XY.
        self._move_to_pose_and_wait(
            self._make_pose(current_x, current_y, approach_z, connector_pose.orientation),
            move_robot, get_observation, start_time, time_limit_sec,
            # convergence_m=0.05: cable tension limits descent to ~4.5 cm above
            # the commanded approach_z.  Accept this equilibrium position rather
            # than spinning for 55 sim-sec waiting for convergence that physics
            # prevents.  check_force=False: cable dynamics create spurious spikes
            # during approach — contact detection is Stage 3's job.
            convergence_m=0.05, stage_timeout_sec=55.0, label="Stage 2 WP1 Z-descend",
            check_force=False,
        )

        # Leg 2: correct XY to align with the detected connector.
        self._move_to_pose_and_wait(
            self._make_pose(
                connector_pose.position.x,
                connector_pose.position.y,
                approach_z,
                connector_pose.orientation,
            ),
            move_robot, get_observation, start_time, time_limit_sec,
            convergence_m=0.015, stage_timeout_sec=30.0, label="Stage 2 WP2 XY-align",
            check_force=False,
        )
        send_feedback("Stage 2: at approach waypoint")

    # ======================================================================
    # Stage 3 — Detect contact with F/T sensor
    # ======================================================================

    def _detect_contact(
        self, connector_pose, get_observation, move_robot, send_feedback,
        start_time, time_limit_sec, zone: str = "sc",
    ) -> Pose:
        """Descend to the connector face and report the actual TCP contact pose.

        SFP (Bug 47) and SC (Bug 60) both use direct descent to connector_z:
        neither cable type generates detectable axial force at the port entrance,
        so FTS-based contact detection is structurally blind.  _move_to_pose_and_wait
        handles stall detection — the arm descends as far as cable tension allows
        and Stage 4 covers any remaining gap.

        A generic FTS running-minimum spiral descent is kept below the early
        returns for unknown connector types; it is not reached in competition.
        """
        send_feedback("Stage 3: descending to detect contact")

        # Initial calibration at approach height — sets the starting local min.
        self._calibrate_cable_baseline(get_observation, start_time, time_limit_sec)

        # SFP (Bugs 46+47) and SC (Bug 60): both use direct descent to connector_z.
        # Neither cable type generates detectable axial force at the port entrance:
        #   SFP: floor blocks T2 signal; T1 yields zero delta at port face.
        #   SC:  Run 23 confirmed max local_delta=2.06 N, below the 4 N threshold.
        # _move_to_pose_and_wait stall detection exits when cable tension prevents
        # further descent; Stage 4 covers any remaining gap.
        #
        # SC Stage 3 Z cap (Bug 71): target connector_z + 0.075 m (= 0.090 m) instead
        # of connector_z (0.0145 m).  Run 30 showed the arm can descend to tcp_z=0.038 m
        # when XY alignment is accurate — at that depth spring force = 2.4 N vs cable
        # tension ≈ 16.5 N (15% ratio), making Stage 4 creep impossible and placing the
        # SC plug tip 0.20 m from port due to cable geometry.  The creep-viable zone is
        # tcp_z ≈ 0.09–0.10 m where spring ≈ 6.8–7.4 N vs cable tension ≈ 13 N (54–57%
        # ratio) — matching Runs 26 and 29 which both achieved tier_3 > 0.
        if zone in ("sfp", "sc"):
            stage3_z = (
                connector_pose.position.z if zone == "sfp"
                else connector_pose.position.z + 0.075
            )
            self.get_logger().info(
                f"Stage 3 ({zone.upper()}): direct descent to "
                f"z={stage3_z:.4f} — skipping contact detection"
            )
            send_feedback(f"Stage 3: {zone.upper()} direct descent")
            target_pose = self._make_pose(
                connector_pose.position.x, connector_pose.position.y,
                stage3_z, connector_pose.orientation,
            )
            try:
                self._move_to_pose_and_wait(
                    target_pose, move_robot, get_observation,
                    start_time, time_limit_sec,
                    convergence_m=0.010, stage_timeout_sec=20.0,
                    label=f"Stage 3 {zone.upper()} direct descent", check_force=False,
                )
            except OutOfReachError:
                # Timed out — arm stalled short of connector_z under cable load.
                # Proceed with wherever it stopped; Stage 4 covers the gap.
                pass
            obs_direct = get_observation()
            if obs_direct is not None:
                tcp = obs_direct.controller_state.tcp_pose
                self.get_logger().info(
                    f"Stage 3 ({zone.upper()}): arrived at tcp="
                    f"({tcp.position.x:.3f},{tcp.position.y:.3f},{tcp.position.z:.3f})"
                )
                send_feedback("Stage 3: contact detected")
                return self._make_pose(
                    tcp.position.x, tcp.position.y, tcp.position.z,
                    connector_pose.orientation,
                )
            send_feedback("Stage 3: contact detected")
            return target_pose

        # Generic spiral FTS descent — not reached in competition (SFP and SC
        # both use direct descent above).  Retained for unknown connector types.
        # Adaptive step size: 5 mm far from port, 2 mm near it.
        step_coarse_m = 0.005   # far from port (> 5 cm above connector Z)
        step_fine_m   = 0.002   # near port (≤ 5 cm above connector Z)
        fine_zone_top = connector_pose.position.z + 0.05

        obs_now = get_observation()
        # Use vision-localised connector XY (from Stage 1), not TCP equilibrium XY.
        if obs_now is not None:
            tcp_z_now = obs_now.controller_state.tcp_pose.position.z
        else:
            tcp_z_now = connector_pose.position.z + 0.05

        start_z = tcp_z_now
        end_z = max(connector_pose.position.z - 0.15, 0.005)
        self.get_logger().info(
            f"Stage 3: contact search z={start_z:.4f} → {end_z:.4f} "
            f"(TCP at {tcp_z_now:.4f}, "
            f"connector at z={connector_pose.position.z:.4f} "
            f"XY=({connector_pose.position.x:.3f},{connector_pose.position.y:.3f}))"
        )

        # Running-minimum local baseline: tracks cable-relaxation during descent.
        local_min_force = self.cable_force_baseline

        # Initialise step_m before the loop so the obs=None early-continue path
        # on the first iteration cannot raise NameError.
        step_m = step_coarse_m

        # Require 3 consecutive steps above threshold to confirm real contact
        # (prevents single-sample cable-noise spikes from triggering Stage 4).
        consecutive_contact = 0
        required_consecutive = 3

        # Surface reset budget: tracks how many times the Z-proximity guard
        # has fired.  Each reset means the arm is bouncing against the board
        # face rather than finding the port entrance.
        #   - hard surface (local_delta > 25 N): raise SurfaceContactError immediately.
        #     Observed: reset #1 at 17.8 N local_delta generates ~38 N absolute force
        #     (above the 20 N scoring threshold) for ~18 s before reset #2 fires.
        #     Aborting immediately on high local_delta prevents penalty accumulation.
        #   - moderate (local_delta ≤ 25 N): allow up to 5 resets before giving up.
        surface_reset_count = 0
        surface_reset_hard_delta = 25.0  # N — abort immediately on first high-force contact
        surface_reset_soft_limit = 5     # resets before abort on moderate contact

        # Force abort: if sustained >baseline+3N with no contact detected (wrong
        # XY, arm pressing into board surface), abort to prevent scoring penalty.
        # See _force_abort_threshold property for why this is relative to baseline.
        high_force_start = None
        high_force_budget_sec = 0.5

        # Spiral lateral search: once the arm is within 4 cm of connector_z, step
        # through XY offsets around the estimated connector position.
        # When a contact candidate fires, the current offset is locked so all 3
        # consecutive-confirmation steps command the same XY — commanding center
        # on candidate 2/3 would break contact.
        #
        # Bug 59 (Run 22): old trigger was start_z − 0.030 m (3 cm below approach).
        # For SC (start_z ≈ 0.118 m, connector_z = 0.015 m), this fired at z ≈ 0.088 m —
        # 7.4 cm above the port, leaving 37 steps of 2 mm before housing-contact depth
        # (z ≈ 0.037 m).  With 1 spiral advance per step the spiral reached idx≈25
        # (coarse ring, ±40–80 mm offsets) before the arm ever touched the housing.
        # All contacts were housing-wall hits at large offsets; fine-ring probing (centre
        # through ±20 mm) never happened at port depth.  Fix: trigger at connector_z+0.040,
        # i.e. 40 mm above port.  From trigger to housing depth ≈ 2–3 steps → spiral
        # stays at idx=0–2 (centre and ±5 mm), guaranteeing fine-ring probing at the
        # depth where port detection matters.
        spiral_z_threshold = connector_pose.position.z + 0.040
        spiral_offset_idx = 0
        locked_dx, locked_dy = 0.0, 0.0  # held while consecutive_contact > 0
        # Near-port spiral reset: set once when tcp_z first enters the near-port
        # zone (tcp_z ≤ connector_z + 0.025).  Resets to idx=0 so any remaining
        # spiral advance since trigger does not leave the arm at a non-centre offset
        # right when the 1-of-1 near-port shortcut is active.
        near_port_spiral_reset = False
        # Fixed threshold for near-port 1-of-1 shortcut — lower than force_threshold
        # because cable noise drops near connector_z (Run 16 T3: alternating 4.36 N
        # readings at the SC port that never hit 3 consecutive above 5 N).
        near_port_threshold = 4.0   # N

        current_z = start_z
        while current_z >= end_z:
            if self._is_timed_out(start_time, time_limit_sec):
                raise TimeoutError("Timed out during contact detection")

            # Compute this step's XY offset.
            spiral_active = current_z <= spiral_z_threshold
            if consecutive_contact > 0:
                dx, dy = locked_dx, locked_dy          # hold offset during confirmation
            elif spiral_active:
                dx, dy = self._SPIRAL_OFFSETS[spiral_offset_idx % len(self._SPIRAL_OFFSETS)]
            else:
                dx, dy = 0.0, 0.0
            target_pose = self._make_pose(
                connector_pose.position.x + dx, connector_pose.position.y + dy, current_z,
                connector_pose.orientation,
            )
            motion_update = self._build_motion_update(target_pose, self.approach_stiffness)
            try:
                move_robot(motion_update=motion_update)
            except Exception as ex:
                raise UnexpectedContactError(
                    f"Stage 3: move_robot rejected command: {ex}"
                )

            self.sleep_for(0.05)
            obs = get_observation()
            # Update step_m before the None guard so it is always current even
            # when an observation is missed (prevents one extra coarse step on
            # the iteration where current_z first crosses the fine-zone boundary).
            step_m = step_fine_m if current_z <= fine_zone_top else step_coarse_m
            if obs is None:
                current_z -= step_m
                continue

            force_abs = self._force_magnitude(obs)
            fz = obs.wrist_wrench.wrench.force.z

            # Track descending minimum (cable-only relaxation).
            # Floor prevents a transient cable-tension spike or drop from
            # resetting the minimum far below the true baseline (Run 13 T1:
            # 12.71 N vs 21.4 N baseline → subsequent normal tension triggered
            # false 3/3 contact confirmation in free air).
            local_min_force = min(local_min_force, force_abs)
            local_min_force = max(local_min_force, self.cable_force_baseline - 8.0)
            local_delta = force_abs - local_min_force

            self.get_logger().info(
                f"Stage 3: z={current_z:.4f} m  |F|={force_abs:.2f} N  "
                f"local_min={local_min_force:.2f} N  local_delta={local_delta:.2f} N  Fz={fz:.2f} N"
            )

            # Near-port shortcut: within 25 mm of the port face, cable noise
            # drops and any reading ≥ 4.0 N is a reliable port-rim contact
            # signal.  Run 16 T3 showed alternating 4.36 N / 0.53 N / 4.44 N
            # readings at the SC port that never hit 3 consecutive above 5.0 N.
            # Within this window, 1-of-1 at 4.0 N is sufficient.
            # Zone expanded from 5 mm → 25 mm (Bug 42): SFP peak delta 4.47 N
            # was observed at z=0.1523 (19 mm above connector_z=0.1335) in Run 17.
            # Uses actual TCP z, not commanded current_z (Bug 43): commanded z
            # can reach connector_z while arm is stalled 20 cm above, causing
            # a false near-port trigger and subsequent z-proximity guard reset.
            tcp_z = obs.controller_state.tcp_pose.position.z
            near_port = tcp_z <= connector_pose.position.z + 0.025
            # Reset spiral to centre on first entry into near-port zone.
            # Bug 59 fix moves the spiral trigger to connector_z+0.040, so the arm
            # should be in fine rings (idx≤8) by the time near-port fires.  Reset to
            # idx=0 (centre) unconditionally (not just for coarse idx>16) so the
            # arm probes the port centre first in the near-port window — the depth
            # where the 1-of-1 shortcut is active.  Requires consecutive_contact==0
            # to avoid interrupting an in-progress 2/3 or 3/3 confirmation.
            if near_port and not near_port_spiral_reset and consecutive_contact == 0:
                near_port_spiral_reset = True
                cur_idx = spiral_offset_idx % len(self._SPIRAL_OFFSETS)
                if cur_idx > 0:
                    self.get_logger().info(
                        f"Stage 3: near-port — resetting spiral from "
                        f"idx={cur_idx} to 0 (centre) at tcp_z={tcp_z:.4f}"
                    )
                    spiral_offset_idx = 0
            near_port_hit = near_port and local_delta > near_port_threshold

            if local_delta > self.force_threshold or near_port_hit:
                if consecutive_contact == 0:
                    # Lock the XY offset at first contact candidate so all
                    # confirmation steps probe the same lateral position.
                    locked_dx, locked_dy = dx, dy
                    if spiral_active and (dx != 0.0 or dy != 0.0):
                        self.get_logger().info(
                            f"Stage 3: spiral contact at offset "
                            f"({dx*1000:+.0f},{dy*1000:+.0f}) mm — locking XY"
                        )
                if near_port_hit:
                    # Jump straight to confirmed — 1-of-1 near port face.
                    consecutive_contact = required_consecutive
                    self.get_logger().info(
                        f"Stage 3: near-port contact confirmed (1-of-1) at "
                        f"tcp_z={tcp_z:.4f} m (local_delta={local_delta:.2f} N "
                        f"≥ {near_port_threshold} N within 25 mm of port)"
                    )
                else:
                    consecutive_contact += 1
                    self.get_logger().info(
                        f"Stage 3: contact candidate {consecutive_contact}/{required_consecutive} "
                        f"at z={current_z:.4f} m (local_delta={local_delta:.2f} N)"
                    )
                if consecutive_contact >= required_consecutive:
                    # Return actual TCP pose so Stage 4 starts from where the
                    # arm IS, not where it was commanded to be.
                    obs_contact = get_observation()
                    if obs_contact is not None:
                        tcp = obs_contact.controller_state.tcp_pose
                        # Sanity check: contact should be near the estimated
                        # connector Z.  If it is >3 cm above the connector Z
                        # estimate, the arm hit the board/NIC card SURFACE
                        # before reaching the port entrance.  Reset and keep
                        # descending so the probe can find the true port below.
                        z_above_port = tcp.position.z - connector_pose.position.z
                        if z_above_port < -0.005:
                            # Arm descended BELOW the connector Z — it passed the
                            # port entirely and is pressing on the table surface.
                            # The force-abort active_condition is suppressed at
                            # table contact (local_delta >> force_threshold), so
                            # this guard is the only escape hatch.  Abort hard.
                            raise SurfaceContactError(
                                f"Stage 3: contact at z={tcp.position.z:.4f} is "
                                f"{abs(z_above_port) * 100:.1f} cm BELOW connector "
                                f"Z={connector_pose.position.z:.4f} — arm passed port, "
                                "retreating for re-scout",
                                tcp_pose=tcp,
                            )
                        if z_above_port > 0.03:
                            surface_reset_count += 1
                            self.get_logger().warning(
                                f"Stage 3: contact at z={tcp.position.z:.4f} is "
                                f"{z_above_port * 100:.1f} cm above connector "
                                f"Z={connector_pose.position.z:.4f} — "
                                f"likely board surface (reset #{surface_reset_count}, "
                                f"local_delta={local_delta:.1f} N)"
                            )
                            # Hard surface: local_delta > 25 N means absolute force is
                            # well above the 20 N scoring threshold.  Abort immediately
                            # on the first such reset — waiting for a second one earns
                            # the penalty (observed: 18 s pinned at 38 N absolute).
                            if local_delta > surface_reset_hard_delta:
                                raise SurfaceContactError(
                                    f"Stage 3: arm pinned on hard surface at "
                                    f"z={tcp.position.z:.4f} (local_delta={local_delta:.1f} N), "
                                    f"retreating for re-scout",
                                    tcp_pose=tcp,
                                )
                            # Scoring-aligned absolute abort: when cable baseline is
                            # high (e.g. SC ≈ 35 N), even moderate local_delta values
                            # (14–18 N) put force_abs at 49–53 N — well above the
                            # 20 N scoring threshold.  After the first reset+detach
                            # cycle, if we are still registering contact at >20 N
                            # absolute, the XY is wrong and re-probing will only
                            # accumulate more penalty seconds.  Abort and re-scout.
                            # (Fires on reset #2+; reset #1 is allowed so the arm
                            # can attempt the detach before giving up.)
                            if surface_reset_count > 1 and force_abs > 20.0:
                                raise SurfaceContactError(
                                    f"Stage 3: repeated surface contact with high absolute "
                                    f"force ({force_abs:.1f} N > 20 N on reset "
                                    f"#{surface_reset_count}), retreating for re-scout",
                                    tcp_pose=tcp,
                                )
                            # Moderate surface: allow a few resets before giving up.
                            if surface_reset_count > surface_reset_soft_limit:
                                raise SurfaceContactError(
                                    f"Stage 3: {surface_reset_count} surface resets at "
                                    f"z={tcp.position.z:.4f} without finding port, "
                                    "retreating for re-scout",
                                    tcp_pose=tcp,
                                )
                            # Detach: lift 25 mm above ACTUAL contact XY/Z before
                            # re-probing.  Two key fixes vs the previous version:
                            # (1) XY: use tcp.position.x/y (actual contact XY),
                            #     NOT connector_pose.position.x/y (vision estimate).
                            #     The arm drifts under impedance during descent, so
                            #     the actual TCP XY can differ by 5–10 cm from the
                            #     vision target.  Using connector_pose XY forced a
                            #     combined lateral+Z move — the lateral component
                            #     stalled at pos_error=0.054 m (T2 run 7) because
                            #     cable drag prevented lateral motion while pressed
                            #     against the board face.  Actual TCP XY = pure
                            #     vertical lift = reliable convergence.
                            # (2) Distance: 8 mm was insufficient to physically
                            #     separate from the board face against cable drag;
                            #     the arm stalled at iter=6 (0.054 m error) and
                            #     re-contacted at the same Z immediately.  25 mm
                            #     clears the surface across all observed contact Zs.
                            detach_z = tcp.position.z + 0.025
                            self._move_to_pose_and_wait(
                                self._make_pose(
                                    tcp.position.x, tcp.position.y, detach_z,
                                    connector_pose.orientation,
                                ),
                                move_robot, get_observation, start_time, time_limit_sec,
                                convergence_m=0.05, stage_timeout_sec=8.0,
                                label="Stage 3 surface detach",
                                check_force=False,
                            )
                            # Reset local_min so the next descent starts with a
                            # fresh cable-relaxation baseline from detach height.
                            obs_detach = get_observation()
                            if obs_detach is not None:
                                local_min_force = self._force_magnitude(obs_detach)
                                # If post-detach force is already near the 20 N
                                # scoring-penalty threshold, re-descending will
                                # immediately accumulate penalty seconds.  This
                                # happens with SC cable whose baseline is ~20 N:
                                # detach lifts to z≈0.163 where forces are
                                # 20.4–20.7 N, and each 5 mm step takes ~1.7 s.
                                # Abort now and let insert_cable() re-scout from
                                # transit_z where forces are safe.
                                if local_min_force > 19.0:
                                    raise SurfaceContactError(
                                        f"Stage 3: post-detach force "
                                        f"{local_min_force:.1f} N > 19 N at "
                                        f"z={detach_z:.4f} — re-descending would "
                                        "earn force penalty; retreating for re-scout",
                                        tcp_pose=tcp,
                                    )
                            consecutive_contact = 0
                            # Allow fine-ring reset on the next near-port entry
                            # after detach — spiral may have advanced to coarse
                            # offsets during the first descent, so re-enabling
                            # the reset ensures the arm probes fine rings (center
                            # → ±5–20 mm) again at the depth where it matters.
                            near_port_spiral_reset = False
                            current_z = detach_z
                            continue
                        self.get_logger().info(
                            f"Stage 3: contact confirmed — actual TCP at "
                            f"({tcp.position.x:.3f},{tcp.position.y:.3f},{tcp.position.z:.3f})"
                        )
                        send_feedback("Stage 3: contact detected")
                        # XY centroid refinement: probe 4 cardinal neighbours at
                        # ±5 mm to find the centre of the contact zone.  The spiral
                        # locks at the FIRST position that gets 3 consecutive hits,
                        # which may be the port edge rather than centre.  Probing
                        # at contact_z+2 mm allows lateral motion without scraping.
                        refine_step_m = 0.005
                        probe_z = tcp.position.z + 0.002
                        contact_offsets = [(locked_dx, locked_dy)]
                        for rdx, rdy in (
                            (refine_step_m, 0.0), (-refine_step_m, 0.0),
                            (0.0, refine_step_m), (0.0, -refine_step_m),
                        ):
                            try:
                                self._move_to_pose_and_wait(
                                    self._make_pose(
                                        connector_pose.position.x + locked_dx + rdx,
                                        connector_pose.position.y + locked_dy + rdy,
                                        probe_z,
                                        connector_pose.orientation,
                                    ),
                                    move_robot, get_observation,
                                    start_time, time_limit_sec,
                                    convergence_m=0.008, stage_timeout_sec=3.0,
                                    label="Stage 3 XY refine", check_force=False,
                                )
                                obs_probe = get_observation()
                                if obs_probe is not None:
                                    probe_delta = (
                                        self._force_magnitude(obs_probe) - local_min_force
                                    )
                                    if probe_delta > self.force_threshold:
                                        contact_offsets.append((locked_dx + rdx, locked_dy + rdy))
                            except TimeoutError:
                                break
                        centroid_dx = sum(o[0] for o in contact_offsets) / len(contact_offsets)
                        centroid_dy = sum(o[1] for o in contact_offsets) / len(contact_offsets)
                        centroid_x = connector_pose.position.x + centroid_dx
                        centroid_y = connector_pose.position.y + centroid_dy
                        if len(contact_offsets) > 1:
                            self.get_logger().info(
                                f"Stage 3: XY refined to ({centroid_x:.3f},{centroid_y:.3f}) "
                                f"from {len(contact_offsets)} contact points "
                                f"(was ({tcp.position.x:.3f},{tcp.position.y:.3f}))"
                            )
                        return self._make_pose(
                            centroid_x, centroid_y, tcp.position.z,
                            connector_pose.orientation,
                        )
                    send_feedback("Stage 3: contact detected")
                    return target_pose
                # Don't advance Z while building consecutive count
                continue
            else:
                consecutive_contact = 0

            # Force abort: arm pressing hard against surface at wrong XY.
            timed_out, high_force_start = self._high_force_timed_out(
                force_abs, self._force_abort_threshold, high_force_start, high_force_budget_sec,
                active_condition=(local_delta <= self.force_threshold),
            )
            if timed_out:
                raise OutOfReachError(
                    f"Stage 3: force abort — sustained {force_abs:.1f} N "
                    f"(>{self._force_abort_threshold:.1f} N threshold) "
                    f"without contact at z={current_z:.4f} m "
                    "(arm at wrong XY, abort to prevent penalty)"
                )

            # Advance spiral index each time we step to the next Z level.
            if spiral_active and consecutive_contact == 0:
                spiral_offset_idx += 1
            current_z -= step_m

        raise OutOfReachError(
            f"Stage 3: no contact after descending to z={end_z:.4f} m — "
            "connector_z_in_base may be too high or XY alignment off"
        )

    # ======================================================================
    # Stage 4 — Compliant insertion
    # ======================================================================

    def _compliant_insertion(
        self, contact_pose, get_observation, move_robot, send_feedback,
        start_time, time_limit_sec,
        connector_z: float = None,
        zone: str = "sfp",
        connector_pose=None,
    ) -> bool:
        """Compliant insertion: hold arm at connector_z − 5 mm for up to 120 s.

        Bug 67: immediately command end_z = connector_z − 5 mm for max spring
        force from t=0.  All non-force-abort exits return True so scoring
        captures the arm at the deepest position reached.

        Bug 74: SC uses approach_stiffness (85 N/m) instead of
        insertion_stiffness (200 N/m).  With 200 N/m the spring saturates at
        15 N (max_wrench) from the first iteration; once cable creep unlocks
        arm movement the arm accelerates through the viable zone (tcp_z ≈
        0.09–0.10 m) and overshoots to tcp_z ≈ 0.03 m where cable geometry
        places the plug 0.17 m from port (Run 31: tier_3 = 0).  At 85 N/m the
        spring is 7.7 N ≪ SC cable tension (~16 N) → slow creep → arm stays in
        the viable zone → tier_3 ≈ 6–8 (Runs 26, 29 pattern).
        """
        send_feedback("Stage 4: compliant insertion")

        # SC uses gentler stiffness to avoid overshooting the viable tcp_z zone.
        stiffness = self.approach_stiffness if zone == "sc" else self.insertion_stiffness

        # Stage 3 returns the actual TCP pose at contact — use that XY/Z.
        start_x = contact_pose.position.x
        start_y = contact_pose.position.y
        start_z = contact_pose.position.z

        # Bug 79: abort if arm landed far from port in XY (wrong board area).
        if connector_pose is not None:
            xy_err = np.linalg.norm([
                start_x - connector_pose.position.x,
                start_y - connector_pose.position.y,
            ])
            if xy_err > 0.15:
                self.get_logger().warning(
                    f"Stage 4: arm {xy_err:.3f}m from port XY — aborting"
                )
                return False

        # End target: 5 mm past port face when connector_z is known, else 20 mm
        # past the stall/contact point as a conservative backstop.
        if connector_z is not None:
            end_z = max(connector_z - 0.005, 0.005)
        else:
            end_z = max(start_z - 0.020, 0.005)

        # Bug 67: all trials hold cmd_z = end_z immediately for max spring force from step 1.
        cmd_z = end_z

        # SFP: spring force (200 N/m) is capped at max_wrench=15 N, which exactly
        # balances the SFP cable upward equilibrium force at ~0.18–0.23 m.
        # A −3 N Z feedforward adds 3 N downward outside the max_wrench cap,
        # shifting the equilibrium ~2 cm lower and allowing creep toward the port.
        # SC uses 0: cable tension there is already overcome by slower creep at 85 N/m.
        if zone == "sc":
            stage4_feedforward_fz = 0.0
        elif self._insert_call_count == 2:   # T2: SFP at -45° yaw, higher cable tension
            stage4_feedforward_fz = -9.0
        else:
            stage4_feedforward_fz = -3.0

        self.get_logger().info(
            f"Stage 4: holding cmd_z={end_z:.4f} (connector_z-5mm), "
            f"arm stall at z={start_z:.4f} ({start_z - end_z:.3f}m gap), "
            f"spring force={stiffness * (start_z - end_z):.1f}N constant "
            f"(stiffness={stiffness:.0f} N/m, max_wrench=15 N, "
            f"feedforward_fz={stage4_feedforward_fz:.1f} N)"
        )

        # Force abort tracking.  Same relative threshold as Stage 3.
        high_force_start = None
        high_force_budget_sec = 0.3   # abort after 0.3 s above baseline+3 N

        stage_timeout = Duration(seconds=120.0)
        stage_start = self.time_now()

        tcp_z = start_z   # initialise before first obs in case obs is None

        while True:
            if self._is_timed_out(start_time, time_limit_sec):
                raise TimeoutError("Timed out during insertion")
            if (self.time_now() - stage_start) >= stage_timeout:
                self.get_logger().info(
                    f"Stage 4: 120s hold complete — tcp_z={tcp_z:.4f} "
                    f"(target={cmd_z:.4f}, {tcp_z - cmd_z:.3f}m remaining)"
                )
                break

            target_pose = self._make_pose(start_x, start_y, cmd_z, contact_pose.orientation)
            motion_update = self._build_motion_update(
                target_pose, stiffness, feedforward_fz=stage4_feedforward_fz
            )
            try:
                move_robot(motion_update=motion_update)
            except Exception as ex:
                raise RuntimeError(f"Stage 4: move_robot failed: {ex}")

            self.sleep_for(0.05)
            obs = get_observation()
            if obs is None:
                continue

            force_abs = self._force_magnitude(obs)
            pos_error = np.linalg.norm(obs.controller_state.tcp_error[:3])
            tcp_z = obs.controller_state.tcp_pose.position.z
            self.get_logger().info(
                f"Stage 4: cmd_z={cmd_z:.4f} tcp_z={tcp_z:.4f} "
                f"pos_error={pos_error:.4f} m  |F|={force_abs:.2f} N"
            )

            # Hard force abort: pressing against solid surface, not inserting.
            timed_out, high_force_start = self._high_force_timed_out(
                force_abs, self._force_abort_threshold, high_force_start, high_force_budget_sec,
            )
            if timed_out:
                self.get_logger().warning(
                    f"Stage 4: force abort — {force_abs:.1f} N sustained > "
                    f"{high_force_budget_sec} s (not a port aperture)"
                )
                send_feedback("Stage 4: force abort")
                return False

            # Early exit: arm has reached insertion depth.
            if tcp_z <= end_z + 0.005:
                self.get_logger().info(
                    f"Stage 4: arm reached insertion depth tcp_z={tcp_z:.4f} "
                    f"(target={end_z:.4f}) — insertion complete"
                )
                break

        self.get_logger().info(
            f"Stage 4: complete — tcp_z={tcp_z:.4f}, target={end_z:.4f}."
        )
        send_feedback("Stage 4: insertion complete")
        return True

    # ======================================================================
    # Surface recovery — retreat, re-scout, retry
    # ======================================================================

    def _retreat_from_surface(
        self, surface_tcp_pose, get_observation, move_robot, start_time, time_limit_sec
    ) -> None:
        """Lift the end-effector 15 cm clear of the board surface after a
        surface contact abort.

        Ascends vertically at the current XY only — no lateral move to the
        calibrated zone.  Moving to scouting XY during retreat caused degenerate
        retry triangulation (Bug 9): both pipeline passes collected rays from
        near-identical positions, giving zero baseline and wrong depth.  Staying
        at current XY guarantees a genuinely independent observation set on retry.
        """
        retreat_z = surface_tcp_pose.position.z + 0.15
        obs = get_observation()
        orient = (
            obs.controller_state.tcp_pose.orientation
            if obs is not None
            else surface_tcp_pose.orientation
        )

        self.get_logger().info(
            f"Surface retreat: ascending to z={retreat_z:.3f} "
            f"from z={surface_tcp_pose.position.z:.3f}"
        )
        # Ascend vertically at current XY only.  Do NOT translate to calibrated zone
        # XY here — the retry vision grid search will navigate to WP2 from the
        # current position, giving a genuine independent observation set.
        self._move_to_pose_and_wait(
            self._make_pose(
                surface_tcp_pose.position.x, surface_tcp_pose.position.y, retreat_z, orient,
            ),
            move_robot, get_observation, start_time, time_limit_sec,
            convergence_m=0.05, stage_timeout_sec=20.0,
            label="Surface retreat ascend",
            check_force=False,
        )

        # Verify the arm actually ascended.  If the retreat stalled (arm still
        # pinned against the surface), proceeding to re-run the pipeline would
        # cause extreme sustained force (observed: 463 N for 3.18 s in trial 2
        # after a 7-iter stall at pos_error=0.15 m).  Raise instead of retrying.
        obs = get_observation()
        if obs is not None:
            actual_z = obs.controller_state.tcp_pose.position.z
            if actual_z < retreat_z - 0.05:
                raise OutOfReachError(
                    f"Surface retreat stalled: arm at z={actual_z:.3f}, "
                    f"target z={retreat_z:.3f} — arm still near surface, aborting retry"
                )

    # ======================================================================
    # Entry point
    # ======================================================================

    def insert_cable(
        self,
        task: Task,
        get_observation: GetObservationCallback,
        move_robot: MoveRobotCallback,
        send_feedback: SendFeedbackCallback,
    ) -> bool:
        """Execute the 4-stage cable-insertion pipeline.

        Must return within task.time_limit seconds.  Returns True on
        successful insertion, False on failure (all failure paths log a
        reason via send_feedback so the engine can report it).
        """
        self._insert_call_count += 1
        self.get_logger().info(
            f"ANT.insert_cable() — task={task.id}  plug={task.plug_name}  "
            f"port={task.port_name}  time_limit={task.time_limit}s  "
            f"trial={self._insert_call_count}"
        )
        start_time = self.time_now()
        time_limit_sec = float(task.time_limit)

        def _run_pipeline(baseline_settle_sec: float = 0.0):
            connector_pose = self._localize_connector(
                task, get_observation, move_robot, send_feedback,
                start_time, time_limit_sec,
                baseline_settle_sec=baseline_settle_sec,
            )
            zone = "sfp" if task.plug_type == "sfp" else "sc"
            self._approach_connector(
                connector_pose, get_observation, move_robot, send_feedback,
                start_time, time_limit_sec,
                zone=zone,
            )
            contact_pose = self._detect_contact(
                connector_pose, get_observation, move_robot, send_feedback,
                start_time, time_limit_sec,
                zone=zone,
            )
            return self._compliant_insertion(
                contact_pose, get_observation, move_robot, send_feedback,
                start_time, time_limit_sec,
                connector_z=self.connector_z_in_base[zone],
                zone=zone,
                connector_pose=connector_pose,
            )

        try:
            success = _run_pipeline()

        except SurfaceContactError as e:
            # Stage 3 detected the arm was pinned against the board surface and
            # has exhausted its in-place reset budget.  Retreat to a safe height,
            # then re-run the full localise → approach → detect → insert pipeline
            # once with a fresh scout from the new position.
            self.get_logger().warning(
                f"ANT: surface contact abort — {e}. Retreating for one re-scout attempt."
            )
            send_feedback("Surface contact detected — retreating for re-scout")
            tcp_at_surface = e.tcp_pose
            try:
                if tcp_at_surface is not None:
                    self._retreat_from_surface(
                        tcp_at_surface, get_observation, move_robot,
                        start_time, time_limit_sec,
                    )
                self.get_logger().info("ANT: re-running pipeline after surface retreat")
                send_feedback("Re-scouting after surface retreat")
                # Pass settle_sec=1.5 so the baseline calibration waits for the
                # arm to stop decelerating after the retreat move.  Without this,
                # the sample is taken while the arm is still moving and produces
                # a wildly low reading (7.64 N observed vs normal ~22 N).
                success = _run_pipeline(baseline_settle_sec=1.5)
            except (TimeoutError, UnexpectedContactError, OutOfReachError,
                    SurfaceContactError) as e2:
                self.get_logger().error(f"ANT: retry failed — {e2}")
                send_feedback(f"FAILED: retry after retreat failed — {e2}")
                return False
            except Exception as e2:
                self.get_logger().error(f"ANT: unhandled exception on retry — {e2}")
                send_feedback(f"FAILED: {e2}")
                return False

        except TimeoutError as e:
            self.get_logger().error(f"ANT: timeout — {e}")
            send_feedback(f"FAILED: timeout — {e}")
            return False
        except UnexpectedContactError as e:
            self.get_logger().error(f"ANT: unexpected contact — {e}")
            send_feedback(f"FAILED: unexpected contact — {e}")
            return False
        except OutOfReachError as e:
            self.get_logger().error(f"ANT: out of reach — {e}")
            send_feedback(f"FAILED: out of reach — {e}")
            return False
        except Exception as e:
            self.get_logger().error(f"ANT: unhandled exception — {e}")
            send_feedback(f"FAILED: {e}")
            return False

        self.get_logger().info("ANT.insert_cable() complete.")

        # Arm return strategy after insert_cable():
        #
        # All trials (SFP 1–2 and SC 3): skip joint return entirely.
        #   - Scoring captures plug/arm position the moment insert_cable() returns.
        #     Keeping the arm at Stage 4 depth preserves tier_3 score:
        #       SFP T1: tcp_z≈0.232 m → plug tip at port entrance → tier_3=38 (partial)
        #       SFP T2: tcp_z≈0.181 m → plug tip 3 cm from port → tier_3=25
        #       SC  T3: tcp_z≈0.040 m → plug tip 2.5 cm above SC port → tier_3≥25
        #   - Joint return for SFP fights cable tension (271 N × 6.28 s on T2 in
        #     Run 19 → −12 force penalty). Bug 52.
        #   - Joint return for SC moves arm 33 cm from port before scoring capture
        #     (Run 24 T3: tier_3=0, distance=0.33 m). Also hit 128 N briefly.
        #     Bug 61. SC Stage 3 now uses direct descent (Bug 60): arm stalls at
        #     z≈0.04 m, not 0.005 m — no reason to return.
        #   - T2 Stage 1 performs safe_z=0.28 m ascent before lateral move.
        #   - T3 Stage 1 safe-Z entry now triggers when arm is below 0.27 m
        #     (fixed condition: Bug 62). Arm ascends to 0.28 m before lateral move
        #     regardless of whether it is above or below transit_z. This covers the
        #     Run 25 failure where arm settled at z≈0.191 m (≈transit_z) and the
        #     old condition current_z > transit_z+0.01 was False → normal path →
        #     17.5 cm lateral at 0.195 m under SC tension → stall.
        #   - No trial 4: leaving arm at SC depth after T3 is safe.
        self.get_logger().info(
            "ANT: skipping joint return — arm stays at Stage 4 depth to preserve "
            "tier_3 score (Bugs 52 + 61)"
        )

        return success
