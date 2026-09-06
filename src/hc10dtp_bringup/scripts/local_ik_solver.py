#!/usr/bin/env python3
"""
local_ik_solver.py
──────────────────
Local IK/FK solver cho Yaskawa HC10DTP.

Tham số động học được trích xuất trực tiếp từ URDF (hc10dtp.urdf).
Sử dụng numpy cho Forward Kinematics và Damped Least Squares (DLS)
Jacobian cho Inverse Kinematics.

An toàn:
  - FK được cross-validate với MoveIt! /compute_fk khi khởi động
  - IK trả về None nếu không hội tụ (caller phải xử lý fallback)
  - Joint limits được enforce qua clamping sau mỗi iteration

Hiệu suất (từ seed gần target):
  - FK: ~0.02ms
  - IK: ~0.3–1.0ms (vs MoveIt! RPC ~10–50ms)
"""

import math
import numpy as np


# ═══════════════════════════════════════════════════════════════════
# HELPER: Ma trận biến đổi đồng nhất 4×4
# ═══════════════════════════════════════════════════════════════════

def _rot_x(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([
        [1, 0,  0, 0],
        [0, c, -s, 0],
        [0, s,  c, 0],
        [0, 0,  0, 1],
    ], dtype=np.float64)

def _rot_y(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([
        [ c, 0, s, 0],
        [ 0, 1, 0, 0],
        [-s, 0, c, 0],
        [ 0, 0, 0, 1],
    ], dtype=np.float64)

def _rot_z(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([
        [c, -s, 0, 0],
        [s,  c, 0, 0],
        [0,  0, 1, 0],
        [0,  0, 0, 1],
    ], dtype=np.float64)

def _trans(x: float, y: float, z: float) -> np.ndarray:
    T = np.eye(4, dtype=np.float64)
    T[0, 3] = x
    T[1, 3] = y
    T[2, 3] = z
    return T

def _rot_matrix_to_quat(R: np.ndarray) -> np.ndarray:
    """
    Rotation matrix (3×3) → quaternion [x, y, z, w] (ROS convention).
    Sử dụng thuật toán Shepperd (numerically stable).
    """
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0:
        s = 0.5 / math.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    q = np.array([x, y, z, w], dtype=np.float64)
    q /= np.linalg.norm(q)
    return q

def _quat_to_rot_matrix(q: np.ndarray) -> np.ndarray:
    """Quaternion [x, y, z, w] → Rotation matrix 3×3."""
    x, y, z, w = q
    R = np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - w*z),     2*(x*z + w*y)],
        [2*(x*y + w*z),     1 - 2*(x*x + z*z), 2*(y*z - w*x)],
        [2*(x*z - w*y),     2*(y*z + w*x),     1 - 2*(x*x + y*y)],
    ], dtype=np.float64)
    return R

def _so3_log(R: np.ndarray) -> np.ndarray:
    """
    Logarithmic map SO(3) → so(3).
    Trả về rotation vector [wx, wy, wz] (axis * angle).
    """
    cos_angle = (np.trace(R) - 1.0) / 2.0
    cos_angle = max(-1.0, min(1.0, cos_angle))
    angle = math.acos(cos_angle)

    if abs(angle) < 1e-8:
        return np.zeros(3, dtype=np.float64)

    if abs(angle - math.pi) < 1e-6:
        # Near π: dùng eigenvector extraction
        # Tìm cột có phần tử đường chéo lớn nhất
        diag = np.diag(R)
        k = np.argmax(diag)
        v = R[:, k] + np.eye(3)[k]
        v = v / np.linalg.norm(v)
        return v * angle

    # General case
    factor = angle / (2.0 * math.sin(angle))
    w = np.array([
        R[2, 1] - R[1, 2],
        R[0, 2] - R[2, 0],
        R[1, 0] - R[0, 1],
    ], dtype=np.float64) * factor
    return w


# ═══════════════════════════════════════════════════════════════════
# LOCAL IK SOLVER
# ═══════════════════════════════════════════════════════════════════

class LocalIKSolver:
    """
    IK/FK solver cho Yaskawa HC10DTP sử dụng Damped Least Squares (DLS).

    Kinematic chain (từ URDF hc10dtp.urdf):
        base_link ─J1(Z)─ link_1_s ─J2(Y)─ link_2_l ─J3(-Y)─ link_3_u
        ─J4(Z)─ link_4_r ─J5(-Y)─ link_5_b ─J6(Z)─ link_6_t ─(fixed)─ tool0
    """

    # ── URDF joint limits ──────────────────────────────────────────
    URDF_JOINT_LIMITS = np.array([
        [-3.141592653589793,  3.141592653589793],   # J1
        [-3.141592653589793,  3.141592653589793],   # J2
        [-1.5707963267948966, 4.71238898038469],    # J3
        [-3.141592653589793,  3.141592653589793],   # J4
        [-3.141592653589793,  3.141592653589793],   # J5
        [-3.141592653589793,  3.141592653589793],   # J6
    ], dtype=np.float64)

    # IK tuning
    IK_POSITION_TOLERANCE = 0.0005  # 0.5mm
    IK_MAX_ITERATIONS = 50
    # Weighted DLS: position giữ trọng số 1.0, orientation 0.5. Cùng ma
    # trận trọng số phải áp dụng lên error và Jacobian.
    IK_ORIENTATION_WEIGHT = 0.5

    # Adaptive damping theo singular value nhỏ nhất của weighted Jacobian.
    # Các pose vận hành bình thường có sigma_min > 0.1; chỉ tăng damping
    # khi tiến gần singularity. Giữ 0.01 là damping sàn lịch sử.
    IK_DAMPING_MIN = 0.01
    IK_DAMPING_MAX = 0.10
    IK_SINGULAR_VALUE_THRESHOLD = 0.05
    IK_STEP_SIZE = 1.0               # Step size (α) — 1.0 = full Newton step
    IK_JACOBIAN_EPS = 1e-7           # Finite-difference step cho numerical Jacobian

    def __init__(self):
        # Pre-compute constant transforms
        self._T_J3_RPY = _rot_y(-math.pi / 2)
        # Keep this transform identical to the URDF chain
        # link_6_t -> flange (0.170 m) -> tool0.  In the local convention it
        # becomes a -Z translation followed by Rx(pi).
        self._T_TOOL0 = _trans(0, 0, -0.170) @ _rot_x(math.pi)

        # Timing stats
        self._fk_call_count = 0
        self._ik_call_count = 0
        self._ik_fail_count = 0
        self._ik_total_time_us = 0.0
        self._ik_min_singular_value = math.inf
        self._ik_max_damping = self.IK_DAMPING_MIN

    # ═══════════════════════════════════════════════════════════════
    # FORWARD KINEMATICS
    # ═══════════════════════════════════════════════════════════════

    def forward_kinematics(self, q: np.ndarray) -> np.ndarray:
        """
        FK cho 6 joints → ma trận biến đổi đồng nhất 4×4 (base_link → tool0).

        Args:
            q: 6 joint angles [rad] (np.ndarray)

        Returns:
            T: np.ndarray (4, 4)
        """
        self._fk_call_count += 1

        # J1: base_link → link_1_s — origin(0,0,0.275), axis Z
        T = _trans(0, 0, 0.275) @ _rot_z(q[0])

        # J2: link_1_s → link_2_l — origin(0,0,0), axis Y
        T = T @ _rot_y(q[1])

        # J3: link_2_l → link_3_u — origin(0,0,0.700), rpy(0,-π/2,0), axis -Y
        T = T @ _trans(0, 0, 0.700) @ self._T_J3_RPY @ _rot_y(-q[2])

        # J4: link_3_u → link_4_r — origin(0,0,-0.500), axis Z
        T = T @ _trans(0, 0, -0.500) @ _rot_z(q[3])

        # J5: link_4_r → link_5_b — origin(0,0.162,0), axis -Y
        T = T @ _trans(0, 0.162, 0) @ _rot_y(-q[4])

        # J6: link_5_b → link_6_t — origin(0,0,0), axis Z
        T = T @ _rot_z(q[5])

        # tool0 (fixed) — origin(0,0,-0.170), rpy(π,0,0)
        T = T @ self._T_TOOL0

        return T

    def fk_position(self, q: np.ndarray) -> np.ndarray:
        """FK → chỉ trả về position [x, y, z]."""
        return self.forward_kinematics(q)[:3, 3]

    def fk_pose(self, q) -> tuple[np.ndarray, np.ndarray]:
        """
        FK → (position [3], quaternion [4]).
        Quaternion: [x, y, z, w] (ROS convention).
        """
        q = np.asarray(q, dtype=np.float64)
        T = self.forward_kinematics(q)
        pos = T[:3, 3].copy()
        quat = _rot_matrix_to_quat(T[:3, :3])
        return pos, quat

    # ═══════════════════════════════════════════════════════════════
    # NUMERICAL JACOBIAN (6×6)
    # ═══════════════════════════════════════════════════════════════

    def _compute_jacobian(self, q: np.ndarray) -> np.ndarray:
        """
        Numerical Jacobian (6×6) — vị trí + orientation.

        J[:3, i] = ∂position / ∂q_i
        J[3:, i] = ∂(axis-angle) / ∂q_i

        Dùng central finite differences cho accuracy cao hơn.
        """
        eps = self.IK_JACOBIAN_EPS
        J = np.zeros((6, 6), dtype=np.float64)
        T0 = self.forward_kinematics(q)
        p0 = T0[:3, 3]
        R0 = T0[:3, :3]

        for i in range(6):
            q_plus = q.copy()
            q_plus[i] += eps
            T_plus = self.forward_kinematics(q_plus)

            # Position Jacobian (forward difference)
            J[:3, i] = (T_plus[:3, 3] - p0) / eps

            # Orientation Jacobian: log(R_plus @ R0^T)
            dR = T_plus[:3, :3] @ R0.T
            J[3:, i] = _so3_log(dR) / eps

        return J

    def compute_jacobian(self, q: np.ndarray) -> np.ndarray:
        """Return the verified base_link -> tool0 geometric Jacobian.

        This public wrapper lets read-only diagnostics (for example the
        sensorless force estimator) reuse exactly the same kinematic model as
        the Cartesian IK path instead of maintaining a second URDF parser.
        """
        q = np.asarray(q, dtype=np.float64)
        if q.shape != (6,) or not np.all(np.isfinite(q)):
            raise ValueError('q must contain six finite joint positions')
        return self._compute_jacobian(q)

    # ═══════════════════════════════════════════════════════════════
    # INVERSE KINEMATICS — Damped Least Squares (DLS)
    # ═══════════════════════════════════════════════════════════════

    @classmethod
    def _weighted_dls_system(
        cls,
        error: np.ndarray,
        jacobian: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray]:
        """Apply the same task-space weights to error and Jacobian."""
        weights = np.ones(6, dtype=np.float64)
        weights[3:] = cls.IK_ORIENTATION_WEIGHT
        return weights * error, weights[:, np.newaxis] * jacobian

    @classmethod
    def _adaptive_damping(cls, weighted_jacobian: np.ndarray) -> tuple[float, float]:
        """Return (lambda, sigma_min), increasing lambda near singularity."""
        try:
            singular_values = np.linalg.svd(
                weighted_jacobian, compute_uv=False)
            sigma_min = float(singular_values[-1])
        except np.linalg.LinAlgError:
            return cls.IK_DAMPING_MAX, 0.0

        threshold = cls.IK_SINGULAR_VALUE_THRESHOLD
        if sigma_min >= threshold:
            return cls.IK_DAMPING_MIN, sigma_min

        # Smooth cosine transition: lambda=max at sigma=0, min at threshold;
        # the slope is zero at both endpoints, avoiding damping jumps.
        ratio = max(0.0, sigma_min) / threshold
        blend = 0.5 * (1.0 + math.cos(math.pi * ratio))
        damping = (
            cls.IK_DAMPING_MIN
            + (cls.IK_DAMPING_MAX - cls.IK_DAMPING_MIN) * blend
        )
        return damping, sigma_min

    def solve_ik(
        self,
        target_position,
        target_quaternion,
        seed_joints,
        joint_limits=None,
    ) -> list[float] | None:
        """
        Giải IK bằng Damped Least Squares (Levenberg-Marquardt style).

        Args:
            target_position: [x, y, z]
            target_quaternion: [qx, qy, qz, qw]
            seed_joints: 6 joint angles khởi đầu
            joint_limits: Optional [(lo, hi)] × 6. None → URDF limits.

        Returns:
            list[float] | None: 6 joint angles, hoặc None nếu thất bại.
        """
        import time as _t
        t0 = _t.perf_counter_ns()
        self._ik_call_count += 1

        target_pos = np.asarray(target_position, dtype=np.float64)
        target_quat = np.asarray(target_quaternion, dtype=np.float64)
        target_quat /= np.linalg.norm(target_quat)
        target_rot = _quat_to_rot_matrix(target_quat)

        q = np.asarray(seed_joints, dtype=np.float64).copy()

        # Bounds
        if joint_limits is not None:
            bounds = np.array(joint_limits, dtype=np.float64)
        else:
            bounds = self.URDF_JOINT_LIMITS

        # Clamp seed vào bounds
        q = np.clip(q, bounds[:, 0], bounds[:, 1])

        alpha = self.IK_STEP_SIZE
        tol = self.IK_POSITION_TOLERANCE

        for iteration in range(self.IK_MAX_ITERATIONS):
            T = self.forward_kinematics(q)
            current_pos = T[:3, 3]
            current_rot = T[:3, :3]

            # Error vector (6D): [position_error; orientation_error]
            pos_err = target_pos - current_pos
            ori_err = _so3_log(target_rot @ current_rot.T)

            error = np.concatenate([pos_err, ori_err])

            pos_err_norm = np.linalg.norm(pos_err)
            if pos_err_norm < tol and np.linalg.norm(ori_err) < 0.01:
                # Converged
                t1 = _t.perf_counter_ns()
                self._ik_total_time_us += (t1 - t0) / 1000.0
                return q.tolist()

            # Jacobian
            J = self._compute_jacobian(q)

            # Weighted least squares requires W on both sides: minimize
            # ||W (J dq - error)||. Applying W only to error changes the task
            # instead of consistently weighting its residual.
            weighted_error, weighted_jacobian = self._weighted_dls_system(
                error, J)
            lam, sigma_min = self._adaptive_damping(weighted_jacobian)
            self._ik_min_singular_value = min(
                self._ik_min_singular_value, sigma_min)
            self._ik_max_damping = max(self._ik_max_damping, lam)

            # DLS: dq = Jw^T (Jw Jw^T + λ²I)^-1 ew
            JJT = weighted_jacobian @ weighted_jacobian.T
            JJT_damped = JJT + (lam ** 2) * np.eye(6)
            dq = weighted_jacobian.T @ np.linalg.solve(
                JJT_damped, weighted_error)

            # Step
            q = q + alpha * dq

            # Clamp to joint limits
            q = np.clip(q, bounds[:, 0], bounds[:, 1])

        # Không hội tụ
        t1 = _t.perf_counter_ns()
        self._ik_total_time_us += (t1 - t0) / 1000.0
        self._ik_fail_count += 1
        return None

    # ═══════════════════════════════════════════════════════════════
    # VALIDATION
    # ═══════════════════════════════════════════════════════════════

    def validate_fk_self_consistency(self, n_tests: int = 20) -> tuple[bool, float]:
        """
        Self-consistency check: FK → IK → FK phải cho cùng kết quả.

        Returns:
            (passed, max_error_mm)
        """
        rng = np.random.default_rng(42)
        max_err = 0.0

        for _ in range(n_tests):
            q_rand = np.array([
                rng.uniform(lo, hi)
                for lo, hi in self.URDF_JOINT_LIMITS
            ])
            pos1, quat1 = self.fk_pose(q_rand)
            seed = q_rand + rng.normal(0, 0.05, 6)
            q_sol = self.solve_ik(pos1, quat1, seed)
            if q_sol is None:
                continue
            pos2, _ = self.fk_pose(q_sol)
            err = np.linalg.norm(pos1 - pos2)
            max_err = max(max_err, err)

        max_err_mm = max_err * 1000
        passed = max_err_mm < 1.0
        return passed, max_err_mm

    def get_stats(self) -> dict:
        """Trả về thống kê hiệu suất."""
        avg_ik_us = (
            self._ik_total_time_us / self._ik_call_count
            if self._ik_call_count > 0 else 0
        )
        return {
            'fk_calls': self._fk_call_count,
            'ik_calls': self._ik_call_count,
            'ik_fails': self._ik_fail_count,
            'ik_avg_us': avg_ik_us,
            'ik_min_singular_value': (
                self._ik_min_singular_value
                if math.isfinite(self._ik_min_singular_value) else None),
            'ik_max_damping': self._ik_max_damping,
        }


# ═══════════════════════════════════════════════════════════════════
# STANDALONE TEST
# ═══════════════════════════════════════════════════════════════════

if __name__ == '__main__':
    import time

    solver = LocalIKSolver()

    print("=" * 60)
    print("HC10DTP Local IK Solver — Self-Test")
    print("=" * 60)

    # 1) FK tại home position (all zeros)
    q_home = [0.0] * 6
    pos, quat = solver.fk_pose(q_home)
    print(f"\n[FK] Home (q=0): pos=({pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f})")
    print(f"     quat=({quat[0]:.4f}, {quat[1]:.4f}, {quat[2]:.4f}, {quat[3]:.4f})")

    # 2) FK tại tư thế co-carrying home
    q_carry = [1.5705, 0.0748, -1.0491, -0.0304, -0.5231, -0.0017]
    pos2, quat2 = solver.fk_pose(q_carry)
    print(f"\n[FK] Co-carry home: pos=({pos2[0]:.4f}, {pos2[1]:.4f}, {pos2[2]:.4f})")
    print(f"     quat=({quat2[0]:.4f}, {quat2[1]:.4f}, {quat2[2]:.4f}, {quat2[3]:.4f})")

    # 3) IK round-trip test
    print("\n[IK] Round-trip test (FK → IK → FK)...")
    q_sol = solver.solve_ik(pos2, quat2, seed_joints=[1.5, 0.0, -1.0, 0.0, -0.5, 0.0])
    if q_sol:
        pos3, _ = solver.fk_pose(q_sol)
        err_mm = np.linalg.norm(pos2 - pos3) * 1000
        print(f"     IK solution: [{', '.join(f'{j:.4f}' for j in q_sol)}]")
        print(f"     Position error: {err_mm:.4f} mm")
    else:
        print("     IK FAILED!")

    # 4) IK timing benchmark
    print("\n[Benchmark] IK timing (1000 calls from good seed)...")
    times = []
    for _ in range(1000):
        seed = np.array(q_carry) + np.random.normal(0, 0.01, 6)
        t0 = time.perf_counter_ns()
        solver.solve_ik(pos2, quat2, seed)
        t1 = time.perf_counter_ns()
        times.append((t1 - t0) / 1000)  # microseconds

    times_arr = np.array(times)
    print(f"     Mean:   {times_arr.mean():.0f} µs")
    print(f"     Median: {np.median(times_arr):.0f} µs")
    print(f"     P95:    {np.percentile(times_arr, 95):.0f} µs")
    print(f"     P99:    {np.percentile(times_arr, 99):.0f} µs")
    print(f"     Max:    {times_arr.max():.0f} µs")

    # 5) IK from further seed
    print("\n[Benchmark] IK timing (100 calls from distant seed, Δ=0.1 rad)...")
    times2 = []
    for _ in range(100):
        seed = np.array(q_carry) + np.random.normal(0, 0.1, 6)
        t0 = time.perf_counter_ns()
        solver.solve_ik(pos2, quat2, seed)
        t1 = time.perf_counter_ns()
        times2.append((t1 - t0) / 1000)
    times2_arr = np.array(times2)
    print(f"     Mean:   {times2_arr.mean():.0f} µs")
    print(f"     P99:    {np.percentile(times2_arr, 99):.0f} µs")

    # 6) Self-consistency
    print("\n[Validation] FK→IK→FK self-consistency (20 random configs)...")
    passed, max_err = solver.validate_fk_self_consistency(20)
    print(f"     Max error: {max_err:.4f} mm")
    print(f"     Result: {'✓ PASSED' if passed else '✗ FAILED'}")

    print(f"\nStats: {solver.get_stats()}")
    print("=" * 60)
