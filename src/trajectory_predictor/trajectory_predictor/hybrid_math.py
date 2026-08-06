#!/usr/bin/env python3
"""
hybrid_math.py
==============
Các hàm toán học thuần túy cho kiến trúc Hybrid GRU (FOLLOWER) + MJM (LEADER).
Không phụ thuộc ROS — có thể import và unit test độc lập.
"""

import numpy as np


def fitts_law_duration(
    x_current: np.ndarray,
    x_goal: np.ndarray,
    a: float = 4.4455,
    b: float = 1.4248,
    w: float = 0.3,
) -> float:
    """Ước lượng thời gian di chuyển (giây) bằng Fitts' Law.

    Công thức:  t_f = a + b * log2(2D / W)

    Trong đó:
        D   = khoảng cách Euclidean từ x_current đến x_goal
        W   = target width (độ rộng vùng đích)
        a,b = hệ số hồi quy Fitts (từ thực nghiệm co-carrying)

    Trường hợp đặc biệt: nếu D ≈ 0 (đã đến đích), trả về a (thời gian tối thiểu).

    Args:
        x_current: vị trí hiện tại, ndarray (3,)
        x_goal:    vị trí đích, ndarray (3,)
        a:         hệ số hằng số (s)
        b:         hệ số độ dốc (s)
        w:         target width (m)

    Returns:
        t_f: thời gian ước lượng (giây), luôn >= a
    """
    x_current = np.asarray(x_current, dtype=np.float64)
    x_goal    = np.asarray(x_goal,    dtype=np.float64)
    D = float(np.linalg.norm(x_current - x_goal))
    if D < 1e-6:
        return float(a)
    return float(a + b * np.log2(2.0 * D / w))


def minimum_jerk_positions(
    x_0: np.ndarray,
    x_f: np.ndarray,
    t_total: float,
    dt: float,
) -> np.ndarray:
    """Tạo quỹ đạo Minimum Jerk từ x_0 đến x_f trong t_total giây.

    Sử dụng phương trình đa thức bậc 5 chuẩn hóa (điều kiện biên: v_0=a_0=v_f=a_f=0):

        tau    = t / T                    in [0, 1]
        s(tau) = 10*tau^3 - 15*tau^4 + 6*tau^5   in [0, 1]
        x(t)   = x_0 + (x_f - x_0) * s(tau)

    Dạng chuẩn hóa tương đương toán học với hệ 6 phương trình nhưng đơn giản hơn,
    phù hợp vì chỉ điều khiển vị trí (v_0=a_0=0).

    Args:
        x_0:     vị trí bắt đầu (điểm chuyển pha FOLLOWER->LEADER), ndarray (3,)
        x_f:     vị trí đích / GOAL, ndarray (3,)
        t_total: thời gian tổng (giây) — kết quả từ fitts_law_duration()
        dt:      bước thời gian (giây) = 1/30 ~ 0.0333s (nhất quán với Kinect ~30Hz)

    Returns:
        positions: ndarray (N, 3) — N = int(t_total/dt) + 1
    """
    x_0 = np.asarray(x_0, dtype=np.float64)
    x_f = np.asarray(x_f, dtype=np.float64)
    t_total = max(float(t_total), float(dt))

    n_steps = int(t_total / dt) + 1
    tau     = np.linspace(0.0, 1.0, n_steps)
    s       = 10.0*tau**3 - 15.0*tau**4 + 6.0*tau**5
    # Broadcasting: s (N,) outer với delta_x (3,) -> (N, 3)
    positions = x_0 + np.outer(s, (x_f - x_0))
    return positions  # ndarray (N, 3)
