#!/usr/bin/env python3

import csv
import math


INPUT = "/home/zaids_laptop/f1tenth_sim/reference/levine/levine_centerline_safe.csv"
OUTPUT = "/home/zaids_laptop/f1tenth_sim/reference/levine/levine_speed_profile.csv"

MAX_SPEED = 5.0
MIN_SPEED = 2.0

MAX_LATERAL_ACCEL = 7.0
MAX_ACCEL = 4.0
MAX_BRAKE = 6.0


def curvature(p1, p2, p3):

    ax, ay = p1
    bx, by = p2
    cx, cy = p3

    ab = math.hypot(bx - ax, by - ay)
    bc = math.hypot(cx - bx, cy - by)
    ac = math.hypot(cx - ax, cy - ay)

    if ab < 1e-9 or bc < 1e-9 or ac < 1e-9:
        return 0.0

    cross = abs(
        (bx - ax) * (cy - ay)
        - (by - ay) * (cx - ax)
    )

    return 2.0 * cross / (ab * bc * ac)


# ------------------------------------------------------------
# Load path
# ------------------------------------------------------------

waypoints = []

with open(INPUT) as f:

    for line in f:

        line = line.strip()

        if not line or line.startswith("#"):
            continue

        values = line.split(";")

        waypoints.append({
            "s": float(values[0]),
            "x": float(values[1]),
            "y": float(values[2]),
            "yaw": float(values[3])
        })


n = len(waypoints)


# ------------------------------------------------------------
# Segment lengths
# ds[i] = distance from waypoint i to waypoint i+1
# ------------------------------------------------------------

ds = []

for i in range(n):

    j = (i + 1) % n

    dx = waypoints[j]["x"] - waypoints[i]["x"]
    dy = waypoints[j]["y"] - waypoints[i]["y"]

    ds.append(
        math.hypot(dx, dy)
    )


# ------------------------------------------------------------
# Curvature-limited raw speed
# ------------------------------------------------------------

curvatures = []
raw_speed = []

for i in range(n):

    p1 = (
        waypoints[(i - 1) % n]["x"],
        waypoints[(i - 1) % n]["y"]
    )

    p2 = (
        waypoints[i]["x"],
        waypoints[i]["y"]
    )

    p3 = (
        waypoints[(i + 1) % n]["x"],
        waypoints[(i + 1) % n]["y"]
    )

    kappa = curvature(
        p1,
        p2,
        p3
    )

    curvatures.append(kappa)

    if kappa < 1e-6:

        speed = MAX_SPEED

    else:

        speed = math.sqrt(
            MAX_LATERAL_ACCEL /
            kappa
        )

    speed = max(
        MIN_SPEED,
        min(
            MAX_SPEED,
            speed
        )
    )

    raw_speed.append(speed)


# ------------------------------------------------------------
# Start with curvature limit
# ------------------------------------------------------------

planned_speed = raw_speed.copy()


# ------------------------------------------------------------
# Closed-loop braking + acceleration passes
#
# Repeat several times because the track is a loop.
# ------------------------------------------------------------

for _ in range(20):

    # ----------------------------
    # Backward pass
    # Brake BEFORE slower sections
    # ----------------------------

    for i in reversed(range(n)):

        next_i = (
            i + 1
        ) % n

        maximum_speed_here = math.sqrt(
            planned_speed[next_i] ** 2 +
            2.0 *
            MAX_BRAKE *
            ds[i]
        )

        planned_speed[i] = min(
            planned_speed[i],
            maximum_speed_here,
            raw_speed[i]
        )

    # ----------------------------
    # Forward pass
    # Limit acceleration after corners
    # ----------------------------

    for i in range(n):

        next_i = (
            i + 1
        ) % n

        maximum_next_speed = math.sqrt(
            planned_speed[i] ** 2 +
            2.0 *
            MAX_ACCEL *
            ds[i]
        )

        planned_speed[next_i] = min(
            planned_speed[next_i],
            maximum_next_speed,
            raw_speed[next_i]
        )


# ------------------------------------------------------------
# Save
# ------------------------------------------------------------

with open(
    OUTPUT,
    "w",
    newline=""
) as f:

    writer = csv.writer(f)

    writer.writerow([
        "index",
        "s_m",
        "x_m",
        "y_m",
        "curvature_1pm",
        "raw_speed_mps",
        "planned_speed_mps"
    ])

    for i, wp in enumerate(waypoints):

        writer.writerow([
            i,
            f'{wp["s"]:.4f}',
            f'{wp["x"]:.4f}',
            f'{wp["y"]:.4f}',
            f'{curvatures[i]:.5f}',
            f'{raw_speed[i]:.3f}',
            f'{planned_speed[i]:.3f}'
        ])


# ------------------------------------------------------------
# Report
# ------------------------------------------------------------

max_kappa = max(curvatures)
tightest_index = curvatures.index(
    max_kappa
)

minimum_speed = min(planned_speed)
minimum_speed_index = planned_speed.index(
    minimum_speed
)

print()
print("===== BRAKING-AWARE VELOCITY PROFILE =====")

print(
    f"Waypoints:          {n}"
)

print(
    f"Maximum speed:      "
    f"{max(planned_speed):.2f} m/s"
)

print(
    f"Minimum speed:      "
    f"{minimum_speed:.2f} m/s"
)

print(
    f"Max lateral accel:  "
    f"{MAX_LATERAL_ACCEL:.2f} m/s^2"
)

print(
    f"Max acceleration:   "
    f"{MAX_ACCEL:.2f} m/s^2"
)

print(
    f"Max braking:        "
    f"{MAX_BRAKE:.2f} m/s^2"
)

print(
    f"Maximum curvature:  "
    f"{max_kappa:.3f} 1/m"
)

print(
    f"Tightest radius:    "
    f"{1.0 / max_kappa:.3f} m"
)

print(
    f"Tightest waypoint:  "
    f"{tightest_index}"
)

print(
    f"Minimum-speed WP:   "
    f"{minimum_speed_index}"
)

print()
print(
    "Speed around tightest corner:"
)

for offset in range(
    -20,
    21,
    5
):

    i = (
        tightest_index +
        offset
    ) % n

    print(
        f"WP {i:3d}: "
        f"raw={raw_speed[i]:.2f} m/s | "
        f"planned={planned_speed[i]:.2f} m/s"
    )

print()
print("Saved to:")
print(OUTPUT)
