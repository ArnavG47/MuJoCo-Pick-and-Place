import numpy as np

START = np.array([0.0, 0.0, 0.7])
PREGRASP = np.array([0.0, 0.0, 0.525])   # 100 mm above grasp (Guide 6.4)
GRASP = np.array([0.0, 0.0, 0.425])      # cube centre
LIFTED = np.array([0.0, 0.0, 0.575])     # lift 150 mm
ABOVE_TARGET = np.array([0.3, 0.0, 0.575])  # translate 300 mm in x
AT_TARGET = np.array([0.3, 0.0, 0.425])  # back down to target surface
RETRACTED = np.array([0.3, 0.0, 0.525])  # 100 mm up

# (name, duration_s, end_position)
PHASES = [
    ("SETTLE_OBJECT",    0.5,  START),
    ("MOVE_TO_PREGRASP", 0.5,  PREGRASP),
    ("APPROACH",         1.0,  GRASP),         # 100 mm at 100 mm/s
    ("ENGAGE",           0.25, GRASP),
    ("HOLD_CONTACT",     0.25, GRASP),
    ("LIFT",             1.0,  LIFTED),
    ("TRANSLATE",        1.0,  ABOVE_TARGET),
    ("HOLD_AT_TARGET",   1.0,  ABOVE_TARGET),
    ("LOWER",            1.0,  AT_TARGET),
    ("RELEASE",          0.25, AT_TARGET),
    ("RETRACT",          0.5,  RETRACTED),
]

def build_schedule(phases, start):
    """Turn durations into absolute times: list of (name, t0, t1, p0, p1)."""
    schedule = []
    t = 0.0
    p = start
    # schedule.append("start", t, t+2, p, p)
    # t += 2;
    for name, duration, end_pos in phases:
        schedule.append((name, t, t+duration, p, end_pos))
        t += duration
        p = end_pos
    return schedule


SCHEDULE = build_schedule(PHASES, START)

#testing schedule
for row in SCHEDULE:
    print(row[0], row[1], row[2])

def phase_at(t):
    """Return (phase_name, carrier_position) at time t."""
    for name, t0, t1, p0, p1 in SCHEDULE:
        if t0 <= t < t1:
            s = (t - t0) / (t1 - t0)
            return (name, p0 + s * (p1 - p0))
    #if time is past the last scheduled action then remain at final position in schedule
    return ('DONE', SCHEDULE[-1][4])


for t in [0.0, 0.75, 1.5, 2.1, 7.0, 8.0]:
    print(t, phase_at(t))

