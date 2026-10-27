import time

import mujoco
import mujoco.viewer
import numpy as np
from build_models import build_finger_scene, GRASP_Z

START = np.array([0.0, 0.0, 0.7])
PREGRASP = np.array([0.0, 0.0, 0.525])   # 100 mm above grasp (Guide 6.4)
GRASP = np.array([0.0, 0.0, 0.44])      # cube centre
LIFTED = np.array([0.0, 0.0, 0.575])     # lift 150 mm
ABOVE_TARGET = np.array([0.3, 0.0, 0.575])  # translate 300 mm in x
AT_TARGET = np.array([0.3, 0.0, 0.425])  # back down to target surface
RETRACTED = np.array([0.3, 0.0, 0.525])  # 100 mm up
GRASP_OFFSET = np.array([0, 0, GRASP_Z])
CLOSE, OPEN = 0.8, 0

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

def step_to_time(step, dt): return step * dt

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

def phase_at(t):
    """Return (phase_name, carrier_position) at time t."""
    for name, t0, t1, p0, p1 in SCHEDULE:
        if t0 <= t < t1:
            s = (t - t0) / (t1 - t0)
            return (name, p0 + s * (p1 - p0))
    #if time is past the last scheduled action then remain at final position in schedule
    return ('DONE', SCHEDULE[-1][4])


if __name__ == "__main__":
    #testing schedule
    for row in SCHEDULE:
        print(row[0], row[1], row[2])

    for t in [0.0, 0.75, 1.5, 2.1, 7.0, 8.0]:
        print(t, phase_at(t))

    model: mujoco.MjModel = build_finger_scene().compile()
    data = mujoco.MjData(model)

    mid = model.body("carrier").mocapid[0]
    aid = model.actuator("g_driving_actuator").id
    T_END = SCHEDULE[-1][2] + 1.0   # end of RETRACT + 1 s
    n_steps = int(round(T_END/model.opt.timestep))

    prev_name = None
    with mujoco.viewer.launch_passive(model, data) as viewer:
        for step in range(0, n_steps):
            if not viewer.is_running():
                break
            step_start = time.time()

            name, pos = phase_at(step_to_time(step, model.opt.timestep))
            data.mocap_pos[mid] = pos + GRASP_OFFSET
            if name != prev_name:
                print(f"{step_to_time(step, model.opt.timestep):.5f}s  enter {name}")
                # later: if name == "ENGAGE": close gripper / turn on suction
                if name == "ENGAGE": data.ctrl[aid] = CLOSE
                if name == "RELEASE": data.ctrl[aid] = OPEN
                prev_name = name

            #simulation step is incremented here
            mujoco.mj_step(model, data)

            viewer.sync()
            elapsed = time.time() - step_start
            remaining = model.opt.timestep - elapsed
            if remaining > 0:
                time.sleep(remaining)

        print("cube position:", data.body("cube").xpos)

        while viewer.is_running():
            viewer.sync()
            time.sleep(0.01)

