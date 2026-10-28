import time

import mujoco
import mujoco.viewer
import numpy as np
from build_models import build_finger_scene, build_suction_scene, GRASP_Z, SUCTION_GRASP_Z, SUCTION_DEFAULTS
import sys

START = np.array([0.0, 0.0, 0.7])
PREGRASP = np.array([0.0, 0.0, 0.525])   # 100 mm above grasp (Guide 6.4)
GRASP = np.array([0.0, 0.0, 0.44])      # cube centre
LIFTED = np.array([0.0, 0.0, 0.575])     # lift 150 mm
ABOVE_TARGET = np.array([0.3, 0.0, 0.575])  # translate 300 mm in x
AT_TARGET = np.array([0.3, 0.0, 0.425])  # back down to target surface
RETRACTED = np.array([0.3, 0.0, 0.525])  # 100 mm up

def make_phases(grasp):
    up_pre  = np.array([0, 0, 0.085])
    up_lift = np.array([0, 0, 0.135])
    over    = np.array([0.3, 0, 0])
    PREGRASP     = grasp + up_pre # 100 mm above grasp 
    LIFTED       = grasp + up_lift # lift 150mm
    ABOVE_TARGET = LIFTED + over
    AT_TARGET    = grasp + over 
    RETRACTED    = grasp+up_pre + over 

    # (name, duration_s, end_position)
    return [ 
        ("SETTLE_OBJECT",    0.5,  START),
        ("MOVE_TO_PREGRASP", 0.5,  PREGRASP),
        ("APPROACH",         1.0,  grasp),         # 100 mm at 100 mm/s
        ("ENGAGE",           0.25, grasp),
        ("HOLD_CONTACT",     0.25, grasp),
        ("LIFT",             1.0,  LIFTED),
        ("TRANSLATE",        1.0,  ABOVE_TARGET),
        ("HOLD_AT_TARGET",   1.0,  ABOVE_TARGET),
        ("LOWER",            1.0,  AT_TARGET),
        ("RELEASE",          0.25, AT_TARGET),
        ("RETRACT",          0.5,  RETRACTED),
    ]   # same 11 rows as now

def step_to_time(step, dt): return step * dt

TOOLS = {
    "finger":  dict(build=lambda: build_finger_scene(),
                    grasp_z=GRASP_Z,
                    grasp=np.array([0, 0, 0.445]),
                    engage={"g_driving_actuator": 0.8}, #actuator name and ctrl value
                    release={"g_driving_actuator": 0.0}),
    "suction": dict(build=lambda: build_suction_scene(**SUCTION_DEFAULTS),# TODO build_suction_scene with SUCTION_DEFAULTS,
                    grasp_z= SUCTION_GRASP_Z,
                    grasp=np.array([0, 0, 0.448]),
                    engage={"s_left_vacuum": 1, "s_right_vacuum": 1},# TODO both vacuum actuators → 1,
                    release={"s_left_vacuum": 0, "s_right_vacuum": 0})
}


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



def phase_at(t, schedule):
    """Return (phase_name, carrier_position) at time t."""
    for name, t0, t1, p0, p1 in schedule:
        if t0 <= t < t1:
            s = (t - t0) / (t1 - t0)
            return (name, p0 + s * (p1 - p0))
    #if time is past the last scheduled action then remain at final position in schedule
    return ('DONE', schedule[-1][4])


if __name__ == "__main__":
    tool = TOOLS[sys.argv[1] if len(sys.argv) > 1 else "finger"]

    SCHEDULE = build_schedule(make_phases(tool["grasp"]), START)
    for t in [0.0, 0.75, 1.5, 2.1, 7.0, 8.0]:
        print(t, phase_at(t, SCHEDULE))

    model: mujoco.MjModel = tool["build"]().compile()
    data = mujoco.MjData(model)


    mid = model.body("carrier").mocapid[0]
    T_END = SCHEDULE[-1][2] + 1.0   # end of RETRACT + 1 s
    n_steps = int(round(T_END/model.opt.timestep))

    prev_name = None
    with mujoco.viewer.launch_passive(model, data) as viewer:
        for step in range(0, n_steps):
            if not viewer.is_running():
                break
            step_start = time.time()

            name, pos = phase_at(step_to_time(step, model.opt.timestep), SCHEDULE)
            data.mocap_pos[mid] = pos + np.array([0, 0, tool["grasp_z"]])
            if name != prev_name:
                print(f"{step_to_time(step, model.opt.timestep):.5f}s  enter {name}")
                # later: if name == "ENGAGE": close gripper / turn on suction
                if name == "ENGAGE": 
                    for act_name, u in tool["engage"].items():
                        data.ctrl[model.actuator(act_name).id] = u
                if name == "RELEASE":
                    for act_name, u in tool["release"].items():
                        data.ctrl[model.actuator(act_name).id] = u

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

