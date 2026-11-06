import time
import mujoco
import mujoco.viewer
import numpy as np
from numpy.typing import NDArray
from build_models import build_finger_scene, build_suction_scene, GRASP_Z, SUCTION_GRASP_Z, SUCTION_DEFAULTS
import sys

START = np.array([0.0, 0.0, 0.7])

LIFT_HEIGHT = 0.135            # m, commanded lift
LIFT_FRAC   = 0.95             # clear by ≥ 95% of commanded lift
SLIP_MAX    = 0.010            # m, Guide 11.3
ROT_MAX     = np.radians(15)   # rad, Guide 11.3
PLACE_TOL   = 0.010            # m, tolerance for final cube position
TRANSPORT   = {"LIFT", "TRANSLATE", "HOLD_AT_TARGET", "LOWER"}

def make_phases(grasp):
    up_pre  = np.array([0, 0, 0.085])
    up_lift = np.array([0, 0, LIFT_HEIGHT])
    over    = np.array([0.3, 0, 0])
    PREGRASP     = grasp + up_pre # 85 mm above grasp
    LIFTED       = grasp + up_lift # lift 135 mm
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

##HELPER FUNCTIONS
def step_to_time(step, dt): return step * dt

def rotation_angle(R0: np.array, R: np.array):
    """Angle (rad) of the rotation that takes R0 to R. Both 3x3 numpy arrays."""
    R_rel = np.transpose(R0) @ R
    cos_a = np.clip((np.trace(R_rel) - 1)/2, -1, 1)
    return np.arccos(cos_a)

def classify(lift, max_slip, max_rot, final_pos, target_pos, z0, drop_code):
    """Return None if the trial passed, else the first failure code (in time order)."""
    fails = []
    if lift == 0: fails.append("NO_LIFT")
    if lift < LIFT_HEIGHT*LIFT_FRAC: fails.append("INSUFFICIENT_LIFT")
    if max_slip > SLIP_MAX: fails.append(drop_code)
    if max_rot > ROT_MAX: fails.append("EXCESS_ROTATION")
    # xy on the target, and back down at pick height (not stuck to the tool)
    if np.linalg.norm(final_pos[:2] - target_pos[:2]) > PLACE_TOL or abs(final_pos[2] - z0) > PLACE_TOL:
        fails.append("FAILED_RELEASE")
    primary = fails[0] if fails else None
    return primary, fails


TOOLS = {
    "finger":  dict(build=lambda: build_finger_scene(),
                    grasp_z=GRASP_Z,
                    grasp=np.array([0, 0, 0.44]), # Target pad center location relative to world at the moment of grasping
                    engage={"g_driving_actuator": 0.8}, #actuator name and ctrl value
                    release={"g_driving_actuator": 0.0},
                    drop_code="SLIP_OR_DROP"),
    "suction": dict(build=lambda: build_suction_scene(**SUCTION_DEFAULTS),# TODO build_suction_scene with SUCTION_DEFAULTS,
                    grasp_z= SUCTION_GRASP_Z,
                    grasp=np.array([0, 0, 0.448]), # Target cup lip plane location relative to world at the moment of grasping
                    engage={"s_left_vacuum": 1, "s_right_vacuum": 1},# TODO both vacuum actuators → 1,
                    release={"s_left_vacuum": 0, "s_right_vacuum": 0},
                    drop_code="CUP_LOSS_OR_DROP")
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
    mujoco.mj_forward(model, data) #needed to evaluate the current position and remove the 0's initialization
    cube = data.body("cube")
    grasp_site = data.site("grasp")
    target_pos = data.site("target").xpos.copy()
    rel0 = R0 = z0 = None
    lift, max_slip, max_rot = 0.0, 0.0, 0.0

    with mujoco.viewer.launch_passive(model, data) as viewer:
        for step in range(0, n_steps):
            if not viewer.is_running():
                break
            step_start = time.time()

            name, pos = phase_at(step_to_time(step, model.opt.timestep), SCHEDULE)
            data.mocap_pos[mid] = pos + np.array([0, 0, tool["grasp_z"]])
            if name != prev_name:
                print(f"{step_to_time(step, model.opt.timestep):.5f}s  enter {name}")
                # reference pose: tool has held the cube for HOLD_CONTACT, cube still on the table
                if name == "LIFT":
                    rel0 = (cube.xpos - grasp_site.xpos).copy()
                    R0 = cube.xmat.reshape(3, 3).copy()
                    z0 = cube.xpos[2]
                if name == "TRANSLATE":
                    lift = cube.xpos[2] - z0
                # we want to actuators for each EOAT to close once at engage
                if name == "ENGAGE": 
                    for act_name, u in tool["engage"].items(): data.ctrl[model.actuator(act_name).id] = u
                if name == "RELEASE":
                    for act_name, u in tool["release"].items(): data.ctrl[model.actuator(act_name).id] = u
                prev_name = name

            #simulation step is incremented here
            mujoco.mj_step(model, data)

            # slip = change in the cube's offset from the tool; rotation relative to the reference pose
            if name in TRANSPORT:
                slip = np.linalg.norm((cube.xpos - grasp_site.xpos) - rel0)
                max_slip = max(max_slip, slip)
                max_rot = max(max_rot, rotation_angle(R0, cube.xmat.reshape(3, 3)))

            viewer.sync()
            elapsed = time.time() - step_start
            remaining = model.opt.timestep - elapsed
            if remaining > 0:
                time.sleep(remaining)

        code, fails = classify(lift, max_slip, max_rot, cube.xpos, target_pos, z0, tool["drop_code"])
        print("PASS" if code is None else f"FAIL: {code} \nAll fails: {', '.join(fails)}",
              f"\nlift={lift*1000:.0f}mm slip={max_slip*1000:.1f}mm rot={np.degrees(max_rot):.1f}deg")

        while viewer.is_running():
            viewer.sync()
            time.sleep(0.01)

