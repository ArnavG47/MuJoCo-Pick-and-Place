import contextlib
import time
import mujoco
import mujoco.viewer
import numpy as np
import math
import imageio.v3 as iio
from numpy.typing import NDArray
from build_models import build_finger_scene, build_suction_scene, GRASP_Z, SUCTION_GRASP_Z, SUCTION_DEFAULTS, CLOSED_TIP_Z
from objects import object_info, cube_info
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
    "finger":  dict(build=lambda obj=None: build_finger_scene(obj),
                    long_axis_x=False,   # objects.object_info: narrow side faces the fingers
                    grasp_z=GRASP_Z,
                    engage={"g_driving_actuator": 0.8}, #actuator name and ctrl value
                    release={"g_driving_actuator": 0.0},
                    drop_code="SLIP_OR_DROP"),
    "suction": dict(build=lambda obj=None: build_suction_scene(obj, **SUCTION_DEFAULTS),
                    long_axis_x=True,    # long side along the cup spacing
                    grasp_z= SUCTION_GRASP_Z,
                    engage={"s_left_vacuum": 1, "s_right_vacuum": 1},
                    release={"s_left_vacuum": 0, "s_right_vacuum": 0},
                    drop_code="CUP_LOSS_OR_DROP")
}

# --- Randomisation (NEXT_STEPS item 1) ---
PERTURB = dict(
    xy = 0.003,           # m, max offset in x and in y (uniform ±). Plan: 3 mm
    yaw= 25*math.pi/180,      # rad, max extra turn about z (uniform ±). Plan: 10 deg
    mass=(0.8, 1.2),  # multiplier range (low, high). Plan: 0.8–1.2
    fric=(0.5, 1.0),  # multiplier range. Plan: 0.7–1.0
)

def perturb(model: mujoco.MjModel, data: mujoco.MjData, info: dict,
            rng: np.random.Generator) -> dict:
    """Randomise one trial in place. Call after MjData(), before mj_forward().
    Returns the values used, so they can go in the CSV."""
    bid = model.body(info["body"]).id

    # TODO 1: start index of the object's freejoint in data.qpos
    qadr = model.jnt_qposadr[model.body_jntadr[bid]]  # index of the first of 7 qpos values for the freejoint

    # TODO 2: draw dx, dy, dyaw with rng.uniform(low, high)
    dx = rng.uniform(-PERTURB["xy"], PERTURB["xy"]) 
    dy = rng.uniform(-PERTURB["xy"], PERTURB["xy"])
    dyaw = rng.uniform(-PERTURB["yaw"], PERTURB["yaw"])

    # TODO 3: shift x and y (z stays: the object must still sit on the table)
    data.qpos[qadr + 0] += dx
    data.qpos[qadr + 1] += dy
    # TODO 4: turn about world z
    #   - build q_yaw as a numpy array of 4
    #   - copy the old quat out first (np.array(...) or .copy())
    qyaw = np.array([np.cos(dyaw/2), 0, 0, np.sin(dyaw/2)])   # (w, x, y, z)
    qold = data.qpos[qadr+3:qadr+7].copy()   # (w, x, y, z)
    #   - mujoco.mju_mulQuat(<where to write>, q_yaw, old)
    mujoco.mju_mulQuat(data.qpos[qadr+3:qadr+7], qyaw, qold)

    # TODO 5: draw k_mass; scale model.body_mass[bid] and model.body_inertia[bid]
    k_mass = rng.uniform(PERTURB["mass"][0], PERTURB["mass"][1])
    model.body_mass[bid] *= k_mass
    model.body_inertia[bid] *= k_mass

    # TODO 6: draw k_fric; scale the SLIDING friction (column 0) of EVERY geom
    k_fric = rng.uniform(PERTURB["fric"][0], PERTURB["fric"][1])
    model.geom_friction[:, 0] *= k_fric

    return dict(dx=dx, dy=dy, dyaw=dyaw, k_mass=k_mass, k_fric=k_fric)

def grasp_point(tool_name: str, info: dict) -> NDArray:
    """World position (m) of the tool's grasp site at the moment of grasping.

    info["top_z"] is the world z of the object's highest point (cube: 0.45).
    The object's box centre is at world x = y = 0.
    Your old values for the cube: finger 0.44, suction 0.448.
    """
    # TODO (you): suction -> cup lips just pressed into the top face (adhesion needs contact)
    if tool_name == "suction": return np.array([0, 0, info["top_z"]-0.002])
    # TODO (you): finger  -> pads below the top, deep enough to hold tall objects, but not so deep
    #             the pads hit the table on short ones (info["height"])
    depth = min(0.035, info["height"] - (CLOSED_TIP_Z - GRASP_Z) - 0.005)    # margin ≈ 0.005
    return np.array([0, 0, info["top_z"] - depth])   # default for the cube and most objects


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


VIDEO_FPS  = 20
VIDEO_SIZE = (360, 480)   # (height, width) px; must fit the default 640x480 offscreen buffer


def make_camera() -> mujoco.MjvCamera:
    """Fixed 3/4 view of the pick spot (x=0) and the target (x=0.3)."""
    cam = mujoco.MjvCamera()
    cam.lookat[:] = [0.15, 0.0, 0.56]
    cam.distance, cam.azimuth, cam.elevation = 1.05, 135, -18
    return cam


def run_trial(tool_name: str, obj: str | None = None, show: bool = False,
              record: str | None = None, verbose: bool = True, hold: bool = True, seed: int | None = None) -> dict | None:
    """Run one pick-and-place trial. Returns a result row (dict), or None if the viewer was closed mid-trial.

    tool_name: key in TOOLS. obj: key in objects.OBJECTS, or None for the cube.
    show: open the passive viewer and run in real time. record: path to save a GIF, or None.
    hold: with show, keep the window open after the trial until you close it (False = close it automatically).
    """
    tool = TOOLS[tool_name]
    info = cube_info() if obj is None else object_info(obj, tool["long_axis_x"])
    SCHEDULE = build_schedule(make_phases(grasp_point(tool_name, info)), START)


    model: mujoco.MjModel = tool["build"](obj).compile()
    data = mujoco.MjData(model)
    if seed is not None: 
        rng = np.random.default_rng(seed)
        pert_vals = perturb(model, data, info, rng)

    mid = model.body("carrier").mocapid[0]
    T_END = SCHEDULE[-1][2] + 1.0   # end of RETRACT + 1 s
    n_steps = int(round(T_END/model.opt.timestep))

    prev_name = None
    mujoco.mj_forward(model, data) #needed to evaluate the current position and remove the 0's initialization
    cube = data.body(info["body"])     # the object being picked (named cube from Lesson 1)
    grasp_site = data.site("grasp")
    target_pos = data.site("target").xpos.copy()
    rel0 = R0 = z0 = None
    lift, max_slip, max_rot = 0.0, 0.0, 0.0

    renderer, cam, frames = None, None, []
    frame_every = int(round(1 / (VIDEO_FPS * model.opt.timestep)))   # steps between video frames
    if record:
        renderer = mujoco.Renderer(model, *VIDEO_SIZE)
        cam = make_camera()

    closed_early = False
    viewer_ctx = mujoco.viewer.launch_passive(model, data) if show else contextlib.nullcontext()
    with viewer_ctx as viewer:
        for step in range(0, n_steps):
            if show and not viewer.is_running():
                closed_early = True   # trial didn't finish, so there is no valid result
                break
            step_start = time.time()

            name, pos = phase_at(step_to_time(step, model.opt.timestep), SCHEDULE)
            data.mocap_pos[mid] = pos + np.array([0, 0, tool["grasp_z"]])
            if name != prev_name:
                if verbose:
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

            if renderer is not None and step % frame_every == 0:
                renderer.update_scene(data, camera=cam)
                frames.append(renderer.render())

            if show:
                viewer.sync()
                remaining = model.opt.timestep - (time.time() - step_start)
                if remaining > 0:
                    time.sleep(remaining)

        if not closed_early:
            code, fails = classify(lift, max_slip, max_rot, cube.xpos, target_pos, z0, tool["drop_code"])
            if verbose:
                print("PASS" if code is None else f"FAIL: {code} \nAll fails: {', '.join(fails)}",
                      f"\nlift={lift*1000:.0f}mm slip={max_slip*1000:.1f}mm rot={np.degrees(max_rot):.1f}deg")

        while show and hold and viewer.is_running():
            viewer.sync()
            time.sleep(0.01)

    if renderer is not None:
        renderer.close()
        if not closed_early:
            iio.imwrite(record, frames, duration=1000 / VIDEO_FPS, loop=0)   # GIF, duration in ms per frame
    if closed_early:
        return None

    row = dict(tool=tool_name, object=info["key"], passed=code is None,
                primary=code or "", all_fails=";".join(fails),
                lift_mm=round(lift * 1000, 1), slip_mm=round(max_slip * 1000, 1),
                rot_deg=round(float(np.degrees(max_rot)), 1),
                place_err_mm=round(float(np.linalg.norm(cube.xpos[:2] - target_pos[:2])) * 1000, 1),
                height_mm=round(info["height"] * 1000), width_x_mm=round(info["width_x"] * 1000),
                mass_kg=info["mass"])
    if seed is not None:
        row.update(seed=seed, 
                   dx_mm = round(pert_vals["dx"]*1000, 2), 
                   dy_mm = round(pert_vals["dy"]*1000, 2), 
                   dyaw_deg = np.degrees(pert_vals["dyaw"]),
                   k_mass = pert_vals["k_mass"], 
                   k_fric = pert_vals["k_fric"])
    return row


if __name__ == "__main__":
    # python src/trial_runner.py finger|suction [object_key]   (no object = the cube)
    tool_name = sys.argv[1] if len(sys.argv) > 1 else "finger"
    obj = sys.argv[2] if len(sys.argv) > 2 else None
    run_trial(tool_name, obj, show=True)

