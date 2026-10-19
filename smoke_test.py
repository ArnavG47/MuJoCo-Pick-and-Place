"""Smoke test for the MuJoCo end-effector trade study (Implementation Guide 3.4).

Usage:
    python smoke_test.py            # headless checks only
    python smoke_test.py --viewer   # also open the interactive viewer (needs a display)
"""
import os
import sys
import time

# Offscreen rendering backend on Linux. Set BEFORE importing mujoco.
# Use "egl" (GPU / headless) by default. If it fails, try: MUJOCO_GL=osmesa
os.environ.setdefault("MUJOCO_GL", "egl")

import mujoco
import numpy as np
import pandas as pd
import optuna
import imageio.v3 as iio

XML = """
<mujoco>
  <option timestep="0.001"/>
  <worldbody>
    <light pos="0 0 2"/>
    <geom name="floor" type="plane" size="1 1 0.1"/>
    <body name="cube" pos="0 0 0.5">
      <freejoint/>
      <geom type="box" size="0.025 0.025 0.025" mass="0.1" rgba="0.8 0.3 0.3 1"/>
    </body>
  </worldbody>
</mujoco>
"""

results = []


def check(name, fn):
    try:
        msg = fn()
        print(f"[PASS] {name}" + (f" - {msg}" if msg else ""))
        results.append((name, True))
    except Exception as e:
        print(f"[FAIL] {name} - {type(e).__name__}: {e}")
        results.append((name, False))


model = mujoco.MjModel.from_xml_string(XML)
data = mujoco.MjData(model)


def t_load():
    return f"MuJoCo {mujoco.__version__}, {model.nbody} bodies"


def t_viewer():
    if "--viewer" not in sys.argv:
        return "skipped (run with --viewer to test)"
    import mujoco.viewer
    with mujoco.viewer.launch_passive(model, data) as v:
        t0 = time.time()
        while v.is_running() and time.time() - t0 < 3:
            mujoco.mj_step(model, data)
            v.sync()
            time.sleep(model.opt.timestep)
    return "viewer opened for 3 s"


def t_headless():
    mujoco.mj_resetData(model, data)
    t0 = time.time()
    for _ in range(10_000):
        mujoco.mj_step(model, data)
    z = data.qpos[2]
    assert abs(z - 0.025) < 0.005, f"cube did not settle on floor, z={z:.4f}"
    return f"10,000 steps in {time.time() - t0:.2f} s, cube z={z:.4f} m"


def t_render():
    with mujoco.Renderer(model, height=240, width=320) as r:
        r.update_scene(data)
        img = r.render()
    iio.imwrite("smoke_frame.png", img)
    return f"saved smoke_frame.png {img.shape}"


def t_parquet():
    df = pd.DataFrame({"trial_id": np.arange(5), "success": [True, False, True, True, False]})
    df.to_parquet("smoke.parquet")
    back = pd.read_parquet("smoke.parquet")
    assert back.equals(df)
    return "wrote and reloaded smoke.parquet"


def t_optuna():
    optuna.logging.set_verbosity(optuna.logging.WARNING)
    study = optuna.create_study(directions=["minimize", "minimize"])
    study.optimize(lambda t: (t.suggest_float("x", 0, 1), t.suggest_float("y", 0, 1)), n_trials=1)
    assert len(study.trials) == 1
    return "1 dummy trial done"


check("1. Load model", t_load)
check("1b. Viewer", t_viewer)
check("2. Headless 10,000 steps", t_headless)
check("3. Offscreen frame", t_render)
check("4. Parquet write/read", t_parquet)
check("5. Optuna dummy trial", t_optuna)

failed = [n for n, ok in results if not ok]
print("\nALL PASSED" if not failed else f"\nFAILED: {failed}")
sys.exit(1 if failed else 0)
