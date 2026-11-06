"""Run every tool on every object and save one result row per trial. Headless by default.

python src/run_batch.py                 # all objects, both tools -> results/smoke.csv
python src/run_batch.py --record        # also save a GIF per trial in results/videos/
python src/run_batch.py --show          # watch each trial in the viewer (real time, ~9 s per trial)
python src/run_batch.py --objects cube ink_box --tools suction

With --show, each trial opens its own window and closes it when done. Closing a window
yourself mid-trial stops the batch; the trials finished so far are still saved.
"""
import argparse
import os
import time

import pandas as pd

from objects import OBJECTS
from trial_runner import TOOLS, run_trial

OUT_CSV   = "results/smoke.csv"
VIDEO_DIR = "results/videos"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--objects", nargs="+", default=["cube", *OBJECTS], help="object keys ('cube' = the Lesson 1 cube)")
    ap.add_argument("--tools", nargs="+", default=list(TOOLS))
    ap.add_argument("--record", action="store_true", help="save a GIF per trial")
    ap.add_argument("--show", action="store_true", help="watch each trial in the viewer (real time)")
    ap.add_argument("--out", default=OUT_CSV)
    args = ap.parse_args()

    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    if args.record:
        os.makedirs(VIDEO_DIR, exist_ok=True)

    trials = [(obj, tool) for obj in args.objects for tool in args.tools]
    rows = []
    for obj, tool in trials:
        video = os.path.join(VIDEO_DIR, f"{tool}_{obj}.gif") if args.record else None
        t0 = time.time()
        row = run_trial(tool, None if obj == "cube" else obj, show=args.show, hold=False,
                        record=video, verbose=False)
        if row is None:   # viewer window closed mid-trial
            print(f"{obj:12s} {tool:8s} viewer closed - stopping batch")
            break
        rows.append(row)
        status = "PASS" if row["passed"] else f"FAIL {row['all_fails']}"
        print(f"{obj:12s} {tool:8s} {status:40s} lift={row['lift_mm']:6.1f}mm "
              f"slip={row['slip_mm']:5.1f}mm rot={row['rot_deg']:5.1f}deg  ({time.time() - t0:.1f}s)")

    if not rows:
        print("no trials finished, nothing saved")
        return
    df = pd.DataFrame(rows)
    df.to_csv(args.out, index=False)
    print(f"\nsaved {len(df)} trials -> {args.out}")
    print(df.pivot(index="object", columns="tool", values="passed").to_string())
    print("pass rate:", df.groupby("tool")["passed"].mean().round(2).to_dict())


if __name__ == "__main__":
    main()
