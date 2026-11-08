"""Print a results table: passes, pass rate, 95% confidence interval, most common failure.

python src/summarize.py results/robust.csv

The confidence interval uses the Wilson method. Example: 20 passes out of 20 gives 84-100%,
so 20/20 only shows the pass rate is at least about 84%.
"""
import sys

import pandas as pd
from scipy.stats import binomtest

csv_path = sys.argv[1] if len(sys.argv) > 1 else "results/robust.csv"
df = pd.read_csv(csv_path)

# Markdown table, so it can be pasted straight into the README
print("| object | tool | passed | rate | 95% CI | most common failure |")
print("|---|---|---|---|---|---|")

for (obj, tool), trials in df.groupby(["object", "tool"]):
    n = len(trials)                    # number of trials
    k = int(trials["passed"].sum())    # number that passed (True counts as 1)

    ci = binomtest(k, n).proportion_ci(method="wilson")
    low = round(ci.low * 100)
    high = round(ci.high * 100)
    rate = round(k / n * 100)

    failed = trials[trials["passed"] == False]
    if len(failed) > 0:
        counts = failed["primary"].value_counts()    # failure code -> how many times
        top_failure = f"{counts.index[0]} ({counts.iloc[0]})"
    else:
        top_failure = ""

    print(f"| {obj} | {tool} | {k}/{n} | {rate}% | {low}-{high}% | {top_failure} |")
