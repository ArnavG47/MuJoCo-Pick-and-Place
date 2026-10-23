import mujoco
import numpy as np

WAYPOINTS = [
    (0.0, np.array([0.0, 0.0, 0.7])),     # start
    (0.5, np.array([0, 0, 0.525])), # TODO: pregrasp   - you pick the time
    (1.5, np.array([0, 0, 0.425])), # TODO: grasp    ()  - 100 mm at 100 mm/s, so +1.0 s after pregrasp
    (1.75, np.array([0, 0, 0.425])), # TODO: hold     same position   - +0.25 s
    (2.75, np.array([0, 0, 0.575])),# TODO: lifted   (0, 0, 0.575)  - you pick the duration
    (3.75, np.array([0.3, 0, 0.575])) # TODO: above target (0.3, 0, 0.575)
]

def carrier_target(t):
    for (t0, p0), (t1, p1) in zip(WAYPOINTS, WAYPOINTS[1:]):
        if t0 <= t < t1:
            s = (t - t0) / (t1 - t0)
            return p0 + s * (p1 - p0)
    # past the last waypoint: stay there
    return WAYPOINTS[-1][1]


for t in [0.0, 0.5, 1, 3]:
    print("time: ", t, " target: ", carrier_target(t))


model = mujoco.MjModel.from_xml_path("models/table.xml")
data = mujoco.MjData(model)

mid = model.body("carrier").mocapid[0]
T_END = WAYPOINTS[-1][0] +  1.0
n_steps = int(round(T_END/model.opt.timestep))

for step in range(1,n_steps):
    data.mocap_pos[mid] = carrier_target(data.time) #data.time is the current simulation time
    mujoco.mj_step(model, data) #increments simulation time for the loop
    if(step%(0.5/model.opt.timestep) == 0):
        print("time is ", round(data.time, 3), "; position is ", data.site("flange").xpos)