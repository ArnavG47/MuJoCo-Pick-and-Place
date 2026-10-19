import mujoco

model = mujoco.MjModel.from_xml_path("models/table.xml")
data = mujoco.MjData(model)

print("bodies:", model.nbody, "geoms:", model.ngeom, "sites:", model.nsite)

# TODO 1: compute positions without moving time, then print the target site position
#have to compute the next step of simulation
mujoco.mj_forward(model, data)

pos_cube = data.body("cube").xpos
pos_target = data.site("target").xpos
print("cube position:", pos_cube, "target position:", pos_target)


# TODO 2: how many steps is 2 seconds? (hint: timestep is in model.opt.timestep)
n_steps = round(int(2/model.opt.timestep))

for _ in range(n_steps):
    mujoco.mj_step(model, data)

print("time:", round(data.time))
print("cube centre:", int(data.body("cube").xpos[0]), int(data.body("cube").xpos[1]), int(data.body("cube").xpos[2]))

# TODO 3: reset, step 2 s again, and print the cube centre.
mujoco.mj_resetData(model, data)

for _ in range(n_steps):
    mujoco.mj_step(model, data)
print("cube centre iter2:", int(data.body("cube").xpos[0]), int(data.body("cube").xpos[1]), int(data.body("cube").xpos[2]))