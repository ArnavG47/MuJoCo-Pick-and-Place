import mujoco

model = mujoco.MjModel.from_xml_path("models/table.xml")
data = mujoco.MjData(model)

print("bodies:", model.nbody, "geoms:", model.ngeom, "sites:", model.nsite)

# TODO 1: compute positions without moving time, then print the target site position
mujoco.mj_forward(model, data)
pos_cube = data.body("cube").xpos.copy()

pos_target = data.site("target").xpos.copy()
print("cube position:", pos_cube, "target position:", pos_target)


# TODO 2: how many steps is 2 seconds? (hint: timestep is in model.opt.timestep)
n_steps = int(round(2/model.opt.timestep))

for _ in range(n_steps):
    mujoco.mj_step(model, data)

print("time:", round(data.time, 4))
cube_center_1 = data.body("cube").xpos.copy()
print("cube centre:", cube_center_1)

# TODO 3: reset, step 2 s again, and print the cube centre.
mujoco.mj_resetData(model, data)

for _ in range(n_steps):
    mujoco.mj_step(model, data)
cube_center_2 = data.body("cube").xpos.copy()
print("cube centre iter2:", cube_center_2)

if round(cube_center_1[2], 3) == round(cube_center_2[2], 3):
    print("z positions match: pass")
else:
    print("z positions don't match: fail")
