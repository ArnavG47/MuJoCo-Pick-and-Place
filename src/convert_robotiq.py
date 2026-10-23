import mujoco
from pathlib import Path

URDF = Path("assets/grippers/work/robotiq_description/robotiq_2f85.urdf")
OUT = Path("models/converted/robotiq_2f85.xml")

spec = mujoco.MjSpec.from_file(str(URDF))

# TODO 1: turn off fusestatic (hint: spec.compiler has an attribute for it)

spec.compiler.fusestatic = False
model = spec.compile()

# TODO 2: print the counts: model.nbody, model.njnt, model.neq, model.ngeom, model.nmesh
bodies = model.nbody
print("bodies: ", bodies, "\n joints: ", model.njnt, "\n eq: ", model.neq, "\n num geometries:",
      model.ngeom, "\n meshes: ", model.nmesh)
# TODO 3: loop over joints; print each name and range
#   hint: model.joint(i).name, model.jnt_range[i]

# TODO 4: loop over bodies; print each name and mass
#   hint: model.body(i).name, model.body_mass[i]
total_mass = 0
for i in range(0,bodies):
    total_mass += model.body_mass[i]

print("total mass is: ", total_mass)
# TODO 5: if model.neq > 0, print what each equality constraint links
#   hint: model.eq_obj1id[i], model.eq_obj2id[i] are joint ids -> model.joint(id).name

# TODO 6: save with spec.to_file(...)
spec.to_file(str(OUT))
