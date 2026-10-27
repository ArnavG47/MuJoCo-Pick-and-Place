import mujoco
from pathlib import Path

URDF = Path("assets/grippers/work/robotiq_description/robotiq_2f85.urdf")
OUT = Path("models/converted/robotiq_2f85.xml")

spec = mujoco.MjSpec.from_file(str(URDF))

# TODO 1: turn off fusestatic (hint: spec.compiler has an attribute for it)

spec.compiler.fusestatic = False

# Self-collision off: geoms collide if (A.contype & B.conaffinity) or (B.contype & A.conaffinity).
# Gripper: contype=2, conaffinity=1. World objects use the default 1/1.
#   gripper vs gripper: (2 & 1) or (2 & 1) = 0  -> no contact
#   gripper vs cube:    (2 & 1) or (1 & 1) = 1  -> contact
GRIPPER_CONTYPE = 2
GRIPPER_CONAFFINITY = 1
for g in spec.geoms:
    g.contype = GRIPPER_CONTYPE
    g.conaffinity = GRIPPER_CONAFFINITY

DAMPING = [0.05, 0, 0]
ARMATURE = 0.001
for j in spec.joints:
    j.damping = DAMPING
    j.armature = ARMATURE

#add finger actuator to driving joint
DRIVER_JOINT = "robotiq_85_left_knuckle_joint"
KP= 5
FORCE_LIMIT = 15

actuator = spec.add_actuator(
    name="driving_actuator",
    target= DRIVER_JOINT,
    trntype=mujoco.mjtTrn.mjTRN_JOINT,
    ctrlrange=[0, 0.8],
    forcerange=[-FORCE_LIMIT, FORCE_LIMIT]
)
actuator.set_to_position(kp=KP)

model: mujoco.MjModel = spec.compile()

# TODO 2: print the counts: model.nbody, model.njnt, model.neq, model.ngeom, model.nmesh
bodies = model.nbody
print("bodies: ", bodies, "\n joints: ", model.njnt, "\n eq: ", model.neq, "\n num geometries:",
      model.ngeom, "\n meshes: ", model.nmesh)
# Joint names and limits (rad). Mirrored joints have negative ranges.
print("\njoints:")
for i in range(model.njnt):
    lo, hi = model.jnt_range[i]
    print(f"  {i}: {model.joint(i).name:40s} range [{lo:+.3f}, {hi:+.3f}]")

joint_id = model.actuator("driving_actuator").trnid[0]
print("\n actuators:", model.nu, "\ndriving joint name:", model.joint(joint_id).name)

# TODO 4: loop over bodies; print each name and mass
#   hint: model.body(i).name, model.body_mass[i]
total_mass = 0
for i in range(0,bodies):
    total_mass += model.body_mass[i]

print("\ntotal mass is: ", total_mass)
# Couplings from URDF <mimic>: follower = c0 + c1 * leader (eq_data[:2]).
print("\nequalities:")
for i in range(model.neq):
    eq_type = mujoco.mjtEq(model.eq_type[i]).name
    follower = model.joint(model.eq_obj1id[i]).name
    leader = model.joint(model.eq_obj2id[i]).name
    c0, c1 = model.eq_data[i][:2]
    print(f"  {eq_type}: {follower} = {c0:+.3f} {c1:+.3f} * {leader}")

# TODO 6: save with spec.to_file(...)
spec.to_file(str(OUT))
