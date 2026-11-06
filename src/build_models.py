import mujoco
from objects import add_object

SCENE_XML = "models/table.xml"
GRIPPER_XML = "models/converted/robotiq_2f85.xml"
OUT = "models/generated/scene_2f85.xml"
OUT_SUCTION = "models/generated/scene_epick.xml"


G_PREFIX = "g_"
S_PREFIX = "s_"
FLIP_DOWN = [0, 1, 0, 0]   # quat: 180° about x, fingers point down
GRASP_Z = 0.14            # m from gripper base to pad centre (estimate, tune in viewer)
CLOSED_TIP_Z = 0.1628     # m from gripper base to closed tip 
WELD_SOLREF = [0.002, 1]
FLANGE_START = [0, 0, 0.7]

def make_flange_scene():
    scene = mujoco.MjSpec.from_file(str(SCENE_XML))

    #create a normal carrier body which will have a real velocity unlike a mocap
    flange = scene.worldbody.add_body(name="flange", pos=FLANGE_START)
    flange.add_freejoint()
    
    #weld flange to carrier. The function assumes the mocap name is 'carrier'
    scene.add_equality(type=mujoco.mjtEq.mjEQ_WELD, objtype=mujoco.mjtObj.mjOBJ_BODY,
                         name1="carrier", name2="flange",
                          data=[0,0,0, 0,0,0, 1,0,0,0, 1], solref=WELD_SOLREF)
    
    # add a frame to the flange body, rotated by FLIP_DOWN
    frame = flange.add_frame(name="rot_frame", quat=[0, 1, 0, 0])
    return scene, flange, frame

def build_finger_scene(obj: str | None = None) -> mujoco.MjSpec:
    """obj: key in objects.OBJECTS, or None for the cube."""
    scene, flange, frame = make_flange_scene()
    if obj is not None:
        add_object(scene, obj, long_axis_x=False)   # narrow side faces the fingers
    gripper = mujoco.MjSpec.from_file(str(GRIPPER_XML))

    # TODO 2: attach the gripper spec into that frame with PREFIX
    #   hint: scene.attach(gripper, prefix=..., frame=...)
    scene.attach(gripper, prefix=G_PREFIX, frame=frame)
    # TODO 3: add a site named "grasp" on the gripper base body at [0, 0, GRASP_Z]
    #   hint: scene.body(PREFIX + "robotiq_85_base_link").add_site(name=..., pos=..., size=[0.005, 0, 0], rgba=[0, 0, 1, 1])
    scene.body(G_PREFIX + "robotiq_85_base_link").add_site(name="grasp", pos=[0, 0, GRASP_Z],
                                                         size=[0.005, 0, 0], rgba=[0, 1, 0, 1])
    scene.body(G_PREFIX + "robotiq_85_base_link").add_site(name="closed_tip", pos=[0, 0, CLOSED_TIP_Z],
                                                         size=[0.005, 0, 0], rgba=[1, 0, 0, 1])

    return scene

EPICK_LEN, EPICK_R = 0.1023, 0.044   # m, from epick_body.xacro
CUP_H = 0.015                        # m, from epick_single_suction_cup.xacro
BAR_T, BAR_W = 0.01, 0.02            # m, my choice (no crossbar in the xacro)
SUCTION_GRASP_Z = EPICK_LEN + BAR_T + CUP_H   # m, flange to cup lip plane
SUCTION_DEFAULTS = dict(cup_r=0.012, spacing=0.025, f_cup=10.0)

def make_epick_spec(cup_r: float, spacing: float, f_cup: float) -> mujoco.MjSpec:
    """Dual-cup EPick in tool frame (+z toward object). cup_r, spacing in m; f_cup in N."""
    spec = mujoco.MjSpec()

    # TODO 1: epick = spec.worldbody.add_body(name="epick")
    epick = spec.worldbody.add_body(name="epick")
    epick.add_geom(type=mujoco.mjtGeom.mjGEOM_CYLINDER, 
                   size=[EPICK_R, EPICK_LEN/2], pos=[0, 0, EPICK_LEN/2])
    
    # TODO 2: crossbar = epick.add_body(name="crossbar", pos=[0, 0, EPICK_LEN])
    crossbar = epick.add_body(name="crossbar", pos=[0, 0, EPICK_LEN])
    crossbar.add_geom(pos=[0, 0, BAR_T/2], type=mujoco.mjtGeom.mjGEOM_BOX, 
                      size=[spacing/2 + cup_r, BAR_W/2, BAR_T/2])

    for side, sign in (("left", -1), ("right", +1)):
        cup = crossbar.add_body(name=f"{side}_cup", pos=[sign*spacing/2, 0, BAR_T])
        cup.add_geom(type=mujoco.mjtGeom.mjGEOM_CYLINDER, size=[cup_r, CUP_H/2], pos=[0, 0, CUP_H/2])
        cup.add_site(name=f"{side}_cup_site", pos=[0, 0, CUP_H], rgba=[1, 0, 0, 1])
        actuator = spec.add_actuator(
            name=f"{side}_vacuum",
            trntype=mujoco.mjtTrn.mjTRN_BODY, 
            target=f"{side}_cup", 
            ctrlrange=[0, 1]
            )
        actuator.set_to_adhesion(f_cup)
    return spec

def build_suction_scene(obj: str | None = None, **params) -> mujoco.MjSpec:
    """obj: key in objects.OBJECTS, or None for the cube. params: make_epick_spec args."""
    scene, flange, frame = make_flange_scene()
    if obj is not None:
        add_object(scene, obj, long_axis_x=True)    # long side runs along the cup spacing
    suction = make_epick_spec(**params)

    # TODO 2: attach the suction spec into that frame with PREFIX
    scene.attach(suction, prefix=S_PREFIX, frame=frame)
    # TODO 3: add a site named "grasp" on the gripper base body at [0, 0, GRASP_Z]
    scene.body(S_PREFIX + "epick").add_site(name="grasp", pos=[0, 0, SUCTION_GRASP_Z],
                                                         size=[0.005, 0, 0], rgba=[0, 1, 0, 1])
    return scene

if __name__ == "__main__":
    spec = build_finger_scene()
    spec.compile()          # fail early if attach broke something
    spec.to_file(OUT)
    print("gripper saved", OUT)

    spec = build_suction_scene(**SUCTION_DEFAULTS)
    spec.compile()          # fail early if attach broke something
    spec.to_file(OUT_SUCTION)
    print("suction saved", OUT_SUCTION)