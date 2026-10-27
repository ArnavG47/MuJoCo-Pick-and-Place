import mujoco

SCENE_XML = "models/table.xml"
GRIPPER_XML = "models/converted/robotiq_2f85.xml"
OUT = "models/generated/scene_2f85.xml"

PREFIX = "g_"
FLIP_DOWN = [0, 1, 0, 0]   # quat: 180° about x, fingers point down
GRASP_Z = 0.125            # m from gripper base to pad centre (estimate, tune in viewer)
WELD_SOLREF = [0.002, 1]
TOOL_START = [0, 0, 0.7]

def make_tool_scene() -> tuple[mujoco.MjSpec, mujoco.MjsBody]:
    scene = mujoco.MjSpec.from_file(str(SCENE_XML))

    #create a normal carrier body which will have a real velocity unlike a mocap
    tool = scene.worldbody.add_body(name="tool", pos=TOOL_START)
    tool.add_freejoint()
    
    #weld tool to carrier. The function assumes the mocap name is 'carrier'
    scene.add_equality(type=mujoco.mjtEq.mjEQ_WELD, objtype=mujoco.mjtObj.mjOBJ_BODY,
                         name1="carrier", name2="tool",
                          data=[0,0,0, 0,0,0, 1,0,0,0, 1], solref=WELD_SOLREF)
    return scene, tool

def build_finger_scene() -> mujoco.MjSpec:
    scene, tool = make_tool_scene()
    gripper = mujoco.MjSpec.from_file(str(GRIPPER_XML))

    # TODO 1: add a frame to the tool body, rotated by FLIP_DOWN
    frame = tool.add_frame(quat=[0, 1, 0, 0])

    # TODO 2: attach the gripper spec into that frame with PREFIX
    #   hint: scene.attach(gripper, prefix=..., frame=...)
    scene.attach(gripper, prefix=PREFIX, frame=frame)
    # TODO 3: add a site named "grasp" on the gripper base body at [0, 0, GRASP_Z]
    #   hint: scene.body(PREFIX + "robotiq_85_base_link").add_site(name=..., pos=..., size=[0.005, 0, 0], rgba=[0, 0, 1, 1])
    scene.body(PREFIX + "robotiq_85_base_link").add_site(name="grasp", pos=[0, 0, GRASP_Z],
                                                         size=[0.005, 0, 0], rgba=[.5, .5, 0, 1])

    return scene


if __name__ == "__main__":
    spec = build_finger_scene()
    spec.compile()          # fail early if attach broke something
    spec.to_file(OUT)
    print("saved", OUT)
