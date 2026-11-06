"""Object library: pre-converted Google Scanned Objects (MJCF, convex collision) placed on the table.

Source: github.com/kevinzakka/mujoco_scanned_objects (see assets/licenses/PROVENANCE.md).
Each object folder has model.xml, a visual mesh (group 2, no collisions) and 32 V-HACD
collision hulls (group 3). The body has no joint and no mass set; we add both here.
"""
import os
from functools import lru_cache

import mujoco
import numpy as np

OBJ_DIR   = "assets/objects/scanned"
TABLE_TOP = 0.40     # m, top face of table_top in models/table.xml
DROP_GAP  = 0.001    # m, start 1 mm above the table; settles during SETTLE_OBJECT
OBJ_PREFIX = "obj_"
OBJ_BODY  = OBJ_PREFIX + "model"   # body name after attach

# key -> (folder name, nominal mass in kg).
# Masses are rough estimates of the real products (not measured, not from the dataset).
OBJECTS = {
    "mug":         ("ACE_Coffee_Mug_Kristen_16_oz_cup", 0.35),
    "lego_figure": ("Android_Lego", 0.10),
    "pill_bottle": ("Beta_Glucan", 0.12),
    "ink_box":     ("Canon_Pixma_Ink_Cartridge_8", 0.05),
    "dino_toy":    ("Great_Dinos_Triceratops_Toy", 0.08),
    "flat_bottle": ("JarroSil_Activated_Silicon", 0.15),
    "tall_bottle": ("Jarrow_Formulas_Glucosamine_Hci_Mega_1000_100_ct", 0.20),
    "candy_box":   ("Nestle_Nips_Hard_Candy_Peanut_Butter", 0.12),
}


def _load_spec(key: str) -> mujoco.MjSpec:
    """Object spec with absolute asset paths, a freejoint, and its nominal mass."""
    folder, mass = OBJECTS[key]
    path = os.path.abspath(os.path.join(OBJ_DIR, folder))
    spec = mujoco.MjSpec.from_file(os.path.join(path, "model.xml"))

    # Absolute file paths, so the meshes still resolve after attach into the table scene.
    for mesh in spec.meshes:
        mesh.file = os.path.join(path, mesh.file)
    for tex in spec.textures:
        tex.file = os.path.join(path, tex.file)

    # Mass: only the collision hulls carry mass. Scale their density so the total = nominal mass.
    body = spec.body("model")
    visual = [g for g in body.geoms if g.contype == 0]
    hulls  = [g for g in body.geoms if g.contype != 0]
    for g in visual:
        g.density = 0
    m_default = spec.copy().compile().body("model").mass[0]   # mass at default density 1000 kg/m^3
    for g in hulls:
        g.density = 1000 * mass / m_default

    body.add_freejoint()
    return spec


@lru_cache(maxsize=None)
def object_info(key: str, long_axis_x: bool) -> dict:
    """Placement and size of an object on the table, in world frame (m).

    long_axis_x=False: rotate so the narrow side faces the fingers (they close along world x).
    long_axis_x=True:  rotate so the long side runs along x (the two cups are spaced along x).
    """
    model = _load_spec(key).compile()
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    # Axis-aligned box around all collision hulls, in the object's own frame.
    pts = []
    for g in range(model.ngeom):
        if model.geom_contype[g] == 0:
            continue
        mid = model.geom_dataid[g]
        v = model.mesh_vert[model.mesh_vertadr[mid]: model.mesh_vertadr[mid] + model.mesh_vertnum[mid]]
        pts.append(v @ data.geom_xmat[g].reshape(3, 3).T + data.geom_xpos[g])
    pts = np.vstack(pts)
    lo, hi = pts.min(0), pts.max(0)
    size = hi - lo

    # Yaw 90 deg about z if the wanted axis is currently along y.
    turn = (size[0] > size[1]) != long_axis_x
    yaw = np.pi / 2 if turn else 0.0
    quat = [np.cos(yaw / 2), 0, 0, np.sin(yaw / 2)]
    c = (lo + hi) / 2
    c_rot = np.array([-c[1], c[0]]) if turn else c[:2]           # box centre after the yaw
    width_x, width_y = (size[1], size[0]) if turn else (size[0], size[1])

    return dict(
        key=key,
        body=OBJ_BODY,
        pos=[-c_rot[0], -c_rot[1], TABLE_TOP + DROP_GAP - lo[2]],  # box centre lands on world x=y=0
        quat=quat,
        height=size[2],
        top_z=TABLE_TOP + size[2],     # world z of the object's highest point once resting
        width_x=width_x,               # along the finger closing / cup spacing axis
        width_y=width_y,
        mass=OBJECTS[key][1],
    )


def add_object(scene: mujoco.MjSpec, key: str, long_axis_x: bool) -> dict:
    """Replace the cube in `scene` with object `key`. Returns object_info."""
    info = object_info(key, long_axis_x)
    scene.delete(scene.body("cube"))
    frame = scene.worldbody.add_frame(pos=info["pos"], quat=info["quat"])
    scene.attach(_load_spec(key), prefix=OBJ_PREFIX, frame=frame)
    return info


def cube_info() -> dict:
    """Same fields for the 50 mm cube already in models/table.xml."""
    return dict(key="cube", body="cube", height=0.05, top_z=TABLE_TOP + 0.05,
                width_x=0.05, width_y=0.05, mass=0.1)
