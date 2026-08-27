"""Costruzione della scena in CoppeliaSim.

La scena non viene salvata come file binario .ttt ma ricostruita da questo
script a ogni esecuzione.
"""

from coppeliasim_zmqremoteapi_client import RemoteAPIClient

import config as cfg


# --- Utility di basso livello ---------------------------------------------


def connect():
    """Apre la connessione e restituisce l'oggetto sim."""
    client = RemoteAPIClient()
    sim = client.require("sim")
    if sim.getSimulationState() != sim.simulation_stopped:
        raise RuntimeError(
            "La simulazione è in esecuzione. Fermala in CoppeliaSim "
            "(STOP) prima di ricostruire la scena."
        )
    return sim


def clear_scene(sim) -> None:
    """Rimuove il sottoalbero costruito da una precedente esecuzione."""
    try:
        root = sim.getObject(f"/{cfg.SCENE_ROOT}")
    except Exception:
        return 
    handles = sim.getObjectsInTree(root, sim.handle_all, 0)
    sim.removeObjects(handles)
    print("Scena precedente rimossa.")


def make_shape(sim, primitive, sizes, position, color, name,
               transparency=None, parent=None):
    """Crea una primitiva statica, non collidibile, già posizionata.
    Statica e non collidibile perché la simulazione è puramente cinematica:
    non ci interessa la dinamica dei corpi, ci interessa la traiettoria.
    """
    handle = sim.createPrimitiveShape(primitive, sizes, 0)
    sim.setObjectAlias(handle, name)
    sim.setObjectPosition(handle, position, sim.handle_world)
    sim.setShapeColor(handle, None, sim.colorcomponent_ambient_diffuse, color)

    if transparency is not None:
        sim.setShapeColor(handle, None, sim.colorcomponent_transparency,
                          [transparency])

    sim.setObjectInt32Param(handle, sim.shapeintparam_static, 1)
    sim.setObjectInt32Param(handle, sim.shapeintparam_respondable, 0)

    if parent is not None:
        sim.setObjectParent(handle, parent, True)

    return handle


# --- Costruzione dell'ambiente --------------------------------------------


def build_environment(sim, root):
    """Phantom di tessuto, lesione target e marker del punto di ingresso."""

    phantom = make_shape(
        sim,
        sim.primitiveshape_cuboid,
        cfg.PHANTOM_SIZE,
        cfg.PHANTOM_CENTER,
        cfg.PHANTOM_COLOR,
        "Phantom",
        transparency=cfg.PHANTOM_TRANSPARENCY,
        parent=root,
    )

    lesion = make_shape(
        sim,
        sim.primitiveshape_spheroid,
        [cfg.LESION_RADIUS * 2] * 3,
        cfg.LESION_POSITION,
        cfg.LESION_COLOR,
        "Lesion",
        parent=root,
    )

    # Dummy sul punto di ingresso
    entry = sim.createDummy(0.01)
    sim.setObjectAlias(entry, "EntryPoint")
    sim.setObjectPosition(entry, cfg.entry_point().tolist(), sim.handle_world)
    sim.setObjectParent(entry, root, True)

    # Dummy sul centro della lesione
    target = sim.createDummy(0.01)
    sim.setObjectAlias(target, "Target")
    sim.setObjectPosition(target, cfg.LESION_POSITION, sim.handle_world)
    sim.setObjectParent(target, root, True)

    return phantom, lesion, entry, target


# --- Costruzione del robot ------------------------------------------------


def _const(sim, *names):
    """Restituisce la prima costante disponibile tra quelle indicate.
    """
    for name in names:
        if hasattr(sim, name):
            return getattr(sim, name)
    raise AttributeError(f"nessuna di queste costanti esiste: {names}")


def make_joint(sim, kind, name, position, orientation, parent):
    """Crea un giunto cinematico già orientato e agganciato al padre.
    """
    if kind == "prismatic":
        jtype = _const(sim, "joint_prismatic", "joint_prismatic_subtype")
    else:
        jtype = _const(sim, "joint_revolute", "joint_revolute_subtype")

    mode = _const(sim, "jointmode_kinematic", "jointmode_passive")
    handle = sim.createJoint(jtype, mode, 0, [0.04, 0.06])

    sim.setObjectAlias(handle, name)

    sim.setObjectPosition(handle, position, sim.handle_world)
    sim.setObjectOrientation(handle, orientation, sim.handle_world)
    sim.setObjectParent(handle, parent, True)

    key = name.replace("Joint", "")
    low, span = cfg.JOINT_LIMITS[key]
    sim.setJointInterval(handle, False, [low, span])

    return handle


def build_robot(sim, root):
    """Catena cinematica a cinque gradi di libertà.

    Gerarchia: JointX -> CarriageX -> JointY -> CarriageY -> JointZ ->
    Column -> JointTilt -> Holder -> JointIns -> Needle -> NeedleTip
    """
    z0 = cfg.GANTRY_Z
    origin = [0.0, 0.0, z0]

    # Orientamenti necessari a puntare l'asse Z locale dove serve.
    along_x = [0.0, 1.5708, 0.0]      # Z locale -> +X mondo
    along_y = [-1.5708, 0.0, 0.0]     # Z locale -> +Y mondo
    along_z = [0.0, 0.0, 0.0]         # Z locale -> +Z mondo
    downward = [3.1416, 0.0, 0.0]     # Z locale -> -Z mondo

    grey = [0.55, 0.55, 0.58]
    steel = [0.75, 0.75, 0.78]

    # Guida fissa del gantry: elemento puramente visivo, non fa parte della
    # catena e resta figlio della radice.
    make_shape(sim, sim.primitiveshape_cuboid, [1.10, 0.10, 0.03],
               [0.45, 0.0, z0 + 0.04], grey, "GantryRail", parent=root)

    jx = make_joint(sim, "prismatic", "JointX", origin, along_x, root)
    carriage_x = make_shape(sim, sim.primitiveshape_cuboid, [0.10, 0.12, 0.05],
                            origin, steel, "CarriageX", parent=jx)

    jy = make_joint(sim, "prismatic", "JointY", origin, along_y, carriage_x)
    carriage_y = make_shape(sim, sim.primitiveshape_cuboid, [0.07, 0.09, 0.05],
                            [0.0, 0.0, z0 - 0.035], steel, "CarriageY",
                            parent=jy)

    jz = make_joint(sim, "prismatic", "JointZ", origin, along_z, carriage_y)
    column = make_shape(sim, sim.primitiveshape_cuboid, [0.05, 0.05, 0.07],
                        [0.0, 0.0, z0 - 0.075], grey, "Column", parent=jz)

    jtilt = make_joint(sim, "revolute", "JointTilt", origin, along_y, column)
    holder = make_shape(sim, sim.primitiveshape_cuboid, [0.045, 0.045, 0.05],
                        [0.0, 0.0, z0 - 0.03], [0.20, 0.35, 0.60],
                        "Holder", parent=jtilt)

    jins = make_joint(sim, "prismatic", "JointIns", origin, downward, holder)

    # L'ago è un cilindro il cui asse coincide con l'asse Z locale. A corsa
    # nulla si estende dal pivot verso il basso per tutta la sua lunghezza.
    needle = make_shape(
        sim,
        sim.primitiveshape_cylinder,
        [cfg.NEEDLE_DIAMETER, cfg.NEEDLE_DIAMETER, cfg.NEEDLE_LENGTH],
        [0.0, 0.0, z0 - cfg.NEEDLE_LENGTH / 2.0],
        [0.85, 0.85, 0.90],
        "Needle",
        parent=jins,
    )

    # Dummy sulla punta
    tip = sim.createDummy(0.006)
    sim.setObjectAlias(tip, "NeedleTip")
    sim.setObjectPosition(tip, [0.0, 0.0, z0 - cfg.NEEDLE_LENGTH],
                          sim.handle_world)
    sim.setObjectParent(tip, needle, True)

    return [jx, jy, jz, jtilt, jins]


def main() -> None:
    sim = connect()
    clear_scene(sim)

    root = sim.createDummy(0.02)
    sim.setObjectAlias(root, cfg.SCENE_ROOT)
    sim.setObjectPosition(root, [0.0, 0.0, 0.0], sim.handle_world)

    build_environment(sim, root)
    joints = build_robot(sim, root)

    # Configurazione di riposo
    for handle, value in zip(joints, cfg.HOME_Q):
        sim.setJointPosition(handle, float(value))

    print("Scena costruita.")
    print(f"  superficie cutanea      z = {cfg.SKIN_Z:.3f} m")
    print(f"  profondità inserimento    = {cfg.insertion_depth() * 1000:.1f} mm")


if __name__ == "__main__":
    main()