
from numpy import random
import mujoco

RAMMER_BASE_NAME = "rammer_"
RAMMER_MAX_THROTTLE = 50.0


SENTINEL_BASE_NAME = "sentinel_"
SENTINEL_DIMENSIONS = {
    "base_height": 0.2,
    "base_width": 0.8,
    "neck_height": 0.4,
    "neck_radius": 0.1,
    "head_radius": 0.2,
    "cannon_length": 0.3,
    "cannon_radius": 0.08
}
SENTINEL_MASSES = {
    "base": 50.0,
    "neck": 5.0,
    "head": 5.0,
    "cannon": 2.0
}

SENTINEL_MAX_PAN_CONTROL = 100.0
SENTINEL_MAX_TILT_CONTROL = 100.0

def spawn_enemies(spec, x_range=(-8, 8), y_range=(-8, 8), num_rammers=1, num_sentinels=1):
    """Programmatically builds enemy droids in memory using MjSpec."""
    
    for i in range(num_rammers):
        x, y = random.uniform(x_range[0], x_range[1]), random.uniform(y_range[0], y_range[1])
        spawn_rammer(spec, name=f"{RAMMER_BASE_NAME}{i}", x=x, y=y)

    for i in range(num_sentinels):
        x, y = random.uniform(x_range[0], x_range[1]), random.uniform(y_range[0], y_range[1])
        spawn_sentinel(spec, name=f"{SENTINEL_BASE_NAME}{i}", x=x, y=y)

def spawn_rammer(spec, name, x, y):
    """Programmatically builds a differential-drive Rammer droid."""
    
    # --- Chassis ---
    enemy_body = spec.worldbody.add_body(name=f"{name}", pos=[x, y, 0.5])
    enemy_body.add_freejoint()
    # Chassis Box (half-extents: L=0.2, W=0.15, H=0.1)
    enemy_body.add_geom(
        type=mujoco.mjtGeom.mjGEOM_BOX,
        size=[0.2, 0.15, 0.1], 
        rgba=[0.8, 0.1, 0.1, 1],
        mass=10.0
    )
    
    # --- Rear Left Drive Wheel ---
    # Positioned back (-0.15) and left (+0.2)
    rl_wheel = enemy_body.add_body(name=f"{name}_wl", pos=[-0.15, 0.2, -0.05])
    rl_joint = rl_wheel.add_joint(name=f"{name}_jl", type=mujoco.mjtJoint.mjJNT_HINGE, axis=[0, 1, 0])
    rl_wheel.add_geom(type=mujoco.mjtGeom.mjGEOM_CYLINDER, size=[0.1, 0.05], rgba=[0.1, 0.1, 0.1, 1], euler=[90, 0, 0])
    
    # --- Rear Right Drive Wheel ---
    # Positioned back (-0.15) and right (-0.2)
    rr_wheel = enemy_body.add_body(name=f"{name}_wr", pos=[-0.15, -0.2, -0.05])
    rr_joint = rr_wheel.add_joint(name=f"{name}_jr", type=mujoco.mjtJoint.mjJNT_HINGE, axis=[0, 1, 0])
    rr_wheel.add_geom(type=mujoco.mjtGeom.mjGEOM_CYLINDER, size=[0.1, 0.05], rgba=[0.1, 0.1, 0.1, 1], euler=[90, 0, 0])
    
    # --- Front Free Wheel (Spherical Caster) ---
    # Positioned forward (+0.2) and centered (0)
    f_wheel = enemy_body.add_body(name=f"{name}_wf", pos=[0.2, 0, -0.05])
    # Ball joint allows it to roll passively in any direction
    f_wheel.add_joint(name=f"{name}_jf", type=mujoco.mjtJoint.mjJNT_BALL)
    f_wheel.add_geom(type=mujoco.mjtGeom.mjGEOM_SPHERE, size=[0.1], rgba=[0.5, 0.5, 0.5, 1])

    # --- Actuators ---
    # Left Motor
    spec.add_actuator(
        name=f"{name}_ml", 
        trntype=mujoco.mjtTrn.mjTRN_JOINT,
        target=rl_joint.name,
        ctrllimited=True, 
        ctrlrange=[-RAMMER_MAX_THROTTLE, RAMMER_MAX_THROTTLE]
    )
    
    # Right Motor
    spec.add_actuator(
        name=f"{name}_mr", 
        trntype=mujoco.mjtTrn.mjTRN_JOINT,
        target=rr_joint.name,
        ctrllimited=True, 
        ctrlrange=[-RAMMER_MAX_THROTTLE, RAMMER_MAX_THROTTLE]
    )

def spawn_sentinel(spec, name, x, y):
    """Programmatically builds heavy Pan-Tilt Sentinel turrets."""
    
    # --- The Heavy Base ---
    s_base = spec.worldbody.add_body(name=f"{name}", pos=[x, y, SENTINEL_DIMENSIONS["base_height"] / 2])
    s_base.add_freejoint()
    # Mass=50 makes it incredibly hard for the player to push around
    s_base.add_geom(
        type=mujoco.mjtGeom.mjGEOM_BOX, 
        size=[SENTINEL_DIMENSIONS["base_width"] / 2, SENTINEL_DIMENSIONS["base_width"] / 2,SENTINEL_DIMENSIONS["base_height"] / 2], 
        rgba=[0.2, 0.2, 0.2, 1], 
        mass=SENTINEL_MASSES["base"]
    )

    # --- The Neck ---
    # A thin cylinder to visually separate the head from the base
    s_neck = s_base.add_body(name=f"{name}_neck", pos=[0, 0, SENTINEL_DIMENSIONS["neck_height"] / 2 + SENTINEL_DIMENSIONS["base_height"] / 2])
    s_neck.add_geom(
        type=mujoco.mjtGeom.mjGEOM_CYLINDER, 
        size=[SENTINEL_DIMENSIONS["neck_radius"], SENTINEL_DIMENSIONS["neck_height"] / 2], 
        rgba=[0.2, 0.2, 0.2, 1], 
        mass=SENTINEL_MASSES["neck"]
    )
    
    # --- The Head (Pan / Yaw Axis) ---
    # Positioned right on top of the base (Z = +0.4)
    s_head = s_neck.add_body(name=f"{name}_head", pos=[0, 0, SENTINEL_DIMENSIONS["head_radius"] + SENTINEL_DIMENSIONS["neck_height"] / 2])
    pan_joint = s_head.add_joint(name=f"{name}_pan", type=mujoco.mjtJoint.mjJNT_HINGE, axis=[0, 0, 1])
    s_head.add_geom(
        type=mujoco.mjtGeom.mjGEOM_SPHERE, 
        size=[SENTINEL_DIMENSIONS["head_radius"]], 
        rgba=[0.8, 0.6, 0.1, 1], 
        mass=SENTINEL_MASSES["head"]
    )
    
    # --- The Cannon (Tilt / Pitch Axis) ---
    # Shifted slightly forward on the X axis so it doesn't clip the sphere
    s_cannon = s_head.add_body(name=f"{name}_cannon", pos=[0.0, 0, 0])
    tilt_joint = s_cannon.add_joint(name=f"{name}_tilt", type=mujoco.mjtJoint.mjJNT_HINGE, axis=[0, 1, 0])
    
    # We rotate the capsule 90 degrees so it points forward along the X-axis
    s_cannon.add_geom(
        type=mujoco.mjtGeom.mjGEOM_CYLINDER, 
        pos=[SENTINEL_DIMENSIONS["cannon_length"] / 2, 0, 0],
        size=[SENTINEL_DIMENSIONS["cannon_radius"], SENTINEL_DIMENSIONS["cannon_length"] / 2], 
        rgba=[0.8, 0.1, 0.1, 1], 
        euler=[0, 90, 0], 
        mass=SENTINEL_MASSES["cannon"]
    )
    
    # --- Actuators ---
    # Very high torque to whip that heavy cannon around
    spec.add_actuator(
        name=f"{name}_mpan",
        trntype=mujoco.mjtTrn.mjTRN_JOINT,
        target=pan_joint.name,
        ctrllimited=True,
        ctrlrange=[-SENTINEL_MAX_PAN_CONTROL, SENTINEL_MAX_PAN_CONTROL]
    )
    spec.add_actuator(
        name=f"{name}_mtilt",
        trntype=mujoco.mjtTrn.mjTRN_JOINT,
        target=tilt_joint.name,
        ctrllimited=True,
        ctrlrange=[-SENTINEL_MAX_TILT_CONTROL, SENTINEL_MAX_TILT_CONTROL]
    )