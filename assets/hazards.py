
from numpy import random
import mujoco

RAMMER_BASE_NAME = "rammer_"
RAMMER_MAX_THROTTLE = 50.0
SENTINEL_BASE_NAME = "sentinel_"
SENTINEL_MAX_PAN_CONTROL = 100.0
SENTINEL_MAX_TILT_CONTROL = 100.0

def spawn_enemies(spec, num_rammers=1, num_sentinels=1):
    """Programmatically builds enemy droids in memory using MjSpec."""
    
    for i in range(num_rammers):
        x, y = random.uniform(-8, 8), random.uniform(-8, 8)
        spawn_rammer(spec, name=f"{RAMMER_BASE_NAME}{i}", x=x, y=y)

    for i in range(num_sentinels):
        x, y = random.uniform(-8, 8), random.uniform(-8, 8)
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
    s_base = spec.worldbody.add_body(name=f"{name}", pos=[x, y, 0.1])
    s_base.add_freejoint()
    # Mass=50 makes it incredibly hard for the player to push around
    s_base.add_geom(
        type=mujoco.mjtGeom.mjGEOM_CYLINDER, 
        size=[0.4, 0.1], 
        rgba=[0.2, 0.2, 0.2, 1], 
        mass=50.0
    )

    # --- The Neck ---
    # A thin cylinder to visually separate the head from the base
    s_neck = s_base.add_body(name=f"{name}_neck", pos=[0, 0, 0.5])
    s_neck.add_geom(
        type=mujoco.mjtGeom.mjGEOM_CYLINDER, 
        size=[0.1, 0.4], 
        rgba=[0.2, 0.2, 0.2, 1], 
        mass=5.0
    )
    
    # --- The Head (Pan / Yaw Axis) ---
    # Positioned right on top of the base (Z = +0.4)
    s_head = s_neck.add_body(name=f"{name}_head", pos=[0, 0, 0.4])
    pan_joint = s_head.add_joint(name=f"{name}_pan", type=mujoco.mjtJoint.mjJNT_HINGE, axis=[0, 0, 1])
    s_head.add_geom(type=mujoco.mjtGeom.mjGEOM_SPHERE, size=[0.2], rgba=[0.8, 0.6, 0.1, 1], mass=5.0)
    
    # --- The Cannon (Tilt / Pitch Axis) ---
    # Shifted slightly forward on the X axis so it doesn't clip the sphere
    s_cannon = s_head.add_body(name=f"{name}_cannon", pos=[0.0, 0, 0])
    tilt_joint = s_cannon.add_joint(name=f"{name}_tilt", type=mujoco.mjtJoint.mjJNT_HINGE, axis=[0, 1, 0])
    
    # We rotate the capsule 90 degrees so it points forward along the X-axis
    s_cannon.add_geom(
        type=mujoco.mjtGeom.mjGEOM_CYLINDER, 
        pos=[0.15, 0, 0],
        size=[0.08, 0.3], 
        rgba=[0.8, 0.1, 0.1, 1], 
        euler=[0, 90, 0], 
        mass=2.0
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