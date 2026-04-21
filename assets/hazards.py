
from numpy import random
import mujoco

def spawn_enemies(spec, num_rammers=1, num_sentinels=1):
    """Programmatically builds enemy droids in memory using MjSpec."""
    
    for i in range(num_rammers):
        x, y = random.uniform(-8, 8), random.uniform(-8, 8)
        
        # 1. Add a new body to the world
        enemy_body = spec.worldbody.add_body(name=f"rammer_{i}", pos=[x, y, 0.5])
        
        # 2. Give it a free joint so physics apply to it
        enemy_body.add_freejoint()
        
        # 3. Define its shape, color, and mass programmatically
        # Note: box size is defined by half-extents [x, y, z]
        enemy_body.add_geom(
            type=mujoco.mjtGeom.mjGEOM_BOX,
            size=[0.15, 0.15, 0.1], 
            rgba=[0.8, 0.1, 0.1, 1],
            mass=15.0
        )

    for i in range(num_sentinels):
        x, y = random.uniform(-8, 8), random.uniform(-8, 8)
        
        enemy_body = spec.worldbody.add_body(name=f"sentinel_{i}", pos=[x, y, 0.5])
        enemy_body.add_freejoint()
        
        # Note: cylinder size is [radius, half-height]
        enemy_body.add_geom(
            type=mujoco.mjtGeom.mjGEOM_CYLINDER,
            size=[0.25, 0.2],
            rgba=[0.8, 0.6, 0.1, 1],
            mass=8.0
        )