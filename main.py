import time
import os
import numpy as np
import mujoco
import mujoco.viewer
from assets import hazards

# Design parameters
# Rover Parameters
ROVER_MAX_THROTTLE = 50.0
ROVER_THROTTLE_STEP = 10.0

# Hazard Parameters
NUM_RAMMERS = 3
NUM_SENTINELS = 2


# A state tracker to manage continuous input from discrete key presses. This is necessary because the key callback only triggers on key press events, not on key release events.
class RoverControl:
    def __init__(self):
        self.forward = 0.0
        self.turn = 0.0
        self.throttle_step = ROVER_THROTTLE_STEP
        self.max_throttle = ROVER_MAX_THROTTLE

control = RoverControl()

def keyboard_callback(keycode):
    """Handles key presses from the MuJoCo viewer using a Throttle system."""    
    try:
        match keycode:
            case 265:
                control.forward = min(control.forward + control.throttle_step, control.max_throttle)
            case 264:
                control.forward = max(control.forward - control.throttle_step, -control.max_throttle)
            case 263:
                control.turn = min(control.turn + control.throttle_step, control.max_throttle)     # Turn left
            case 262:
                control.turn = max(control.turn - control.throttle_step, -control.max_throttle)    # Turn right
            case 32:
                control.forward = 0.0  # Stop forward/backward movement
                control.turn = 0.0     # Stop turning
    except ValueError:
        # Ignore weirdness with keyboard shortcuts defined for the viewer
        pass
            



def main():
    print("Welcome to Droids!")

    # locate assets
    current_dir = os.path.dirname(os.path.abspath(__file__))
    assets_dir = os.path.join(current_dir, "assets")
    #scene_path = os.path.join(assets_dir, "scene.xml")
    scene_path = os.path.join(assets_dir, "multidroid_scene.xml")

    # initialize the scene and simulation
    spec = mujoco.MjSpec.from_file(scene_path)

    # programmatically add enemies to the scene by modifying the MjSpec in memory before compiling it into a model
    hazards.spawn_enemies(spec, num_rammers=NUM_RAMMERS, num_sentinels=NUM_SENTINELS)

    model = spec.compile()
    data = mujoco.MjData(model)

    # setup lighting shadow box to track the rover and ensure it casts shadows on the ground
    light_id  = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_LIGHT, "sun_light")
    rover_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "player_rover")
    print(f"Shadow box extent radius: {model.stat.extent}")
    SAFE_Z_HEIGHT = model.stat.extent/2

    # Get the internal ID numbers for our rover motors so we can control them in the simulation loop.
    left_motors = [
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "motor_fl"),
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "motor_ml"),
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "motor_bl"),
    ]
    right_motors = [
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "motor_fr"),
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "motor_mr"),
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "motor_br"),
    ]

    # Launch the passive viewer for rendering the sim/game
    with mujoco.viewer.launch_passive(model, data, key_callback=keyboard_callback) as viewer:
        
        # --- CAMERA TRACKING SETUP ---
        # Get the internal ID of our new fixed camera
        cam_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, "chase_cam")
        
        # Tell the viewer to lock onto this specific fixed camera
        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
        viewer.cam.fixedcamid = cam_id
        
        # Adjust the camera angle and distance (TODO tune these!)
        viewer.cam.distance = 4.0      # How far away the camera is
        viewer.cam.elevation = -30.0   # Angle looking down (negative means looking down)
        viewer.cam.azimuth = 90.0      # Orbit angle around the rover
        # -----------------------------
        
        # Run the simulation loop
        while viewer.is_running():
            step_start_time = time.time()

            #print(f"Shadow box extent radius: {model.stat.extent}")

            # update the shadow box position to track the rover's position
            #print(f"Light pos before update: {model.light_pos[light_id]}")
            rover_pos = data.xpos[rover_body_id]
            #print(f"Rover position: {rover_pos}")
            model.light_pos[light_id] = [rover_pos[0], rover_pos[1], SAFE_Z_HEIGHT]
            #print(f"Light pos after update: {model.light_pos[light_id]}")


            # convert control inputs to left/right wheel inputs
            # mapping represent a skid-steer throttle-control scheme
            left_torque = control.forward - control.turn
            right_torque = control.forward + control.turn

            # apply the torques to the motors' physics state
            for motor_id in left_motors:
                data.ctrl[motor_id] = left_torque
            for motor_id in right_motors:
                data.ctrl[motor_id] = right_torque

            # === RAMMER ENEMY AI TRACKING ===
            # Get the player's exact 2D position (X, Y)
            player_pos = data.body("player_rover").xpos[:2]

            for i in range(NUM_RAMMERS): # Assuming num_rammers = 3
                # Get the Rammer's 2D position
                rammer_pos = data.body(f"{hazards.RAMMER_BASE_NAME}{i}").xpos[:2]
                
                # Get the vector pointing from the rammer to the player
                direction = player_pos - rammer_pos
                distance = np.linalg.norm(direction)
                
                if distance > 0.1: # Prevent division by zero if they crash perfectly
                    direction = direction / distance # Normalize
                    
                    # Get the Rammer's local forward vector (its X-axis in world space)
                    # MuJoCo's xmat is a 3x3 rotation matrix. Column 0 is the local X axis.
                    rammer_mat = data.body(f"{hazards.RAMMER_BASE_NAME}{i}").xmat.reshape(3, 3)
                    forward = rammer_mat[:, 0][:2]
                    forward = forward / np.linalg.norm(forward)
                    
                    # 2D Cross Product to find turn error (Positive = Player is Left, Negative = Right)
                    turn_error = forward[0] * direction[1] - forward[1] * direction[0]
                    
                    # Dot Product to check if facing the player
                    face_error = np.dot(forward, direction)
                    
                    # AI Controller
                    # Drive forward if looking generally in the right direction
                    base_speed = hazards.RAMMER_MAX_THROTTLE if face_error > 0 else 0.0 
                    
                    # Turn aggressively based on the cross product error
                    turn_speed = turn_error * hazards.RAMMER_MAX_THROTTLE 

                    r_left_torque = base_speed - turn_speed
                    r_right_torque = base_speed + turn_speed
                    
                    # Apply to motors
                    data.actuator(f"{hazards.RAMMER_BASE_NAME}{i}_ml").ctrl[0] = r_left_torque
                    data.actuator(f"{hazards.RAMMER_BASE_NAME}{i}_mr").ctrl[0] = r_right_torque
            # ==========================

            # === SENTINEL TURRET AI TRACKING ===
            # Get the player's 3D center of mass
            player_pos_3d = data.body("player_rover").xpos
            
            for i in range(NUM_SENTINELS): # Assuming num_sentinels = 2
                # --- PAN (Yaw) CONTROL ---
                head_pos = data.body(f"{hazards.SENTINEL_BASE_NAME}{i}_head").xpos
                direction = player_pos_3d - head_pos
                dist_2d = np.linalg.norm(direction[:2])
                
                if dist_2d > 0.1:
                    dir_2d = direction[:2] / dist_2d
                    
                    # Get the Head's current forward vector in 2D
                    head_mat = data.body(f"{hazards.SENTINEL_BASE_NAME}{i}_head").xmat.reshape(3, 3)
                    head_forward_2d = head_mat[:, 0][:2]
                    head_forward_2d = head_forward_2d / np.linalg.norm(head_forward_2d)
                    
                    # Cross product for Left/Right error
                    pan_error = head_forward_2d[0] * dir_2d[1] - head_forward_2d[1] * dir_2d[0]
                    
                    # Apply Torque (Multiplier defines tracking aggressiveness)
                    data.actuator(f"{hazards.SENTINEL_BASE_NAME}{i}_mpan").ctrl[0] = pan_error * hazards.SENTINEL_MAX_PAN_CONTROL


                # --- TILT (Pitch) CONTROL ---
                cannon_pos = data.body(f"{hazards.SENTINEL_BASE_NAME}{i}_cannon").xpos
                dir_3d = player_pos_3d - cannon_pos
                dist_3d = np.linalg.norm(dir_3d)
                
                if dist_3d > 0.1:
                    dir_3d = dir_3d / dist_3d
                    
                    # Get the Cannon's current UP vector (Z-axis is column 2)
                    cannon_mat = data.body(f"{hazards.SENTINEL_BASE_NAME}{i}_cannon").xmat.reshape(3, 3)
                    cannon_up_3d = cannon_mat[:, 2]
                    
                    # Dot product of the Up vector and the Target vector
                    # If this is positive, the target is "above" the barrel
                    tilt_error = np.dot(cannon_up_3d, dir_3d)
                    
                    # Apply Torque
                    # (Note: depending on the hinge's right-hand rule, we may need to invert this sign. 
                    # If it aims AWAY from you, change this to positive!)
                    data.actuator(f"{hazards.SENTINEL_BASE_NAME}{i}_mtilt").ctrl[0] = -tilt_error * hazards.SENTINEL_MAX_TILT_CONTROL
            # ============================
            
            # step the simulation forward
            mujoco.mj_step(model, data)

            # sync the viewer to the simulation
            viewer.sync()

            # time keeping 
            time_until_next_step = model.opt.timestep - (time.time() - step_start_time)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)


if __name__ == "__main__":
    main()
