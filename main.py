import time
import os
import mujoco
import mujoco.viewer

# A state tracker to manage continuous input from discrete key presses. This is necessary because the key callback only triggers on key press events, not on key release events.
class RoverControl:
    def __init__(self):
        self.forward = 0.0
        self.turn = 0.0
        self.throttle_step = 10.0
        self.max_throttle = 50.0

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
    scene_path = os.path.join(assets_dir, "scene.xml")

    # initialize the scene and simulation
    model = mujoco.MjModel.from_xml_path(scene_path)
    data = mujoco.MjData(model)

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


            # convert control inputs to left/right wheel inputs
            # mapping represent a skid-steer throttle-control scheme
            left_torque = control.forward - control.turn
            right_torque = control.forward + control.turn

            # apply the torques to the motors' physics state
            for motor_id in left_motors:
                data.ctrl[motor_id] = left_torque
            for motor_id in right_motors:
                data.ctrl[motor_id] = right_torque

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
