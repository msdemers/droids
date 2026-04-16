import time
import os
import mujoco
import mujoco.viewer

# A state tracker to manage continuous input from discrete key presses. This is necessary because the key callback only triggers on key press events, not on key release events.
class RoverControl:
    def __init__(self):
        self.forward = 0.0
        self.turn = 0.0
        self.last_update = time.time()

control = RoverControl()

def keyboard_callback(keycode):
    """Handle key presses from the MuJoCo viewer. This function is called whenever a key is pressed while the viewer is active."""
    
    try:
        # convert numeric keycode to character
        key = chr(keycode).lower()
        match key:
            case 'w':
                control.forward = 5.0  # Move forward
                control.last_update = time.time()  # Update the last update time
            case 's':
                control.forward = -5.0 # Move backward
                control.last_update = time.time()
            case 'a':
                control.turn = 5.0     # Turn left
                control.last_update = time.time()
            case 'd':
                control.turn = -5.0    # Turn right
                control.last_update = time.time()
            case ' ':
                control.forward = 0.0  # Stop forward/backward movement
                control.last_update = time.time()
    except ValueError:
        # Ignore non-character key presses
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
        # Run the simulation loop
        while viewer.is_running():
            step_start_time = time.time()

            # check for release of keys, accounting for delay
            if time.time() - control.last_update > 0.1:
                control.forward = 0.0
                control.turn = 0.0

            # convert control inputs to left/right wheel inputs
            # mapping represent a skid-steer control scheme
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
