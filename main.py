import time
import os
import mujoco
import mujoco.viewer

def keyboard_callback(keycode):
    if keycode == 265: # up arrow key
        print("Up arrow key pressed!")


def main():
    print("Welcome to Droids!")

    # locate assets
    current_dir = os.path.dirname(os.path.abspath(__file__))
    assets_dir = os.path.join(current_dir, "assets")
    scene_path = os.path.join(assets_dir, "scene.xml")

    # initialize the scene and simulation
    model = mujoco.MjModel.from_xml_path(scene_path)
    data = mujoco.MjData(model)

    # Launch the passive viewer for rendering the sim/game
    with mujoco.viewer.launch_passive(model, data, key_callback=keyboard_callback) as viewer:
        # Run the simulation loop
        while viewer.is_running():
            step_start_time = time.time()

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
