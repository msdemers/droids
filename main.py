import time
import mujoco
import mujoco.viewer

def keyboard_callback(keycode):
    if keycode == 265: # up arrow key
        print("Up arrow key pressed!")


def main():
    print("Welcome to Droids!")

    """Initialize the simulation and run the game loop."""
    xml_string = """
    <mujoco>
        <worldbody>
            <light diffuse=".5 .5 .5" pos="0 0 3" dir="0 0 -1"/>
            <geom type="plane" size="1 1 0.1" rgba=".9 0 0 1"/>
        </worldbody>
    </mujoco>
    """
    model = mujoco.MjModel.from_xml_string(xml_string)
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
