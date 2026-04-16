# Droids ☄️🤖

A classic *Asteroids*-style game rebuilt from the ground up as a 3D rigid-body physics simulation using the **MuJoCo** physics engine.

The goal of this project is to explore kinematics, collision detection, and rigid-body forces in an interactive, playful environment.

## 🛠 Project Structure

  * `main.py`: The core game loop, user input handling, and MuJoCo viewer synchronization.
  * `assets/scene.xml`: The MuJoCo XML (MJCF) file defining the physical world, the scene, and the Droid chassis.

## 🚀 Quickstart

This project uses [`uv`](https://www.google.com/search?q=%5Bhttps://github.com/astral-sh/uv%5D\(https://github.com/astral-sh/uv\)) to manage dependencies and Python environments for lightning-fast, reproducible setups.

### Linux & Windows

1.  **Clone the repo:**
    ```bash
    git clone https://github.com/msdemers/droids.git
    cd droids
    ```
2.  **Run the simulation:**
    ```bash
    uv run python main.py
    ```
    *(Note: `uv run` will automatically create an isolated environment, install MuJoCo/NumPy, and launch the game.)*

-----

### 🍏 macOS (Apple Silicon) Notes

macOS is strict about how graphical applications are launched. To run the MuJoCo viewer successfully, you need a **Framework build** of Python (not a standalone build) so the viewer can link to the necessary dynamic libraries.

1.  **Install a Framework Build of Python 3.12:** Do not rely `uv`'s default downloaded Python versions. Instead, either download and install the official macOS installer from [python.org](https://www.python.org/downloads/macos/), or use homebrew.
    ```bash
    brew install python@3.12
    ```
2.  **Clone the repo:**
    ```bash
    git clone https://github.com/msdemers/droids.git
    cd droids
    ```
3.  **Build the environment targeting the Framework build:**
    If you used the python.org installer, run
    ```bash
    uv venv --python /Library/Frameworks/Python.framework/Versions/3.12/bin/python3.12
    uv sync
    ```
    If you used hombrew, run 
    ```bash
    uv venv --python /opt/homebrew/bin/python3.12
    uv sync
    ```
4.  **Run the simulation using `mjpython`:**
    ```bash
    uv run mjpython main.py
    ```
    *(`mjpython` is a wrapper included with MuJoCo that handles macOS graphical window entitlements).*