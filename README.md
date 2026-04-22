# Droids ☄️🤖

https://github.com/user-attachments/assets/75aaf3be-cdb9-4fe5-9581-b8c1bbfc3be2

A classic *Asteroids*-style game rebuilt from the ground up as a 3D rigid-body physics simulation using the **MuJoCo** physics engine. It's *Asteroids*-style in that the player navigates a 2-D environment to avoid enemies. However:
1. Those enemies aim for you.
2. Some of them shoot at you.
3. You're driving a skid-steer rover, not flying a ship.
4. The 3-D physics yeild much more interesting interactions and consequences.

The goal of this project is to explore kinematics, collision detection, and rigid-body forces in an interactive, playful environment.

## 🛠 Project Structure

  * `main.py`: The core game loop, user input handling, enemy behavior, and the MuJoCo viewer synchronization.
  * `assets/`: The directory containing the core MuJoCo XML (MJCF) file, the player rover droid, and programatically spawned hazards/enemies.

## 🚀 Installation

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

    *If* you used the python.org installer, run
    ```bash
    uv venv --python /Library/Frameworks/Python.framework/Versions/3.12/bin/python3.12
    uv sync
    ```
    *Otherwise, if* you used hombrew, run 
    ```bash
    uv venv --python /opt/homebrew/bin/python3.12
    uv sync
    ```
5.  **Run the simulation using `mjpython`:**
    ```bash
    uv run mjpython main.py
    ```
    *(`mjpython` is a wrapper included with MuJoCo that handles macOS graphical window entitlements).*

## 🎮 Controls

The rover uses a skid-steer drive system and features an articulated physics-based cannon. Check your terminal while playing for a live telemetry HUD!

| System | Action | Input |
| :--- | :--- | :--- |
| **Driving** | Throttle Forward / Reverse | <kbd>↑</kbd> <kbd>↓</kbd> |
| | Turn Left / Right | <kbd>←</kbd> <kbd>→</kbd> |
| | Brake / Cut Throttle | <kbd>Left Shift</kbd> |
| **Weapons** | Cannon Pitch Up | <kbd>2</kbd> |
| | Cannon Pitch Down | <kbd>1</kbd> |
| | Fire Weapon | <kbd>Space</kbd> |
