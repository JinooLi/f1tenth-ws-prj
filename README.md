# test_ws

## How to use?
0. Required Environment.
    - OS: Linux or WSL
    - Docker
    - Text Editor: VSCode
        - Extension: DevContainer

1. Clone this repository. (It is recommended to have Docker installed and DevContainer extension in VSCode on a Linux or WSL environment.)

    ```bash
    git clone https://github.com/JinooLi/test-ws.git
    ```

2. Open this directory (repository) in VSCode.

    ```bash
    cd test_ws
    code .
    ```

3. Press `Ctrl + Shift + P`, search for `rebuild container`, and build the DevContainer.

4. Start working.

## Component

- [.devcontainer](./.devcontainer) : Contains devcontainer.json and Dockerfile for setting up the work environment.

- [.vscode](./.vscode) : VSCode environment settings.

- [scripts](./scripts/) : Directory for storing various shell scripts.
    - [init.sh](./scripts/init.sh) : Script to run at the end of setting up the DevContainer.

- [src](./src/) : Directory containing the source code of ROS2 packages.
    - [car-control](./src/car-control/) : Node for vehicle control algorithms -> [readme](https://github.com/JinooLi/car-control/tree/main)
    - [devices](./src/devices/) : Contains ROS packages for Lidar, IMU, and VESC.
    - [simulation](./src/simulation/) : Contains ROS packages for the f1tenth simulator.

## How to use submodule

- Add submodule
    ```bash
    git submodule add <repo>
    ```

- Update submodules
    ```bash
    git submodule update --init --recursive
    ```
