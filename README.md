# HMI (Human-Machine Interface)

The **Human-Machine Interface (HMI)** is designed to facilitate interaction between human operators and the robot. This layer will provide a user-friendly interface for monitoring and controlling robot functions, enabling efficient and accurate commands to be sent to the robot.

## Goal 🏆️

The goal of this project is to create an intuitive and reliable interface that allows for straightforward control and monitoring of the robot’s functions. By standardizing communication methods and display components, the HMI aims to simplify operator interaction with the robot, ensuring flexibility for both current and future use cases.

## Overview ℹ️

This repository currently uses **React** (built with Vite) to build a responsive and dynamic interface. React will be used to display input options and such on the touchscreen on the robot. This should also allow for remote monitoring in the future via a tablet or laptop to get more advanced information. As development progresses, ROS will be integrated to allow for direct communication with the robot.

## Build instructions 🛠️

### Prerequisites

You will need to install:

- Docker

Or:

- NodeJS
- Yarn

> [!NOTE]
> You will need all of these installed (especially yarn and node) if you plan on doing any development in this repository

### Building

#### ROS Communication

For the site to be able to connect and interact with ros, you will need to:

1. include the contents of ./ros in your ros environment, and
2. have rosbridge-suite installed and running for the client to be able to communicate to ROS (as in, fetch nodes, start/stop the robot, etc.).

You can do this in your ROS environment by:

```bash
# Install ROSBridge-Suite
apt update # fetch package list
apt install ros-<ros distro>-rosbridge-suite

# Start ROSBridge (you may need to colcon build first)
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

#### Hosting the App

This repository includes a Dockerfile that will automatically build and host the React app via Nginx. Since the backend is essentially ROS, this hosting is very basic but works fine. It can be run by:

```bash
docker build -t grr-hmi .
docker run -p 8080:80 grr-hmi # Will serve on http://localhost:8080/. Replace '8080:80' with '80:80' to serve on http://localhost/ instead.
```

Alternatively, if you wish to use some other service to host the app, you will need NodeJS and Yarn. You can compile the app like so:

```bash
cd app # Working directory is in ./app
yarn install # Install packages
yarn build
```

You should then find the compiled assets + JS in `./app/dist`.

## Development instructions 💻️

> [!IMPORTANT]
> This project was made with yarn, please do not use other package managers such as npm!

To get started with development, clone this repository, and then you can spin up a development server like so:

```bash
cd app # Working directory is in ./app
yarn install # Install packages
yarn dev # Starts the development server
```

To be able to pass CI and merge pull requests, code has to be formatted properly and all tests must pass. For convenience, these commands have been added to package.json:

```bash
yarn format # Formats code with prettier
yarn format:check # Checks if any files contain formatting issues without writing anything
yarn test # Runs all tests and outputs results
yarn coverage # Outputs overall coverage of tests
```

> [!NOTE]
> Running tests, particularly for the first time, may take some time. This is normal, and will be optimized #soon.

Additionally, if you are using VSCode, there are extensions that will help with this. [Prettier](https://marketplace.visualstudio.com/items?itemName=esbenp.prettier-vscode) can be configured to format your code on save and [Vitest](https://marketplace.visualstudio.com/items?itemName=vitest.explorer) will run your tests and give you inline coverage.

> [!NOTE]
> As of writing, the latest version of Vitest appears to be unstable. It's recommended to install v1.14.3 (next to install: manage (settings icon) > install specific version) and uncheck auto-update.

## Structure 🗃️

The project is organized into several key directories, each serving a distinct purpose:

- **`app/`**: Contains the React application built with Vite, forming the core of the HMI's user interface.
  - **`src/`**: Houses the source code for the React application, including components, types, and utility functions.
  - **`public/`**: Stores static assets like the favicon and other images.
  - **`node_modules/`**: Project dependencies managed by Yarn.
- **`ros/src/`**: Contains ROS-related code for interfacing with the robot.
  - **`node_info/`**: ROS package providing node information to the app
  - **`node_info_interface/`**: ROS package for interfacing with the app (for services)

This structure may change throughout the development of this project.

## License ⚖️

This repository is licensed under the MIT License. See the [LICENSE](https://github.com/Gold-Rush-Robotics/hmi/blob/main/LICENSE) for more info.
