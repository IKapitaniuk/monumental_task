# General Remark
This code is intended to be used on `Ubuntu` distro 20.04+ 
# Dependecies
This code dependes on the following external libraries:
- Boost Libraries: ASIO, Beast, Program Options. These can be installed by 
  ```
  sudo apt install libboost-system-dev libboost-program-options-dev
  ```
- Eigen3 
  ```
  sudo apt install libeigen3-dev
  ```
- Ruckig. This library placed  inside `external` folder of this project
- Google Test. This library placed  inside `external` folder of this project

# Build instruction
To build the application use the following commands
```
mkdir build
cd build 
cmake ..
cmake --build .
```

# How to run
The builded application is placed inside `build/bin` folder.
To run it use the following command from insdie the `build/bin` folder:
```
./app --config-file=../config/robot_config.json --port=8081
```
where `--config-file=<path_to_config_file>` is a path to the robot configuration in the JSON format, and `--port=<server_port_number>`, is a backend server port number