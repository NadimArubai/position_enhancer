# Position Enhancer

A ROS2 package for enhancing object position estimates using GTSAM (Georgia Tech Smoothing and Mapping) library for optimal triangulation from multiple observations.

## Overview

This package provides a service-based solution for improving object position estimates by fusing multiple range and bearing measurements from different robot poses. It uses factor graph optimization with GTSAM to compute the most probable object position and its uncertainty.

## Features

- **Service-based architecture**: Stateless service for position enhancement
- **GTSAM integration**: Uses factor graph optimization for optimal triangulation
- **Uncertainty estimation**: Provides covariance estimates for enhanced positions
- **Flexible input**: Supports variable numbers of observations with custom uncertainty models
- **ROS2 compatible**: Built with modern ROS2 standards

## Package Structure

```
position_enhancer/
├── CMakeLists.txt
├── package.xml
├── src/
│   └── position_solver_node.cpp
├── srv/
│   └── EnhancePosition.srv
├── launch/
│   └── position_enhancer.launch.py
├── scripts/
│   ├── position_solver_client_node.py
│   ├── position_solver_client_node_v1.py
│   ├── position_solver_client_node_v2.py
│   ├── position_solver_client_node_v3.py
│   └── position_solver_non_blocking_client_node.py
└── README.md
```

## Dependencies

### Required ROS2 Packages
- `rclcpp`
- `geometry_msgs`
- `std_msgs`
- `tf2`
- `tf2_geometry_msgs`
- `gtsam_msgs`

### Required System Dependencies
- `GTSAM`4.2 (Georgia Tech Smoothing and Mapping library)
- `Eigen3`

## Installation

1. **Clone and build the package:**
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   git clone https://github.com/NadimArubai/position_enhancer.git
   cd ~/ros2_ws
   colcon build --packages-select position_enhancer
   source install/setup.bash
   ```

2. **Install GTSAM:**
   ```bash
   # Ubuntu/Debian
   sudo apt-get install libgtsam-dev libgtsam-unstable-dev
   
   # Or build from source:
   git clone https://github.com/borglab/gtsam.git
   mkdir gtsam/build && cd gtsam/build
   cmake .. && make -j$(nproc)
   sudo make install
   ```

## Usage

### Starting the Service

**Method 1: Using launch file**
```bash
ros2 launch position_enhancer position_enhancer.launch.py
```

**Method 2: Direct node execution**
```bash
ros2 run position_enhancer position_solver_node
```

### Service Interface

**Service Name:** `/enhance_position`

**Service Type:** `gtsam_msgs/srv/EnhancePosition`

**Request Fields:**
- `robot_poses[]`: Array of geometry_msgs/Pose (robot poses when observations were made)
- `ranges[]`: Array of float64 (measured ranges to object)
- `bearings[]`: Array of float64 (measured bearings to object)
- `range_uncertainties[]`: Array of float64 (range measurement uncertainties)
- `bearing_uncertainties[]`: Array of float64 (bearing measurement uncertainties)

**Response Fields:**
- `success`: bool (whether optimization succeeded)
- `message`: string (status message)
- `enhanced_pose`: geometry_msgs/Pose (optimized object position)
- `pose_uncertainty_x`: float64 (standard deviation in x direction)
- `pose_uncertainty_y`: float64 (standard deviation in y direction)
- `pose_uncertainty_theta`: float64 (standard deviation in orientation)

### Client Examples

Several client implementations are provided:

1. **Basic client** (`position_solver_client_node.py`): Simple synchronous client
2. **Debug client** (`position_solver_client_node_v3.py`): Enhanced debugging with service discovery
3. **Non-blocking client** (`position_solver_non_blocking_client_node.py`): Asynchronous client for multiple concurrent requests

**Example client usage:**
```bash
ros2 run position_enhancer position_solver_client_node.py
```

## Configuration Parameters

The service node supports the following parameters:

- `default_range_noise` (default: 0.1): Default range uncertainty when not specified
- `default_bearing_noise` (default: 0.05): Default bearing uncertainty when not specified  
- `optimization_iterations` (default: 100): Maximum optimization iterations
- `optimization_tolerance` (default: 1e-5): Optimization convergence tolerance

Set parameters via launch file or command line:
```bash
ros2 run position_enhancer position_solver_node --ros-args -p default_range_noise:=0.2 -p default_bearing_noise:=0.1
```

## Algorithm Details

The package uses a factor graph optimization approach:

1. **Input Validation**: Checks for consistent array sizes and minimum observations
2. **Initial Estimate**: Computes rough position estimate by averaging observations
3. **Factor Graph Construction**: Creates bearing-range factors for each observation
4. **Optimization**: Uses Levenberg-Marquardt optimizer to find optimal solution
5. **Uncertainty Estimation**: Computes marginal covariances for uncertainty bounds

## Performance Considerations

- **Minimum Observations**: At least 2 observations recommended for meaningful triangulation
- **Computational Cost**: Optimization time increases with number of observations
- **Memory Usage**: Stateless design processes each request independently

## Troubleshooting

### Common Issues

1. **Service not found**: Ensure the service node is running and check service name
2. **GTSAM not found**: Verify GTSAM installation and CMake configuration
3. **Optimization failures**: Check input data quality and consider increasing default uncertainties

### Debugging

Use the debug client (`v3`) to list available services:
```bash
ros2 run position_enhancer position_solver_client_node_v3.py
```

## License

Apache License 2.0

## Authors

- Nadim Arubai - nadim.arubai@gmail.com

## Acknowledgments

- GTSAM development team for the excellent optimization library
- ROS2 community for the robust middleware framework
