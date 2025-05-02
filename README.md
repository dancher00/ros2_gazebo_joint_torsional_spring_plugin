# ROS2 Gazebo Joint Torsional Spring Plugin

A Gazebo plugin for ROS2 that adds torsional spring behavior to robot joints.

## Installation

```bash
# Create a ROS2 workspace (if you don't have one)
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone the repository
git clone https://github.com/USERNAME/ros2_gazebo_joint_torsional_spring_plugin.git

# Build the package
cd ~/ros2_ws
colcon build --packages-select ros2_gazebo_joint_torsional_spring_plugin

# Source the environment
source install/setup.bash
```

## Usage

Add the plugin to your URDF model:

```xml
<robot>
  <joint name="your_joint" type="revolute">
    <!-- joint properties -->
    <limit lower="-0.5" upper="0.5" effort="10" velocity="1.0" />
  </joint>
    
  <gazebo>
    <plugin name="your_joint_torsional_spring" filename="libgazebo_joint_torsional_spring.so">
      <kx>20.0</kx>            <!-- Spring stiffness coefficient (N·m/rad) -->
      <set_point>0.0</set_point> <!-- Equilibrium point (radians) -->
      <joint>your_joint</joint>  <!-- Joint name -->
    </plugin>
  </gazebo>
</robot>
```

## Parameters

| Parameter | Type | Description |
|-----------|------|-------------|
| `kx` | double | Spring stiffness coefficient in N·m/rad |
| `set_point` | double | Equilibrium angle in radians |
| `joint` | string | Name of the joint to apply the spring to |

## Troubleshooting

If the spring doesn't work:

1. Ensure the joint's `effort` and `velocity` limit is greater than zero
2. Try increasing the `kx` value for a stronger spring
3. Check that the plugin loads successfully (see Gazebo logs)
4. Verify the joint name is correct

## License

MIT License
