# Jetacker Configuration

Tricycle/parallel-link steering robot configuration.

## Controller Type
- Uses `tricycle_controller` or `ackermann_steering_controller`
- Front steering: single virtual joint (`front_steering_joint`)
- Rear drive: independent left/right wheel velocity

## Files
- `jetacker.urdf` - Robot description (symlink to models/jetacker/jetacker.urdf)
- `tricycle_controller.yaml` - Controller config (TBD)
- `ekf.yaml` - Sensor fusion (TBD)
- `nav2_params.yaml` - Navigation params (TBD)
- `slam_toolbox.yaml` - SLAM config (can copy from slam_bot initially)
