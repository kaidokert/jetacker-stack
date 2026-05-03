# Gazebo Pose Publisher Investigation

**Gazebo Version**: gz-sim 8.10.0

## Problem

Ground truth verification requires model pose data from Gazebo. The official `PosePublisher` plugin stops publishing after Gazebo world reset.

## Investigation Goals

1. **Phase 1**: Create working world-level pose publisher (our code, our control)
2. **Phase 2**: Understand why official PosePublisher fails after reset

## Directory Structure

```
pose_publisher/
├── upstream/           # Official PosePublisher from gz-sim 8.10.0
│   ├── PosePublisher.cc
│   └── PosePublisher.hh
├── custom/            # Our world-level version
│   ├── WorldPosePublisher.cc
│   └── WorldPosePublisher.hh
├── CMakeLists.txt     # Builds custom plugin
└── README.md          # This file
```

## Key Differences: Model-Level vs World-Level

### Upstream PosePublisher (Model-Level)
- **Attachment**: Attached to MODEL entity via SDF `<plugin>` tag
- **Lifecycle**: Tied to model - destroyed when model is destroyed
- **Reset Behavior**: FAILS - model gets recreated, plugin instance lost
- **Topic Pattern**: `/model/<model_name>/pose`
- **Use Case**: Publishing link/visual/collision poses relative to model

### Our WorldPosePublisher (World-Level)
- **Attachment**: Attached to WORLD entity via SDF `<plugin>` tag
- **Lifecycle**: Tied to world - survives model recreation
- **Reset Behavior**: SUCCESS - world persists, plugin continues
- **Topic Pattern**: `/world/<world_name>/model/<model_name>/pose`
- **Use Case**: Publishing absolute model poses for ground truth verification

## Build Instructions

Inside gazebo container:
```bash
cd /workspace/workdir/gz_plugins/pose_publisher
mkdir -p build && cd build
cmake ..
make
```

Plugin library: `build/libWorldPosePublisher.so`

## Usage in SDF

```xml
<world name="my_world">
  <plugin
    filename="libWorldPosePublisher"
    name="gz::sim::systems::WorldPosePublisher">
    <!-- Model names to publish (comma-separated) -->
    <model_names>slam_bot,jetacker</model_names>
    <!-- Update frequency in Hz (default: 50) -->
    <update_frequency>50</update_frequency>
  </plugin>

  <!-- Models -->
  <model name="slam_bot">
    <!-- ... -->
  </model>
</world>
```

## Phase 2 Investigation: Why PosePublisher Fails

**Hypothesis**: Model-level plugins are destroyed during world reset because models are recreated.

**To Test**:
1. Add verbose logging to upstream PosePublisher
2. Recompile and test with reset
3. Monitor when PostUpdate stops being called
4. Check if Configure is called again after reset
5. Determine if this is a bug or intended behavior

**Expected Finding**: Model entity gets destroyed during reset, taking the plugin instance with it. SceneBroadcaster might be special-cased to survive, but model plugins are not.

## Related Issues

- SceneBroadcaster topics not published: Earlier investigation showing `/world/*/model/*/pose` topics exist but have no publishers
