# DeepRacer Gazebo System Plugin

Provides ROS 2 services for manipulating visual elements, managing model/link states, and controlling physics in Gazebo simulations. It serves as the core interface between ROS 2 nodes and the Gazebo simulation environment for DeepRacer training and evaluation scenarios.

## Overview

The DeepRacer Gazebo System Plugin is a Gazebo Harmonic system plugin that exposes 21 ROS 2 services for:
- **Visual Manipulation**: Colors, transparency, visibility, poses, and meshes
- **Model/Link State Management**: Position, orientation, and velocity control
- **Model Properties**: Retrieve detailed model structure information
- **Physics Control**: Pause/unpause simulation physics
- **Light Management**: Discovery and enumeration of simulation lights

## Services Provided

### Visual Discovery Services

#### `/get_light_names`
**Type**: `deepracer_msgs/srv/GetLightNames`
- **Purpose**: Retrieve names of all lights in the simulation
- **Request**: Empty
- **Response**: Array of light names, success status, status message

```bash
ros2 service call /get_light_names deepracer_msgs/srv/GetLightNames
```

#### `/get_visual_names`
**Type**: `deepracer_msgs/srv/GetVisualNames`
- **Purpose**: Get visual names for specified links (or all links if empty)
- **Request**: Array of link names (optional)
- **Response**: Arrays of visual names and corresponding link names

```bash
ros2 service call /get_visual_names deepracer_msgs/srv/GetVisualNames "{link_names: ['racecar::base_link']}"
```

### Visual Property Services

#### `/get_visual`
**Type**: `deepracer_msgs/srv/GetVisual`
- **Purpose**: Retrieve complete properties of a specific visual
- **Request**: Link name, visual name
- **Response**: Pose, colors (ambient, diffuse, specular, emissive), transparency, visibility, geometry info

```bash
ros2 service call /get_visual deepracer_msgs/srv/GetVisual "{link_name: 'racecar::base_link', visual_name: 'base_visual'}"
```

#### `/get_visuals`
**Type**: `deepracer_msgs/srv/GetVisuals`
- **Purpose**: Bulk retrieval of visual properties for multiple visuals
- **Request**: Arrays of link names and visual names
- **Response**: Arrays of all visual properties with individual status indicators

```bash
ros2 service call /get_visuals deepracer_msgs/srv/GetVisuals "{link_names: ['link1', 'link2'], visual_names: ['visual1', 'visual2']}"
```

### Visual Manipulation Services

#### `/set_visual_color`
**Type**: `deepracer_msgs/srv/SetVisualColor`
- **Purpose**: Update color properties of a single visual
- **Request**: Link name, visual name, color values (ambient, diffuse, specular, emissive)
- **Response**: Success status and message

```bash
ros2 service call /set_visual_color deepracer_msgs/srv/SetVisualColor "{link_name: 'racecar::base_link', visual_name: 'base_visual', diffuse: {r: 1.0, g: 0.0, b: 0.0, a: 1.0}}"
```

#### `/set_visual_colors`
**Type**: `deepracer_msgs/srv/SetVisualColors`
- **Purpose**: Bulk color updates for multiple visuals
- **Request**: Arrays of link names, visual names, and color values
- **Response**: Success status and message

#### `/set_visual_transparency`
**Type**: `deepracer_msgs/srv/SetVisualTransparency`
- **Purpose**: Set transparency of a single visual
- **Request**: Link name, visual name, transparency value (0.0-1.0)
- **Response**: Success status and message

```bash
ros2 service call /set_visual_transparency deepracer_msgs/srv/SetVisualTransparency "{link_name: 'racecar::base_link', visual_name: 'base_visual', transparency: 0.5}"
```

#### `/set_visual_transparencies`
**Type**: `deepracer_msgs/srv/SetVisualTransparencies`
- **Purpose**: Bulk transparency updates for multiple visuals
- **Request**: Arrays of link names, visual names, and transparency values
- **Response**: Success status and message

#### `/set_visual_visible`
**Type**: `deepracer_msgs/srv/SetVisualVisible`
- **Purpose**: Show/hide a single visual
- **Request**: Link name, visual name, visibility boolean
- **Response**: Success status and message

```bash
ros2 service call /set_visual_visible deepracer_msgs/srv/SetVisualVisible "{link_name: 'racecar::base_link', visual_name: 'base_visual', visible: false}"
```

#### `/set_visual_visibles`
**Type**: `deepracer_msgs/srv/SetVisualVisibles`
- **Purpose**: Bulk visibility updates for multiple visuals
- **Request**: Arrays of link names, visual names, and visibility values
- **Response**: Arrays of individual status results

### Visual Pose and Geometry Services

#### `/set_visual_pose`
**Type**: `deepracer_msgs/srv/SetVisualPose`
- **Purpose**: Update pose (position and orientation) of a single visual
- **Request**: Link name, visual name, pose (position + orientation)
- **Response**: Success status and message

```bash
ros2 service call /set_visual_pose deepracer_msgs/srv/SetVisualPose "{link_name: 'racecar::base_link', visual_name: 'base_visual', pose: {position: {x: 1.0, y: 0.0, z: 0.5}}}"
```

#### `/set_visual_poses`
**Type**: `deepracer_msgs/srv/SetVisualPoses`
- **Purpose**: Bulk pose updates for multiple visuals
- **Request**: Arrays of link names, visual names, and poses
- **Response**: Success status and message

#### `/set_visual_mesh`
**Type**: `deepracer_msgs/srv/SetVisualMesh`
- **Purpose**: Update mesh geometry of a single visual
- **Request**: Link name, visual name, filename, scale
- **Response**: Success status and message

```bash
ros2 service call /set_visual_mesh deepracer_msgs/srv/SetVisualMesh "{link_name: 'racecar::base_link', visual_name: 'base_visual', filename: 'file://path/to/mesh.dae', scale: {x: 1.0, y: 1.0, z: 1.0}}"
```

#### `/set_visual_meshes`
**Type**: `deepracer_msgs/srv/SetVisualMeshes`
- **Purpose**: Bulk mesh updates for multiple visuals
- **Request**: Arrays of link names, visual names, filenames, and scales
- **Response**: Success status and message

### Model State Services

#### `/get_model_states`
**Type**: `deepracer_msgs/srv/GetModelStates`
- **Purpose**: Retrieve pose and velocity of models
- **Request**: Arrays of model names and reference frame names
- **Response**: Array of model states with pose and twist information

```bash
ros2 service call /get_model_states deepracer_msgs/srv/GetModelStates "{model_names: ['racecar'], relative_entity_names: ['world']}"
```

#### `/set_model_states`
**Type**: `deepracer_msgs/srv/SetModelStates`
- **Purpose**: Update pose and velocity of models
- **Request**: Array of model states (name, pose, twist, reference frame)
- **Response**: Arrays of individual status results

```bash
ros2 service call /set_model_states deepracer_msgs/srv/SetModelStates "{model_states: [{model_name: 'racecar', pose: {position: {x: 2.0, y: 1.0, z: 0.0}}, reference_frame: 'world'}]}"
```

### Model Properties Services

#### `/get_model_properties`
**Type**: `deepracer_msgs/srv/GetModelProperties`
- **Purpose**: Retrieve detailed structural information about a model
- **Request**: Model name
- **Response**: Parent model, canonical body name, body names, geometry names, joint names, child models, static flag, success status

```bash
ros2 service call /get_model_properties deepracer_msgs/srv/GetModelProperties "{model_name: 'racecar'}"
```

**Response Fields**:
- `parent_model_name`: Name of the parent model (empty if top-level)
- `canonical_body_name`: Primary body name in format `model::link`
- `body_names[]`: All body/link names associated with the model
- `geom_names[]`: All geometry names in the model
- `joint_names[]`: All joint names attached to the model
- `child_model_names[]`: Names of any child models
- `is_static`: Boolean indicating if the model is static
- `success`: Operation success status
- `status_message`: Descriptive status message

### Link State Services

#### `/get_link_states`
**Type**: `deepracer_msgs/srv/GetLinkStates`
- **Purpose**: Retrieve pose and velocity of links
- **Request**: Arrays of link names and reference frame names
- **Response**: Array of link states with pose and twist information

```bash
ros2 service call /get_link_states deepracer_msgs/srv/GetLinkStates "{link_names: ['racecar::base_link'], reference_frames: ['world']}"
```

#### `/set_link_states`
**Type**: `deepracer_msgs/srv/SetLinkStates`
- **Purpose**: Update pose and velocity of links
- **Request**: Array of link states (name, pose, twist, reference frame)
- **Response**: Arrays of individual status results

### Physics Control Services

#### `/pause_physics_dr`
**Type**: `std_srvs/srv/Empty`
- **Purpose**: Pause the simulation physics
- **Request**: Empty
- **Response**: Empty

```bash
ros2 service call /pause_physics_dr std_srvs/srv/Empty
```

#### `/unpause_physics_dr`
**Type**: `std_srvs/srv/Empty`
- **Purpose**: Resume the simulation physics
- **Request**: Empty
- **Response**: Empty

```bash
ros2 service call /unpause_physics_dr std_srvs/srv/Empty
```

## Usage Examples

### Basic Visual Manipulation
```bash
# Get all light names
ros2 service call /get_light_names deepracer_msgs/srv/GetLightNames

# Change car color to red
ros2 service call /set_visual_color deepracer_msgs/srv/SetVisualColor \
  "{link_name: 'racecar::base_link', visual_name: 'base_visual', \
    diffuse: {r: 1.0, g: 0.0, b: 0.0, a: 1.0}}"

# Make car semi-transparent
ros2 service call /set_visual_transparency deepracer_msgs/srv/SetVisualTransparency \
  "{link_name: 'racecar::base_link', visual_name: 'base_visual', transparency: 0.5}"

# Hide car visual
ros2 service call /set_visual_visible deepracer_msgs/srv/SetVisualVisible \
  "{link_name: 'racecar::base_link', visual_name: 'base_visual', visible: false}"
```

### Model State Management
```bash
# Get current car position
ros2 service call /get_model_states deepracer_msgs/srv/GetModelStates \
  "{model_names: ['racecar'], relative_entity_names: ['world']}"

# Move car to new position
ros2 service call /set_model_states deepracer_msgs/srv/SetModelStates \
  "{model_states: [{model_name: 'racecar', \
    pose: {position: {x: 5.0, y: 2.0, z: 0.1}}, \
    reference_frame: 'world'}]}"

# Get detailed model structure information
ros2 service call /get_model_properties deepracer_msgs/srv/GetModelProperties \
  "{model_name: 'racecar'}"
```

### Physics Control
```bash
# Pause simulation
ros2 service call /pause_physics_dr std_srvs/srv/Empty

# Resume simulation
ros2 service call /unpause_physics_dr std_srvs/srv/Empty
```

## Configuration

### World File Integration
Add the plugin to your Gazebo world file:

```xml
<world name="deepracer_world">
  <plugin filename="libdeepracer_gazebo_system_plugin.so" 
          name="gz::sim::systems::DeepRacerGazeboSystemPlugin">
  </plugin>
  <!-- World content -->
</world>
```

### Environment Variables
Ensure the plugin library is in your Gazebo plugin path:

```bash
export GZ_SIM_SYSTEM_PLUGIN_PATH=/path/to/plugin/lib:$GZ_SIM_SYSTEM_PLUGIN_PATH
```

## Dependencies

- **ROS 2 Jazzy**
- **Gazebo Harmonic**
- **deepracer_msgs**
- **std_msgs**
- **geometry_msgs**
- **std_srvs**

## Troubleshooting

### Common Issues

1. **Service Not Available**
   ```bash
   # Check if plugin is loaded
   ros2 service list | grep -E "(visual|model|physics)"
   ```

2. **Entity Not Found Errors**
   ```bash
   # List available models/links
   gz model --list
   gz topic -e -t /world/default/model/info
   ```

3. **Plugin Not Loading**
   - Verify `GZ_SIM_SYSTEM_PLUGIN_PATH` includes plugin directory
   - Check world file plugin configuration
   - Ensure all dependencies are installed

### Debug Information

Enable debug logging:
```bash
export RCUTILS_LOGGING_SEVERITY=DEBUG
export GZ_VERBOSITY=4
```

## Testing

The plugin includes comprehensive unit tests covering all 21 services:
Note: The tests must be explicitly built by using the -DBUILD_TESTING argument when building the package.

```bash
# Build for testing
colcon build --packages-select deepracer_gazebo_system_plugin --cmake-args -DBUILD_TESTING=ON

# Run tests
colcon test --packages-select deepracer_gazebo_system_plugin

# View test results
colcon test-result --verbose
```
