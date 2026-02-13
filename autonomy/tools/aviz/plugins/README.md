# AViz Plugins

This directory contains the plugin system for AViz visualizer, similar to ROS2 rviz's `rviz_default_plugins`.

## Overview

The plugin system allows for modular display types to be registered and dynamically created. Each plugin represents a type of visualization (e.g., RobotModel, PointCloud2, LaserScan) that can be added to the visualization scene.

## Architecture

### PluginFactory

The `PluginFactory` is a singleton that manages all registered display plugins. It provides:

- **Plugin Registration**: Register display plugins with metadata and factory functions
- **Plugin Creation**: Create display instances by class_id
- **Plugin Discovery**: Query available plugins and their descriptions

### Plugin Registration

Plugins are registered using the `PluginFactory::registerPlugin()` method, which takes:

1. **PluginDescription**: Metadata about the plugin (class_id, name, description, message types)
2. **DisplayCreator**: A factory function that creates instances of the display

### Plugin Description File

The `plugins_description.xml` file provides a human-readable description of all available plugins, similar to rviz's plugin description format. This file is for documentation purposes and can be used by UI tools to show available plugins.

## Usage

### Registering Plugins

All default plugins are registered by calling `RegisterDefaultPlugins()` during application initialization:

```cpp
#include "autonomy/tools/aviz/plugins/register_plugins.hpp"

int main(int argc, char** argv) {
    // ... initialization ...
    
    // Register all default plugins
    RegisterDefaultPlugins();
    
    // ... rest of application ...
}
```

### Creating Displays

To create a display instance:

```cpp
#include "autonomy/tools/aviz/plugins/plugin_factory.hpp"

auto& factory = PluginFactory::Instance();
auto display = factory.createDisplay("aviz/RobotModel", "MyRobot");
if (display) {
    display->initialize(display_context);
    display->setEnabled(true);
}
```

### Adding a New Plugin

To add a new display plugin:

1. **Create the Display Class**: Inherit from `Display` base class
   ```cpp
   class MyDisplay : public Display {
   public:
       explicit MyDisplay(const std::string& name);
       // ... implement Display interface ...
   };
   ```

2. **Register in `register_plugins.cpp`**:
   ```cpp
   void RegisterDefaultPlugins() {
       auto& factory = PluginFactory::Instance();
       
       PluginDescription desc;
       desc.class_id = "aviz/MyDisplay";
       desc.class_name = "MyDisplay";
       desc.description = "Description of my display";
       desc.base_class = "Display";
       
       factory.registerPlugin(desc, [](const std::string& name) {
           return std::make_unique<MyDisplay>(name);
       });
   }
   ```

3. **Add to `plugins_description.xml`**:
   ```xml
   <class
     name="aviz/MyDisplay"
     type="MyDisplay"
     base_class_type="Display"
   >
     <description>
       Description of my display.
     </description>
   </class>
   ```

## Available Plugins

### Implemented

- **aviz/RobotModel**: Displays robot models from URDF files
  - Supports visual and collision geometry
  - Supports alpha transparency
  - Can load from file or string

### Planned

- **aviz/PointCloud2**: Point cloud visualization
- **aviz/LaserScan**: Laser scan visualization
- **aviz/Image**: Image display
- **aviz/Grid**: Grid display
- **aviz/Path**: Path visualization
- **aviz/Odometry**: Odometry visualization
- **aviz/Marker**: Visualization markers

## Comparison with rviz

This plugin system is inspired by ROS2 rviz's plugin architecture but simplified:

- **No pluginlib**: Uses static registration instead of dynamic library loading
- **Simpler API**: Direct factory pattern instead of pluginlib's class loader
- **Same concepts**: Plugin descriptions, factory pattern, and display lifecycle

## Future Enhancements

- Dynamic plugin loading from shared libraries
- Plugin configuration persistence
- Plugin property panels in UI
- Plugin dependencies and ordering
