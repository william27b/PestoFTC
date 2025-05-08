# Core Components

PestoFTC is built around several core components that work together to provide a comprehensive robotics development experience.

## Main Components

### 1. Vision System
Located in the `vision` package, this component provides:
- Advanced image processing capabilities
- Object detection and tracking
- Camera management utilities
- Vision pipeline implementations

### 2. Drivebase Systems
Found in the `drivebases` package, offering:
- Various drivebase implementations
- Motion control systems
- Path following capabilities
- Odometry support

### 3. Device Management
In the `devices` package, providing:
- Hardware device abstractions
- Sensor management
- Motor and servo control
- Device configuration utilities

### 4. Command System
Located in the `commands` package, featuring:
- Command-based programming model
- Command scheduling
- Command groups
- Command lifecycle management

### 5. Tuning Tools
In the `tuners` package, offering:
- PID tuning utilities
- Parameter optimization
- Real-time tuning capabilities
- Data logging and analysis

### 6. Geometric Utilities
Found in the `geometries` package, providing:
- 2D and 3D geometry classes
- Path planning utilities
- Spatial transformations
- Geometric calculations

### 7. Algorithms
In the `algorithms` package, featuring:
- Path planning algorithms
- Motion profiling
- State estimation
- Control algorithms

## Integration
These components are designed to work together seamlessly. For example:
- The vision system can feed data to the drivebase system
- The command system can utilize device management
- Tuning tools can work with any component
- Geometric utilities support all other components

## Best Practices
1. Start with the basic components you need
2. Gradually integrate more complex features
3. Use the command system for robot control
4. Utilize tuning tools for optimization
5. Implement vision when needed for autonomous tasks 