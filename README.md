# PiRacer ROS2
ROS2 package for [Waveshare PiRacer](https://www.waveshare.com/piracer-ai-kit.htm).

## I2C Devices
* `0x40` PCA9685 (steering servo control)
  * steering servo connected to channel `0`
  * compatible with Adafruit PCA9685 Python library
* `0x41` Unknown
* `0x3c` SSD1306 OLED display
* `0x60` PCA9685 (motor control)
  * motor control connected to channel `3`
  * compatible with Adafruit PCA9685 Python library
  * compatible with Adafruit motor hat
    * use `motor3`
* `0x70` Unknown

## Launching

### Launch hierarchy
Below is the hierarchy of the launch files. `hardware_nodes` is necessary for any of the other launch files to do
anything, but for practical purposes it should never need to be launched by itself.
```mermaid
    flowchart TD
        full_stack(full_stack) --> ackermann_control(ackermann_control)
            ackermann_control -->|launch_hardware| hardware_nodes
            ackermann_control --> ackermann_controller([ackermann_controller])
            ackermann_control -->|launch_bridge| twist_bridge(twist_bridge)
        
        full_stack --> hardware_nodes(hardware_nodes)
            hardware_nodes --> display_driver([display_driver])
            hardware_nodes --> power_monitor_driver([power_monitor_driver])
            hardware_nodes --> steering_driver([steering_driver])
            hardware_nodes --> throttle_driver([throttle_driver])
        
        full_stack --> open_loop_control(open_loop_control)
            open_loop_control -->|launch_ackermann| ackermann_control
            open_loop_control --> arc_behavior([arc_behavior])
            open_loop_control --> straight_behavior([straight_behavior])
        
        full_stack --> teleop_control(teleop_control)
            teleop_control -->|launch_hardware|hardware_nodes
            teleop_control --> teleop_controller([teleop_controller])
        
        full_stack --> v2x_node([v2x_node])
        full_stack --> autonomy_manager([autonomy_manager])
        full_stack --> vive_driver([vive_driver])

        full_stack -->|launch_bridge|control_bridge(control_bridge)
    
        classDef launch_file fill:#D5E8D4,stroke:#82B366,color:black;
        classDef bridge_launch_file fill:#DAE8FC,stroke:#6C8EBF,color:black;
        classDef nodes fill:#FFFFFF,stroke:#000000,color:black;
        
        class display_driver,power_monitor_driver,steering_driver,throttle_driver,ackermann_controller,teleop_controller,arc_behavior,straight_behavior,v2x_node,autonomy_manager,vive_driver nodes;
        class ackermann_control,hardware_nodes,open_loop_control,teleop_control,full_stack launch_file;
        class twist_bridge,control_bridge bridge_launch_file;
```
### Environment Variables
These need to be set *before* launching the code for the first time.
1. `CAR_ID`: ID of the car, will be concatenated with string `car` to set the ROS namespace. 
Example values: `1`, `2`, `3` for `car1`, `car2`, or `car3` respectfully.
1. `MQTT_BROKER_URI`: URI of MQTT broker to be used by `autonomy_manager` and `v2x_node`. 
Defaults to `127.0.0.1` if not set.

Example of setting `MQTT_BROKER_URI`: `export MQTT_BROKER_URI=127.0.0.1` 
For long-term deployment, setting these in `.bashrc` is recommended.

### Launch file options
1. `full_stack.launch.py`: Launches all possible control modes at once, as well as the autonomy_manager.
2. `open_loop_control.py`: Launches the open loop controllers and the ackermann_controller node.
3. `ackermann_control.launch.py`: For use with the twist mode of the mqtt_bridge package.
4. `teleop_control.launch.py`: For use with a gamepad / controller.

### Command-line arguments
1. `config_file`: Full path to configuration file.
    * Default value: `piracer/config/default_config.yaml`
    * These config files can be found at the dedicated 
    [piracer_configs](https://github.com/vcuopencity/piracer_configs-internal) repo.

1. `launch_bridge`: Controls the launching of bridges from the `mqtt_bridge` package.
    * Default value: `True`
    * This should be left as true unless the bridges are being launched manually elsewhere.
  
### Launch examples
The following examples will only work after the package has already been successfully built using `colcon build 
--packages-select piracer` and the install directory sourced, via `source install/setup.bash`. Also assuming the 
[environment variables](#environment-variables) have been set.
1. `ros2 launch piracer full_stack.launch.py config_file:="/home/ubuntu/dev_ws/src/piracer_configs-internal/car1_config.yaml"`
    * Launches every control mode and the autonomy manager, using `car1_config.yaml`.
    * Assuming `CAR_ID=1`.
1. `ros2 launch piracer teleop_control.launch.py config_file:="/home/ubuntu/dev_ws/src/piracer_configs-internal/car2_config.yaml"`
    * Launches the controller-driven mode, using `car2_config.yaml`.
    * Assuming `CAR_ID=2`.
    
```yaml
car2:
  autonomy_manager:
    ros__parameters:
 #...
```

## Usage

### Control Modes
The following are the currently implemented control modes. Switching between these is handled within the `autonomy_manager`
node, using ROS2 services. The commands from the `command` bridge to enable these modes are strings and are 
case-insensitive. The **default** can be set through the `initial_mode` parameter for the `autonomy_manager` node,
as seen in the `/config/default_config.yaml` file.

1. `direct`: `twist` messages containing steering_angle and throttle commands are sent from the 
   `twist` bridge and parsed by the `ackermann_controller` node.
   * Enabled via sending the string `direct` through the command bridge
2. `auto`: Does nothing as currently implemented.
   * Enabled via sending the string `auto` through the command bridge
3. `experiment`: Performs whatever experiment is currently being worked through.
   * Enabled via sending the string `experiment` through the command bridge
   
### Bridges
The following are bridges from the `mqtt_bridge` package that must be launched and in the appropriate namespace (have 
the same `agent_name`) for cars to function  properly for `direct` mode and mode switching. They are automatically
launched as needed unless `launch_bridge` is set to `False`.

1. `twist`: sends `twist` messages to the `ackermann_controller` node while the car is in `direct` mode
   * As the `autonomy_mode` node relies on a service that lives in the `twist` bridge to enable / disable 
     `direct` mode, it hangs until the `twist` bridge is launched, outputting to the logger that it is
     waiting for the service to be available.
1. `command`: allows for mode switching, as previously described in the [Control Modes](#control-modes) section.


### Spring 2023 Demo

1. Pushing code to the car, using VSCode to develop, run this command from the 
`piracer-internal` folder on the development machine: `rsync -rlptzv --progress --exclude=.git . "ubuntu@piracer-01:~/dev_ws/src/
piracer-internal"`

1. Ensure the [environment variables](#environment-variables) have been set on the car. 

1. Running the code on the car: 
`ros2 launch piracer full_stack.launch.py config_file:="/home/ubuntu/dev_ws/src/piracer_configs-internal/car1_config.yaml"`

1. Entering the experiment state (making it go forward and enter the following state machine on 'straight' mode): 
`ros2 topic pub -1 /car1/cmd/in cmd_msgs/msg/Command "{operational_mode: experiment}"`

1. Stopping the vehicle after experiment is completed (by putting it into direct command mode):
`ros2 topic pub -1 /car1/cmd/in cmd_msgs/msg/Command "{operational_mode: direct}"`
