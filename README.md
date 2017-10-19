---
# Lysander robot
---

# Launch files

## lysander
### <span id="hector_mapping.launch"/>hector_mapping.launch
### lysander.launch
* Args
    - <span id="do_hector_mapping"/>do_hector_mapping -- default ***true***
    
        True means launch the hector_mapping node.
        
    - <span id="do_motor_controller"/>do_motor_controller -- default ***true***
    
        True means launch the motor controller node.
          
    - <span id="do_rplidar"/>do_rplidar -- default ***true***
    
        True means launch the RPLIDAR node.
      
* Setup tf 
    - base_link to laser_frame
    - base_stabalized to base_link unless [do_hector_mapping](#do_hector_mapping)
    - base_footprint_to_base_stabalized

* If [do_motor_controller](#do_motor_controller), launch [motor.launch](#motor.launch).

* If [do_rplidar](#do_rplidar), launch rplidar_ros/rplidarNode.
* 
* If [do_hector_mapping](#do_hector_mapping), launch [hector_mapping.launch](#hector_mapping.launch).s

## motor_controller
### <span id="motor.launch"/>motor.launch