# Steps to start UGV wave rover

## Launch bringup (complete drivers are launched)

export UGV_MODEL=rasp_rover
ros2 launch ugv_bringup bringup_rover.launch.py 

## The individual modules can be run by executing below commands
## Drive the car 

ros2 run ugv_bringup ugv_driver

## other
ros2 run ugv_bringup ugv_bringup

To echo topics
ros2 topic list


## Behavior control

ros2 run ugv_tools behavior_ctrl

    Forward data unit meters
        
    ```jsx
    ros2 action send_goal /behavior ugv_interface/action/Behavior "{command: '[{\"T\": 1, \"type\": \"drive_on_heading\", \"data\": 0.1}]'}"
    ```
    
    Back data unit meters
    
    ```jsx
    ros2 action send_goal /behavior ugv_interface/action/Behavior "{command: '[{\"T\": 1, \"type\": \"back_up\", \"data\": 0.1}]'}"
    ```
    
    Rotation data unit degree ,positive number left rotation, negative number right rotation
    
    ```jsx
    ros2 action send_goal /behavior ugv_interface/action/Behavior "{command: '[{\"T\": 1, \"type\": \"spin\", \"data\": -1}]'}"
    ```
    
    Stop
    
    ```jsx
    ros2 action send_goal /behavior ugv_interface/action/Behavior "{command: '[{\"T\": 1, \"type\": \"stop\", \"data\": 0}]'}"
    ```

## Launch camera

ros2 launch ugv_vision camera.launch.py
