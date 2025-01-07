# My Rover react frontend application

Communication between robot and react app handled by AWS IOT Core.
Video streamed from robot handled by AWS Kinesis Web RTC viewer (custom).

## Controlling the robot

To move the robot we need to send twist message in following format to robot using IOT Core.
{
"linear": {
"x": 0,
"y": 0,
"z": 0,
},
"angular": {
"x": 0,
"y": 0,
"z": 0,
}
}

## Controlling the robot using mouse -> joystick component

react-joystick-component is used together with mouse to get x,y cordinates
ie.
forward --> x
backward -> -x
left --> z
right -> -z

### Implementation 1:

For Joystick control convert y --> linear x and x --> angular z.
ie send linear X and angular Z together

### Implementation 2:

The first implementation was sending linear and angular together which was causing issues with Wave Rover.
So 2nd implementation sends either linear or angular velocity

## Controlling the robot using actual joystick component

-- TO DO
