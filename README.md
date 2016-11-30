# bno055_usb_stick
A ROS driver of Bosch BNO055 USB Stick

## Device
Bosch BNO055 USB Stick http://www.mouser.jp/new/bosch/bosch-bno055-usb-stick/

## Dependencies
bno055_usb_stick_msgs https://github.com/yoshito-n-students/bno055_usb_stick_msgs

## Published Topics
output (bno055_usb_stick_msgs/Output)

imu (sensor_msgs/Imu)

pose (geometry_msgs/PoseStamped)
* associated with the fixed frame

magnetic_field (sensor_msgs/MagneticField)

temperature (sensor_msgs/Temperature)

## Provided tf Transforms
fixed -> bno055

## Parameters
~fixed_frame_id (string, default: fixed)

~frame_id (string, default: bno055)

~port (string, default: /dev/ttyACM0)

~timeout (float, default: 1.)
* timeout on serial port communication in seconds

~mode (string, default: ndof)
* nodf: estimate absolute orientation with an accelerometer, gyroscope, and magnetometer
* imu: estimate relative orientation with an accelerometer and gyroscope
