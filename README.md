# bno055_usb_stick
A ROS driver of Bosch BNO055 USB Stick

## Device
Bosch BNO055 USB Stick http://www.mouser.jp/new/bosch/bosch-bno055-usb-stick/

## Dependencies
bno055_usb_stick_msgs https://github.com/yoshito-n-students/bno055_usb_stick_msgs

## Published Topics
**output** (bno055_usb_stick_msgs/Output)

**imu** (sensor_msgs/Imu)

**pose** (geometry_msgs/PoseStamped)
* associated with the fixed frame

**magnetic_field** (sensor_msgs/MagneticField)

**temperature** (sensor_msgs/Temperature)

## Provided tf Transforms
**fixed -> bno055**
* never broadcasted unless ~publish_tf is true
* frame ids can be changed by ~tf_frame_id and ~tf_child_frame_id

## Parameters
**~port** (string, default: "/dev/ttyACM0")
* path to serial port which your BNO055 USB Stick is connected

**~timeout** (float, default: 1.0)
* timeout on serial port communication in seconds

**~mode** (string, default: "ndof")
* "nodf": estimate absolute orientation with an accelerometer, gyroscope, and magnetometer
* "imu": estimate relative orientation with an accelerometer and gyroscope

**~frame_id** (string, default: "bno055")
* frame id of messages except pose and tf

**~pose_frame_id** (string, default: "fixed")
* frame id of pose messages

**~publish_tf** (bool, default: false)
* publish tf messages or not

**~tf_frame_id** (string, default: "fixed")
* parent frame id of tf messages

**~tf_child_frame_id** (string, default: "bno055")
* child frame id of tf messages

**~invert_tf** (bool, default: false)
* invert transfom in tf message or not
* frame ids in tf message will NOT be swapped

## Troubleshooting
`[ERROR] []: start: open: Permission denied`
* give permission to the current user by `sudo usermod -aG dialout $USER`. This command is for ubuntu 16.04 and may vary for other environments.