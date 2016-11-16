#include <geometry_msgs/PoseStamped.h>
#include <ros/node_handle.h>
#include <ros/init.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Temperature.h>
#include <tf/transform_broadcaster.h>

#include <bno055_usb_stick/bno055_usb_stick.hpp>
#include <bno055_usb_stick/decoder.hpp>
#include <bno055_usb_stick_msgs/Output.h>

#include <boost/asio/io_service.hpp>
#include <boost/shared_ptr.hpp>

namespace bus = bno055_usb_stick;

ros::Publisher imu_pub;
ros::Publisher pose_pub;
boost::shared_ptr<tf::TransformBroadcaster> tf_pub;
ros::Publisher mag_pub;
ros::Publisher temp_pub;

void publish(const bno055_usb_stick_msgs::Output &output) {
    imu_pub.publish(bus::Decoder::toImuMsg(output));
    pose_pub.publish(bus::Decoder::toPoseMsg(output));
    tf_pub->sendTransform(bus::Decoder::toTfMsg(output));
    mag_pub.publish(bus::Decoder::toMagMsg(output));
    temp_pub.publish(bus::Decoder::toTempMsg(output));
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "bno055_usb_stick_node");
    ros::NodeHandle nh;

    imu_pub = nh.advertise< sensor_msgs::Imu >("imu", 1);
    pose_pub = nh.advertise< geometry_msgs::PoseStamped >("pose", 1);
    tf_pub.reset(new tf::TransformBroadcaster);
    mag_pub = nh.advertise< sensor_msgs::MagneticField >("magnetic_field", 1);
    temp_pub = nh.advertise< sensor_msgs::Temperature >("temperature", 1);

    boost::asio::io_service asio_service;

    bus::BNO055USBStick device(asio_service, publish);

    while (nh.ok()) {
        asio_service.run_one();
    }

    return 0;
}