#include <geometry_msgs/PoseStamped.h>
#include <ros/node_handle.h>
#include <ros/init.h>
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

void publish(const bno055_usb_stick_msgs::Output &output) {
    imu_pub.publish(bus::Decoder::toImuMsg(output));
    pose_pub.publish(bus::Decoder::toPoseMsg(output));
    tf_pub->sendTransform(bus::Decoder::toTfMsg(output));
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "bno055_usb_stick_node");
    ros::NodeHandle nh;

    imu_pub = nh.advertise< sensor_msgs::Imu >("/imu", 1);
    pose_pub = nh.advertise< geometry_msgs::PoseStamped >("/pose", 1);
    tf_pub.reset(new tf::TransformBroadcaster);

    boost::asio::io_service asio_service;

    bus::BNO055USBStick device(asio_service, publish);

    while (nh.ok()) {
        asio_service.run_one();
    }

    return 0;
}