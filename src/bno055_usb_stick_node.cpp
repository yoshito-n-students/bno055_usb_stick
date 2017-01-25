#include <string>

#include <geometry_msgs/PoseStamped.h>
#include <ros/node_handle.h>
#include <ros/init.h>
#include <ros/param.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Temperature.h>
#include <tf/transform_broadcaster.h>

#include <bno055_usb_stick/bno055_usb_stick.hpp>
#include <bno055_usb_stick/decoder.hpp>
#include <bno055_usb_stick_msgs/Output.h>

#include <boost/asio/io_service.hpp>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>

namespace bus = bno055_usb_stick;

ros::Publisher out_pub;
ros::Publisher imu_pub;
ros::Publisher pose_pub;
boost::shared_ptr< tf::TransformBroadcaster > tf_pub;
ros::Publisher mag_pub;
ros::Publisher temp_pub;

void publish(const bno055_usb_stick_msgs::Output &output, const std::string &fixed_frame_id) {
  out_pub.publish(output);
  imu_pub.publish(bus::Decoder::toImuMsg(output));
  pose_pub.publish(bus::Decoder::toPoseMsg(output, fixed_frame_id));
  tf_pub->sendTransform(bus::Decoder::toTFTransform(output, fixed_frame_id));
  mag_pub.publish(bus::Decoder::toMagMsg(output));
  temp_pub.publish(bus::Decoder::toTempMsg(output));
}

int main(int argc, char *argv[]) {
  // init ROS
  ros::init(argc, argv, "bno055_usb_stick_node");
  ros::NodeHandle nh;

  // load parameters
  const std::string fixed_frame_id(ros::param::param< std::string >("~fixed_frame_id", "fixed"));

  // setup publishers
  out_pub = nh.advertise< bno055_usb_stick_msgs::Output >("output", 1);
  imu_pub = nh.advertise< sensor_msgs::Imu >("imu", 1);
  pose_pub = nh.advertise< geometry_msgs::PoseStamped >("pose", 1);
  tf_pub.reset(new tf::TransformBroadcaster);
  mag_pub = nh.advertise< sensor_msgs::MagneticField >("magnetic_field", 1);
  temp_pub = nh.advertise< sensor_msgs::Temperature >("temperature", 1);

  // construct the worker
  boost::asio::io_service asio_service;
  bus::BNO055USBStick device(asio_service, boost::bind(publish, _1, fixed_frame_id));

  // run the worker
  while (nh.ok()) {
    asio_service.run_one();
  }

  return 0;
}