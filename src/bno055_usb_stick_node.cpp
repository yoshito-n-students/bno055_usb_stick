
#include <ros/init.h>

#include <bno055_usb_stick/bno055_usb_stick.hpp>

#include <boost/asio/io_service.hpp>

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "bno055_usb_stick_node");

    boost::asio::io_service asio_service;

    bno055_usb_stick::BNO055USBStick device(asio_service);

    asio_service.run();

    return 0;
}