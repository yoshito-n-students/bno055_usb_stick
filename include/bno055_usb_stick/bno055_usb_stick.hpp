#ifndef BNO055_USB_STICK_BNO055_USB_STICK_HPP
#define BNO055_USB_STICK_BNO055_USB_STICK_HPP

#include <algorithm>
#include <deque>
#include <sstream>
#include <string>

#include <ros/console.h>
#include <ros/duration.h>
#include <ros/names.h>
#include <ros/param.h>

#include <bno055_usb_stick/constants.hpp>
#include <bno055_usb_stick/decoder.hpp>
#include <bno055_usb_stick/match_conditions.hpp>
#include <bno055_usb_stick_msgs/Output.h>

#include <boost/asio/deadline_timer.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/asio/read_until.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/asio/streambuf.hpp>
#include <boost/asio/write.hpp>
#include <boost/bind.hpp>
#include <boost/cstdint.hpp>
#include <boost/function.hpp>
#include <boost/system/error_code.hpp>
#include <boost/system/system_error.hpp>

namespace bno055_usb_stick {

class BNO055USBStick {
public:
  typedef boost::function< void(const bno055_usb_stick_msgs::Output &) > Callback;

public:
  BNO055USBStick(boost::asio::io_service &asio_service, const Callback &callback,
                 const std::string &ns = "~")
      : port_(ros::param::param< std::string >(ros::names::append(ns, "port"), "/dev/ttyACM0")),
        timeout_(ros::param::param(ros::names::append(ns, "timeout"), 1.)),
        mode_(ros::param::param< std::string >(ros::names::append(ns, "mode"), "ndof")),
        serial_(asio_service), timer_(asio_service), callback_(callback), decoder_(ns) {
    start();
  }

  virtual ~BNO055USBStick() { stop(); }

private:
  void start() {
    // stop the previous operaiton if there
    stop();

    // setup the serial port
    try {
      serial_.open(port_);

      typedef boost::asio::serial_port Serial;
      serial_.set_option(Serial::baud_rate(115200));
      serial_.set_option(Serial::flow_control(Serial::flow_control::none));
      serial_.set_option(Serial::parity(Serial::parity::none));
      serial_.set_option(Serial::stop_bits(Serial::stop_bits::one));
      serial_.set_option(Serial::character_size(8));
    } catch (const boost::system::system_error &error) {
      // retry if something is wrong
      ROS_ERROR_STREAM("start: " << error.what());
      restart();
      return;
    }

    // pack commands
    commands_.clear();
    if (mode_ == "ndof") {
      for (const boost::uint8_t **command = Constants::toNDOFCommands(); *command; ++command) {
        commands_.push_back(*command);
      }
    } else if (mode_ == "imu") {
      for (const boost::uint8_t **command = Constants::toIMUCommands(); *command; ++command) {
        commands_.push_back(*command);
      }
    } else {
      ROS_WARN_STREAM("Unknown mode \""
                      << mode_ << "\" was given. Will use the default mode \"ndof\" instead.");
      for (const boost::uint8_t **command = Constants::toNDOFCommands(); *command; ++command) {
        commands_.push_back(*command);
      }
    }
    for (const boost::uint8_t **command = Constants::startStreamCommands(); *command; ++command) {
      commands_.push_back(*command);
    }

    // trigger send packed commands
    startSendCommand();
  }

  void startSendCommand() {
    if (commands_.empty()) {
      ROS_ERROR("startSendCommand: No command in the queue");
      restart();
      return;
    }

    // trigger send the top command in the queue
    const boost::uint8_t *command(commands_.front());
    boost::asio::async_write(serial_,
                             boost::asio::buffer(command, Constants::getCommandLength(command)),
                             boost::bind(&BNO055USBStick::handleSendCommand, this, _1, _2));

    // schedule restarting in case of timeout
    startWaitDeadline(&BNO055USBStick::restart);
  }

  void handleSendCommand(const boost::system::error_code &error, const std::size_t bytes) {
    // cancel the timeout action
    cancelWaitDeadline();

    if (error) {
      ROS_ERROR_STREAM("handleSendCommand: " << error.message());
      restart();
      return;
    }

    // pop the top command from the queue
    // dumpWritten("handleSendCommand: written: ", bytes);
    commands_.pop_front();

    // trigger wait the response for the command
    startWaitResponse();
  }

  void startWaitResponse() {
    // trigger read a responce
    boost::asio::async_read_until(serial_, buffer_, ResponseCondition(),
                                  boost::bind(&BNO055USBStick::handleWaitResponse, this, _1, _2));

    // schedule restarting in case of timeout
    startWaitDeadline(&BNO055USBStick::restart);
  }

  void handleWaitResponse(const boost::system::error_code &error, const std::size_t bytes) {
    // cancel the timeout action
    cancelWaitDeadline();

    if (error) {
      ROS_ERROR_STREAM("handleWaitResponse: " << error.message());
      restart();
      return;
    }

    // clear the read response (cannot parse it because the protocol is unknown...)
    // dumpRead("handleWaitResponse: read: ", bytes);
    buffer_.consume(bytes);

    // trigger send the next command, or wait data stream
    if (!commands_.empty()) {
      startSendCommand();
    } else {
      startWaitData();
    }
  }

  void startWaitData() {
    // trigger read a data
    boost::asio::async_read_until(serial_, buffer_, DataCondition(),
                                  boost::bind(&BNO055USBStick::handleWaitData, this, _1, _2));

    // schedule restarting in case of timeout
    startWaitDeadline(&BNO055USBStick::restart);
  }

  void handleWaitData(const boost::system::error_code &error, const std::size_t bytes) {
    // cancel the timeout action
    cancelWaitDeadline();

    if (error) {
      ROS_ERROR_STREAM("handleWaitData: " << error.message());
      restart();
      return;
    }

    // decode the received data and execute the user callback
    // dumpRead("handleWaitData: read: ", bytes);
    if (callback_) {
      const boost::uint8_t *data_end(
          boost::asio::buffer_cast< const boost::uint8_t * >(buffer_.data()) + bytes);
      const boost::uint8_t *data_begin(data_end - Constants::DAT_LEN);
      const bno055_usb_stick_msgs::Output output(decoder_.decode(data_begin));
      callback_(output);
    }

    // clear the parsed data
    buffer_.consume(bytes);

    // trigger wait the next data
    startWaitData();
  }

  void stop() {
    // finalize the serial port
    try {
      if (serial_.is_open()) {
        serial_.close();
      }
    } catch (const boost::system::system_error &error) {
      // just print the error because can do nothing further
      ROS_ERROR_STREAM("stop: " << error.what());
    }
  }

  void restart() {
    // schedule start a new operation
    startWaitDeadline(&BNO055USBStick::start);
  }

  void startWaitDeadline(void (BNO055USBStick::*handler)()) {
    timer_.expires_from_now(timeout_.toBoost());
    timer_.async_wait(boost::bind(&BNO055USBStick::handleWaitDeadline, this, _1, handler));
  }

  void cancelWaitDeadline() { timer_.cancel(); }

  void handleWaitDeadline(const boost::system::error_code &error,
                          void (BNO055USBStick::*handler)()) {
    // make sure this callback is called by a deadline expiration
    if (error == boost::asio::error::operation_aborted) {
      // ROS_INFO("handleWaitDeadline: the deadline disabled");
      return;
    } else if (error) {
      ROS_ERROR_STREAM("handleWaitDeadline: " << error.message());
      return;
    }

    // try to cancel all operations on the serial port
    try {
      if (serial_.is_open()) {
        serial_.cancel();
      }
    } catch (const boost::system::system_error &error_on_cancel) {
      ROS_ERROR_STREAM("handleWaitDeadline: " << error_on_cancel.what());
    }

    // execute the given handler
    if (handler) {
      (this->*handler)();
    }
  }

  void dumpWritten(const std::string &prefix, const std::size_t bytes) {
    std::ostringstream oss;
    const boost::uint8_t *begin(commands_.front());
    const boost::uint8_t *end(begin + bytes);
    for (const boost::uint8_t *c = begin; c != end; ++c) {
      oss << "0x" << std::setw(2) << std::setfill('0') << std::hex << int(*c) << " ";
    }
    ROS_INFO_STREAM(prefix << oss.str());
  }

  void dumpRead(const std::string &prefix, const std::size_t bytes) {
    std::ostringstream oss;
    const boost::uint8_t *begin(boost::asio::buffer_cast< const boost::uint8_t * >(buffer_.data()));
    const boost::uint8_t *end(begin + bytes);
    for (const boost::uint8_t *c = begin; c != end; ++c) {
      oss << "0x" << std::setw(2) << std::setfill('0') << std::hex << int(*c) << " ";
    }
    ROS_INFO_STREAM(prefix << oss.str());
  }

private:
  // parameters
  const std::string port_;
  const ros::Duration timeout_;
  const std::string mode_;

  // buffers
  std::deque< const boost::uint8_t * > commands_;
  boost::asio::streambuf buffer_;

  // async objects
  boost::asio::serial_port serial_;
  boost::asio::deadline_timer timer_;

  // callback given by the user
  const Callback callback_;

  // orientation and sensor decoder
  Decoder decoder_;
};
}

#endif // BNO055_USB_STICK_BNO055_USB_STICK_HPP