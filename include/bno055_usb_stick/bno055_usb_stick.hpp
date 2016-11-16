#ifndef BNO055_USB_STICK_BNO055_USB_STICK_HPP
#define BNO055_USB_STICK_BNO055_USB_STICK_HPP

#include <algorithm>
#include <deque>
#include <sstream>
#include <string>

#include <ros/console.h>
#include <ros/names.h>
#include <ros/param.h>

#include <bno055_usb_stick/constants.hpp>
#include <bno055_usb_stick/decoder.hpp>
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
    typedef boost::function< void(const bno055_usb_stick_msgs::Output &)> Callback;

  public:
    BNO055USBStick(boost::asio::io_service &asio_service, const Callback &callback,
                   const std::string &ns = "~")
        : serial_(asio_service), timer_(asio_service), callback_(callback) {
        start(ns);
    }

    virtual ~BNO055USBStick() { stop(); }

  private:
    void start(const std::string &ns) {
        namespace rp = ros::param;
        namespace rn = ros::names;

        // load parameters
        const std::string port(rp::param< std::string >(rn::append(ns, "port"), "/dev/ttyACM0"));
        const std::string mode(rp::param< std::string >(rn::append(ns, "mode"), "ndof"));

        // setup the serial port
        try {
            serial_.open(port);

            typedef boost::asio::serial_port Serial;
            serial_.set_option(Serial::baud_rate(115200));
            serial_.set_option(Serial::flow_control(Serial::flow_control::none));
            serial_.set_option(Serial::parity(Serial::parity::none));
            serial_.set_option(Serial::stop_bits(Serial::stop_bits::one));
            serial_.set_option(Serial::character_size(8));
        } catch (const boost::system::system_error &error) {
            ROS_ERROR_STREAM("start: " << error.what());
            return;
        }

        // pack commands
        commands_.clear();
        if (mode == "ndof") {
            for (const boost::uint8_t **command = Constants::toNDOFCommands(); *command;
                 ++command) {
                commands_.push_back(*command);
            }
        } else if (mode == "imu") {
            for (const boost::uint8_t **command = Constants::toIMUCommands(); *command; ++command) {
                commands_.push_back(*command);
            }
        } else {
            ROS_WARN_STREAM("Unknown mode \""
                            << mode << "\" was given. Will use the default mode \"ndof\" instead.");
            for (const boost::uint8_t **command = Constants::toNDOFCommands(); *command;
                 ++command) {
                commands_.push_back(*command);
            }
        }
        for (const boost::uint8_t **command = Constants::startStreamCommands(); *command;
             ++command) {
            commands_.push_back(*command);
        }

        // trigger send packed commands
        startSendCommand();
    }

    void startSendCommand() {
        if (commands_.empty()) {
            ROS_ERROR("startSendCommand: No command in the queue");
            return;
        }

        // trigger send the top command in the queue
        const boost::uint8_t *command(commands_.front());
        boost::asio::async_write(serial_,
                                 boost::asio::buffer(command, Constants::getCommandLength(command)),
                                 boost::bind(&BNO055USBStick::handleSendCommand, this, _1, _2));
    }

    void handleSendCommand(const boost::system::error_code &error, const std::size_t bytes) {
        if (error) {
            ROS_ERROR_STREAM("handleSendCommand: " << error.message());
            return;
        }

        // pop the top command from the queue
        // dumpWritten("handleSendCommand: written: ");
        commands_.pop_front();

        // trigger wait the response for the command
        startWaitResponse();
    }

    void startWaitResponse() {
        boost::asio::async_read_until(
            serial_, buffer_, "\r\n",
            boost::bind(&BNO055USBStick::handleWaitResponse, this, _1, _2));
    }

    void handleWaitResponse(const boost::system::error_code &error, const std::size_t bytes) {
        if (error) {
            ROS_ERROR_STREAM("handleWaitResponse: " << error.message());
            return;
        }

        // clear the read response (cannot parse it because the protocol is unknown...)
        // dumpRead("handleWaitResponse: read: ");
        buffer_.consume(bytes);

        // trigger send the next command, or wait data stream
        if (!commands_.empty()) {
            startSendCommand();
        } else {
            startWaitData();
        }
    }

    void startWaitData() {
        boost::asio::async_read_until(serial_, buffer_, "\r\n",
                                      boost::bind(&BNO055USBStick::handleWaitData, this, _1, _2));
    }

    void handleWaitData(const boost::system::error_code &error, const std::size_t bytes) {
        if (error) {
            ROS_ERROR_STREAM("handleWaitData: " << error.message());
            return;
        }

        // parse the received data
        // dumpRead("handleWaitData: read: ");
        if (buffer_.size() >= Constants::DAT_LEN) {
            const boost::uint8_t *data(
                boost::asio::buffer_cast< const boost::uint8_t * >(buffer_.data()) +
                buffer_.size() - Constants::DAT_LEN);

            if (std::equal(data, data + Constants::HDR_LEN, Constants::streamHeader())) {
                const bno055_usb_stick_msgs::Output output(Decoder::decode(data));
                // ROS_INFO_STREAM("handleWaitData: output:\n" << output);
                if (callback_) {
                    callback_(output);
                }
            } else {
                ROS_WARN("handleWaitData: Data header mismatch");
            }
        } else {
            ROS_WARN_STREAM("handleWaitData: Too short data size (" << buffer_.size()
                                                                    << "). Continue anyway.");
        }

        // clear the parsed data
        buffer_.consume(bytes);

        // trigger wait the next data
        startWaitData();
    }

    void stop() {}

    void dumpWritten(const std::string &prefix) {
        std::ostringstream oss;
        const boost::uint8_t *begin(commands_.front());
        const boost::uint8_t *end(begin + Constants::getCommandLength(commands_.front()));
        for (const boost::uint8_t *c = begin; c != end; ++c) {
            oss << "0x" << std::setw(2) << std::setfill('0') << std::hex << int(*c) << " ";
        }
        ROS_INFO_STREAM(prefix << oss.str());
    }

    void dumpRead(const std::string &prefix) {
        std::ostringstream oss;
        const boost::uint8_t *begin(
            boost::asio::buffer_cast< const boost::uint8_t * >(buffer_.data()));
        const boost::uint8_t *end(begin + buffer_.size());
        for (const boost::uint8_t *c = begin; c != end; ++c) {
            oss << "0x" << std::setw(2) << std::setfill('0') << std::hex << int(*c) << " ";
        }
        ROS_INFO_STREAM(prefix << oss.str());
    }

  private:
    // buffers
    std::deque< const boost::uint8_t * > commands_;
    boost::asio::streambuf buffer_;

    // async objects
    boost::asio::serial_port serial_;
    boost::asio::deadline_timer timer_;

    // callback given by the user
    const Callback callback_;
};
}

#endif // BNO055_USB_STICK_BNO055_USB_STICK_HPP