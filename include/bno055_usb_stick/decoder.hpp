#ifndef BNO055_USB_STICK_DECODER_HPP
#define BNO055_USB_STICK_DECODER_HPP

#include <algorithm>
#include <string>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <ros/names.h>
#include <ros/param.h>
#include <ros/time.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Temperature.h>
#include <tf/transform_datatypes.h>

#include <bno055_usb_stick/constants.hpp>
#include <bno055_usb_stick_msgs/CalibrationStatus.h>
#include <bno055_usb_stick_msgs/EulerAngles.h>
#include <bno055_usb_stick_msgs/Output.h>

#include <boost/cstdint.hpp>

namespace bno055_usb_stick {
class Decoder {
public:
  Decoder(const std::string &ns)
      : frame_id_(ros::param::param< std::string >(ros::names::append(ns, "frame_id"), "bno055")) {}

  virtual ~Decoder() {}

  bno055_usb_stick_msgs::Output decode(const boost::uint8_t *data) {
    bno055_usb_stick_msgs::Output output;
    output.header.stamp = ros::Time::now();
    output.header.frame_id = frame_id_;
    output.acceleration = decodeAcc(data + Constants::ACC_POS);
    output.magnetometer = decodeMag(data + Constants::MAG_POS);
    output.gyroscope = decodeGyr(data + Constants::GYR_POS);
    output.euler_angles = decodeEul(data + Constants::EUL_POS);
    output.quaternion = decodeQua(data + Constants::QUA_POS);
    output.linear_acceleration = decodeLia(data + Constants::LIA_POS);
    output.gravity_vector = decodeGrv(data + Constants::GRV_POS);
    output.temperature = decodeTemp(data + Constants::TEMP_POS);
    output.calibration_status = decodeCalibStat(data + Constants::CALIB_STAT_POS);
    return output;
  }

  static tf::StampedTransform toTFTransform(const bno055_usb_stick_msgs::Output &output,
                                            const std::string &frame_id,
                                            const std::string &child_frame_id,
                                            const bool do_invert) {
    tf::Quaternion quaternion;
    tf::quaternionMsgToTF(output.quaternion, quaternion);
    return tf::StampedTransform(do_invert ? tf::Transform(quaternion).inverse()
                                          : tf::Transform(quaternion),
                                output.header.stamp, frame_id, child_frame_id);
  }

  static sensor_msgs::Imu toImuMsg(const bno055_usb_stick_msgs::Output &output) {
    sensor_msgs::Imu imu;
    imu.header = output.header;
    imu.orientation = output.quaternion;
    imu.angular_velocity = output.gyroscope;
    imu.linear_acceleration = output.acceleration;

    // To indicate no covariance estimate, set the 1st elements of matrice -1
    imu.orientation_covariance[0] = -1;
    std::fill(imu.orientation_covariance.begin() + 1, imu.orientation_covariance.end(), 0.);
    imu.angular_velocity_covariance[0] = -1;
    std::fill(imu.angular_velocity_covariance.begin() + 1, imu.angular_velocity_covariance.end(),
              0.);
    imu.linear_acceleration_covariance[0] = -1;
    std::fill(imu.linear_acceleration_covariance.begin() + 1,
              imu.linear_acceleration_covariance.end(), 0.);

    return imu;
  }

  static geometry_msgs::PoseStamped toPoseMsg(const bno055_usb_stick_msgs::Output &output,
                                              const std::string &frame_id) {
    geometry_msgs::PoseStamped pose;
    pose.header = output.header;
    pose.header.frame_id = frame_id;
    pose.pose.position.x = pose.pose.position.y = pose.pose.position.z = 0.;
    pose.pose.orientation = output.quaternion;
    return pose;
  }

  static sensor_msgs::MagneticField toMagMsg(const bno055_usb_stick_msgs::Output &output) {
    sensor_msgs::MagneticField mag;
    mag.header = output.header;
    mag.magnetic_field = output.magnetometer;
    std::fill(mag.magnetic_field_covariance.begin(), mag.magnetic_field_covariance.end(), 0.);
    return mag;
  }

  static sensor_msgs::Temperature toTempMsg(const bno055_usb_stick_msgs::Output &output) {
    sensor_msgs::Temperature temp;
    temp.header = output.header;
    temp.temperature = output.temperature;
    temp.variance = 0.;
    return temp;
  }

private:
  static geometry_msgs::Vector3 decodeAcc(const boost::uint8_t *data) {
    geometry_msgs::Vector3 acc;
    acc.x = decodeVal(data[1], data[0], Constants::ACC_DENOM);
    acc.y = decodeVal(data[3], data[2], Constants::ACC_DENOM);
    acc.z = decodeVal(data[5], data[4], Constants::ACC_DENOM);
    return acc;
  }

  static geometry_msgs::Vector3 decodeMag(const boost::uint8_t *data) {
    geometry_msgs::Vector3 mag;
    mag.x = decodeVal(data[1], data[0], Constants::MAG_DENOM);
    mag.y = decodeVal(data[3], data[2], Constants::MAG_DENOM);
    mag.z = decodeVal(data[5], data[4], Constants::MAG_DENOM);
    return mag;
  }

  static geometry_msgs::Vector3 decodeGyr(const boost::uint8_t *data) {
    geometry_msgs::Vector3 gyr;
    gyr.x = decodeVal(data[1], data[0], Constants::GYR_DENOM) * M_PI / 180.;
    gyr.y = decodeVal(data[3], data[2], Constants::GYR_DENOM) * M_PI / 180.;
    gyr.z = decodeVal(data[5], data[4], Constants::GYR_DENOM) * M_PI / 180.;
    return gyr;
  }

  static bno055_usb_stick_msgs::EulerAngles decodeEul(const boost::uint8_t *data) {
    bno055_usb_stick_msgs::EulerAngles eul;
    eul.heading = decodeVal(data[1], data[0], Constants::EUL_DENOM) * M_PI / 180.;
    eul.roll = decodeVal(data[3], data[2], Constants::EUL_DENOM) * M_PI / 180.;
    eul.pitch = decodeVal(data[5], data[4], Constants::EUL_DENOM) * M_PI / 180.;
    return eul;
  }

  static geometry_msgs::Quaternion decodeQua(const boost::uint8_t *data) {
    geometry_msgs::Quaternion qua;
    qua.w = decodeVal(data[1], data[0], Constants::QUA_DENOM);
    qua.x = decodeVal(data[3], data[2], Constants::QUA_DENOM);
    qua.y = decodeVal(data[5], data[4], Constants::QUA_DENOM);
    qua.z = decodeVal(data[7], data[6], Constants::QUA_DENOM);
    return qua;
  }

  static geometry_msgs::Vector3 decodeLia(const boost::uint8_t *data) {
    geometry_msgs::Vector3 lia;
    lia.x = decodeVal(data[1], data[0], Constants::LIA_DENOM);
    lia.y = decodeVal(data[3], data[2], Constants::LIA_DENOM);
    lia.z = decodeVal(data[5], data[4], Constants::LIA_DENOM);
    return lia;
  }

  static geometry_msgs::Vector3 decodeGrv(const boost::uint8_t *data) {
    geometry_msgs::Vector3 grv;
    grv.x = decodeVal(data[1], data[0], Constants::GRV_DENOM);
    grv.y = decodeVal(data[3], data[2], Constants::GRV_DENOM);
    grv.z = decodeVal(data[5], data[4], Constants::GRV_DENOM);
    return grv;
  }

  static double decodeTemp(const boost::uint8_t *data) { return data[0] / Constants::TEMP_DENOM; }

  static bno055_usb_stick_msgs::CalibrationStatus decodeCalibStat(const boost::uint8_t *data) {
    bno055_usb_stick_msgs::CalibrationStatus calib_stat;
    calib_stat.system = (*data >> 6) & 0x3;
    calib_stat.gyroscope = (*data >> 4) & 0x3;
    calib_stat.accelerometer = (*data >> 2) & 0x3;
    calib_stat.magnetometer = *data & 0x3;
    return calib_stat;
  }

  static double decodeVal(const boost::uint8_t msb, const boost::uint8_t lsb, const double denom) {
    return boost::int16_t((boost::int16_t(msb) << 8) | lsb) / denom;
  }

private:
  const std::string frame_id_;
};
} // namespace bno055_usb_stick
#endif // BNO055_USB_STICK_DECODER_HPP