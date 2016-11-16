#ifndef BNO055_USB_STICK_DECODER_HPP
#define BNO055_USB_STICK_DECODER_HPP

#include <ros/time.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Temperature.h>

#include <bno055_usb_stick/constants.hpp>

#include <boost/cstdint.hpp>

namespace bno055_usb_stick {
class Decoder {
  public:
    Decoder() {}

    virtual ~Decoder() {}

    void decode(const char *data, sensor_msgs::Imu &imu, sensor_msgs::MagneticField &mag,
                sensor_msgs::Temperature &temp) const {
        const ros::Time stamp(ros::Time::now());

        // decode imu
        imu.angular_velocity = decodeGyr(data + 12);
        imu.orientation = decodeQua(data + 24);
        imu.linear_acceleration = decodeAcc(data + 32);

        // decode mag
        mag.magnetic_field = decodeMag(data + 6);

        // decode temp
        temp.temperature = decodeTemp(data + 44);
    }

  private:
    static geometry_msgs::Vector3 decodeAcc(const char *data) {
        geometry_msgs::Vector3 acc;
        acc.x = decodeVal(data[1], data[0], Constants::ACC_DENOM);
        acc.y = decodeVal(data[3], data[2], Constants::ACC_DENOM);
        acc.z = decodeVal(data[5], data[4], Constants::ACC_DENOM);
        return acc;
    }

    static geometry_msgs::Vector3 decodeGyr(const char *data) {
        geometry_msgs::Vector3 gyr;
        gyr.x = decodeVal(data[1], data[0], Constants::GYR_DENOM) * M_PI / 180.;
        gyr.y = decodeVal(data[3], data[2], Constants::GYR_DENOM) * M_PI / 180.;
        gyr.z = decodeVal(data[5], data[4], Constants::GYR_DENOM) * M_PI / 180.;
        return gyr;
    }

    static geometry_msgs::Quaternion decodeQua(const char *data) {
        geometry_msgs::Quaternion qua;
        qua.w = decodeVal(data[1], data[0], Constants::QUA_DENOM);
        qua.x = decodeVal(data[3], data[2], Constants::QUA_DENOM);
        qua.y = decodeVal(data[5], data[4], Constants::QUA_DENOM);
        qua.z = decodeVal(data[7], data[6], Constants::QUA_DENOM);
        return qua;
    }

    static geometry_msgs::Vector3 decodeMag(const char *data) {
        geometry_msgs::Vector3 mag;
        mag.x = decodeVal(data[1], data[0], Constants::MAG_DENOM);
        mag.y = decodeVal(data[3], data[2], Constants::MAG_DENOM);
        mag.z = decodeVal(data[5], data[4], Constants::MAG_DENOM);
        return mag;
    }

    static double decodeTemp(const char *data) { return data[0] / Constants::TEMP_DENOM; }

    static double decodeVal(const char msb, const char lsb, const double denom) {
        return boost::int16_t((boost::int16_t(msb) << 8) | lsb) / denom;
    }
};
}
#endif // BNO055_USB_STICK_DECODER_HPP