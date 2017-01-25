#ifndef BNO055_USB_STICK_CONSTANTS_HPP
#define BNO055_USB_STICK_CONSTANTS_HPP

#include <cstddef>

#include <boost/cstdint.hpp>

namespace bno055_usb_stick {

struct Constants {
  // Length of a data stream packet
  enum { DAT_LEN = 0x38, HDR_LEN = 5 };

  // Form of a data stream packet
  enum {
    HDR_POS = 0,
    BDY_POS = HDR_POS + HDR_LEN,
    ACC_POS = BDY_POS,
    MAG_POS = ACC_POS + 6,
    GYR_POS = MAG_POS + 6,
    EUL_POS = GYR_POS + 6,
    QUA_POS = EUL_POS + 6,
    LIA_POS = QUA_POS + 8,
    GRV_POS = LIA_POS + 6,
    TEMP_POS = GRV_POS + 6,
    CALIB_STAT_POS = TEMP_POS + 1
  };

  // Denominators to convert a byte/word to a real value
  enum {
    ACC_DENOM = 100,
    MAG_DENOM = 16,
    GYR_DENOM = 16,
    EUL_DENOM = 16,
    QUA_DENOM = 1 << 14,
    LIA_DENOM = 100,
    GRV_DENOM = 100,
    TEMP_DENOM = 1
  };

  static std::size_t getCommandLength(const boost::uint8_t *command) { return command[1]; }

  static const boost::uint8_t **toNDOFCommands() {
    static const boost::uint8_t _00[] = {0xaa, 0x13, 0x01, 0x16, 0x02, 0x00, 0x01, 0x00, 0x00, 0x28,
                                         0x07, 0x00, 0x01, 0x01, 0x00, 0x00, 0x00, 0x0d, 0x0a};
    static const boost::uint8_t _01[] = {0xaa, 0x13, 0x01, 0x16, 0x02, 0x00, 0x01, 0x00, 0x00, 0x28,
                                         0x3d, 0x00, 0x01, 0x01, 0x00, 0x01, 0x1c, 0x0d, 0x0a};
    static const boost::uint8_t *commands[] = {_00, _01, NULL};
    return commands;
  }

  static const boost::uint8_t **toIMUCommands() {
    static const boost::uint8_t _00[] = {0xaa, 0x13, 0x01, 0x16, 0x02, 0x00, 0x01, 0x00, 0x00, 0x28,
                                         0x07, 0x00, 0x01, 0x01, 0x00, 0x00, 0x00, 0x0d, 0x0a};
    static const boost::uint8_t _01[] = {0xaa, 0x13, 0x01, 0x16, 0x02, 0x00, 0x01, 0x00, 0x00, 0x28,
                                         0x3d, 0x00, 0x01, 0x01, 0x00, 0x01, 0x18, 0x0d, 0x0a};
    static const boost::uint8_t *commands[] = {_00, _01, NULL};
    return commands;
  }

  static const boost::uint8_t **startStreamCommands() {
    static const boost::uint8_t _00[] = {0xaa, 0x06, 0x06, 0x00, 0x0d, 0x0a};
    static const boost::uint8_t _01[] = {0xaa, 0x13, 0x01, 0x16, 0x02, 0x00, 0x01, 0x00, 0x00, 0x28,
                                         0x07, 0x00, 0x01, 0x01, 0x00, 0x00, 0x00, 0x0d, 0x0a};
    static const boost::uint8_t _02[] = {0xaa, 0x0a, 0x03, 0x02, 0x01,
                                         0x00, 0x0a, 0x02, 0x0d, 0x0a};
    static const boost::uint8_t _03[] = {0xaa, 0x14, 0x04, 0x01, 0x0e, 0x00, 0x00,
                                         0x00, 0x28, 0x00, 0x0a, 0x02, 0x08, 0x12,
                                         0x00, 0x00, 0x00, 0x00, 0x0d, 0x0a};
    static const boost::uint8_t _04[] = {0xaa, 0x15, 0x04, 0x02, 0x0e, 0x00, 0x00,
                                         0x00, 0x28, 0x00, 0x0a, 0x02, 0x1a, 0x1e,
                                         0x00, 0x00, 0x01, 0x07, 0x00, 0x0d, 0x0a};
    static const boost::uint8_t _05[] = {0xaa, 0x06, 0x06, 0xff, 0x0d, 0x0a};
    static const boost::uint8_t *commands[] = {_00, _01, _02, _03, _04, _05, NULL};
    return commands;
  }

  static const boost::uint8_t **stopStreamCommands() {
    static const boost::uint8_t _00[] = {0xaa, 0x06, 0x06, 0x00, 0x0d, 0x0a};
    static const boost::uint8_t *commands[] = {_00, NULL};
    return commands;
  }
};
}

#endif // BNO055_USB_STICK_CONSTANTS_HPP