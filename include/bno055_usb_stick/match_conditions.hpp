#ifndef BNO055_USB_STICK_MATCH_CONDITIONS_HPP
#define BNO055_USB_STICK_MATCH_CONDITIONS_HPP

#include <algorithm>
#include <utility>

#include <boost/array.hpp>
#include <boost/asio/read_until.hpp>
#include <boost/type_traits/integral_constant.hpp>

namespace bno055_usb_stick {

struct ResponseCondition {
  template < typename Iterator >
  std::pair< Iterator, bool > operator()(const Iterator begin, const Iterator end) {
    typedef typename Iterator::value_type Value;
    typedef std::pair< Iterator, bool > Result;
    static const boost::array< Value, 1 > prefix = {static_cast< Value >(0xaa)};
    static const boost::array< Value, 2 > suffix = {static_cast< Value >(0x0d),
                                                    static_cast< Value >(0x0a)};

    // find a prefix of response
    const Iterator response_begin(std::search(begin, end, prefix.begin(), prefix.end()));
    if (response_begin == end) {
      // no prefix found in the buffer.
      // return the result that means
      // the searched part of buffer should not be evaluated again.
      return Result(end - std::min< std::size_t >(end - begin, prefix.size() - 1), false);
    }

    // find the length of response following the prefix
    const std::size_t buffer_length(end - response_begin);
    if (buffer_length == prefix.size()) {
      // the response length has never recieved.
      // return the result that means the found prefix should be evaluated again.
      return Result(response_begin, false);
    }
    const std::size_t response_length(*(response_begin + prefix.size()));

    // find a suffix of response
    if (response_length <= buffer_length) {
      const Iterator response_end(response_begin + response_length);
      if (std::equal(suffix.begin(), suffix.end(), response_end - suffix.size())) {
        // the suffix found.
        // return the result that means a entire response was found.
        return Result(response_end, true);
      }
      // no suffix found
      // return the result of rest of buffer
      return operator()(response_begin + 1, end);
    }

    // reaching here means that the found prefix should be evaluated again
    // unless the buffer does not contain another complete response
    const Result result_rest(operator()(response_begin + 1, end));
    if (result_rest.second) {
      // found a complete response in the rest of buffer
      return result_rest;
    }
    // the found prefix should be evaluated again
    return Result(response_begin, false);
  }
};

struct DataCondition {
  template < typename Iterator >
  std::pair< Iterator, bool > operator()(Iterator begin, Iterator end) {
    typedef typename Iterator::value_type Value;
    typedef std::pair< Iterator, bool > Result;
    static const boost::array< Value, 5 > header = {
        static_cast< Value >(0xaa), static_cast< Value >(0x38), static_cast< Value >(0x01),
        static_cast< Value >(0x00), static_cast< Value >(0x86)};
    static const std::size_t data_length(header[1]);
    static const boost::array< Value, 2 > suffix = {static_cast< Value >(0x0d),
                                                    static_cast< Value >(0x0a)};

    // find a header of data
    const Iterator data_begin(std::search(begin, end, header.begin(), header.end()));
    if (data_begin == end) {
      // no header found in the buffer.
      // return the result that means
      // the searched part of buffer should not be evaluated again.
      return Result(end - std::min< std::size_t >(end - begin, header.size() - 1), false);
    }

    // find a suffix of data
    const std::size_t buffer_length(end - data_begin);
    if (data_length <= buffer_length) {
      const Iterator data_end(data_begin + data_length);
      if (std::equal(suffix.begin(), suffix.end(), data_end - suffix.size())) {
        // the suffix found.
        // return the result that means the complete data was found.
        return Result(data_end, true);
      }
      // no suffix found
      // return the result of rest of buffer
      return operator()(data_begin + 1, end);
    }

    // reaching here means that the found header should be evaluated again
    // unless the buffer does not contain another complete data
    const Result result_rest(operator()(data_begin + 1, end));
    if (result_rest.second) {
      // found a complete data in the rest of buffer
      return result_rest;
    }
    // the found header should be evaluated again
    return Result(data_begin, false);
  }
};
} // namespace bno055_usb_stick

namespace boost {
namespace asio {
template <> struct is_match_condition< bno055_usb_stick::ResponseCondition > : boost::true_type {};
template <> struct is_match_condition< bno055_usb_stick::DataCondition > : boost::true_type {};
} // namespace asio
} // namespace boost

#endif // BNO055_USB_STICK_MATCH_CONDITIONS_HPP