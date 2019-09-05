#ifndef NOVATEL_GPS_DRIVER_GPHDT_H
#define NOVATEL_GPS_DRIVER_GPHDT_H

#include <novatel_gps_driver/parsers/message_parser.h>
#include <novatel_gps_msgs/Gphdt.h>

namespace novatel_gps_driver
{
  class GphdtParser : MessageParser<novatel_gps_msgs::GphdtPtr>
  {
  public:
    uint32_t GetMessageId() const override;

    const std::string GetMessageName() const override;

    novatel_gps_msgs::GphdtPtr ParseAscii(const NmeaSentence& sentence) noexcept(false) override;

    static const std::string MESSAGE_NAME;
  };
}

#endif //NOVATEL_GPS_DRIVER_GPHDT_H
