// *****************************************************************************
//
// Copyright (C) 2016 All Right Reserved, Southwest Research Institute® (SwRI®)
//
// THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
// KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
// PARTICULAR PURPOSE.
//
// *****************************************************************************

#ifndef NOVATEL_GPS_DRIVER_GPRMC_H
#define NOVATEL_GPS_DRIVER_GPRMC_H

#include <novatel_gps_driver/parsers/message_parser.h>
#include <novatel_gps_msgs/Gprmc.h>

namespace novatel_gps_driver
{
  class GprmcParser : public MessageParser<novatel_gps_msgs::GprmcPtr>
  {
  public:
    GprmcParser() : MessageParser<novatel_gps_msgs::GprmcPtr>(),
                    was_last_gps_valid_(false)
    {}

    uint32_t GetMessageId() const override;

    const std::string GetMessageName() const override;

    novatel_gps_msgs::GprmcPtr ParseAscii(const NmeaSentence& sentence) throw(ParseException) override;

    bool WasLastGpsValid() const;

    static const std::string MESSAGE_NAME;
    static constexpr double KNOTS_TO_MPS = 0.5144444;

  private:
    bool was_last_gps_valid_;
  };
}

#endif //NOVATEL_GPS_DRIVER_GPRMC_H
