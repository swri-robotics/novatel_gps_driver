#include <novatel_gps_driver/parsers/gphdt.h>
#include <boost/make_shared.hpp>
#include <swri_string_util/string_util.h>

const std::string novatel_gps_driver::GphdtParser::MESSAGE_NAME = "GPHDT";

uint32_t novatel_gps_driver::GphdtParser::GetMessageId() const
{
  return 0;
}

const std::string novatel_gps_driver::GphdtParser::GetMessageName() const
{
  return MESSAGE_NAME;
}

novatel_gps_msgs::GphdtPtr novatel_gps_driver::GphdtParser::ParseAscii(const novatel_gps_driver::NmeaSentence& sentence) noexcept(false)
{
  const size_t EXPECTED_LEN = 3;

  if (sentence.body.size() != EXPECTED_LEN)
  {
    std::stringstream error;
    error << "Expected GPHDT length = "
          << EXPECTED_LEN << ", "
          << "actual length = " << sentence.body.size();
    throw ParseException(error.str());
  }

  novatel_gps_msgs::GphdtPtr msg = boost::make_shared<novatel_gps_msgs::Gphdt>();
  msg->message_id = sentence.body[0];

  double heading;
  if (swri_string_util::ToDouble(sentence.body[1], heading))
  {
      msg->heading = heading;
  }
  else
  {
    throw ParseException("Error parsing heading as double in GPHDT");
  }

  msg->t = sentence.body[2];
  return msg;
}

