// *****************************************************************************
//
// Copyright (C) 2019 All Right Reserved, Southwest Research Institute® (SwRI®)
//
// THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
// KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
// PARTICULAR PURPOSE.
//
// *****************************************************************************

#include <novatel_gps_driver/parsers/header.h>
#include <novatel_gps_driver/parsers/psrdop2.h>
#include <sstream>

const std::string novatel_gps_driver::Psrdop2Parser::MESSAGE_NAME = "PSRDOP2";

uint32_t novatel_gps_driver::Psrdop2Parser::GetMessageId() const
{
  return MESSAGE_ID;
}

const std::string novatel_gps_driver::Psrdop2Parser::GetMessageName() const
{
  return MESSAGE_NAME;
}

novatel_gps_driver::Psrdop2Parser::MessageType
novatel_gps_driver::Psrdop2Parser::ParseBinary(const novatel_gps_driver::BinaryMessage& bin_msg)
{
  uint32_t num_systems = ParseUInt32(&bin_msg.data_[16]);
  if (bin_msg.data_.size() != (BINARY_SYSTEM_LENGTH * num_systems) + BINARY_BODY_LENGTH)
  {
    std::stringstream error;
    error << "Unexpected PSRDOP2 message size: " << bin_msg.data_.size();
    throw ParseException(error.str());
  }

  auto ros_msg = std::make_shared<novatel_gps_msgs::msg::NovatelPsrdop2>();

  HeaderParser header_parser;
  ros_msg->novatel_msg_header = header_parser.ParseBinary(bin_msg);
  ros_msg->novatel_msg_header.message_name = MESSAGE_NAME;

  ros_msg->gdop = ParseFloat(&bin_msg.data_[0]);

  ros_msg->pdop = ParseFloat(&bin_msg.data_[4]);

  ros_msg->hdop = ParseFloat(&bin_msg.data_[8]);

  ros_msg->vdop = ParseFloat(&bin_msg.data_[12]);

  ros_msg->systems.reserve(num_systems);
  for (uint32_t i = 0; i < num_systems; i++)
  {
    size_t system_offset = BINARY_BODY_LENGTH + i * BINARY_SYSTEM_LENGTH;
    novatel_gps_msgs::msg::NovatelPsrdop2System system;

    system.system = GetSystemName(ParseUInt32(&bin_msg.data_[system_offset]));
    system.tdop = ParseFloat(&bin_msg.data_[system_offset+4]);

    ros_msg->systems.push_back(system);
  }

  return ros_msg;
}

novatel_gps_driver::Psrdop2Parser::MessageType
novatel_gps_driver::Psrdop2Parser::ParseAscii(const novatel_gps_driver::NovatelSentence& sentence)
{
  if (sentence.body.size() < ASCII_BODY_FIELDS)
  {
    std::stringstream error;
    error << "Unexpected number of body fields in PSRDOP2 log: " << sentence.body.size();
    throw ParseException(error.str());
  }

  uint32_t num_systems = 0;
  ParseUInt32(sentence.body[4], num_systems);

  if (sentence.body.size() != ASCII_BODY_FIELDS + num_systems * ASCII_SYSTEM_FIELDS)
  {
    std::stringstream error;
    error << "Size of PSRDOP2 log (" << sentence.body.size() << ") did not match expected size ("
          << ASCII_BODY_FIELDS + num_systems * ASCII_SYSTEM_FIELDS << ").";
    throw ParseException(error.str());
  }

  bool valid = true;
  auto msg = std::make_unique<novatel_gps_msgs::msg::NovatelPsrdop2>();
  HeaderParser h_parser;
  msg->novatel_msg_header = h_parser.ParseAscii(sentence);
  valid &= ParseFloat(sentence.body[0], msg->gdop);
  valid &= ParseFloat(sentence.body[1], msg->pdop);
  valid &= ParseFloat(sentence.body[2], msg->hdop);
  valid &= ParseFloat(sentence.body[3], msg->vdop);

  msg->systems.reserve(num_systems);
  for (size_t i = 0; i < num_systems; i++)
  {
    novatel_gps_msgs::msg::NovatelPsrdop2System system;
    size_t offset = 5 + i * ASCII_SYSTEM_FIELDS;
    system.system = sentence.body[offset];
    valid &= ParseFloat(sentence.body[offset+1], system.tdop);
    msg->systems.push_back(system);
  }

  if (!valid)
  {
    std::stringstream error;
    error << "Error parsing PSRDOP2 log.";
    throw ParseException(error.str());
  }
  return msg;
}

std::string novatel_gps_driver::Psrdop2Parser::GetSystemName(uint32_t system_id)
{
  switch (system_id)
  {
    case 0:
      return "GPS";
    case 1:
      return "GLONASS";
    case 2:
      return "GALILEO";
    case 3:
      return "BEIDOU";
    case 4:
      return "NAVIC";
    case 99:
      return "AUTO";
    default:
      return "UNKNOWN";
  }
}