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

#include <novatel_gps_driver/parsers/inscov.h>
#include <novatel_gps_driver/parsers/header.h>
#include <boost/make_shared.hpp>

const std::string novatel_gps_driver::InscovParser::MESSAGE_NAME = "INSCOV";

uint32_t novatel_gps_driver::InscovParser::GetMessageId() const
{
  return MESSAGE_ID;
}

const std::string novatel_gps_driver::InscovParser::GetMessageName() const
{
  return MESSAGE_NAME;
}

novatel_gps_msgs::InscovPtr
novatel_gps_driver::InscovParser::ParseBinary(const BinaryMessage& bin_msg) throw(ParseException)
{
  if (bin_msg.data_.size() != BINARY_LENGTH)
  {
    std::stringstream error;
    error << "Unexpected inscov message size: " << bin_msg.data_.size();
    throw ParseException(error.str());
  }
  novatel_gps_msgs::InscovPtr ros_msg = boost::make_shared<novatel_gps_msgs::Inscov>();
  HeaderParser h_parser;
  ros_msg->novatel_msg_header = h_parser.ParseBinary(bin_msg);
  ros_msg->novatel_msg_header.message_name = GetMessageName();

  ros_msg->week = ParseUInt32(&bin_msg.data_[0]);
  ros_msg->seconds = ParseDouble(&bin_msg.data_[4]);
  int offset = 12;
  for (int i = 0; i < 9; i++, offset += 8)
  {
    ros_msg->position_covariance[i] = ParseDouble(&bin_msg.data_[offset]);
  }
  for (int i = 0; i < 9; i++, offset += 8)
  {
    ros_msg->attitude_covariance[i] = ParseDouble(&bin_msg.data_[offset]);
  }
  for (int i = 0; i < 9; i++, offset += 8)
  {
    ros_msg->velocity_covariance[i] = ParseDouble(&bin_msg.data_[offset]);
  }
  return ros_msg;
}

novatel_gps_msgs::InscovPtr
novatel_gps_driver::InscovParser::ParseAscii(const NovatelSentence& sentence) throw(ParseException)
{
  if (sentence.body.size() != ASCII_FIELDS)
  {
    std::stringstream error;
    error << "Unexpected number of fields in INSCOV log: " << sentence.body.size();
    throw ParseException(error.str());
  }
  novatel_gps_msgs::InscovPtr ros_msg = boost::make_shared<novatel_gps_msgs::Inscov>();
  HeaderParser h_parser;
  ros_msg->novatel_msg_header = h_parser.ParseAscii(sentence);

  bool valid = true;

  valid &= ParseUInt32(sentence.body[0], ros_msg->week);
  valid &= ParseDouble(sentence.body[1], ros_msg->seconds);

  int offset = 2;
  for (int i = 0; i < 9; i++, offset++)
  {
    valid &= ParseDouble(sentence.body[offset], ros_msg->position_covariance[i]);
  }
  for (int i = 0; i < 9; i++, offset++)
  {
    valid &= ParseDouble(sentence.body[offset], ros_msg->attitude_covariance[i]);
  }
  for (int i = 0; i < 9; i++, offset++)
  {
    valid &= ParseDouble(sentence.body[offset], ros_msg->velocity_covariance[i]);
  }

  if (!valid)
  {
    throw ParseException("Error parsing INSCOV log.");
  }

  return ros_msg;
}
