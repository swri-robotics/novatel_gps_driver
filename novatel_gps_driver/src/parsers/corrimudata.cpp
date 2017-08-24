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

#include <novatel_gps_driver/parsers/corrimudata.h>
#include <novatel_gps_driver/parsers/header.h>
#include <boost/make_shared.hpp>

const std::string novatel_gps_driver::CorrImuDataParser::MESSAGE_NAME = "CORRIMUDATA";

uint32_t novatel_gps_driver::CorrImuDataParser::GetMessageId() const
{
  return MESSAGE_ID;
}

const std::string novatel_gps_driver::CorrImuDataParser::GetMessageName() const
{
  return MESSAGE_NAME;
}

novatel_gps_msgs::NovatelCorrectedImuDataPtr
novatel_gps_driver::CorrImuDataParser::ParseBinary(const novatel_gps_driver::BinaryMessage& bin_msg) throw(ParseException)
{
  if (bin_msg.data_.size() != BINARY_LENGTH)
  {
    std::stringstream error;
    error << "Unexpected corrimudata message size: " << bin_msg.data_.size();
    throw ParseException(error.str());
  }
  novatel_gps_msgs::NovatelCorrectedImuDataPtr ros_msg = boost::make_shared<novatel_gps_msgs::NovatelCorrectedImuData>();
  HeaderParser h_parser;
  ros_msg->novatel_msg_header = h_parser.ParseBinary(bin_msg);
  ros_msg->novatel_msg_header.message_name = "CORRIMUDATA";

  ros_msg->gps_week_num = ParseUInt32(&bin_msg.data_[0]);
  ros_msg->gps_seconds = ParseDouble(&bin_msg.data_[4]);
  ros_msg->pitch_rate = ParseDouble(&bin_msg.data_[12]);
  ros_msg->roll_rate = ParseDouble(&bin_msg.data_[20]);
  ros_msg->yaw_rate = ParseDouble(&bin_msg.data_[28]);
  ros_msg->lateral_acceleration = ParseDouble(&bin_msg.data_[36]);
  ros_msg->longitudinal_acceleration = ParseDouble(&bin_msg.data_[44]);
  ros_msg->vertical_acceleration = ParseDouble(&bin_msg.data_[52]);

  return ros_msg;
}

novatel_gps_msgs::NovatelCorrectedImuDataPtr
novatel_gps_driver::CorrImuDataParser::ParseAscii(const novatel_gps_driver::NovatelSentence& sentence) throw(ParseException)
{
  if (sentence.body.size() != ASCII_FIELDS)
  {
    std::stringstream error;
    error << "Unexpected number of fields in CORRIMUDATA log: " << sentence.body.size();
    throw ParseException(error.str());
  }
  novatel_gps_msgs::NovatelCorrectedImuDataPtr msg = boost::make_shared<novatel_gps_msgs::NovatelCorrectedImuData>();
  HeaderParser h_parser;
  msg->novatel_msg_header = h_parser.ParseAscii(sentence);

  bool valid = true;

  valid &= ParseUInt32(sentence.body[0], msg->gps_week_num);
  valid &= ParseDouble(sentence.body[1], msg->gps_seconds);
  valid &= ParseDouble(sentence.body[2], msg->pitch_rate);
  valid &= ParseDouble(sentence.body[3], msg->roll_rate);
  valid &= ParseDouble(sentence.body[4], msg->yaw_rate);
  valid &= ParseDouble(sentence.body[5], msg->lateral_acceleration);
  valid &= ParseDouble(sentence.body[6], msg->longitudinal_acceleration);
  valid &= ParseDouble(sentence.body[7], msg->vertical_acceleration);

  if (!valid)
  {
    throw ParseException("Error parsing CORRIMUDATA log.");
  }

  return msg;
}
