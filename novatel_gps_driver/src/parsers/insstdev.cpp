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

#include <novatel_gps_driver/parsers/insstdev.h>
#include <novatel_gps_driver/parsers/header.h>
#include <boost/make_shared.hpp>

const std::string novatel_gps_driver::InsstdevParser::MESSAGE_NAME = "INSSTDEV";

uint32_t novatel_gps_driver::InsstdevParser::GetMessageId() const
{
  return MESSAGE_ID;
}

const std::string novatel_gps_driver::InsstdevParser::GetMessageName() const
{
  return MESSAGE_NAME;
}

novatel_gps_msgs::InsstdevPtr
novatel_gps_driver::InsstdevParser::ParseBinary(const novatel_gps_driver::BinaryMessage& bin_msg) throw(ParseException)
{
  if (bin_msg.data_.size() != BINARY_LENGTH)
  {
    std::stringstream error;
    error << "Unexpected INSSTDEV message size: " << bin_msg.data_.size();
    throw ParseException(error.str());
  }
  novatel_gps_msgs::InsstdevPtr ros_msg = boost::make_shared<novatel_gps_msgs::Insstdev>();
  HeaderParser h_parser;
  ros_msg->novatel_msg_header = h_parser.ParseBinary(bin_msg);
  ros_msg->novatel_msg_header.message_name = GetMessageName();
  ros_msg->latitude_dev = ParseFloat(&bin_msg.data_[0]);
  ros_msg->latitude_dev = ParseFloat(&bin_msg.data_[4]);
  ros_msg->height_dev = ParseFloat(&bin_msg.data_[8]);
  ros_msg->north_velocity_dev = ParseFloat(&bin_msg.data_[12]);
  ros_msg->east_velocity_dev = ParseFloat(&bin_msg.data_[16]);
  ros_msg->up_velocity_dev = ParseFloat(&bin_msg.data_[20]);
  ros_msg->roll_dev = ParseFloat(&bin_msg.data_[24]);
  ros_msg->pitch_dev = ParseFloat(&bin_msg.data_[28]);
  ros_msg->azimuth_dev = ParseFloat(&bin_msg.data_[32]);
  uint32_t status = ParseUInt32(&bin_msg.data_[36]);
  GetExtendedSolutionStatusMessage(status, ros_msg->extended_solution_status);
  ros_msg->time_since_update = ParseUInt16(&bin_msg.data_[40]);

  return ros_msg;
}

novatel_gps_msgs::InsstdevPtr
novatel_gps_driver::InsstdevParser::ParseAscii(const novatel_gps_driver::NovatelSentence& sentence) throw(ParseException)
{
  if (sentence.body.size() != ASCII_FIELDS)
  {
    std::stringstream error;
    error << "Unexpected number of fields in INSSTDEV log: " << sentence.body.size();
    throw ParseException(error.str());
  }
  novatel_gps_msgs::InsstdevPtr msg = boost::make_shared<novatel_gps_msgs::Insstdev>();
  HeaderParser h_parser;
  msg->novatel_msg_header = h_parser.ParseAscii(sentence);

  bool valid = true;

  valid &= ParseFloat(sentence.body[0], msg->latitude_dev);
  valid &= ParseFloat(sentence.body[1], msg->longitude_dev);
  valid &= ParseFloat(sentence.body[2], msg->height_dev);
  valid &= ParseFloat(sentence.body[3], msg->north_velocity_dev);
  valid &= ParseFloat(sentence.body[4], msg->east_velocity_dev);
  valid &= ParseFloat(sentence.body[5], msg->up_velocity_dev);
  valid &= ParseFloat(sentence.body[6], msg->roll_dev);
  valid &= ParseFloat(sentence.body[7], msg->pitch_dev);
  valid &= ParseFloat(sentence.body[8], msg->azimuth_dev);
  uint32_t status;
  valid &= ParseUInt32(sentence.body[9], status);
  GetExtendedSolutionStatusMessage(status, msg->extended_solution_status);

  if (!valid)
  {
    throw ParseException("Error parsing INSSTDEV log.");
  }

  return msg;
}
