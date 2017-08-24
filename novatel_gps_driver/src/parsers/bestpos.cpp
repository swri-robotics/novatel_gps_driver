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

#include <novatel_gps_driver/parsers/bestpos.h>

#include <novatel_gps_driver/parsers/header.h>

#include <boost/make_shared.hpp>

namespace novatel_gps_driver
{
  const std::string BestposParser::MESSAGE_NAME = "BESTPOS";

  uint32_t BestposParser::GetMessageId() const
  {
    return MESSAGE_ID;
  }

  const std::string BestposParser::GetMessageName() const
  {
    return MESSAGE_NAME;
  }

  novatel_gps_msgs::NovatelPositionPtr BestposParser::ParseBinary(const BinaryMessage& bin_msg) throw(ParseException)
  {
    if (bin_msg.data_.size() != BINARY_LENGTH)
    {
      std::stringstream error;
      error << "Unexpected BESTPOS message length: " << bin_msg.data_.size();
      throw ParseException(error.str());
    }
    novatel_gps_msgs::NovatelPositionPtr ros_msg =
        boost::make_shared<novatel_gps_msgs::NovatelPosition>();
    HeaderParser header_parser;
    ros_msg->novatel_msg_header = header_parser.ParseBinary(bin_msg);
    ros_msg->novatel_msg_header.message_name = MESSAGE_NAME;

    uint16_t solution_status = ParseUInt16(&bin_msg.data_[0]);
    if (solution_status > 22)
    {
      std::stringstream error;
      error << "Unknown solution status: " << solution_status;
      throw ParseException(error.str());
    }
    ros_msg->solution_status = SOLUTION_STATUSES[solution_status];
    uint16_t pos_type = ParseUInt16(&bin_msg.data_[4]);
    if (pos_type > 74)
    {
      std::stringstream error;
      error << "Unknown position type: " << pos_type;
      throw ParseException(error.str());
    }
    ros_msg->position_type = POSITION_TYPES[pos_type];
    ros_msg->lat = ParseDouble(&bin_msg.data_[8]);
    ros_msg->lon = ParseDouble(&bin_msg.data_[16]);
    ros_msg->height = ParseDouble(&bin_msg.data_[24]);
    ros_msg->undulation = ParseFloat(&bin_msg.data_[32]);
    uint16_t datum_id = ParseUInt16(&bin_msg.data_[36]);
    if (datum_id > 86)
    {
      std::stringstream error;
      error << "Unknown datum: " << datum_id;
      throw ParseException(error.str());
    }
    ros_msg->datum_id = DATUMS[datum_id];
    ros_msg->lat_sigma = ParseFloat(&bin_msg.data_[40]);
    ros_msg->lon_sigma = ParseFloat(&bin_msg.data_[44]);
    ros_msg->height_sigma = ParseFloat(&bin_msg.data_[48]);
    ros_msg->base_station_id.resize(4);
    std::copy(&bin_msg.data_[52], &bin_msg.data_[56], &ros_msg->base_station_id[0]);
    ros_msg->diff_age = ParseFloat(&bin_msg.data_[56]);
    ros_msg->solution_age = ParseFloat(&bin_msg.data_[60]);
    ros_msg->num_satellites_tracked = bin_msg.data_[64];
    ros_msg->num_satellites_used_in_solution = bin_msg.data_[65];
    ros_msg->num_gps_and_glonass_l1_used_in_solution = bin_msg.data_[66];
    ros_msg->num_gps_and_glonass_l1_and_l2_used_in_solution = bin_msg.data_[67];
    GetExtendedSolutionStatusMessage(bin_msg.data_[69],
                                     ros_msg->extended_solution_status);
    GetSignalsUsed(bin_msg.data_[70], ros_msg->signal_mask);

    return ros_msg;
  }

  novatel_gps_msgs::NovatelPositionPtr BestposParser::ParseAscii(const NovatelSentence& sentence) throw(ParseException)
  {
    novatel_gps_msgs::NovatelPositionPtr msg =
        boost::make_shared<novatel_gps_msgs::NovatelPosition>();
    HeaderParser h_parser;
    msg->novatel_msg_header = h_parser.ParseAscii(sentence);

    if (sentence.body.size() != ASCII_LENGTH)
    {
      std::stringstream error;
      error << "Unexpected number of BESTPOS message fields: " << sentence.body.size();
      throw ParseException(error.str());
    }

    bool valid = true;

    msg->solution_status = sentence.body[0];
    msg->position_type = sentence.body[1];
    valid = valid && ParseDouble(sentence.body[2], msg->lat);
    valid = valid && ParseDouble(sentence.body[3], msg->lon);
    valid = valid && ParseDouble(sentence.body[4], msg->height);
    valid = valid && ParseFloat(sentence.body[5], msg->undulation);
    msg->datum_id = sentence.body[6];
    valid = valid && ParseFloat(sentence.body[7], msg->lat_sigma);
    valid = valid && ParseFloat(sentence.body[8], msg->lon_sigma);
    valid = valid && ParseFloat(sentence.body[9], msg->height_sigma);
    msg->base_station_id = sentence.body[10];
    valid = valid && ParseFloat(sentence.body[11], msg->diff_age);
    valid = valid && ParseFloat(sentence.body[12], msg->solution_age);
    valid = valid && ParseUInt8(sentence.body[13], msg->num_satellites_tracked);
    valid = valid && ParseUInt8(sentence.body[14], msg->num_satellites_used_in_solution);
    valid = valid && ParseUInt8(sentence.body[15], msg->num_gps_and_glonass_l1_used_in_solution);
    valid = valid && ParseUInt8(sentence.body[16], msg->num_gps_and_glonass_l1_and_l2_used_in_solution);

    // skip reserved field
    uint32_t extended_solution_status = 0;
    valid = valid && ParseUInt32(sentence.body[18], extended_solution_status, 16);
    GetExtendedSolutionStatusMessage(
        extended_solution_status, msg->extended_solution_status);

    // skip reserved field
    uint32_t signal_mask = 0;
    valid = valid && ParseUInt32(sentence.body[20], signal_mask, 16);
    GetSignalsUsed(signal_mask, msg->signal_mask);

    if (!valid)
    {
      throw ParseException("Invalid field in BESTPOS message");
    }

    return msg;
  }
};
