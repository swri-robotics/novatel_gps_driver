// *****************************************************************************
//
// Copyright (c) 2019, Southwest Research Institute® (SwRI®)
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of Southwest Research Institute® (SwRI®) nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL SOUTHWEST RESEARCH INSTITUTE BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// *****************************************************************************


#include <sstream>

#include <novatel_gps_driver/parsers/insupdatestatus.h>

#include <novatel_gps_driver/parsers/header.h>

namespace novatel_gps_driver
{
  namespace
  {
    const size_t MAX_DMI_UPDATE_STATUS = 5;
    const std::string DMI_UPDATE_STATUSES[] = {
        "INACTIVE", "ACTIVE", "USED", "RESERVED", "BAD_MISC", "HIGH_ROTATION"};
    const size_t MAX_ALIGN_UPDATE_STATUS = 5;
    const std::string ALIGN_UPDATE_STATUSES[] = {
        "INACTIVE", "ACTIVE", "USED", "RESERVED", "RESERVED", "BAD_MISC"};
  }

  const std::string InsUpdateStatusParser::MESSAGE_NAME = "INSUPDATESTATUS";

  uint32_t InsUpdateStatusParser::GetMessageId() const
  {
    return MESSAGE_ID;
  }

  const std::string InsUpdateStatusParser::GetMessageName() const
  {
    return MESSAGE_NAME;
  }

  InsUpdateStatusParser::MessageType InsUpdateStatusParser::ParseBinary(const BinaryMessage& bin_msg) noexcept(false)
  {
    if (bin_msg.data_.size() != BINARY_LENGTH)
    {
      std::stringstream error;
      error << "Unexpected INSUPDATESTATUS message length: " << bin_msg.data_.size();
      throw ParseException(error.str());
    }
    auto ros_msg = std::make_unique<novatel_gps_msgs::msg::NovatelInsUpdateStatus>();
    HeaderParser header_parser;
    ros_msg->novatel_msg_header = header_parser.ParseBinary(bin_msg);
    ros_msg->novatel_msg_header.message_name = MESSAGE_NAME;

    uint32_t pos_type = ParseUInt32(&bin_msg.data_[0]);
    if (pos_type > MAX_POSITION_TYPE)
    {
      std::stringstream error;
      error << "Unknown position type: " << pos_type;
      throw ParseException(error.str());
    }
    ros_msg->position_type = POSITION_TYPES[pos_type];
    ros_msg->num_psr = ParseInt32(&bin_msg.data_[4]);
    ros_msg->num_adr = ParseInt32(&bin_msg.data_[8]);
    ros_msg->num_dop = ParseInt32(&bin_msg.data_[12]);
    uint32_t dmi_status = ParseUInt32(&bin_msg.data_[16]);
    if (dmi_status > MAX_DMI_UPDATE_STATUS)
    {
      std::stringstream error;
      error << "Unknown DMI update status: " << dmi_status;
      throw ParseException(error.str());
    }
    ros_msg->dmi_update_status = DMI_UPDATE_STATUSES[dmi_status];
    uint32_t align_status = ParseUInt32(&bin_msg.data_[20]);
    if (align_status > MAX_ALIGN_UPDATE_STATUS)
    {
      std::stringstream error;
      error << "Unknown ALIGN update status: " << align_status;
      throw ParseException(error.str());
    }
    ros_msg->align_update_status = ALIGN_UPDATE_STATUSES[align_status];
    GetInsExtendedSolutionStatusMessage(ParseUInt32(&bin_msg.data_[24]), ros_msg->extended_solution_status);
    ros_msg->ins_enabled_updates = ParseUInt32(&bin_msg.data_[28]);

    return ros_msg;
  }

  InsUpdateStatusParser::MessageType InsUpdateStatusParser::ParseAscii(const NovatelSentence& sentence) noexcept(false)
  {
    auto msg = std::make_unique<novatel_gps_msgs::msg::NovatelInsUpdateStatus>();
    HeaderParser h_parser;
    msg->novatel_msg_header = h_parser.ParseAscii(sentence);

    if (sentence.body.size() != ASCII_LENGTH)
    {
      std::stringstream error;
      error << "Unexpected number of INSUPDATESTATUS message fields: " << sentence.body.size();
      throw ParseException(error.str());
    }

    bool valid = true;

    msg->position_type = sentence.body[0];
    valid = valid && ParseInt32(sentence.body[1], msg->num_psr);
    valid = valid && ParseInt32(sentence.body[2], msg->num_adr);
    valid = valid && ParseInt32(sentence.body[3], msg->num_dop);
    msg->dmi_update_status = sentence.body[4];
    msg->align_update_status = sentence.body[5];
    uint32_t extended_solution_status = 0;
    valid = valid && ParseUInt32(sentence.body[6], extended_solution_status, 16);
    GetInsExtendedSolutionStatusMessage(extended_solution_status, msg->extended_solution_status);
    valid = valid && ParseUInt32(sentence.body[7], msg->ins_enabled_updates, 16);
    // skip two reserved fields

    if (!valid)
    {
      throw ParseException("Invalid field in INSUPDATESTATUS message");
    }

    return msg;
  }
}
