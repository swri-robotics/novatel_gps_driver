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
#include <novatel_gps_driver/parsers/gpgsv.h>
#include <novatel_gps_driver/novatel_message_extractor.h>

#include <gtest/gtest.h>

TEST(ParserTestSuite, testBestposAsciiParsing)
{
  novatel_gps_driver::BestposParser parser;
  std::string bestpos_str = "#BESTPOSA,ICOM1,0,87.5,FINESTEERING,1956,157432.000,00000800,7145,6938;SOL_COMPUTED,SINGLE,29.44391220792,-98.61476921244,261.4344,-26.0000,WGS84,2.1382,3.1092,4.0429,\"\",0.000,0.000,8,8,8,8,0,06,00,03*ecf2202b\r\n";
  std::string extracted_str;

  novatel_gps_driver::NovatelMessageExtractor extractor;

  std::vector<novatel_gps_driver::NmeaSentence> nmea_sentences;
  std::vector<novatel_gps_driver::NovatelSentence> novatel_sentences;
  std::vector<novatel_gps_driver::BinaryMessage> binary_messages;
  std::string remaining;

  extractor.ExtractCompleteMessages(bestpos_str, nmea_sentences, novatel_sentences,
                                    binary_messages, remaining);

  ASSERT_EQ(0, nmea_sentences.size());
  ASSERT_EQ(0, binary_messages.size());
  ASSERT_EQ(1, novatel_sentences.size());

  novatel_gps_driver::NovatelSentence sentence = novatel_sentences.front();

  ASSERT_EQ(parser.GetMessageName() + "A", sentence.id);

  novatel_gps_msgs::NovatelPositionPtr msg = parser.ParseAscii(sentence);

  ASSERT_NE(msg.get(), nullptr);

  ASSERT_EQ("SOL_COMPUTED", msg->solution_status);
  ASSERT_DOUBLE_EQ(29.44391220792, msg->lat);
  ASSERT_DOUBLE_EQ(-98.61476921244, msg->lon);
}

TEST(ParserTestSuite, testGpgsvParsing)
{
  novatel_gps_driver::GpgsvParser parser;
  std::string sentence_str = "$GPGSV,3,3,11,12,07,00.,32,13,03,227,36,22,0.,041,*4A\r\n";
  std::string extracted_str;

  novatel_gps_driver::NovatelMessageExtractor extractor;

  std::vector<novatel_gps_driver::NmeaSentence> nmea_sentences;
  std::vector<novatel_gps_driver::NovatelSentence> novatel_sentences;
  std::vector<novatel_gps_driver::BinaryMessage> binary_messages;
  std::string remaining;

  extractor.ExtractCompleteMessages(sentence_str, nmea_sentences, novatel_sentences,
                                    binary_messages, remaining);

  ASSERT_EQ(1, nmea_sentences.size());
  ASSERT_EQ(0, binary_messages.size());
  ASSERT_EQ(0, novatel_sentences.size());

  novatel_gps_driver::NmeaSentence sentence = nmea_sentences.front();

  ASSERT_EQ(parser.GetMessageName(), sentence.id);
  ASSERT_FALSE(sentence.body.empty());

  novatel_gps_msgs::GpgsvPtr msg = parser.ParseAscii(sentence);

  ASSERT_NE(msg.get(), nullptr);

  ASSERT_EQ(3, msg->n_msgs);
  ASSERT_EQ(3, msg->msg_number);
  ASSERT_EQ(3, msg->satellites.size());
  ASSERT_EQ(11, msg->n_satellites);
  ASSERT_EQ(12, msg->satellites[0].prn);
  ASSERT_EQ(7, msg->satellites[0].elevation);
  ASSERT_EQ(0, msg->satellites[0].azimuth);
  ASSERT_EQ(32, msg->satellites[0].snr);
  ASSERT_EQ(13, msg->satellites[1].prn);
  ASSERT_EQ(3, msg->satellites[1].elevation);
  ASSERT_EQ(227, msg->satellites[1].azimuth);
  ASSERT_EQ(36, msg->satellites[1].snr);
  ASSERT_EQ(22, msg->satellites[2].prn);
  ASSERT_EQ(0, msg->satellites[2].elevation);
  ASSERT_EQ(41, msg->satellites[2].azimuth);
  ASSERT_EQ(-1, msg->satellites[2].snr);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}