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

#ifndef NOVATEL_GPS_DRIVER_NOVATEL_SENTENCE_H
#define NOVATEL_GPS_DRIVER_NOVATEL_SENTENCE_H

#include <string>
#include <vector>

namespace novatel_gps_driver
{
  /**
   * Contains an ASCII NovAtel sentence that has been tokenized into a vector
   * of strings.
   */
  struct NovatelSentence
  {
    std::string id;
    std::vector<std::string> header;
    std::vector<std::string> body;
  };
}

#endif //NOVATEL_GPS_DRIVER_NOVATEL_SENTENCE_H
