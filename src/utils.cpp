#include "utils.h"
#include <iostream>
#include <string>
#include <time.h>


float CStopWatch::get_duration() {
  return (float)(finish_t - start_t) / CLOCKS_PER_SEC;
}

void CStopWatch::start() {
  start_t = clock();
}

void CStopWatch::stop() {
  finish_t = clock();
}

// Returns the current date and time
const std::string current_date_time() {
  time_t     now = time(0);
  struct tm  tstruct;
  char       buf[80];
  tstruct = *localtime(&now);
  strftime(buf, sizeof(buf), "%Y-%m-%d-%H%M%S", &tstruct);

  return buf;
}

const std::string get_file_name() {
  std::string input = "";
  std::string filename;
  // std::cout << "\nChoose a file name (default)" << std::endl;
  // getline(std::cin, input);
  if (input == "") {
    return current_date_time();
  }
  return input;
}
