#include "utils.h"
#include <iostream>
#include <string>

using Clock = std::chrono::steady_clock;

unsigned int CStopWatch::get_duration() {
  return std::chrono::duration_cast<std::chrono::milliseconds>(end_t - start_t).count();
}

void CStopWatch::start() {
  start_t = Clock::now();
}

void CStopWatch::stop() {
  end_t = Clock::now();
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
