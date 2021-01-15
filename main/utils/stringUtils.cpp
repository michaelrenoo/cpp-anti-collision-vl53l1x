
#include "stringUtils.hpp"

/**
 * @brief since we use the oldest compiler there is std::to_string is not
 * available -.-
 *
 * @param num
 * @return std::string
 */
std::string int_to_string(int num) {
  uint8_t buff[20];
  memset(buff, 0x00, 20);
  sprintf((char*)buff, "%d", num);
  return std::string((char*)buff);
}

int count(std::string str, char ch) {
  int count = 0;
  for (int i = 0; i < str.size(); i++)
    if (str[i] == ch) count++;
  return count;
}

std::string strip(std::string in_str) {
  std::string out = in_str;
  if (out.length() == 0) return std::string("");
  while (out.at(0) == ' ') {
    if (out.length() <= 1) return std::string("");
    out = out.substr(1);
  }
  if (out.length() == 0) return std::string("");
  while (out.at(out.length() - 1) == ' ') {
    if (out.length() <= 1) return std::string("");
    out = out.substr(0, out.length() - 1);
  }
  return out;
}

std::string strip_quotes(std::string in_str) {
  std::string out = in_str;
  if (out.length() == 0) return std::string("");
  if (out.at(0) == '\"') {
    out = out.substr(1);
  }
  if (out.length() == 0) return std::string("");
  if (out.at(out.length() - 1) == '\"') {
    out = out.substr(0, out.length() - 1);
  }
  return out;
}

std::string get_csv_part(std::string str, int index) {
  std::string sub_str = str;
  int sep_pos;
  for (int i = 0; i <= index; i++) {
    sep_pos = sub_str.find(',');
    if (i == index) {
      return strip(sub_str.substr(0, sep_pos));
    }
    if (sep_pos == sub_str.length()) {
      return std::string("");
    }
    sub_str = sub_str.substr(sep_pos + 1);
  }
  return std::string("");
}