/**
 * @file stringUtils.h
 * @author Juri Bieler (juribieler@gmail.com)
 * @brief missing string utils that are not available in the current version of
 * std
 * @version 0.1
 * @date 2019-07-12
 *
 * @copyright Copyright (c) 2019
 *
 */

#ifndef MAIN_UTILS_STRINGUTILS_H_
#define MAIN_UTILS_STRINGUTILS_H_

#include <stdio.h>
#include <stdlib.h>
#include <cstdint>
#include <cstring>
#include <string>

std::string int_to_string(int num);
int count(std::string str, uint8_t ch);
std::string strip(std::string in_str);
std::string strip_quotes(std::string in_str);
std::string get_csv_part(std::string str, int index);

#endif /* MAIN_UTILS_STRINGUTILS_H_ */