#pragma once

#include <sstream>
#include <cstdarg>
#include <iostream>

// #define COLOR_RED     "\033[31m"
// #define COLOR_GREEN   "\033[32m"
// #define COLOR_RESET   "\033[0m"
// #define COLOR_LYELLOW "\033[1;33m"
// #define COLOR_LGREEN  "\033[1;32m"
// #define COLOR_LBLUE   "\033[1;34m"

#define COLOR_RED     ""
#define COLOR_GREEN   ""
#define COLOR_RESET   ""
#define COLOR_LYELLOW ""
#define COLOR_LGREEN  ""
#define COLOR_LBLUE   ""

namespace Utils {
    inline void Log(const int& type, const char* format, ...) {
        std::ostringstream oss;

        if (type == 1) {
            oss << COLOR_LGREEN << "[+] ";
        }
        else if (type == 2) {
            oss << COLOR_RED << "[X] ";
        }
        else {
            oss << COLOR_LYELLOW << "[-] ";
        }

        va_list args;
        va_start(args, format);
        char buffer[256];
        std::vsnprintf(buffer, sizeof(buffer), format, args);
        va_end(args);

        oss << buffer;
        oss << COLOR_RESET;

        std::cout << oss.str() << std::endl;
    }
} 