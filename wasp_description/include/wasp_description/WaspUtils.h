
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cmath>
#include <sys/stat.h>

static bool isNum(const std::string& str){
    std::string::const_iterator it = str.begin();
    while (it != str.end() && (std::isdigit(*it) || (it == str.begin() && *it == '-') || *it == ' ' || *it == '.')) ++it;
    return !str.empty() && it == str.end();
}

static void updateField(std::string value, double& field){
    if(isNum(value)){
        field = std::stod(value);
    }
}

static bool checkToken(const std::string& line, const std::string& token){
    if(token.length() > line.length()) return false;

    for(uint32_t i = 0; i < token.length(); i++){
        if(token[i] != line[i]) return false;
    }

    return true;
}

static std::string ltrim(std::string str){
    uint32_t begin = 0, end = str.length();

    for(; begin < end && str[begin] == ' '; begin++);

    return str.substr(begin, end - begin);
}

inline bool fileExists (const std::string& name) {
  struct stat buffer;
  return (stat (name.c_str(), &buffer) == 0);
}
