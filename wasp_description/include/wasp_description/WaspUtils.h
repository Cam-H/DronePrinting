
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cmath>

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
