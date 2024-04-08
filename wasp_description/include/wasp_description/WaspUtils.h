
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cmath>
#include <sys/stat.h>

static std::string ltrim(std::string str){
    uint32_t begin = 0, end = str.length();

    for(; begin < end && str[begin] == ' '; begin++);

    return str.substr(begin, end - begin);
}

static bool isNum(std::string str){
    str = ltrim(str);

    std::string::const_iterator it = str.begin();
    while (it != str.end() && (std::isdigit(*it) || (it == str.begin() && *it == '-') || *it == '.')) ++it;
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

inline bool fileExists (const std::string& name) {
  struct stat buffer;
  return (stat (name.c_str(), &buffer) == 0);
}

struct vec3{
    double x;
    double y;
    double z;

    bool operator==(const vec3& a){
        return x == a.x && y == a.y && z == a.z;
    }

    bool operator!=(const vec3& a){
        return !(*this == a);
    }

    vec3 operator+(const vec3& a){
        return {x + a.x, y + a.y, z + a.z};
    }

    vec3 operator-(const vec3& a){
        return {x - a.x, y - a.y, z - a.z};
    }

    vec3 operator*(double scalar) const {
        return {x * scalar, y * scalar, z * scalar};
    }

    vec3 operator/(double scalar) const {
        return {x / scalar, y / scalar, z / scalar};
    }

    double length(){
        return sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
    }

    static double distance(const vec3& a, const vec3& b){
        return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2) + pow(a.z - b.z, 2));
    }

};
