#pragma once

#include <ros/ros.h>
#include <string>
using std::to_string;
using std::string;

class LogHeader{
    public:
        string m_file;
        string m_robot_id;

        LogHeader();

        string getHeader();

        string getHeader(string s);

        void error(string s);

        void debug(string s);
        void debugOnce(string s);

        void info(string s);
};
