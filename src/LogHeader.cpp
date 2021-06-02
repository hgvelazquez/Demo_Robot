#include "LogHeader.h"

using std::to_string;
using std::string;

LogHeader::LogHeader(){
    m_file = "";
    m_robot_id = "";
}

string LogHeader::getHeader(){
    return m_file+"["+m_robot_id+"]: ";
}

string LogHeader::getHeader(string s){
    return getHeader()+s;
}

void LogHeader::error(string s){
    string msg = getHeader(s);
    ROS_ERROR("%s",msg.c_str());
}

void LogHeader::debug(string s){
    string msg = getHeader(s);
    ROS_DEBUG("%s",msg.c_str());
}

void LogHeader::debugOnce(string s){
    string msg = getHeader(s);
    ROS_DEBUG_ONCE("%s",msg.c_str());
}

void LogHeader::info(string s){
    string msg = getHeader(s);
    ROS_INFO("%s",msg.c_str());
}