#pragma once
#include <termios.h>
#include <sstream>
#include <vector>
#include "ros/ros.h"
class KeyboardEvent{
    static int  input_flag; //keyboard flag
    static char buff; // keyboard buff
    static char getch();// get keyboard input. We need all instance to use the same function to catch the keyboard
    vector<char> keynumberlist; // key number list corresponding to
    vector<void (*)(void)> callbacklist;
    void KeyboardPress();
    unsigned int number_of_callbacks;//number of callback functions
public:
    void RosWhileLoopRun(); // this should be
    void KeyboardEvent();
    void PushCallBack(char keynumber,void(*callback)(void)); // push all the preset function callbacks into callbacklist
    void BendingComplete();
};
