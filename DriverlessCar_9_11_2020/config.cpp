#include "config.h"

bool Config::DEBUG = false;

int Config::ZONE = RED_ZONE;

int Config::SIGN_LIFE = 40;

int Config::LANE_WIDTH = 160;

int Config::WIDTH = 320;
int Config::HEIGHT = 240;

float Config::VELOCITY = 50;

int Config::SKY_LINE = 120;

int Config::CANNY_LOW = 25;
int Config::CANNY_HIGH = 50;

int Config::LOW_H = 0;
int Config::LOW_S = 0;
int Config::LOW_V = 0;

int Config::HIGH_H = 180;
int Config::HIGH_S = 255;
int Config::HIGH_V = 100;   // Do sang moi truong

// Traffic Sign Threshold
std::vector<int> Config::BG_SIGN = {150, 40, 40, 180, 255, 200};
std::vector<int> Config::BLUE_SIGN = {105, 20, 20, 160, 200, 200};
std::vector<int> Config::GREEN_SIGN = {5, 20, 20, 65, 180, 200};
std::vector<int> Config::YELLOW_SIGN = {15, 40, 40, 30, 255, 200};
std::vector<int> Config::CYAN_SIGN = {150, 40, 40, 180, 180, 200};
std::vector<int> Config::VIOLET_SIGN = {150, 40, 40, 180, 180, 200};

// Button Pin

int Config::BTN1 = 15;

int Config::BTN2 = 7;

int Config::BTN3 = 16;

int Config::BTN4 = 12;

int Config::BL = 5;

int Config::LIGHT = 10;

int Config::SUB = 25;

// SoftPWM Pin

int Config::softPWMLeft1 = 21;

int Config::softPWMLeft2 = 30;

int Config::softPWMRight1 = 24;

int Config::softPWMRight2 = 23;

// Servo Pin

int Config::STEER_SERVO = 12;

int Config::PROX_PIN = 14;

float Config::kP = 0.9f;
float Config::kI = 0.1f;
float Config::kD = 0.4f;

