#ifndef CONFIG_H
#define CONFIG_H
#include <vector>

enum TrafficSign
{
    Change,
    TurnLeft,
    TurnRight,
    Stop,
    None
};

#define RED_ZONE 0
#define BLUE_ZONE 1

class Config
{
public:
    static bool DEBUG;
    static int LOOK_AT;
    static int LANE_WIDTH;
    static int WIDTH;
    static int HEIGHT;

    static int CANNY_LOW;
    static int CANNY_HIGH;

    static int LOW_H;
    static int LOW_S;
    static int LOW_V;

    static int HIGH_H;
    static int HIGH_S;
    static int HIGH_V;

    static std::vector<int> BG_SIGN;
    static std::vector<int> BLUE_SIGN;
    static std::vector<int> GREEN_SIGN;
    static std::vector<int> YELLOW_SIGN;
    static std::vector<int> CYAN_SIGN;
    static std::vector<int> VIOLET_SIGN;

    static int SKY_LINE;

    static int SIGN_LIFE;
    static int ZONE;

    static int WINDOW_WIDTH;
    static int WINDOW_HEIGHT;

    static int BTN1;
    static int BTN2;
    static int BTN3;
    static int BTN4;

    static int BL;
    static int LIGHT;
    static int SUB;

    static int softPWMLeft1;
    static int softPWMLeft2;
    static int softPWMRight1;
    static int softPWMRight2;

    static int STEER_SERVO;
    static int PROX_PIN;

    static float VELOCITY;

    static float kP;
    static float kI;
    static float kD;
};

#endif // CONFIG_H
