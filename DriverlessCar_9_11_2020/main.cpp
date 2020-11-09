#include "application.h"

int main(int argc, char *argv[])
{
    Application app;

    system("clear");

    if (argc >= 2)
    {
        int speed = atoi(argv[1]);
        std::cout << "Car Speed: " << speed << std::endl;
        Config::VELOCITY = speed;
    }

    if (argc >= 3)
    {
        int brightness = atoi(argv[2]);
        std::cout << "Brightness: " << brightness << std::endl;
        Config::HIGH_V = brightness;
    }

    if (argc >= 4)
    {
        int signLife = atoi(argv[3]);
        std::cout << "Sign Life: " << signLife << std::endl;
        Config::SIGN_LIFE = signLife;
    }

    if (argc >= 5)
    {
        int zone = atoi(argv[4]);
        if (zone == RED_ZONE) {
            std::cout << "Zone: Red" << std::endl;
        } else {
            std::cout << "Zone: BLue" << std::endl;
        }
        Config::ZONE = zone;
    }

    app.Start();

    return 0;
}
