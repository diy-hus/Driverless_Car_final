#include <pthread.h>
#include <zbar.h>
#include "application.h"

float middle(float value, float min, float max)
{
    if( value > max )
        value = max;
    else if( value < min )
        value = min;
    return value;
}

Application::Application()
{
    startWindowThread();
    namedWindow( "Camera", CV_WINDOW_AUTOSIZE );
//    namedWindow( "HSV Slider", CV_WINDOW_AUTOSIZE );
//    createTrackbar("LOW H", "HSV Slider", &Config::YELLOW_SIGN[0], 180);  //chon nguong mau cho vong tron bien phia trong
//    createTrackbar("HIGH H", "HSV Slider", &Config::YELLOW_SIGN[3], 180);  //BG_SIGN  chon nguong mau cho hinh bao ben ngoai
//    createTrackbar("LOW S", "HSV Slider", &Config::YELLOW_SIGN[1], 255);   //dieu chinh slide HSV de chon nguong cho BG_SIGN truoc
//    createTrackbar("HIGH S", "HSV Slider", &Config::YELLOW_SIGN[4], 255);  //dien vao trong config
//    createTrackbar("LOW V", "HSV Slider", &Config::YELLOW_SIGN[2], 255);
//    createTrackbar("HIGH V", "HSV Slider", &Config::YELLOW_SIGN[5], 255);
}

void Application::Start()
{
    Mat image;
    m_car.Init();

    m_car.SetSteerAngle(30);
    time_sleep(1);
    m_car.SetSteerAngle(0);

    TrafficSign currentSign = None;
    int signCountdown = 0;
    bool start = false;
    digitalWrite(Config::LIGHT, LOW);
    while (true) {
        m_capture >> image;  //lấy hình ảnh vào
        char key = waitKey(1);

        if (key == 'q') {
            return;
        }

        if (key == 'r') {
            start = true;
        }

        if(digitalRead (Config::BTN3) == LOW)
        {
            time_sleep(1);
            if(digitalRead (Config::BTN3) == LOW)
            {
                start = false;
                break;
            }
        }

        if (!start)
        {
            imshow("Camera", image);
            if(digitalRead (Config::BTN1) == LOW)
            {
                start = true;
                time_sleep(0.25);
            }
            continue;
        }

        if (digitalRead (Config::PROX_PIN) == LOW)  //nếu căt tiêm cận thì xe se dưng
        {
            m_car.SetSpeed(0);
            m_car.SetSteerAngle(0);
            continue;
        }

        if(digitalRead (Config::BTN2) == LOW)
        {
            start = false;
            m_car.Brake();
            time_sleep(0.25);
            continue;
        }

        //video.write(image);
        image = m_detector.update(image);  //cập nhật hình ảnh mới
        if (m_detector.IsStop())
        {
            m_car.SetSteerAngle(0);  //chỉnh thẳng bánh lái
            m_car.Brake();
            time_sleep(5);
            m_car.SetSpeed(Config::VELOCITY);
            time_sleep(0.5);
        }
        else if (m_detector.HasLane())   //nếu nhìn thấy line thì chạy theo line
        {
            if (m_detector.GetSign() != None)
            {
                currentSign = m_detector.GetSign();
                digitalWrite(Config::LIGHT, HIGH);
                if (currentSign == Change) {
                    signCountdown = Config::SIGN_LIFE;
                    if (Config::ZONE == RED_ZONE) {
                        currentSign = TurnRight;
                    } else {
                        currentSign = TurnLeft;
                    }
                } else if (currentSign == Stop) {
                    signCountdown = 10;
                }
            }

            float errorAngle = 0;

            if (signCountdown != 0)
            {
                if (currentSign == TurnLeft) {
                    circle(image, cv::Point(20, 20), 20, cv::Scalar(0, 0, 255), CV_FILLED);
                } else if (currentSign == TurnRight) {
                    circle(image, cv::Point(300, 20), 20, cv::Scalar(0, 0, 255), CV_FILLED);
                } else if (currentSign == Stop) {
                    circle(image, cv::Point(160, 20), 20, cv::Scalar(0, 0, 255), CV_FILLED);
                }
                errorAngle = middle(m_detector.getErrorAngle(currentSign), -60, 60);
                signCountdown--;
                if (signCountdown == 0) {
                    digitalWrite(Config::LIGHT, LOW);
                    if (currentSign == Stop) {
                        start = false;
                        m_car.Brake();
                        time_sleep(0.25);
                        continue;
                    }
                }
            }
            else
            {
                errorAngle = middle(m_detector.getErrorAngle(None), -60, 60);
            }

            float pidAngle = PID(errorAngle);
            if (abs(errorAngle) > 10)
            {
                m_car.SetSpeed(max(Config::VELOCITY - 10, Config::VELOCITY - (abs(errorAngle) - 10)), pidAngle / 50);
            }
            else
            {
                m_car.SetSpeed(Config::VELOCITY, pidAngle / 50);
            }
            m_car.SetSteerAngle(pidAngle);
        } else {		//trường hợp không thấy line thì chạy thẳng
            m_car.SetSpeed(Config::VELOCITY / 3.0 * 2.0);
            m_car.SetSteerAngle(PID(m_preError));
        }
        imshow("Camera", image);
    }
}

float Application::PID(float error)
{
    float Pout = Config::kP * error;

    m_integral += error;

    if (m_integral > 1) m_integral = 1;
    else if (m_integral < -1) m_integral = -1;
    float Iout = Config::kI * m_integral;

    float Dout = Config::kD * (error - m_preError);

    float output = Pout + Dout;

    output = middle(output, -60, 60);

    m_preError = error;

    return output;
}
