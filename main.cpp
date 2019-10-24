#include <pigpio.h>
#include "PigpioMS/PigpioMS.hpp"
#include "RasPiDS3/RasPiDS3.hpp"
#include "Sensor-master/GY521/GY521.hpp"
#include <iostream>
#include <cmath>
#include <unistd.h>
#include <chrono>

using namespace RPDS3;
using namespace RPMS;
using namespace RPGY521;

MotorSerial ms;
DualShock3 ds3;

int main(void)
{

    constexpr int RUNLED = 13;
    constexpr int SLEEPLED = 27;

    //------MDD------//
    constexpr int BOTTOM_MDD = 2;
    constexpr int DOWN_MDD = 11;
    constexpr int UP_MDD = 10;
    constexpr int TOP_MDD = 16;
    constexpr int TAPE_LED = 12;

    //-----PORT------//
    constexpr int UNC_PORT = 2;
    constexpr int SOLENOID_PORT = 4;
    constexpr int ARM_PORT = 3;

    constexpr int TAPELED = 84;

    constexpr int RIGHT_TOP_LIMIT = 12;
    constexpr int RIGHT_BOTTOM_LIMIT = 16;
    constexpr int LEFT_TOP_LIMIT = 11;
    constexpr int LEFT_BOTTOM_LIMIT = 22;

    constexpr int Y_FRONT_LIMIT = 26;
    constexpr int Y_BACK_LIMIT = 19;
    constexpr int Z_TOP_LIMIT = 9;
    constexpr int Z_BOTTOM_LIMIT = 10;

    constexpr int TOWEL_ARM_PWM = 80;
    constexpr int Y_ARM_PWM = 250;
    constexpr int Z_ARM_PWM = 200;

    ds3.update();
    try
    {
        ms.init();
    }
    catch (std::runtime_error exception)
    {
        std::cout << "error" << std::endl;
        return -1;
    }

    gpioSetMode(13, PI_OUTPUT);
    gpioWrite(13, true);
    gpioSetMode(27, PI_OUTPUT);

    gpioSetMode(RIGHT_TOP_LIMIT, PI_INPUT);
    gpioSetPullUpDown(RIGHT_TOP_LIMIT, PI_PUD_UP);
    gpioSetMode(RIGHT_BOTTOM_LIMIT, PI_INPUT);
    gpioSetPullUpDown(RIGHT_BOTTOM_LIMIT, PI_PUD_UP);
    gpioSetMode(LEFT_TOP_LIMIT, PI_INPUT);
    gpioSetPullUpDown(LEFT_TOP_LIMIT, PI_PUD_UP);
    gpioSetMode(LEFT_BOTTOM_LIMIT, PI_INPUT);
    gpioSetPullUpDown(LEFT_BOTTOM_LIMIT, PI_PUD_UP);

    gpioSetMode(Y_FRONT_LIMIT, PI_INPUT);
    gpioSetPullUpDown(Y_FRONT_LIMIT, PI_PUD_UP);
    gpioSetMode(Y_BACK_LIMIT, PI_INPUT);
    gpioSetPullUpDown(Y_BACK_LIMIT, PI_PUD_UP);
    gpioSetMode(Z_TOP_LIMIT, PI_INPUT);
    gpioSetPullUpDown(Z_TOP_LIMIT, PI_PUD_UP);
    gpioSetMode(Z_BOTTOM_LIMIT, PI_INPUT);
    gpioSetPullUpDown(Z_BOTTOM_LIMIT, PI_PUD_UP);

    double regulation = 1;
    bool sleep_flag = false;
    int coat_select = 0;
    bool hanger_flag = false;
    int tape_led_mode = 0;
    int arm_status = 0;
    int towel_arm_status = 0;

    std::chrono::steady_clock::time_point z_fall_start, z_fall_now, y_pull_start, y_pull_now, box_start, box_now;
    while (coat_select == 0)
    {
        ds3.update();
        if (ds3.button(SELECT) && ds3.button(CROSS))
        {
            coat_select = 1;
            std::cout << "RED" << std::endl;
        }
        else if (ds3.button(SELECT) && ds3.button(TRIANGLE))
        {
            coat_select = -1;
            std::cout << "BLUE" << std::endl;
        }
    }

    while (!(ds3.button(SELECT) && ds3.button(START)))
    {
    }

    RPGY521::GY521 gyro;
    std::cout << "Calibration finished" << std::endl;
    std::cout << "Start Main program" << std::endl;

    gyro.start();
    ds3.yReverseSet(true);

    while (!(ds3.button(SELECT) && ds3.button(START)))
    {
        ds3.update();
        gyro.updata();
        //--------------------------------coat change-----------------------------------//
        if (ds3.button(SELECT) && ds3.press(TRIANGLE))
        {
            coat_select = -coat_select;
            ((coat_select == 1) ? tape_led_mode = 2 : tape_led_mode = 3);
        }
        //--------------------------------sleep mode-------------------------------------//
        do
        {
            ds3.update();
            if ((ds3.button(SELECT) && ds3.button(SQUARE)))
            {
                (sleep_flag == true) ? sleep_flag = false : sleep_flag = true;
                if (sleep_flag == false)
                {
                    std::cout << "zzz" << std::endl;
                }
                else
                {
                    std::cout << "wake up" << std::endl;
                }
            }
        } while (sleep_flag == true);
        //-------------------------------undercarriage-------------------------------------//
        double left_x = ds3.stick(LEFT_X);
        double left_y = ds3.stick(LEFT_Y);
        std::array<double, 4> wheel_velocity;

        if (ds3.button(SELECT) && ds3.button(LEFT))
        {
            gyro.resetYaw(0);
            std::cout << "gyro was reseted" << std::endl;
        }

        ds3.button(R1) ? regulation = 0.5 : regulation = 1;

        double gyro_rad = gyro.yaw * M_PI / 180;
        double rotation = (ds3.stick(RIGHT_T) - ds3.stick(LEFT_T)) * 0.3;

        wheel_velocity[0] = (-std::sin(M_PI / 4 + gyro_rad) * left_x + std::cos(M_PI / 4 + gyro_rad) * left_y + rotation) * regulation;
        wheel_velocity[1] = (-std::cos(M_PI / 4 + gyro_rad) * left_x + std::sin(M_PI / 4 + gyro_rad) * left_y + rotation) * regulation;

        wheel_velocity[2] = -wheel_velocity[0];
        wheel_velocity[3] = -wheel_velocity[1];

        ms.send(BOTTOM_MDD, UNC_PORT, wheel_velocity[1] * 1.6 * regulation + rotation);
        ms.send(DOWN_MDD, UNC_PORT, wheel_velocity[2] * 1.6 * regulation + rotation);
        ms.send(UP_MDD, UNC_PORT, wheel_velocity[0] * 1.6 * regulation + rotation);
        ms.send(TOP_MDD, UNC_PORT, wheel_velocity[3] * 1.6 * regulation + rotation);
        //-----------------------------hanger------------------------------------------------//

        if (ds3.press(SQUARE) && !(ds3.button(SELECT)))
        {
            ((hanger_flag == true) ? hanger_flag = false : hanger_flag = true);
            if (hanger_flag == true)
            {
                ms.send(UP_MDD, SOLENOID_PORT, 251);
                ms.send(UP_MDD, SOLENOID_PORT, 252);
                tape_led_mode = 6;
            }
            else
            {
                ms.send(UP_MDD, SOLENOID_PORT, 0);
                tape_led_mode = 7;
            }
        }

        bool limit_uneffect = false;
        if (ds3.button(SELECT) && ds3.press(CIRCLE))
        {
            ((limit_uneffect = true) ? limit_uneffect = false : limit_uneffect = true);
        }
        //-----------------------------arm-------------------------------------------------//
        bool y_front_limit = false;
        bool y_back_limit = false;
        bool z_top_limit = false;
        bool z_bottom_limit = false;
        bool right_top_limit = false;
        bool right_bottom_limit = false;
        bool left_top_limit = false;
        bool left_bottom_limit = false;

        bool recover = false;

        if (limit_uneffect == false)
        {
            y_front_limit = gpioRead(Y_FRONT_LIMIT);
            y_back_limit = gpioRead(Y_BACK_LIMIT);
            z_top_limit = gpioRead(Z_TOP_LIMIT);
            z_bottom_limit = gpioRead(Z_BOTTOM_LIMIT);
            right_top_limit = gpioRead(RIGHT_TOP_LIMIT);
            right_bottom_limit = gpioRead(RIGHT_BOTTOM_LIMIT);
            left_top_limit = gpioRead(LEFT_TOP_LIMIT);
            left_bottom_limit = gpioRead(LEFT_BOTTOM_LIMIT);
        }

        double right_x = ds3.stick(RIGHT_X);
        double right_y = ds3.stick(RIGHT_Y);
        double right_theta = std::atan2(right_x, right_y);
        double right_distance = std::hypot(right_x, right_y);

        int sent_y = 0;
        int sent_z = 0;

        if (right_distance >= 20)
        {
            if (right_theta >= (M_PI / 4) && right_theta <= (M_PI / 4) * 3)
            {
                sent_y = 0;
                z_top_limit == false ? sent_z = -right_y * 1.8 : sent_z = 0;
            }
            else if (right_theta > (M_PI / 4) * 3 || right_theta < -(M_PI / 4) * 3)
            {
                sent_z = 0;
                y_front_limit == false ? sent_y = right_x * 1.8 : sent_y = 0;
            }
            else if (right_theta >= -(M_PI / 4) * 3 && right_theta <= -(M_PI / 4))
            {
                sent_y = 0;
                z_bottom_limit == true ? sent_z = -right_y * 1.8 : sent_z = 0;
            }
            else if (right_theta > -(M_PI / 4) && right_theta < (M_PI / 4))
            {
                sent_z = 0;
                y_back_limit == false ? sent_y = right_x * 1.8 : sent_y = 0;
            }
            recover = false;
            arm_status = 0;
        }
        else
        {
            if (ds3.press(CROSS) == true)
            {
                recover == true ? recover = false : recover = true;
            }

            if (recover == true)
            {
                switch (arm_status)
                {
                case 0:
                    sent_z = Z_ARM_PWM;
                    if (z_top_limit == true)
                    {
                        sent_z = 0;
                        arm_status = 1;
                    }
                    break;
                case 1:
                    sent_y = Y_ARM_PWM;
                    if (y_front_limit == true)
                    {
                        sent_y = 0;
                        arm_status = 2;
                        z_fall_start = std::chrono::steady_clock::now();
                    }
                    break;

                case 2:
                    z_fall_now = std::chrono::steady_clock::now();
                    sent_z = -Z_ARM_PWM;
                    auto z_fall_time = std::chrono::duration_cast<std::chrono::milliseconds>(z_fall_start - z_fall_now);
                    if (z_bottom_limit == true || z_fall_time.count() > 1500)
                    {
                        sent_z = 0;
                        arm_status = 3;
                        y_pull_start = std::chrono::steady_clock::now();
                    }
                    break;

                case 3:
                    y_pull_now = std::chrono::steady_clock::now();
                    sent_y = -Y_ARM_PWM;
                    auto y_pull_time = std::chrono::duration_cast<std::chrono::milliseconds>(y_pull_start - y_pull_now);
                    if (y_back_limit == true || y_pull_time.count() > 2500)
                    {
                        sent_y = 0;
                        arm_status = 0;
                    }
                    break;
                }
            }
            ms.send(TOP_MDD, ARM_PORT, sent_y * regulation);
            ms.send(UP_MDD, ARM_PORT, -sent_z * regulation);

            int sent_right = 0; //Right from the circuit side
            int sent_left = 0;  //Left from the circuit side
            bool box_permission = false;

            if (ds3.press(CIRCLE))
            {
                //stop_towel_arm = !(stop_towel_arm);
                switch (towel_arm_status)
                {
                case 0: // Primary status that doesn't detect
                    towel_arm_status++;
                    sent_right = 0;
                    sent_left = 0;
                    break;
                case 1:
                    if (limit_uneffect == false)
                    {
                        right_top_limit == true ? sent_right = 0 : sent_right = TOWEL_ARM_PWM;
                        left_top_limit == true ? sent_left = 0 : sent_left = TOWEL_ARM_PWM;
                    }

                    if (right_top_limit == true || left_top_limit == true)
                    {
                        towel_arm_status++;
                        box_permission = true;
                    }
                    break;
                case 2:
                    if (limit_uneffect == false)
                    {
                        right_bottom_limit == true ? sent_right = 0 : sent_right = TOWEL_ARM_PWM;
                        left_top_limit == true ? sent_left = 0 : sent_left = TOWEL_ARM_PWM;
                    }
                    if (right_top_limit == true || left_top_limit == true)
                    {
                        towel_arm_status = 0;
                    }
                    break;
                }
            }
            //-------------------------------box-------------------------------//
            if (ds3.press(TRIANGLE) && !(ds3.button(SELECT)))
            {
                box_start = std::chrono::steady_clock::now();
            }

            box_now = std::chrono::steady_clock::now();
            auto box_wait_time = std::chrono::duration_cast<std::chrono::microseconds>(box_start - box_now);

            if (box_wait_time.count() > 300)
            {
                if (box_permission == true)
                {
                    ms.send(TOP_MDD, SOLENOID_PORT, 251);
                    towel_arm_status = 1;
                }
            }
        }
    }
