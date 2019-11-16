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
    /*constexpr int RIGHT_UP   = 10;
      constexpr int RIGHT_DOWN =  2;
      constexpr int LEFT_UP    = 11;//どっちかと言うと16v
      constexpr int LEFT_DOWN  = 16;//''*/

    //-----PORT------//
    constexpr int MOTOR    = 2;
    constexpr int TOWEL    = 3;
    constexpr int HANGER   = 2;
    constexpr int SOLENOID = 4;
    constexpr int ARM      = 4;

    //constexpr int TAPELED = 84;

    constexpr int RIGHT_TOP_LIMIT = 12;
    constexpr int RIGHT_BOTTOM_LIMIT = 16;//16
    constexpr int LEFT_TOP_LIMIT = 11;//11
    constexpr int LEFT_BOTTOM_LIMIT = 22;//22

    //constexpr int Y_FRONT_LIMIT = 26;
    //constexpr int Y_BACK_LIMIT = 19;
    constexpr int Z_TOP_LIMIT = 9;
    constexpr int Z_BOTTOM_LIMIT = 10;

    constexpr int TOWEL_ARM_PWM = 160;
    constexpr int Y_ARM_PWM = 250;
    constexpr int Z_ARM_PWM = 200;

    constexpr bool final_mode = false;

    int led_mode     = 0;
    bool sent_or_not = true;
    int priventeer   = led_mode;

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

    gpioInitialise();

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

    //gpioSetMode(Y_FRONT_LIMIT, PI_INPUT);
    //gpioSetPullUpDown(Y_FRONT_LIMIT, PI_PUD_UP);
    //gpioSetMode(Y_BACK_LIMIT, PI_INPUT);
    //gpioSetPullUpDown(Y_BACK_LIMIT, PI_PUD_UP);
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
    int box_time = 0;

    std::chrono::steady_clock::time_point z_fall_start, z_fall_now, y_pull_start,y_pull_now,box_start,box_now;
    std::chrono::milliseconds z_fall_time,y_pull_time,box_wait_time;
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
        gpioWrite(SLEEPLED,true);
    }

    while (!(ds3.button(SELECT) && ds3.button(START)))
    {
        ds3.update();
        gpioWrite(RUNLED,true);
        gpioWrite(RUNLED,false);
    }


    RPGY521::GY521 gyro;
    std::cout << "Calibration finished" << std::endl;
    std::cout << "Start Main program" << std::endl;

    gyro.start();
    ds3.yReverseSet(true);


    double now_rotation_vel  = 0;
    double prev_rotation_vel = 0;

    while (!(ds3.button(START) && ds3.button(RIGHT))){
        ds3.update();
        if(ds3.button(SELECT) && ds3.press(SQUARE)){
            sleep_flag = false;
            std::cout << "wake up" << std::endl;
        }


        while (sleep_flag == false)
        {
            /*std::cout << "rt" << gpioRead(RIGHT_TOP_LIMIT) << std::endl;
              std::cout << "rb" << gpioRead(RIGHT_BOTTOM_LIMIT) << std::endl;
              std::cout << "lt" << gpioRead(LEFT_TOP_LIMIT) << std::endl;
              std::cout << "lb" << gpioRead(LEFT_BOTTOM_LIMIT) << std::endl;
              */
            gpioWrite(RUNLED,true);
            gpioWrite(SLEEPLED,true);
            ds3.update();
            gyro.updata();
            //--------------------------------coat change-----------------------------------//
            if (ds3.button(SELECT) && ds3.press(TRIANGLE))
            {
                coat_select = -coat_select;
                ((coat_select == 1) ? tape_led_mode = 2 : tape_led_mode = 3);
            }

            //--------------------------------sleep mode-------------------------------------//
            if ((ds3.button(SELECT) && ds3.press(SQUARE)))
            {
                sleep_flag = true;
                std::cout << "zzz" << std::endl;
                led_mode  = 4;
            }
            //-------------------------------undercarriage-------------------------------------//

            double left_x = ds3.stick(LEFT_X);
            double left_y = ds3.stick(LEFT_Y);
            std::array<double, 3> wheel_velocity;
            static std::chrono::steady_clock::time_point time_count,prev_time_count;
            static double dest_angle = 0;//目標角度
            static double diff_dest = 0;//現在の誤差
            static double prev_diff_dest = 0;//前の誤差
            //static double prev_prev_diff_dest = 0;//前の前の誤差
            static double diff_vel = 0;//速度誤差
            static double prev_diff_vel = 0;//前速度誤差
            static double acceleration = 0;//角加速度誤差
            static double rotation_pwm = 0;//かける補正pwm
            static double prev_rotation_pwm = 0;//前のかけるpwm
            constexpr double Kp = 8.0;//21.0;//p係数
            constexpr double Ki = 1.0;//36.0 / 0.3;//i係数
            constexpr double Kd = 0.0075;//0.075 * 30.0 * 0.3;//d係数
            if (ds3.button(SELECT) && ds3.press(LEFT))
            {
                gyro.resetYaw(0);
                std::cout << "gyro was reseted" << std::endl;
                dest_angle = 0;
                led_mode = 5;
            }
            time_count = std::chrono::steady_clock::now();
            //prev_time_count = time_count;
            double delta_t = std::chrono::duration_cast<std::chrono::milliseconds>(time_count - prev_time_count).count();
            double now_angle = gyro.yaw;//現在角度
            double user_rotation = (ds3.stick(RIGHT_T) - ds3.stick(LEFT_T)) * 0.8;

            static bool front = false;
            static bool right = false;
            static bool left  = false;
            static bool back  = false;

            /*if(!ds3.button(SELECT)){
              if(ds3.press(UP)){
              front == true ? front = false : front = true;
              right = false;
              left  = false;
              back  = false;
              }else if(ds3.press(RIGHT)){
              right == true ? right = false : right = true;
              front = false;
              left  = false;
              back  = false;

              }else if(ds3.press(LEFT)){
              left  == true ? left  = false : left  = true;
              front = false;
              right = false;
              back  = false;

              }else if(ds3.press(DOWN)){
              back  == true ? back  = false : back  = true;
              front = false;
              right = false;
              left  = false;
              }
              }*/

            /*if(front == true){
              dest_angle = 0;
              }else if(right == true){
              dest_angle = -90;
              }else if(left == true){
              dest_angle = 90;
              }else if(back == true){
              dest_angle = 180;
              }*/
            if(std::fabs(user_rotation) > 0){//もしもL2R2が押されたら目標角度を現在の角度にする
                dest_angle = now_angle;
                front = false;
                right = false;
                left  = false;
                back  = false;
                //integral = 0;//誤差蓄積リセット
                //differential = 0;//変化率リセット
            }


            //------------微分計算区間--------------//
            diff_dest = dest_angle - now_angle;
            diff_dest = diff_dest - (int)diff_dest / 180 * 360;//目標角度との差をdiff_destに格納
            diff_vel = (diff_dest - prev_diff_dest) / (delta_t);//1制御周期での微小角度の変化量を角速度とする
            acceleration = (diff_vel - prev_diff_vel) / (delta_t);//1制御周期での微小角速度の変化量を角加速度とする

            now_rotation_vel = Kp * diff_vel + Ki * diff_dest + Kd * acceleration;
            double move_velocity = now_rotation_vel + prev_rotation_vel;
            //--------------更新区間---------------//
            //prev_time_count = time_count;
            prev_diff_dest = diff_dest;
            // prev_prev_diff_dest = prev_diff_dest;
            prev_rotation_vel = now_rotation_vel;
            prev_diff_vel = diff_vel;
            // - user_rotation;
            //double move = prev_rotation_velocity + rotation_velocity - user_rotation;
            //std::cout << move_velocity << std::endl;//-----------------------------------------
            if(move_velocity > 100){
                move_velocity = 100;
            }else if(move_velocity < -100){
                move_velocity = -100;
            }

            prev_time_count = time_count;
            double gyro_rad = now_angle * M_PI / 180;
            wheel_velocity[1] = std::cos(gyro_rad) * left_x + std::sin(gyro_rad) * left_y + move_velocity - user_rotation;
            wheel_velocity[2] = std::cos(gyro_rad + M_PI * 2/3) * left_x + std::sin(gyro_rad + M_PI * 2/3) * left_y + move_velocity - user_rotation;/*rotation_velocity*///move;
            wheel_velocity[0] = std::cos(gyro_rad - M_PI * 2/3) * left_x + std::sin(gyro_rad - M_PI * 2/3) * left_y + move_velocity - user_rotation;/*rotation_velocity*///move;

            if(wheel_velocity[1] < -250){
                wheel_velocity[1] = -250;
            }else if(wheel_velocity[1] > 250){
                wheel_velocity[1] = 250;
            }

            if(wheel_velocity[2] < -250){
                wheel_velocity[2] = -250;
            }else if(wheel_velocity[2] > 250){
                wheel_velocity[2] = 250;
            }

            if(wheel_velocity[0] < -250){
                wheel_velocity[0] = -250;
            }else if(wheel_velocity[0] > 250){
                wheel_velocity[0] = 250;
            }

            ds3.button(R1) ? regulation = 0.5 : regulation = 1;
            ms.send(10, MOTOR, -wheel_velocity[1] * 0.8 * regulation);
            ms.send(11, MOTOR, -wheel_velocity[2] * 0.8 * regulation);
            ms.send(16, MOTOR, -wheel_velocity[0] * 0.8 * regulation);

            //		std::cout << wheel_velocity[1] << std::endl;

            //-----------------------------hanger------------------------------------------------//

            if (ds3.press(DOWN))
            {
                ((hanger_flag == true) ? hanger_flag = false : hanger_flag = true);
                if (hanger_flag == true)
                {
                    std::cout << "hanger" << std::endl;
                    ms.send(2, HANGER, 251);
                    ms.send(2, HANGER, 252);
                    tape_led_mode = 6;
led_mode = 6;
                }
                else
                {
                    ms.send(2, HANGER, 0);
                    tape_led_mode = 7;
led_mode = ;
                }
            }

            static bool limit_uneffect = false;
            if (ds3.button(SELECT) && ds3.press(CIRCLE))
            {
                ((limit_uneffect = true) ? limit_uneffect = false : limit_uneffect = true);
            }
            //-----------------------------arm-------------------------------------------------//
            //bool y_front_limit = false;
            //bool y_back_limit = false;
            bool z_top_limit = false;
            bool z_bottom_limit = false;
            bool right_top_limit = false;
            bool right_bottom_limit = false;
            bool left_top_limit = false;
            bool left_bottom_limit = false;
            int potentiometer = ms.send(16,40,200);
            std::cout <<"potentio:"  << potentiometer << std::endl;

            static bool recover_towel  = false;
            static bool recover_sheets = false;
            static bool recover_tshirt = false;
            static bool disturbanc     = false;

            if (limit_uneffect == false)
            {
                //y_front_limit = gpioRead(Y_FRONT_LIMIT);
                //y_back_limit = gpioRead(Y_BACK_LIMIT);
                z_top_limit = gpioRead(Z_TOP_LIMIT);
                z_bottom_limit = gpioRead(Z_BOTTOM_LIMIT);
                right_top_limit = gpioRead(RIGHT_TOP_LIMIT);
                right_bottom_limit = gpioRead(RIGHT_BOTTOM_LIMIT);
                left_top_limit = gpioRead(LEFT_TOP_LIMIT);
                left_bottom_limit = gpioRead(LEFT_BOTTOM_LIMIT);
            }


            //std::cout <<  y_front_limit  << std::endl;
            //std::cout <<  left_top_limit << std::endl;

            double right_x = ds3.stick(RIGHT_X);
            double right_y = ds3.stick(RIGHT_Y);
            double right_theta = std::atan2(right_x, right_y);
            double right_distance = std::hypot(right_x, right_y);
            //std::cout << right_distance << std::endl;
            //		std::cout << right_distance << std::endl;
            int sent_y = 0;
            int sent_z = 0;


            if (right_distance >= 20)
            {
                sent_y = right_x * 1.8 * coat_select;
                sent_z = right_y * 1.8;

                if(potentiometer < 290 && sent_y > 0){
                    sent_y = 0;
                }else if(z_bottom_limit == true && right_y > 0){
                    sent_z = 0;
                }else if(potentiometer > 675 && sent_y < 0){
                    sent_y = 0;
                }else if(z_top_limit == true && right_y < 0){
                    sent_z = 0;
                }
                recover_towel = false;
                recover_sheets = false;
                recover_tshirt = false;
                disturbanc     = false;
                //arm_status = 0;
            }
            else
            {
                if((ds3.button(L1) && ds3.button(R1)) && ds3.press(TRIANGLE)){
                    disturbanc = true;
                    recover_sheets = false;
                    recover_tshirt = false;
                    recover_towel  = false;
                }

                else if (ds3.press(TRIANGLE))
                {
                    if(arm_status != 4){
                        recover_towel == true ? recover_towel = false : recover_towel = true;
                        recover_sheets = false;
                        recover_tshirt = false;
                        disturbanc = false;

                    }else{
                        arm_status = 2;
                        /*recover_sheets = false;
                          recover_tshirt = false;*/
                    }

                    std::cout << "towel" << std::endl;
                }
                else if (ds3.press(SQUARE))
                {
                    if(arm_status != 4){
                        recover_sheets == true ? recover_sheets = false : recover_sheets = true;
                        recover_towel = false;
                        recover_tshirt = false;
                        disturbanc = false;

                    }else{
                        arm_status = 2;
                        /*recover_towel = false;
                          recover_tshirt = false;*/
                    }
                    std::cout << "sheets" << std::endl;
                }
                else if (ds3.press(CROSS))
                {
                    if(arm_status != 4 && arm_status != 7){
                        recover_tshirt == true ? recover_tshirt = false : recover_tshirt = true;
                        recover_towel = false;
                        recover_sheets = false;
                        disturbanc = false;
                    }else if(arm_status == 4){
                        arm_status = 2;
                    }else{
                        arm_status = 8;
                    }
                    std::cout << "tshirt" << std::endl;
                }else if(ds3.press(START)){
                    recover_towel = false;
                    recover_sheets = false;
                    recover_tshirt = false;
                    disturbanc = false;
                    arm_status = false;
                }


                //std::cout << arm_status << std::endl;
                if (recover_towel)
                {
                    //std::cout << arm_status << std::endl;
                    switch (arm_status)
                    {
                        case 0:
                            sent_z = Z_ARM_PWM;
                            if (z_bottom_limit == true)
                            {
                                sent_z = 0;
                                arm_status = 1;
                            }
                            break;
                        case 1:
                            sent_y = -Y_ARM_PWM;
                            if (potentiometer > 655)
                            {
                                sent_y = 0;
                                arm_status = 4;
                                z_fall_start = std::chrono::steady_clock::now();
                            }
                            break;

                        case 2:
                            z_fall_now = std::chrono::steady_clock::now();
                            sent_z = -Z_ARM_PWM;
                            z_fall_time = std::chrono::duration_cast<std::chrono::milliseconds>(z_fall_start - z_fall_now);
                            if (z_top_limit == true || z_fall_time.count() > 1500)
                            {
                                sent_z = 0;
                                arm_status = 3;
                                y_pull_start = std::chrono::steady_clock::now();
                            }
                            break;

                        case 3:
                            y_pull_now = std::chrono::steady_clock::now();
                            sent_y = Y_ARM_PWM * 0.7;
                            y_pull_time = std::chrono::duration_cast<std::chrono::milliseconds>(y_pull_start - y_pull_now);
                            if (potentiometer < 495 || y_pull_time.count() > 2500)
                            {
                                sent_y = 0;
                                recover_towel = false;
                                arm_status = 0;
                            }
                            break;
                        case 4:
                            if(ds3.press(TRIANGLE)){
                                arm_status = 2;
                            }
                            break;

                    }
                }else if(recover_sheets){
                    switch (arm_status)
                    {
                        case 0:
                            sent_z = Z_ARM_PWM;
                            if (z_bottom_limit == true)
                            {
                                sent_z = 0;
                                arm_status = 1;
                            }
                            break;
                        case 1:
                            sent_y = -Y_ARM_PWM;
                            if (potentiometer > 435)
                            {
                                sent_y = 0;
                                arm_status = 4;
                                z_fall_start = std::chrono::steady_clock::now();
                            }
                            break;

                        case 2:
                            z_fall_now = std::chrono::steady_clock::now();
                            sent_z = -Z_ARM_PWM;
                            z_fall_time = std::chrono::duration_cast<std::chrono::milliseconds>(z_fall_start - z_fall_now);
                            if (z_top_limit == true || z_fall_time.count() > 1500)
                            {
                                sent_z = 0;
                                arm_status = 3;
                                y_pull_start = std::chrono::steady_clock::now();
                            }
                            break;

                        case 3:
                            y_pull_now = std::chrono::steady_clock::now();
                            sent_y = Y_ARM_PWM;
                            y_pull_time = std::chrono::duration_cast<std::chrono::milliseconds>(y_pull_start - y_pull_now);
                            if (potentiometer < 295 || y_pull_time.count() > 2500)
                            {
                                sent_y = 0;
                                arm_status = 5;
                            }
                            break;
                        case 4:
                            if(ds3.press(SQUARE)){
                                arm_status = 2;
                            }
                            break;
                        case 5:
                            sent_z = Z_ARM_PWM;
                            if (z_bottom_limit == true)
                            {
                                sent_z = 0;
                                arm_status = 0;
                                recover_sheets = false;
                            }
                            break;

                    }

                }else if(recover_tshirt){
                    switch (arm_status)
                    {
                        case 0://上がる
                            sent_z = Z_ARM_PWM;
                            if (z_bottom_limit == true)
                            {
                                sent_z = 0;
                                arm_status = 1;
                            }
                            break;
                        case 1://前行く
                            sent_y = -Y_ARM_PWM;
                            if (potentiometer > 480)
                            {
                                sent_y = 0;
                                arm_status = 4;
                            }
                            break;

                        case 2://下がる
                            z_fall_now = std::chrono::steady_clock::now();
                            sent_z = -Z_ARM_PWM;
                            z_fall_time = std::chrono::duration_cast<std::chrono::milliseconds>(z_fall_start - z_fall_now);
                            if (z_top_limit == true || z_fall_time.count() > 1500)
                            {
                                sent_z = 0;
                                arm_status = 3;
                                y_pull_start = std::chrono::steady_clock::now();
                            }
                            break;

                        case 3://引く
                            y_pull_now = std::chrono::steady_clock::now();
                            sent_y = Y_ARM_PWM;
                            y_pull_time = std::chrono::duration_cast<std::chrono::milliseconds>(y_pull_start - y_pull_now);
                            if (potentiometer < 295 || y_pull_time.count() > 2500)
                            {
                                sent_y = 0;
                                //recover_tshirt = false;
                                arm_status = 5;
                            }
                            break;
                        case 4://止まる
                            if(ds3.button(L1) && ds3.press(CROSS)){
                                arm_status = 2;
                            }else{
                                z_fall_start = std::chrono::steady_clock::now();
                            }
                            break;
                        case 5://上げる
                            sent_z = Z_ARM_PWM;
                            std::cout << "www" << std::endl;
                            if (z_bottom_limit == true)
                            {
                                sent_z = 0;
                                arm_status = 6;
                            }
                            break;
                        case 6://前
                            sent_y = -Y_ARM_PWM;
                            if (potentiometer > 500)
                            {
                                sent_y = 0;
                                arm_status = 7;
                            }
                            break;
                        case 7://一旦停止
                            if(ds3.press(CROSS)){
                                arm_status = 8;
                            }else{
                                z_fall_start = std::chrono::steady_clock::now();
                            }
                            break;
                        case 8://下げる
                            z_fall_now = std::chrono::steady_clock::now();
                            sent_z = -Z_ARM_PWM;
                            z_fall_time = std::chrono::duration_cast<std::chrono::milliseconds>(z_fall_start - z_fall_now);
                            if (z_top_limit == true || z_fall_time.count() > 1500)
                            {
                                sent_z = 0;
                                arm_status = 9;
                                y_pull_start = std::chrono::steady_clock::now();
                            }
                            break;
                        case 9://引く
                            y_pull_now = std::chrono::steady_clock::now();
                            sent_y = Y_ARM_PWM;
                            y_pull_time = std::chrono::duration_cast<std::chrono::milliseconds>(y_pull_start - y_pull_now);
                            if (potentiometer < 500 || y_pull_time.count() > 2000)
                            {
                                sent_y = 0;
                                recover_tshirt = false;
                                arm_status = 0;
                            }
                            break;
                    }

                }else if(disturbanc == true){
                    switch (arm_status)
                    {
                        case 0://上がる
                            sent_z = Z_ARM_PWM;
                            if (z_bottom_limit == true)
                            {
                                sent_z = 0;
                                arm_status = 1;
                            }
                            break;
                        case 1://前行く
                            sent_y = -Y_ARM_PWM;
                            if (potentiometer > 545)
                            {
                                sent_y = 0;
                                arm_status = 2;
                            }
                            break;

                        case 2://下がる
                            z_fall_now = std::chrono::steady_clock::now();
                            sent_z = -Z_ARM_PWM;
                            z_fall_time = std::chrono::duration_cast<std::chrono::milliseconds>(z_fall_start - z_fall_now);
                            if (z_top_limit == true || z_fall_time.count() > 1500)
                            {
                                sent_z = 0;
                                arm_status = 3;
                                y_pull_start = std::chrono::steady_clock::now();
                            }
                            break;
                        case 3:
                            sent_y = -Y_ARM_PWM;
                            if (potentiometer > 670)
                            {
                                sent_y = 0;
                                arm_status = 4;
                            }
                            break;
                        case 4:
                            sent_z = Z_ARM_PWM;
                            if (z_bottom_limit == true)
                            {
                                sent_z = 0;
                                arm_status = 0;
                                disturbanc = false;
                            }
                    }
                }


                if(ds3.button(L1) && ds3.button(CIRCLE)){
                    ms.send(16,SOLENOID,251);
                    ms.send(16,SOLENOID,252);
                }else{
                    ms.send(16,SOLENOID,0);
                }

                //std::cout << sent_y << sent_z << std::endl;
                int sent_right = 0; //Right from the circuit side
                int sent_left = 0;  //Left from the circuit side
                static bool box_permission = false;
                static bool circle_on = false;

                if(ds3.press(UP)){
                    circle_on == true ? circle_on = false :circle_on = true;
                }

                if (circle_on)
                {
                    //std::cout << "maru" << std::endl;

                    //stop_towel_arm = !(stop_towel_arm);
                    switch (towel_arm_status)
                    {
                        case 0: // Primary status that doesn't detect
                            towel_arm_status++;
                            sent_right = 0;
                            sent_left = 0;
                            box_permission = false;
                            break;
                        case 1:
                            if (limit_uneffect == false)
                            {
                                right_top_limit == true ? sent_right = 0 : sent_right = TOWEL_ARM_PWM;
                                left_top_limit == true ? sent_left = 0 : sent_left = TOWEL_ARM_PWM;
                            }

                            if (right_top_limit == true && left_top_limit == true)
                            {
                                towel_arm_status++;
                                box_permission = true;
                                circle_on = false;
                            }
                            break;
                        case 2:
                            if (limit_uneffect == false)
                            {
                                right_bottom_limit == true ? sent_right = 0 : sent_right = -1 * TOWEL_ARM_PWM;
                                left_bottom_limit == true ? sent_left = 0 : sent_left = -1 * TOWEL_ARM_PWM;
                            }
                            if (right_bottom_limit == true && left_bottom_limit == true)
                            {
                                towel_arm_status = 0;
                                circle_on = false;
                                box_permission = false;
                            }
                            break;
                    }
                }


                ms.send(10,TOWEL,-sent_right);
                ms.send(2, TOWEL,-sent_left);

                //-------------------------------box-------------------------------/
                static bool up_on = false;

                if (ds3.press(CIRCLE)){
                    up_on == true ? up_on = false : up_on = true;
                    box_start = std::chrono::steady_clock::now();
                    //box_start = std::chrono::steady_clock::now();
                }
                if(up_on == true){
                    z_bottom_limit == true ? sent_z = 0 : sent_z = 100;
                    potentiometer > 295 ? sent_y = Y_ARM_PWM : sent_z = 0;
                    if(potentiometer < 295){
                        up_on = false;
                        sent_y= 0;
                        sent_z = 0;
                    }
                }
            }
            ms.send(10, ARM, sent_y /* regulation*/);
            ms.send(2, ARM,  sent_z /* regulation*/);

            if(coat_select > 0){
                led_mode = 2;
            }else if(coat_select < 0){
                led_mode = 3;
            }else if(sent_right > 0){
                led_mode = 8;
            }else if(sent_right < 0){
                led_mode = 9;
            }else if(sent_y != 0){
                led_mode = 11;
            }else if(sent_z != 0){
                led_mode = 12;
            }

            if(led_mode != priventeer){
                sent_or_not = false;
                led_mode = priventeer;
            }

            if(sent_or_not != true){
                switch(led_mode){
                    case 1:
                        ms.send(84,10,60);
                        break;
                    case 2:
                        ms.send(84,20,60);
                        break;
                    case 3:
                        ms.send(84,30,60);
                        break;
                    case 4:
                        ms.send(84,40,60);
                        break;
                    case 5:
                        ms.send(84,50,60);
                        break;
                    case 6:
                        ms.send(84,60,60);
                        break;
                    case 7:
                        ms.send(84,70,60);
                        break;
                    case 8:
                        ms.send(84,80,100);
                        break;
                    case 9:
                        ms.send(84,90,100);
                        break;
                    case 10:
                        ms.send(84,100,60);
                        break;
                    case 11:
                        ms.send(84,110,60);
                        break;
                    case 12:
                        ms.send(84,120,60);
                        break;
                    case 13:
                        ms.send(84,200,60);
                        break;
                }
                sent_or_not = true;
            }

        }
    }
    ms.send(84,200,10);
    gpioWrite(RUNLED,false);
    gpioWrite(SLEEPLED,false);
    ms.send(255,255,0);
    return 0;
}
