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

	constexpr int RIGHT_TOP_LIMIT = 12;//12
	constexpr int RIGHT_BOTTOM_LIMIT = 16;//16
	constexpr int LEFT_TOP_LIMIT = 11;//11
	constexpr int LEFT_BOTTOM_LIMIT = 22;//22

	constexpr int Y_FRONT_LIMIT = 26;
	constexpr int Y_BACK_LIMIT = 19;
	constexpr int Z_TOP_LIMIT = 9;
	constexpr int Z_BOTTOM_LIMIT = 10;

	constexpr int TOWEL_ARM_PWM = 160;
	constexpr int Y_ARM_PWM = 250;
	constexpr int Z_ARM_PWM = 200;

	constexpr bool final_mode = false; 

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

	while (!(ds3.button(START) && ds3.button(RIGHT)))
	{
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
		do
		{
			if(sleep_flag == true){
				ds3.update();
			}
			//ds3.update();
			if ((ds3.button(SELECT) && ds3.press(SQUARE)))
			{
				(sleep_flag == true) ? sleep_flag = false : sleep_flag = true;
				if (sleep_flag == true)
				{
					std::cout << "zzz" << std::endl;
					ms.send(255,255,0);
					gpioWrite(SLEEPLED,false);
					//ds3.update();
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
		//auto gyro_fixing = std::chrono::steady_clock::now();

		if (ds3.button(SELECT) && ds3.press(LEFT))
		{
			gyro.resetYaw(0);
			std::cout << "gyro was reseted" << std::endl;
		}

		ds3.button(R1) ? regulation = 0.5 : regulation = 1;

		double gyro_rad = gyro.yaw * M_PI / 180;
		double rotation = (ds3.stick(RIGHT_T) - ds3.stick(LEFT_T)) * 0.8;
		static double gyro_pre_value = 0;

		static bool front = false;
		static bool right = false;
		static bool left  = false;
		static bool back  = false;
		static bool correct_rock = false;

		int correct_rotation = gyro.yaw - gyro_pre_value;

		if(std::fabs(rotation) > 0 || correct_rock == false){
			gyro_pre_value = gyro.yaw;
			//gyro_correct_wait = std::chrono::steady_clock::now();
		}

		if(!ds3.button(SELECT)){
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
		}

		if(front == true){
			rotation = rotation + 50 * (gyro.yaw / 180);
			if(gyro.yaw < 1 && gyro.yaw > -1){
				front = false;
				correct_rock = false;
			}else{
				correct_rock = true;
			}
		}else if(right == true){
			rotation = rotation + 50 * ((gyro.yaw - 90)/ 180);
			if(gyro.yaw < 91 && gyro.yaw > 89){
				right = false;
				correct_rock = false;
			}else{
				correct_rock = true;
			}
		}else if(left == true){
			rotation = rotation + 50 * ((gyro.yaw + 90)/ 180);
			if(gyro.yaw < -89 && gyro.yaw > -91){
				left = false;
				correct_rock = false;
			}else{
				correct_rock = true;
			}
		}else if(back == true){
			rotation = rotation + 50 * ((gyro.yaw + 180)/ 180);
			if(gyro.yaw < 1 && gyro.yaw > -1){
				back = false;
				correct_rock = false;
			}else{
				correct_rock = true;
			}
		}else{
            correct_rock = false;
            front = false;
            right = false;
            left  = false;
            back  = false;
        }

		std::cout << rotation << std::endl;
		std::cout << correct_rotation << std::endl;

		wheel_velocity[0] = -std::sin(M_PI/4 + gyro_rad) * left_x + std::cos(M_PI/4 + gyro_rad) * left_y  -rotation - correct_rotation;
		wheel_velocity[1] = -std::cos(M_PI/4 + gyro_rad) * left_x + -std::sin(M_PI/4 + gyro_rad) * left_y - rotation - correct_rotation;
		wheel_velocity[2] = std::sin(M_PI/4 + gyro_rad) * left_x + -std::cos(M_PI/4 + gyro_rad) * left_y - rotation -  correct_rotation;
		wheel_velocity[3] = std::cos(M_PI/4 + gyro_rad) * left_x + std::sin(M_PI/4 + gyro_rad) * left_y - rotation - correct_rotation;

		ms.send(BOTTOM_MDD, UNC_PORT , -wheel_velocity[1] * 0.8 * regulation);
		ms.send(DOWN_MDD,   UNC_PORT , -wheel_velocity[2] * 0.8 * regulation);
		ms.send(UP_MDD,     UNC_PORT , -wheel_velocity[0] * 0.8 * regulation);
		ms.send(TOP_MDD,    UNC_PORT , -wheel_velocity[3] * 0.8 * regulation);


		//-----------------------------hanger------------------------------------------------//

		if (ds3.press(SQUARE) && !(ds3.button(SELECT)))
		{
			((hanger_flag == true) ? hanger_flag = false : hanger_flag = true);
			if (hanger_flag == true)
			{
				std::cout << "hanger" << std::endl;
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

		static bool limit_uneffect = false;
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

		static bool recover = false;

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

			if(y_back_limit  == true && right_x > 0){
				sent_y = 0;
			}else if(z_bottom_limit == true && right_y > 0){
				sent_z = 0;
			}else if(y_front_limit == true && right_x < 0){
				sent_y = 0;
			}else if(z_top_limit == true && right_y < 0){
				sent_z = 0;
			}
			recover = false;
			arm_status = 0;
		}
		else
		{
			if (ds3.press(CROSS))
			{
				recover == true ? recover = false : recover = true;
				std::cout << "cross" << std::endl;
			}

			if (recover == true)
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
						sent_y = Y_ARM_PWM;
						if (y_back_limit == true)
						{
							sent_y = 0;
							arm_status = 2;
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
						sent_y = -Y_ARM_PWM;
						auto y_pull_time = std::chrono::duration_cast<std::chrono::milliseconds>(y_pull_start - y_pull_now);
						if (y_front_limit == true || y_pull_time.count() > 2500)
						{
							sent_y = 0;
							recover = false;
							arm_status = 0;
						}
						break;
				}
			}
		}

		if(ds3.button(L1) && ds3.button(CIRCLE)){
			ms.send(DOWN_MDD,SOLENOID_PORT,251);
			ms.send(DOWN_MDD,SOLENOID_PORT,252);
		}else{
			ms.send(DOWN_MDD,SOLENOID_PORT,0);
		}

		//std::cout << sent_y << sent_z << std::endl;
		int sent_right = 0; //Right from the circuit side
		int sent_left = 0;  //Left from the circuit side
		static bool box_permission = false;
		static bool circle_on = false;

		if(ds3.press(CIRCLE) && !(ds3.button(L1))){
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


		ms.send(DOWN_MDD,ARM_PORT,sent_right);
		ms.send(BOTTOM_MDD,ARM_PORT,sent_left);

		//-------------------------------box-------------------------------/
		static bool triangle_on = false;

		if (ds3.press(TRIANGLE) && !(ds3.button(SELECT))){
			triangle_on = true;
			box_start = std::chrono::steady_clock::now();
			//box_start = std::chrono::steady_clock::now();
		}

		if(triangle_on == true){
			towel_arm_status = 1;
			if(box_permission == true || final_mode == true){
				box_now = std::chrono::steady_clock::now();
				box_wait_time = std::chrono::duration_cast<std::chrono::milliseconds>(box_now - box_start);
				box_time = box_wait_time.count();
				std::cout << box_time << std::endl;
				z_bottom_limit == false ? sent_z = 180 : sent_z = 0;
				if(box_time < 300){
					y_back_limit == false ? sent_y = 180 : sent_y = 0;
					//z_bottom_limit == false ? sent_z = 180 : sent_z = 0;
				}else if(box_time < 600){
					ms.send(TOP_MDD,SOLENOID_PORT,251);
					//triangle_on = false;

				}else{
					//ms.send(TOP_MDD,SOLENOID_PORT,0);
					triangle_on = false;
				}

			}else if(limit_uneffect == true){
				ms.send(TOP_MDD,SOLENOID_PORT,251);
			}
		}else{
			ms.send(TOP_MDD,SOLENOID_PORT,0);
		}
		ms.send(TOP_MDD, ARM_PORT, sent_y * regulation);
		ms.send(UP_MDD, ARM_PORT,  sent_z * regulation);
	}
	gpioWrite(RUNLED,false);
	gpioWrite(SLEEPLED,false);
	ms.send(255,255,0);
	return 0;
}
