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

	while (!(ds3.button(START) && ds3.button(RIGHT)))
	{
		/*std::cout << "rt" << gpioRead(RIGHT_TOP_LIMIT) << std::endl;
		std::cout << "rb" << gpioRead(RIGHT_BOTTOM_LIMIT) << std::endl;
		std::cout << "lt" << gpioRead(LEFT_TOP_LIMIT) << std::endl;
		std::cout << "lb" << gpioRead(LEFT_BOTTOM_LIMIT) << std::endl;
		*/gpioWrite(RUNLED,true);
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
		std::array<double, 3> wheel_velocity;

		if (ds3.button(SELECT) && ds3.press(LEFT))
		{
			gyro.resetYaw(0);
			std::cout << "gyro was reseted" << std::endl;
		}

		ds3.button(R1) ? regulation = 0.5 : regulation = 1;

		double gyro_rad = gyro.yaw * M_PI / 180;
		
		double user_rotation = (ds3.stick(RIGHT_T) - ds3.stick(LEFT_T)) * 0.8;
		
		static double integral  = 0 differential = 0;
		static double gyro_pre_value = 0;
		double diff_angle = gyro.yaw - gyro_pre_value;
		static double integral = integral + diff;
		static
		d_correct_rotation = 
		
		if(std::fabs(user_rotation) > 0){
			gyro_pre_value = gyro.yaw;
			i_correct_rotation = 0;
		}


		constexpr double kp = 21.0;
		constexpr double ki = 36.0 / 0.3;
				

		std::cout << kp * p_correct_rotation << std::endl;

		rotation = rotation + kp*p_correct_rotation - ki*i_correct_rotation;
		prev_rotation - rotation
		//double prev_correct = rotation;
 	       
		if(rotation > 100){
			rotation = 100;
		}else if(rotation < -100){
			rotation = -100;
		}	//+ i_correct_rotation;

		/*if(rotation < -250){
			rotation = -250;
		}else if(rotation > 250){
			rotation = 250;
		}*/
		//std::cout << rotation << std::endl;
		//std::cout << correct_rotation << std::endl;

		wheel_velocity[1] = std::cos(gyro_rad) * left_x + std::sin(gyro_rad) * left_y -rotation;
		wheel_velocity[2] = std::cos(gyro_rad + M_PI * 2/3) * left_x + std::sin(gyro_rad + M_PI * 2/3) * left_y -rotation;
		wheel_velocity[0] = std::cos(gyro_rad - M_PI * 2/3) * left_x + std::sin(gyro_rad - M_PI * 2/3) * left_y -rotation;

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

		ms.send(10, MOTOR, -wheel_velocity[1] * 0.8 * regulation);
		ms.send(11, MOTOR, -wheel_velocity[2] * 0.8 * regulation);
		ms.send(16, MOTOR, -wheel_velocity[0] * 0.8 * regulation);

//		std::cout << wheel_velocity[1] << std::endl;

		//-----------------------------hanger------------------------------------------------//

		if (ds3.press(SQUARE) && !(ds3.button(SELECT)))
		{
			((hanger_flag == true) ? hanger_flag = false : hanger_flag = true);
			if (hanger_flag == true)
			{
				std::cout << "hanger" << std::endl;
				ms.send(2, HANGER, 251);
				ms.send(2, HANGER, 252);
				tape_led_mode = 6;
			}
			else
			{
				ms.send(2, HANGER, 0);
				tape_led_mode = 7;
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
//		std::cout <<"potentio:"  << potentiometer << std::endl;

		static bool recover = false;

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

			if(potentiometer < 50 && sent_y > 0){
				sent_y = 0;
			}else if(z_bottom_limit == true && right_y > 0){
				sent_z = 0;
			}else if(potentiometer > 570 && sent_y < 0){
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
						if (potentiometer < 570)
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
						if (potentiometer > 313 || y_pull_time.count() > 2500)
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


		ms.send(10,TOWEL,sent_right);
		ms.send(2, TOWEL,sent_left);

		//-------------------------------box-------------------------------/
		static bool triangle_on = false;

		if (ds3.press(TRIANGLE) && !(ds3.button(SELECT))){
			triangle_on = true;
			box_start = std::chrono::steady_clock::now();
			//box_start = std::chrono::steady_clock::now();
		}

		if(triangle_on == true){
			if(potentiometer < 50){
				sent_y = 150;
			}else{
				sent_y = 0;
				triangle_on = false;
			}	
		}
		ms.send(10, ARM, sent_y * regulation);
		ms.send(2, ARM,  sent_z * regulation);
	}
	gpioWrite(RUNLED,false);
	gpioWrite(SLEEPLED,false);
	ms.send(255,255,0);
	return 0;
}
