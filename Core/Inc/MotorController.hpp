/*
 * MotorController.hpp
 *
 *  Created on: Jun 1, 2020
 *      Author: tung
 */

#ifndef INC_MOTORCONTROLLER_HPP_
#define INC_MOTORCONTROLLER_HPP_
#include "PID.h"
#include "sbus.h"
#include "mpu_data_type.hpp"

class MotorController{
	public:
	   MotorController();
	   PID_value pid;
	   PID pid_r;
	   PID pid_p;
	   PID pid_y;
	   PID pid_alt;
   void loadPID(PID_value pid){
		this->pid_r.load(pid.Kp1/100, pid.Ki1/100, pid.Kd1/100,0.01);
		this->pid_p.load(pid.Kp2/100, pid.Ki2/100, pid.Kd2/100,0.01);
	    this->pid_y.load(pid.Kp3/100, pid.Ki3/100, pid.Kd3/100,0.01);
	    this->pid_alt.load(pid.Kp4/100, pid.Ki4/100, pid.Kd4/100,0.01);
   }
   ESC_value update(EULER_angle drone_state, uint16_t thurst){
	    ESC_value control;
        this->pid_p.update(0, drone_state.pitch);
        this->pid_r.update(0, drone_state.roll);
        this->pid_y.update(0, drone_state.yaw);


        control.esc_value1 = thurst + pid_r.get_control_value() - pid_p.get_control_value() + pid_y.get_control_value();
        control.esc_value2 = thurst + pid_r.get_control_value() + pid_p.get_control_value() - pid_y.get_control_value();

        control.esc_value3 = thurst - pid_r.get_control_value() - pid_p.get_control_value() - pid_y.get_control_value();
        control.esc_value4 = thurst - pid_r.get_control_value() + pid_p.get_control_value() + pid_y.get_control_value();

        if(control.esc_value1 < 1000){
        	control.esc_value1 = 1000;
        }
        if(control.esc_value1 > 1500){
        	control.esc_value1 = 1500;
        }

        if(control.esc_value2 < 1000){
        	control.esc_value2 = 1000;
        }
        if(control.esc_value2 > 1500){
        	control.esc_value2 = 1500;
        }

        if(control.esc_value3 < 1000){
        	control.esc_value3 = 1000;
        }
        if(control.esc_value3 > 1500){
        	control.esc_value3 = 1500;
        }

        if(control.esc_value4 < 1000){
        	control.esc_value4 = 1000;
        }
        if(control.esc_value4 > 1500){
        	control.esc_value4 = 1500;
        }

        return control;
   }
};
MotorController::MotorController(){
}


#endif /* INC_MOTORCONTROLLER_HPP_ */
