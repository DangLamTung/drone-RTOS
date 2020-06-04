/*
 * MotorController.hpp
 *
 *  Created on: Jun 1, 2020
 *      Author: tung
 */

#ifndef INC_MOTORCONTROLLER_HPP_
#define INC_MOTORCONTROLLER_HPP_
#include "PID.h"

class MotorController{
	public:
	   MotorController(PID_value pid);
	   PID_value pid;
	   PID pid_r;
	   PID pid_p;
	   PID pid_y;
	   PID pid_alt;

};
MotorController::MotorController(PID_value pid){
	this->pid_r = PID(pid.Kp1, pid.Ki1, pid.Kd1);
	this->pid_p = PID(pid.Kp2, pid.Ki2, pid.Kd2);
    this->pid_y = PID(pid.Kp3, pid.Ki3, pid.Kd3);
    this->pid_alt = PID(pid.Kp4, pid.Ki4, pid.Kd4);
}


#endif /* INC_MOTORCONTROLLER_HPP_ */
