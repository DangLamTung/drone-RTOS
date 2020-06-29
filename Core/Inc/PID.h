/*
 * PID.h
 *
 *  Created on: May 23, 2020
 *      Author: tung
 */

#ifndef SRC_PID_H_
#define SRC_PID_H_
#include "LPF.hpp"



typedef struct{
	float Kp1;
	float Ki1;
	float Kd1;

	float Kp2;
	float Ki2;
	float Kd2;

	float Kp3;
	float Ki3;
	float Kd3;

	float Kp4;
	float Ki4;
	float Kd4;
    uint8_t crc;
}PID_value;

typedef struct{

	uint16_t data[12];
    uint8_t crc;
}PID_raw;

PID_raw convert(PID_value pid){
	PID_raw raw;
	raw.data[0] = pid.Kp1;
	raw.data[1] = pid.Ki1;
	raw.data[2] = pid.Kd1;

	raw.data[3] = pid.Kp2;
	raw.data[4] = pid.Ki2;
	raw.data[5] = pid.Kd2;

	raw.data[6] = pid.Kp3;
	raw.data[7] = pid.Ki3;
	raw.data[8] = pid.Kd3;

	raw.data[9] = pid.Kp4;
	raw.data[10] = pid.Ki4;
	raw.data[11] = pid.Kd4;

	return raw;
}
PID_raw pid_decode(uint8_t data[24]){
	PID_raw pid;

	pid.data[0] = (data[0] << 6) | ((data[1]&0xFC) >> 2);
	pid.data[1] = ((data[1] & 0x03) <<12 ) |(data[2] <<4)|((data[3]&0xF0 )>> 4);
	pid.data[2] = ((data[3]&0x0F )<< 10 | (data[4] << 2) |  (data[5] &0xC0)>>6);

	pid.data[3] = ((data[5] & 0x3F)<<8|data[6]);
	pid.data[4] = ((data[7] << 6)|((data[8]&0xFC) >> 2));
	pid.data[5] = ((data[8] & 0x03) <<12)| (data[9] <<4) |((data[10]&0xF0 )>> 4);

	pid.data[6] = ((data[10]&0x0F) << 10) | (data[11] << 2) |  ((data[12] &0xC0)>>6);
	pid.data[7] = ((data[12] & 0x3F)<<8)|(data[13]);
	pid.data[8] = ((data[14] << 6) | ((data[15]&0xFC) >> 2));

	pid.data[9] = ((data[15] & 0x03) << 12) | (data[16] << 4) | ((data[17]&0xF0) >> 4);
	pid.data[10] = ((data[17]&0x0F) << 10) | (data[18]<<2)|((data[19] &0xC0)>>6);
	pid.data[11] = ((data[19] & 0x3F)<<8) | data[20];
    pid.crc = data[21];
	return pid;
}
uint8_t check_CRC_pid(PID_raw pid){
    uint32_t check = 0;
    for(uint8_t i = 0; i<12; i++)
    	check += pid.data[i];

    if(check % 37 != pid.crc)
    	return 0;
    return 1;
}
class PID {
public:
	float Error, pre_Error, pre_pre_Error;
	float P_part, I_part, D_part, Out, pre_out;
    float Kp, Ki, Kd, T;
    LPF Dterm;

	float update(float setpoint, float input){
		Error = setpoint - input;
		P_part = Kp*(Error);
		I_part += Ki*T*Error;
		if(I_part > 500){
			I_part = 500;
		}
		D_part = Kd*(Error - pre_Error)/T;
		Out = P_part + I_part + D_part ;
		pre_Error = Error;
		pre_out = Out;

		return Out;
	}
	int round(double x)
	{
	    if (x < 0.0)
	    	if(x > -1000){
	        return (int)(x - 0.5);
	    	}
	    	else{
	    		return -1000;
	    	}
	    else
	    	if(x < 1000){
		        return (int)(x + 0.5);
	    	}
	    	else{
	    		return 1000;
	    	}
	}
	int get_control_value()
	{
	    if (Out < 0.0)
	    	if(Out > -1000){
	        return (int)(Out - 0.5);
	    	}
	    	else{
	    		return -1000;
	    	}
	    else
	    	if(Out < 1000){
		        return (int)(Out + 0.5);
	    	}
	    	else{
	    		return 1000;
	    	}

	}
    void load(float Kp,float Ki,float Kd,float T){
    	this->Kp = Kp;
    	this->Ki = Ki;
    	this->Kd = Kd;
    	this->T = T;

    	Dterm.load(LPF_10HZ);
    }
	PID();
	virtual ~PID();
};
PID::~PID(){

}

PID::PID(){

}
#endif /* SRC_PID_H_ */
