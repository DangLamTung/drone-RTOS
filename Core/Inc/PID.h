/*
 * PID.h
 *
 *  Created on: May 23, 2020
 *      Author: tung
 */

#ifndef SRC_PID_H_
#define SRC_PID_H_

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
	void update(float setpoint, float input){
		P_part = Kp*(Error - pre_Error);
		I_part = 0.5*Ki*T*(Error + pre_Error);
		D_part = Kd/T*( Error - 2*pre_Error+ pre_pre_Error);
		Out = pre_out + P_part + I_part + D_part ;
		pre_pre_Error = pre_Error;
		pre_Error = Error;
		pre_out = Out;
	}

	PID(float Kp,float Ki,float Kd,float T);
	virtual ~PID();
};
PID::PID(float Kp,float Ki,float Kd,float T){
	this->Kp = Kp;
	this->Ki = Ki;
    this->Kd = Kd;
    this->T = T;
}
#endif /* SRC_PID_H_ */
