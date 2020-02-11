/*
 * EKF.h
 *
 *  Created on: Feb 10, 2020
 *      Author: tung
 */

#ifndef INC_EKF_H_
#define INC_EKF_H_
#include "Matrix.hpp"
#include <math.h>
#include "mpu_data_type.hpp"
class EKF {
private:
//    Matrix x,y,A,B,P,Q,K,S,R,w;
	Matrix x = Matrix(7,1);
	Matrix y = Matrix(3,1);
	Matrix A = Matrix(7,7);
	Matrix B = Matrix(7,3);
	Matrix P = Matrix(7,7);
	Matrix Q = Matrix(7,7);
	Matrix H = Matrix(3,7);
	Matrix K = Matrix(7,3);
	Matrix S = Matrix(3,3);
	Matrix R = Matrix(3,3);
	Matrix I = diag_mat(7, 7);
	Matrix w = Matrix(3,1);
public:
	void loadEKF(double* x_, double *P_, double* Q_, double*R_){
	      this->x.get_value(x_);
	      this->P.get_value(P_);
	      this->Q.get_value(Q_);
	      this->R.get_value(R_);
	}
	EKF(double* x_, double *P_, double* Q_, double*R_);
	quaternion updateEKF(IMU_data data_imu, float dt){
          quaternion state;
          if(!isnan(data_imu.Acc_x) &&!isnan(data_imu.Acc_y)&&!isnan(data_imu.Acc_z)&&!isnan(data_imu.Gyro_x)&&!isnan(data_imu.Gyro_y)&&!isnan(data_imu.Gyro_z)){
          w.data[0] = data_imu.Gyro_x*DEC2RAD;
          w.data[1] = data_imu.Gyro_y*DEC2RAD;
          w.data[2] = data_imu.Gyro_z*DEC2RAD;

          y.data[0] = data_imu.Acc_x;
          y.data[1] = data_imu.Acc_y;
          y.data[2] = data_imu.Acc_z;

          double x0,x1,x2,x3,x0_,x1_,x2_,x3_,x02,x12,x22,x32,x02_,x12_,x22_,x32_;
          x0 = this->x.data[0]*0.5*dt;
          x1 = this->x.data[1]*0.5*dt;
          x2 = this->x.data[2]*0.5*dt;
          x3 = this->x.data[3]*0.5*dt;

          x02 = this->x.data[0]*2;
          x12 = this->x.data[1]*2;
          x22 = this->x.data[2]*2;
          x32 = this->x.data[3]*2;

          x0_ = -x0;
          x1_ = -x1;
          x2_ = -x2;
          x3_ = -x3;

          x02_ = -x02;
          x12_ = -x12;
          x22_ = -x22;
          x32_ = -x32;
         A.data[0] = 1;
         A.data[8] = 1;
         A.data[16] = 1;
         A.data[24] = 1;
         A.data[32] = 1;
         A.data[40] = 1;
         A.data[48] = 1;

         A.data[4] = x1;
         A.data[5] = x2;
         A.data[6] = x3;

         A.data[11] = x0_;
         A.data[12] = x3;
         A.data[13] = x2_;

         A.data[18] = x3_;
         A.data[19] = x0_;
         A.data[20] = x1;

         A.data[25] = x2;
         A.data[26] = x1_;
         A.data[27] = x0_;

         B.data[1] = x1_;
         B.data[2] = x2_;
         B.data[3] = x3_;

         B.data[4] = x0;
         B.data[5] = x3_;
         B.data[6] = x2;

         B.data[7] = x3;
         B.data[8] = x0;
         B.data[9] = x1_;

         B.data[10] = x2_;
         B.data[11] = x1;
         B.data[12] = x0;

         H.data[0] = x22_;
         H.data[1] = x32;
         H.data[2] = x02_;
         H.data[3] = x12;

         H.data[7] = x12;
         H.data[8] = x02;
         H.data[9] = x32;
         H.data[10] = x22;

         H.data[14] = x02;
         H.data[15] = x12_;
         H.data[16] = x22_;
         H.data[17] = x32;


         x = add_mat(mul_mat(A,x),mul_mat(B,w));
         P = mul_mat(mul_mat(A,P), transpose(A));
         Matrix tem = mul_mat(P,transpose(H));
         Matrix S = mul_mat(H,tem);
         S = inverse(S);
         K = mul_mat(mul_mat(tem,transpose(H)),S);
         Matrix inovation = sub_mat(y,mul_mat(H,x));
         x = add_mat(x,mul_mat(K,inovation));
         P = mul_mat(sub_mat(I,mul_mat(K,H)),P);
//         mul_mat(B,w);
         x.print(huart3);


//         y_.print(huart3);
//         w.print(huart3);
          }
         return state;
	}
	virtual ~EKF();
};
EKF::EKF(double* x_, double *P_, double* Q_, double*R_){

}
EKF::~EKF() {
	// TODO Auto-generated destructor stub
	 // free(this->data);
}
#endif /* INC_EKF_H_ */
