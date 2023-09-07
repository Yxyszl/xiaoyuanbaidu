#include <iostream>
#include "LQR_track/LQR.h"
 
using namespace std;
 
void LQR::initial(double L_, double T_, vehicleState car, waypoint waypoint, U U_r, double *Q_, double *R_) {
 
	L = L_;
	T = T_;
	x_car = car.x; y_car = car.y; yaw_car = car.yaw;
	x_d = waypoint.x; y_d = waypoint.y; yaw_d = waypoint.yaw;
	v_d = U_r.v;kesi_d = U_r.kesi;
 
	for (int i = 0; i < 3; i++) {
		Q3[i] = Q_[i];
	}
	for (int j = 0; j < 2; j++) {
		R2[j] = R_[j];
	}
}
 
void LQR::param_struct() {
	Q << Q3[0], 0.0, 0.0,
		0.0, Q3[1], 0.0,
		0.0, 0.0, Q3[2];
	//cout << "Q矩阵为：\n" << Q << endl;
	R << R2[0], 0.0,
		0.0, R2[1];
	//cout << "R矩阵为：\n" << R << endl;
	A_d << 1.0, 0.0, -v_d * T * sin(yaw_d),
		0.0, 1.0, v_d* T* cos(yaw_d),
		0.0, 0.0, 1.0;
	//cout << "A_d矩阵为:\n" << A_d << endl;
	B_d << T * cos(yaw_d), 0.0,
		T* sin(yaw_d), 0.0,
		T* tan(kesi_d), v_d* T / (L * cos(kesi_d) * cos(kesi_d));
	//cout << "B_d矩阵为:\n" << B_d << endl;
	X_e << x_car - x_d, y_car - y_d, yaw_car - yaw_d;
	//cout << "X_e矩阵为:\n" << X_e << endl;
	
}
 
Matrix2x3 LQR::cal_Riccati() {
	int N = 150;//迭代终止次数
	double err = 100;//误差值
	double err_tolerance = 0.01;//误差收敛阈值
	Matrix3x3 Qf = Q;
	Matrix3x3 P = Qf;//迭代初始值
	//cout << "P初始矩阵为\n" << P << endl;
	Matrix3x3 Pn;//计算的最新P矩阵
	for (int iter_num = 0; iter_num < N; iter_num++) {
		Pn = Q + A_d.transpose() * P * A_d - A_d.transpose() * P * B_d * (R + B_d.transpose() * P * B_d).inverse() * B_d.transpose() * P * A_d;//迭代公式
		//cout << "收敛误差为" << (Pn - P).array().abs().maxCoeff() << endl;
		//err = (Pn - P).array().abs().maxCoeff();//
		err = (Pn - P).lpNorm<Eigen::Infinity>();
		if(err < err_tolerance)//
		{
			P = Pn;
			//cout << "迭代次数" << iter_num << endl;
			break;
		}
		P = Pn;
			
	}
	
	//cout << "P矩阵为\n" << P << endl;
	//P = Q;
	Matrix2x3 K = -(R + B_d.transpose() * P * B_d).inverse() * B_d.transpose() * P * A_d;//反馈率K
	return K;
}
 
U LQR::cal_vel() {
	U output;
	param_struct();
	Matrix2x3 K = cal_Riccati();
	Matrix2x1 U = K * X_e;
	//cout << "反馈增益K为：\n" << K << endl;
	//cout << "控制输入U为：\n" << U << endl;
	output.v = U[0] + v_d;
	output.kesi = U[1] + kesi_d;
	return output;
}
 
void LQR::test() //控制器效果测试
{
	/*param_struct();
	while (temp < 1000) {
		Matrix2x3 K = cal_Riccati();
		Matrix2x1 U = K * X_e;
		//cout <<"state variable is:\n" <<X_e << endl;
		//cout <<"control input is:\n"<< U << endl;
		Matrix3x1 X_e_ = A_d * X_e + B_d * U;
		X_e = X_e_;
		temp++;
	}*/
	Matrix3x3 C,D,F;
	C << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;
	F << 1.0, 0.0, 0.0, 0.0, 4.0, 0.0, 0.0, 0.0, 7.0;
	D = (C - F);
	double BBBB = D.lpNorm<Eigen::Infinity>();
	cout << BBBB << endl;
}