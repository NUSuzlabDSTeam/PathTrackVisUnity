#pragma once



//#include "carsim\CarsimCar2014.h"


// 最適化のためのデータ集合
struct opt_slt{

	int n ;  //ステップ数
	double r ;  //入力の重み
	double s;
	double tf;  //ホライゾン長さ
	double u; //入力
	double acc;
	MatrixXd Sf;  //終端重み
	MatrixXd Qd;  
	MatrixXd Ad;  
	MatrixXd Bd;
	MatrixXd Wd;
	MatrixXd Cd;
	



	VectorXd X; //状態ベクトル
	VectorXd U; //入力ベクトル
	VectorXd Rho; //曲率
	VectorXd Vref; //速度リファレンス
	VectorXd Xp; //予測値ベクトル
	MatrixXd A;
	MatrixXd B;
	MatrixXd C;
	MatrixXd W;
	MatrixXd Q;
	MatrixXd R;
	MatrixXd S;
	MatrixXd K; //ゲイン
	MatrixXd D;

};


class LateralControl;
class LongitudinalControl;


// データ保存のための構造体
struct LOG_DATA{

	// ここに保存するデータのメンバを追加する

	// まずは車両から来るデータ
	double t;
	double x;
	double y;
	double u;
	double v;
	double yaw;
	double speed;
	
	double throttle;
	double brake;
	double steer;
	double rpm;

	// 車両への出力情報
	double steer_ref;
	double tire_angle_ref;
	double vel_ref;
	double v_command;

	// 制御用変数とか
	double ctrl_u;
//	double a_ref;
	double v_ref;
//	double th_ref;
	double yaw_modi;
	int n_lap;
//	int p_zero;

	// リファレンスコース
	double theta_e;
	double theta_d;
	double rho;
	double rho_modi;
	double y_e_dot;
	double theta_e_dot;

	// Lane中央
	double lane_u;
	double lane_y_e;
	double lane_theta_d;
	double lane_theta_e;

	// 先読み情報
	double ahead_l; // 先読み長さ．車から他の先読み情報を取得するより先に設定する必要がある．
	double ahead_x;
	double ahead_y;
	double ahead_u;
	double ahead_v;
	double ahead_theta_d;
	double ahead_theta_e;
	
	double U_log0;
	double U_log1;
	double U_log2;
	double U_log3;
	double U_log4;

	// ここに，保存するデータと，保存するときのタグを登録する
	// ここで登録しないと保存されない
	static void RegisterMember(RTCLib::DataStorage<LOG_DATA>& logger )
	{
		logger.register_member("t", &LOG_DATA::t);
		logger.register_member("x", &LOG_DATA::x);
		logger.register_member("y", &LOG_DATA::y);
		logger.register_member("u", &LOG_DATA::u);
		logger.register_member("v", &LOG_DATA::v);
		logger.register_member("yaw", &LOG_DATA::yaw);
		logger.register_member("speed", &LOG_DATA::speed);

		logger.register_member("throttle", &LOG_DATA::throttle);
		logger.register_member("brake", &LOG_DATA::brake);
		logger.register_member("steer", &LOG_DATA::steer);

		logger.register_member("steer_ref", &LOG_DATA::steer_ref);
		logger.register_member("tire_angle_ref", &LOG_DATA::tire_angle_ref);
		logger.register_member("vel_ref", &LOG_DATA::vel_ref);
		logger.register_member("v_command", &LOG_DATA::v_command);
		
		logger.register_member("y_e_dot", &LOG_DATA::y_e_dot);
		logger.register_member("theta_e_dot", &LOG_DATA::theta_e_dot);

		logger.register_member("ctrl_u", &LOG_DATA::ctrl_u);
		//logger.register_member("a_ref", &LOG_DATA::a_ref);
		logger.register_member("v_ref", &LOG_DATA::v_ref);
		//logger.register_member("th_ref", &LOG_DATA::th_ref);
		logger.register_member("yaw_modi", &LOG_DATA::yaw_modi);
		logger.register_member("n_lap", &LOG_DATA::n_lap);
		//logger.register_member("p_zero", &LOG_DATA::p_zero);
		logger.register_member("theta_e", &LOG_DATA::theta_e);

		logger.register_member("theta_d", &LOG_DATA::theta_d);
		logger.register_member("rho", &LOG_DATA::rho);
		logger.register_member("rho_modi", &LOG_DATA::rho_modi);

		// Deviation from references 
		logger.register_member("lane_u", &LOG_DATA::lane_u);
		logger.register_member("lane_v", &LOG_DATA::lane_y_e);
		logger.register_member("lane_theta_d", &LOG_DATA::lane_theta_d);
		logger.register_member("lane_theta_e", &LOG_DATA::lane_theta_e);

		logger.register_member("ahead_x", &LOG_DATA::ahead_x);
		logger.register_member("ahead_y", &LOG_DATA::ahead_y);
		logger.register_member("ahead_u", &LOG_DATA::ahead_u);
		logger.register_member("ahead_v", &LOG_DATA::ahead_v);
		logger.register_member("ahead_theta_d", &LOG_DATA::ahead_theta_d);
		logger.register_member("ahead_theta_e", &LOG_DATA::ahead_theta_e);

		logger.register_member("U_log0", &LOG_DATA::U_log0);
		logger.register_member("U_log1", &LOG_DATA::U_log1);
		logger.register_member("U_log2", &LOG_DATA::U_log2);
		logger.register_member("U_log3", &LOG_DATA::U_log3);
		logger.register_member("U_log4", &LOG_DATA::U_log4);
	}


};


