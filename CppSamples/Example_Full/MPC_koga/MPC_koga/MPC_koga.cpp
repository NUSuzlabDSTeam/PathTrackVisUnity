
// SMDriveCpp.cpp : コンソール アプリケーションのエントリ ポイントを定義します。
//

#include "stdafx.h"
#include "direct.h"


//#include "Course.h"

using namespace std;
using namespace Eigen;


int p_zero;

#include "optdata_st.h"
#include "MPCSates.h"
#include "parameters.h"

// for small functions
#include "utils.h"
#include "CarDynamicsDelegator.h"
#include "lateral_control.h"
#include "longitudinal_control.h"


int _tmain(int argc, _TCHAR* argv[])
{

	RTCLib::StopWatch keisoku;	//計測用
	RTCLib::StopWatch stp;		//制御周期
	RTCLib::StopWatch perform;	//パフォーマンス測定用
	RTCLib::StopWatch msr;

	bool LoopAlive  = true;
	int ctrl_cnt = 0;
	
	//-----------------------------------------------------
	// for debug process
	RTCLib::SharedMemoryAccessor<MPCStates> shr_fordebug;
	shr_fordebug.TryOpen( "MPC_DEBUG_INFO", "MPC_DEBUG_INFO_MUTEX");

	// Launch data viewer if it is possible
#ifdef ENABLE_RESULT_VISUALIZER
	try{
	ShellExecuteA(NULL, "open", "COMSMPCDataView.exe", NULL, NULL, SW_SHOWNORMAL);
	}catch(...)
	{
	}
#endif
	//-----------------------------------------------------


	// 状態リファレンスの読み込み
	RTCLib::CSVLoader map(DRIVING_MAP,1);
	//parameters from csv
	RTCLib::CSVLoader prm("parameter.csv",1);

	// read true path of sim file
	std::ifstream ifs("c:\\simfile_place.txt");
	std::string new_fn;
	ifs >> new_fn;
	ifs.close();

	// vehicle dynamics computer
	CarDynamicsDelegator car(new_fn);

	// ####################### DS モードと CARSIM モードの切り換え ########
	//car.Init(CAR_DYN_DS);
	car.Init(CAR_DYN_CARSIM);
	//car.Init(CAR_DYN_COMS);
	
	// lateral controller
	std::shared_ptr<LateralControl> lat_ctrl(new LateralControl());
	std::shared_ptr<LongitudinalControl> lon_ctrl(new LongitudinalControl());
	std::shared_ptr<CarModel> car_model(new CarModel());
	std::shared_ptr<ControlParameter> ctrl_param(new ControlParameter());
	
	// data logger:
	RTCLib::DataStorage<LOG_DATA> logger;
	LOG_DATA::RegisterMember(logger);

	keisoku.start();

	

	//変数、定数の宣言////////////////////////////////////////////////
	//縦用
	double acc; //目標加速度

	//横用
	double yaw; //現在の車両のヨー角
	double tire_angle; //現在の車両のタイヤ角
	double v; //目標（予測）車両速度
	
	//共通
	double u;	//現在の車両のu座標
	double vel;//現在の車両の速度
	double y_e, y_e_dot, theta_e, theta_e_dot, theta_d; //状態変数
	double pre_input = 0;
	int p; //マップ参照用

	const double pi = 3.141592654;
	//////////////////////////////////////////////////////////////////

	//for...
	//for (int h=0; h<3; h++){

	//変数、定数の初期化//////////////////////////////////////////////
	int h = 0;
	opt_slt& ltrl = lat_ctrl->CreateParameter(prm,h);	//ステアリング
	opt_slt& lgtd = lon_ctrl->CreateParameter(prm,h);	//速度
	parameter& car_mdl = car_model->SetParameter(prm,h);	//車両パラメータ設定
	parameter& ctrl_prm = ctrl_param->SetParameter(prm,h);	//制御用パラメータ設定

	//状態計算用




	double pre_y_e=0, pre_theta_e=0, pre_vel=0, pre_u=0, pre_rho=0, pre_rho_input=0, pre_rho_diff=0, pre_rho_n=0;
	double vel_ref = 0;

	//周回数
	int n_lap = 1;

	//状態量の次元
	#ifdef DELAY
		#ifdef BICYCLE
			double dim_x = 5;
		#else
			#ifndef RHO
				double dim_x = 3;
			#else
				double dim_x = 4;
			#endif
		#endif
	#else
		#ifdef BICYCLE
			double dim_x = 4;
		#else
			#ifndef RHO
				double dim_x = 2;
			#else
				double dim_x = 3;
			#endif
		#endif
	#endif

	//マップの最初からスタート！
	p_zero = 0;
	//////////////////////////////////////////////////////////////////


	stp.start();

			msr.reset();
	// Loop 
	while(LoopAlive)
	{
		// 各項目の処理時間計測のためのカウンタ
		int t1 = 0;
		
		perform.reset();
		perform.start();
#define	MEASURE_BLOCK_TIME	{if(false){printf("Term%d:%f\n",t1++,perform.lap_in_ms());}}

		// input command tests
		string cmd = "";
		int key_ret = _kbhit();
		if(key_ret != 0)
		{
			// if any key is pushed, stop and wait command
			cin >> cmd;
			cout << "Command [" << cmd << "] received" << endl;
		}
		if(cmd=="q" || cmd == "quit" || cmd == "e" || cmd == "exit")
		{
			// loop exit
			LoopAlive = false;
		}



		LOG_DATA car_state;
		
		// 車両から状態の取得
		car.GetLogDataFromCar( car_state );

		// get u and vel from shared memory @okuda
		u = car_state.u;
		vel = car_state.speed;
		y_e = car_state.v;
		tire_angle = car_state.steer * 0.058917;
		p = get_p(u, map, p_zero, 1, ctrl_prm.course_length); 

		//周回数更新///////////////////////////////////////////
		if (u < 20 && pre_u > ctrl_prm.course_length-20)
		{
			n_lap += 1;
			p_zero = 0;
		}
		yaw = car_state.yaw + 2 * pi *(n_lap-1); //for Carsim
		//yaw = car_state.yaw; //for COMS
		pre_u = u;
		///////////////////////////////////////////////////////



		//Rho,Vrefの取得///////////////////////////////////////
		VectorXd Rho = VectorXd::Zero(ltrl.n+1);
		VectorXd Delta_Rho = VectorXd::Zero(ltrl.n);
		double u_f = u; //k時点でのu座標
		double rhoo;
#ifdef RHO
		for(int i=0; i<ltrl.n+1; i++){
#else
		for(int i=0; i<ltrl.n; i++){
#endif

			if(i==0) p = p_zero;
			
			p = get_p(u_f, map, p, 0, ctrl_prm.course_length);

#ifdef RHO
			Rho(i) = get_rho(u_f, map, p);
			if(i!=ltrl.n) {
				ltrl.Rho(2*(i+1)-1) = Rho(i);
			}
			if(i!=0) {
				Delta_Rho(i-1) = Rho(i) - pre_rho_n;
				ltrl.Rho(2*i-2) = Delta_Rho(i-1);
			}
#else
			ltrl.Rho(i) = get_rho(u_f, map, p);
			
			//Rho(i) = get_rho(u_f, map, p);
			//if(i==0) rhoo = Rho(i);
			//if(i!=0) {
			//	//Delta_Rho(i-1) = Rho(i) - pre_rho_n;
			//	//ltrl.Rho(2*i-2) = Delta_Rho(i-1);
			//	Delta_Rho(i-1) = Rho(i) - Rho(i-1);
			//	
			//	ltrl.Rho(i-1) = ctrl_prm.K*((ctrl_prm.T1/ltrl.tf)*Delta_Rho(i-1) + Rho(i-1));
			//}


#endif


#ifdef LONGITUDINAL
			lgtd.Vref(i) = get_vref(u_f, map, p);
#else
			lgtd.Vref(i) = ctrl_prm.v_input;
#endif
		
			u_f += lgtd.Vref(i)*ltrl.tf;


#ifdef RHO
			pre_rho_n = Rho(i);
#endif
		}

#ifdef LONGITUDINAL
		p = get_p(u_f, map, p, 0, ctrl_prm.course_length);
		lgtd.Vref(lgtd.n) = get_vref(u_f, map, p);
#else
		lgtd.Vref(lgtd.n) = ctrl_prm.v_input;
#endif
		///////////////////////////////////////////////////////

	
		
		///////縦方向/////////////////////////////////////////
#ifdef LONGITUDINAL
		lgtd.X(0) = vel;
		lgtd.U = lgtd.K * ( lgtd.A * lgtd.X - lgtd.Vref);
		acc = lgtd.U(0); //目標加速度
		
		lgtd.Xp = lgtd.A * lgtd.X + lgtd.B * lgtd.U;
		
		lgtd.u= pre_vel + acc * ctrl_prm.delta;	//目標速度
		//lgtd.u= hold3 + delta * ( lgtd.U(0) + lgtd.U(1) + lgtd.U(2) + lgtd.U(3) + lgtd.U(4) + lgtd.U(5) );
		lgtd.acc = acc;

#else 
		lgtd.Xp = ctrl_prm.v_const * VectorXd::Ones(lgtd.n+1);
		lgtd.u = ctrl_prm.v_input;
#endif
		/////////////////////////////////////////////////////////

		


		//////横方向/////////////////////////////////////////////
		MEASURE_BLOCK_TIME
		
		//速度ゼロは発散（bicycle model）
		for (int i=0; i<ltrl.n+1; i++){
		if (lgtd.Xp(i) < 8) lgtd.Xp(i) = 8;
		}

		//係数行列の再計算
		//recompute_coefficientmat(ltrl, lgtd, car_mdl, ctrl_prm, dim_x);


		MEASURE_BLOCK_TIME

		//ゲイン行列の計算
		//MatrixXd mat1 = ltrl.C*ltrl.B;
		//MatrixXd tmp_mat = (mat1.transpose() * ltrl.Q * mat1 + ltrl.D.transpose()*ltrl.R*ltrl.D + ltrl.S).inverse();	
		//ltrl.K = -( tmp_mat * mat1.transpose() * ltrl.Q ); 

			
		
		MEASURE_BLOCK_TIME

			
		//状態の取得，計算
		p = get_p(u, map, p_zero, 0, ctrl_prm.course_length);
		theta_d = get_theta_d(u, map, p_zero);//リファレンスのヨー角をマップから引き当て

		theta_e = yaw - theta_d;

		//theta_eの補正
		if(theta_e > pi) theta_e -= 2*pi;
		else if(theta_e < -pi) theta_e += 2*pi;

		y_e_dot = (y_e - pre_y_e)/ ctrl_prm.delta;
		theta_e_dot =  (theta_e - pre_theta_e)/ ctrl_prm.delta;



		//cout << ltrl.B << endl << endl << ltrl.A << endl ;
		//状態ベクトル
		ltrl.X(0)=y_e;
		ltrl.X(1)=y_e_dot;
		ltrl.X(2)=theta_e;
		ltrl.X(3)=theta_e_dot;
		#ifdef DELAY
			ltrl.X(4)=tire_angle;
		#endif


		
		double u_delta =
			- y_e * 0.2//0.15
			- y_e_dot * 0.2//0.1
			- theta_e * 0.8//0.5
			- theta_e_dot * 0.2;//0.1

		//static double U_delta_tmp = 0;

		// Detailed debug print  --> by Okuda 2019/06/15
		printf("u=%.3f, v=%.3f\r\n", u, y_e);


		double U_velRef = 60.0 / 3.6;


		/////////////////////////////////////////////////////////


		//lgtd.u= hold3 + acc * delta;	//目標速度
		//lgtd.acc = acc;

		//lgtd.U(0) = get_throttle(lgtd.u/3.6,acc/3.6); //目標スロットル


		//現ステップの値を保存

		pre_y_e = y_e;
		pre_theta_e = theta_e;
		pre_vel = vel;

		

		MEASURE_BLOCK_TIME
		


		//命令
		//car.SetOutput( ctrl_cnt, 1, ltrl);
		//car.SetOutput( ctrl_cnt, 1, ltrl, lgtd);
		car.SetOutput(ctrl_cnt, 1, u_delta, U_velRef);
		bool do_wait = car.Update();

		MEASURE_BLOCK_TIME

		// ----------- log data -------------
		// 保存したいデータを増やしたい場合，
		// optdata_st.hの中のLOG_DATAのメンバを増やし，
		// registermember内で登録しておき，ここでLOG_DATAにデータを代入する．
		// 内容を書き込んだLOG_DATA構造体を，loggerに追加(push_back)する．
		// あとはlogger.printで保存される．
		LOG_DATA tmp = LOG_DATA();

		// 車両のは場合分けがあるので以下の中で書き込みしましょ
		// CarDynamicsDelegator.cppの中に実体がアルヨ
		car.GetLogDataFromCar( tmp );
		car.GetLogDataForCar( tmp );

		// 制御変数の保存割り振り.
		// 多くなるようなら専用の関数を用意してちょうだい．
		tmp.ctrl_u = ltrl.u;
		tmp.v_ref = lgtd.Vref(0);
		tmp.v_command = lgtd.u;
		tmp.n_lap = n_lap;
		//tmp.p_zero = p_zero;
		//tmp.a_ref = acc;
		//tmp.th_ref = lgtd.U(0);
#ifdef LATERAL
		tmp.yaw_modi = yaw;
		tmp.theta_e = theta_e;
		tmp.U_log0 = u_delta;
		//tmp.U_log0 =ltrl.U(0);
		//tmp.U_log1 =ltrl.U(1);
		//tmp.U_log2 =ltrl.U(2);
		//tmp.U_log3 =ltrl.U(3);
		//tmp.U_log4 =ltrl.U(4);
		tmp.theta_d = theta_d;
		tmp.y_e_dot = y_e_dot;
		tmp.theta_e_dot = theta_e_dot;

		double l1 = 1.0, l3 = 0.5, h1 = 0.3, h2 = 1.0;
		double L = (h1+h2)/h1*l1;
		tmp.rho = (1/L)*tmp.v+(1+l3/L)*tmp.theta_e;
		tmp.rho_modi = l1*tmp.rho;
#endif

		//tmp.rho = rhoo;
		//tmp.rho_modi = ltrl.Rho(0);

		// 割り振ったデータ構造を，ストレージに格納
		logger.push_back( tmp );

		MEASURE_BLOCK_TIME

						cout << msr.lap();


		// ---------- debug print here -------------
		static int debug_print_skip_count = 0;
		const int decimate_number = 10;
		if( debug_print_skip_count++ % decimate_number == 0)
		{

						// computation for debug;
			double interval = keisoku.lap()/decimate_number;


			//Time
			printf("Time=%f, ",
				//shr_dsctrl_info.p_shared_data->nCtrlCnt);  // @ okuda
				car_state.t);
			printf("Average Itvl：%.5f秒\n",interval);

			//Car state
			printf("Car state (x,y,  u,v)= (%.2f,%.2f,  %.2f,%.2f)\n", 
				car_state.x, car_state.y, car_state.u, car_state.v );
			printf("Car state (yaw,speed,steer)= (%.2f,%.2f,%.2f)\n", 
				car_state.yaw, car_state.speed,car_state.steer);
			cout << "laps:" << n_lap << endl << endl;

			

			// --------------------------------------------
		}



		if(do_wait)
		{
			//制御周期100ms
			while( stp.split()  < ltrl.tf )
			{ 
				Sleep(0);	
			}
		}

		stp.reset();


		ctrl_cnt++;


		if (n_lap == 6) break; //n_lap周に入った瞬間に終了
	}

	// 保存処理．ファイルクローズのため，この中カッコは消さない事
	{
		time_t now = time(NULL);
		tm* now_local = localtime(&now);

		char str[256];
		char str2[256];
		strftime(str, sizeof(str)-1, "%Y%m%d_%H%M%S", now_local);
		strftime(str2, sizeof(str)-1, "%Y%m%d", now_local);

		string date_str(str);
		string date_str2(str2);
		//string filename = string("mpcdata")
		//	+ "_" + date_str 
		//	+ "_n" + boost::lexical_cast<string>(ltrl.n)
		//	+ "_r" + boost::lexical_cast<string>(ltrl.r)
		//	+ "_q" + boost::lexical_cast<string>(ltrl.Qd(0))
		//	+ "_in" + (car.mode==CAR_DYN_CARSIM ? "SIM" : "DS")
		//	+ ".csv";
		//ofstream data_out_stream(filename); 
		


		_mkdir(str2);

		string filename = date_str2
			+ "/mpcdata"
			+ "_" + date_str 
			+ ".csv";
		ofstream data_out_stream(filename); 

		// ログデータをストリームに書き出し，クリア
		logger.print(data_out_stream);
		logger.clear();

		
		string prm_filename = date_str2	+ "/parameter" + date_str + ".txt";
		write_param(prm, prm_filename);
	}

	
	// パラメータ探索時はここまでを繰り返す
	//}
	
	// ----------------------------------------
	// Close DataViewProcess
	HWND target;
	target = FindWindowA(NULL, "COMSMPCDataView");
	SendMessage( target, WM_SYSCOMMAND, SC_CLOSE, 0);

	// ---------------------------------------

	return 0;




}