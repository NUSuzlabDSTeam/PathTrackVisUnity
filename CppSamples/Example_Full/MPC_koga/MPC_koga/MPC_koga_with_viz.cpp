
// SMDriveCpp.cpp : コンソール アプリケーションのエントリ ポイントを定義します。
//

#include "stdafx.h"


//#include "Course.h"

using namespace std;
using namespace Eigen;


int p_zero;

#include "optdata_st.h"
#include "MPCSates.h"

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

	//-----------------------------------------------------
	// for debug process
	RTCLib::SharedMemoryAccessor<MPCStates> shr_fordebug;
	shr_fordebug.TryOpen( "MPC_DEBUG_INFO", "MPC_DEBUG_INFO_MUTEX");

	// Launch data viewer if it is possible
	try{
	ShellExecute(NULL, "open", "COMSMPCDataView.exe", NULL, NULL, SW_SHOWNORMAL);
	}catch(...)
	{
	}
	//-----------------------------------------------------

	bool LoopAlive  = true;
	int ctrl_cnt = 0;	
	

	// 状態リファレンスの読み込み
	RTCLib::CSVLoader map("jyousai2_map.csv",1);

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
	
	// data logger:
	RTCLib::DataStorage<LOG_DATA> logger;
	LOG_DATA::RegisterMember(logger);

	keisoku.start();



	//変数、定数の宣言////////////////////////
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
	int p; //マップ参照用

	//拡大行列計算用とか
#ifdef DELAY
	MatrixXd I = MatrixXd::Identity(5,5);
#else
	MatrixXd I = MatrixXd::Identity(4,4);
#endif
	MatrixXd O;
	MatrixXd TMP,TMP_A,TMP_B,TMP_W;
	const double pi = 3.141592654;
	double delta = 0.1;   //制御周期（とりあえず縦横同じ）
	//////////////////////////////////////////////////////////////////


	//parameters from csv
	RTCLib::CSVLoader prm("parameter.csv",1);

	//for...

	//変数、定数の初期化
	// ステア制御用パラメータ構築
	opt_slt& ltrl = lat_ctrl->CreateParameter(prm,0);	//ステアリング
	opt_slt& lgtd = lon_ctrl->CreateParameter(prm,0);

	//状態計算用
	double hold1=0, hold2=0, hold3=0;
	double vel_ref = 0;
	double pre_u = 0;
	int n_lap = 1; //周回数
	p_zero = 0;
#ifdef DELAY
	double alpha = 10;
#endif

	//ヘッダファイルでやりたい．とりあえずここで．
	parameter car_mdl;
	parameter ctrl_prm;
	int n=0;//for文用に変数にしとく

	double M = prm[n][prm.GetColOf("M")];
	double Iz = prm[n][prm.GetColOf("Iz")];
	double lf = prm[n][prm.GetColOf("l_f")];
	double	lr = prm[n][prm.GetColOf("l_r")];
	double	Cf = prm[n][prm.GetColOf("C_f")];
	double	Cr = prm[n][prm.GetColOf("C_r")];

	car_mdl.a(0,0) = (Cf+Cr)/M;
	car_mdl.a(0,1) = (lr*Cr-lf*Cf)/M;
	car_mdl.a(1,0) = (lf*Cf-lr*Cr)/Iz;
	car_mdl.a(1,1) = -(lf*lf*Cf+lr*lr*Cr)/Iz;
	car_mdl.b(0) = Cf/M;
	car_mdl.b(1) = (lf*Cf)/Iz;

	ctrl_prm.alpha = prm[n][prm.GetColOf("alpha")];	//時定数（の逆数？）
	ctrl_prm.steerrate_limit = prm[n][prm.GetColOf("steerrate_limit")];
	ctrl_prm.acceleration_limit = prm[n][prm.GetColOf("acceleration_limit")];
	ctrl_prm.v_const = prm[n][prm.GetColOf("v_const")];
	ctrl_prm.steer_const = prm[n][prm.GetColOf("steer_const")];
	ctrl_prm.course_length = prm[0][prm.GetColOf("course_length")];


	//////////////////////////////////////////////////////////////////

	stp.start();


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
		p = get_p(u, map, p_zero, 1); 

		//周回数更新/////////////////////
		if (u < 20 && pre_u > ctrl_prm.course_length-20)
		{
			n_lap += 1;
			p_zero = 0;
		}

		yaw = car_state.yaw + 2 * pi *(n_lap-1);
		pre_u = u;
		//////////////////////////////////


		//	Rho,Vrefの取得
		double u_f = u; //k時点でのu座標

		for(int i=0; i<ltrl.n; i++){
			if(i==0) p = p_zero;

			p = get_p(u_f, map, p, 0);

			ltrl.Rho(i) = get_rho(u_f, map, p);

#ifdef LONGITUDINAL
			lgtd.Vref(i) = get_vref(u_f, map, p);
#else
			lgtd.Vref(i) = ctrl_prm.v_const;
#endif
			u_f = u_f +  lgtd.Vref(i)*ltrl.tf; 	
		}
		

		lgtd.Vref(lgtd.n) = get_vref(u_f, map, p);
		
		
		///////縦方向////////////////////////////////////////////
#ifdef LONGITUDINAL
		lgtd.X(0) = vel;
		lgtd.U = lgtd.K * ( lgtd.A * lgtd.X - lgtd.Vref);
		acc = lgtd.U(0); //目標加速度
		
		lgtd.Xp = lgtd.A * lgtd.X + lgtd.B * lgtd.U;
		
		lgtd.u= hold3 + acc * delta;	//目標速度
		lgtd.acc = acc;
#else 
		lgtd.Xp = ctrl_prm.v_const * VectorXd::Ones(lgtd.n+1);
		lgtd.u = ctrl_prm.v_const;
#endif
		/////////////////////////////////////////////////////////

		


		//////横方向/////////////////////////////////////////////
#ifdef LATERAL
		MEASURE_BLOCK_TIME
		if (lgtd.Xp(0) == 0) lgtd.Xp(0) = 0.01;

#ifdef DELAY
		//ゲイン行列の再計算
		TMP_A = I;
		for (int m = 0; m < ltrl.n; m++)
		{	
			v = lgtd.Xp(m);
			ltrl.Bd <<	0, 0, 0, 0, ltrl.tf;
			ltrl.Wd <<	0, (car_mdl.a(0,1)-v*v)*ltrl.tf, 0, car_mdl.a(1,1)*ltrl.tf, 0;

			TMP_B = ltrl.Bd;
			TMP_W = ltrl.Wd;

			O = MatrixXd::Zero(5,1);
			for (int l = 0; l < (ltrl.n+1)*5; l+=5)
			{	
				if (l > m*5)
				{	
					v = lgtd.Xp(l/5-(m+1));
					ltrl.Ad <<	1, ltrl.tf, 0, 0, 0,
						0, 1-car_mdl.a(0,0)*ltrl.tf/v, car_mdl.a(0,0)*ltrl.tf, car_mdl.a(0,1)*ltrl.tf/v, car_mdl.b(0)*ltrl.tf,  
						0, 0, 1, ltrl.tf, 0,
						0, -car_mdl.a(1,0)*ltrl.tf/v, car_mdl.a(1,0)*ltrl.tf, 1+car_mdl.a(1,1)*ltrl.tf/v, car_mdl.b(1)*ltrl.tf,
						0, 0, 0, 0, 1-alpha*ltrl.tf;   

					ltrl.B.block(l,m,5,1) = TMP_B;
					TMP_B = ltrl.Ad * TMP_B;

					ltrl.W.block(l,m,5,1) = TMP_W;
					TMP_W = ltrl.Ad * TMP_W;

				}else
				{
					ltrl.B.block(l,m,5,1) = O;
					ltrl.W.block(l,m,5,1) = O;
				}


			}
		}
		
		MEASURE_BLOCK_TIME
			

		TMP_A = I;
		for (int l = 0 ; l < (ltrl.n+1)*5; l+=5)
		{
			if(l==0)
			{
				TMP_A = I;
			}
			else{
			v = lgtd.Xp(l/5-1);
					ltrl.Ad <<	1, ltrl.tf, 0, 0, 0,
						0, 1-car_mdl.a(0,0)*ltrl.tf/v, car_mdl.a(0,0)*ltrl.tf, car_mdl.a(0,1)*ltrl.tf/v, car_mdl.b(0)*ltrl.tf,  
						0, 0, 1, ltrl.tf, 0,
						0, -car_mdl.a(1,0)*ltrl.tf/v, car_mdl.a(1,0)*ltrl.tf, 1+car_mdl.a(1,1)*ltrl.tf/v, car_mdl.b(1)*ltrl.tf,
						0, 0, 0, 0, 1-alpha*ltrl.tf;    
		
			TMP_A = ltrl.Ad * TMP_A;
			}
			
			ltrl.A.block<5,5>(l,0) = TMP_A;
		}
		
#else
		//ゲイン行列の再計算
		TMP_A = I;
		for (int m = 0; m < ltrl.n; m++)
		{	
			v = lgtd.Xp(m);
			ltrl.Bd <<	0, car_mdl.b(0)*ltrl.tf, 0, car_mdl.b(1)*ltrl.tf;
			ltrl.Wd <<	0, (car_mdl.a(0,1)-v*v)*ltrl.tf, 0, car_mdl.a(1,1)*ltrl.tf;

			TMP_B = ltrl.Bd;
			TMP_W = ltrl.Wd;

			O = MatrixXd::Zero(4,1);
			for (int l = 0; l < (ltrl.n+1)*4; l+=4)
			{	
				if (l > m*4)
				{	
					v = lgtd.Xp(l/4-(m+1));
					ltrl.Ad <<	1, ltrl.tf, 0, 0,
						0, 1-car_mdl.a(0,0)*ltrl.tf/v, car_mdl.a(0,0)*ltrl.tf, car_mdl.a(0,1)*ltrl.tf/v,  
						0, 0, 1, ltrl.tf,
						0, -car_mdl.a(1,0)*ltrl.tf/v, car_mdl.a(1,0)*ltrl.tf, 1+car_mdl.a(1,1)*ltrl.tf/v;

					ltrl.B.block(l,m,4,1) = TMP_B;
					TMP_B = ltrl.Ad * TMP_B;

					ltrl.W.block(l,m,4,1) = TMP_W;
					TMP_W = ltrl.Ad * TMP_W;

				}else
				{
					ltrl.B.block(l,m,4,1) = O;
					ltrl.W.block(l,m,4,1) = O;
				}


			}
		}
		
		MEASURE_BLOCK_TIME
			

		TMP_A = I;
		for (int l = 0 ; l < (ltrl.n+1)*4; l+=4)
		{
			if(l==0)
			{
				TMP_A = I;
			}
			else{
			v = lgtd.Xp(l/4-1);
					ltrl.Ad <<	1, ltrl.tf, 0, 0,
						0, 1-car_mdl.a(0,0)*ltrl.tf/v, car_mdl.a(0,0)*ltrl.tf, car_mdl.a(0,1)*ltrl.tf/v,  
						0, 0, 1, ltrl.tf,
						0, -car_mdl.a(1,0)*ltrl.tf/v, car_mdl.a(1,0)*ltrl.tf, 1+car_mdl.a(1,1)*ltrl.tf/v;
		
			TMP_A = ltrl.Ad * TMP_A;
			}
			
			ltrl.A.block<4,4>(l,0) = TMP_A;
		}
#endif
		MEASURE_BLOCK_TIME

		//MatrixXd tmp_mat = (ltrl.B.transpose() * ltrl.C.transpose() * ltrl.Q * ltrl.C * ltrl.B + ltrl.D.transpose()*ltrl.R*ltrl.D).inverse();	
		MatrixXd tmp_mat = (ltrl.B.transpose() * ltrl.C.transpose() * ltrl.Q * ltrl.C * ltrl.B + ltrl.D.transpose()*ltrl.R*ltrl.D + ltrl.S).inverse();	
		ltrl.K = -( tmp_mat * ltrl.B.transpose() * ltrl.C.transpose() * ltrl.Q ); 
		
		MEASURE_BLOCK_TIME
			

		p = get_p(u, map, p_zero, 0);
		theta_d = get_theta_d(u, map, p_zero);

		// @ okuda
		theta_e = yaw - theta_d;

		y_e_dot = (y_e - hold1)/ delta;
		theta_e_dot =  (theta_e - hold2)/ delta;




		//状態ベクトル
		ltrl.X(0)=y_e;
		ltrl.X(1)=y_e_dot;
		ltrl.X(2)=theta_e;
		ltrl.X(3)=theta_e_dot;
#ifdef DELAY
		ltrl.X(4)=tire_angle;
#endif
		
		ltrl.U = ltrl.K * ltrl.C * (ltrl.A *ltrl.X + ltrl.W * ltrl.Rho);
		ltrl.u = ltrl.U(0);

		//for Debug
		MatrixXd X_p = ltrl.A*ltrl.X+ltrl.B*ltrl.U+ltrl.W*ltrl.Rho;
		MatrixXd Y_p = ltrl.C*X_p;
#else
		ltrl.u = ctrl_prm.steer_const;
#endif



		/////////////////////////////////////////////////////////



		
		//lgtd.u= hold3 + acc * delta;	//目標速度
		//lgtd.acc = acc;

		//lgtd.U(0) = get_throttle(lgtd.u/3.6,acc/3.6); //目標スロットル

		hold1 = y_e;
		hold2 = theta_e;
		hold3 = vel;
		

		MEASURE_BLOCK_TIME
		
		//命令
		//car.SetOutput( ctrl_cnt, 1, ltrl);
		car.SetOutput( ctrl_cnt, 1, ltrl, lgtd);
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
		tmp.v_ref = lgtd.u;
		//tmp.a_ref = acc;
		//tmp.th_ref = lgtd.U(0);
		//tmp.yaw_modi = yaw;
		//tmp.n_lap = n_lap;
		tmp.p_zero = p_zero;


		// 割り振ったデータ構造を，ストレージに格納
		logger.push_back( tmp );

		MEASURE_BLOCK_TIME


		// ---------- debug print here -------------
		static int debug_print_skip_count = 0;
		const int decimate_number = 10;
		if( debug_print_skip_count++ % decimate_number == 0)
		{
			// computation for debug;
			double interval = keisoku.lap()/decimate_number;
#ifdef LATERAL
			MatrixXd Delta_U_forDP = ltrl.U.transpose()*ltrl.D.transpose();
			MatrixXd Cost_Delta_U_forDP = ltrl.U.transpose()*ltrl.D.transpose()*ltrl.R*ltrl.D*ltrl.U;
			MatrixXd Cost_State_forDP = Y_p.transpose()*ltrl.Q*Y_p;
			MatrixXd Cost_U_forDP = ltrl.U.transpose()*ltrl.S*ltrl.U;
#endif

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

			//Control state
#ifdef LATERAL
			cout <<"MPC state:[" << ltrl.X.transpose() << "]" << endl;

			cout << "Difference of input(prediction):" << endl << Delta_U_forDP << endl;
			cout << "Input(prediction):" << endl << ltrl.U << endl;
			cout << "State error(prediction):" << endl ;
			for (int i=0; i<ltrl.n; i++){
				printf("%.4f  %.4f\n", 
					Y_p(2*i), Y_p(2*i+1));
			}
			cout << "Difference of input(cost function):" << Cost_Delta_U_forDP << endl;
			cout << "State error(cost function):" << Cost_State_forDP << endl;
			cout << "Input(cost function):" << Cost_U_forDP << endl;
#endif
			cout << "laps:" << n_lap << endl << endl;

			// ---------------------------------------------------------
			// send data through SharedMemory for debug @ Okuda 2015 11/19
			// time
			shr_fordebug.p_shared_data->time.time = car_state.t;
			shr_fordebug.p_shared_data->time.interval = interval;
		
			// states
			shr_fordebug.p_shared_data->car_state = car_state;

			// costs
			shr_fordebug.p_shared_data->cost_du = Cost_Delta_U_forDP(0);
			shr_fordebug.p_shared_data->cost_u = Cost_U_forDP(0);
			shr_fordebug.p_shared_data->cost_state = Cost_State_forDP(0);
			
			// predictions
			shr_fordebug.p_shared_data->valid_predicted_states_length = ltrl.n;
			for ( int i=0; i < ltrl.n; i++ )
			{
				shr_fordebug.p_shared_data->predicted_states[i].U = ltrl.U(i);
				shr_fordebug.p_shared_data->predicted_states[i].X = X_p(i);
				shr_fordebug.p_shared_data->predicted_states[i].Vref = lgtd.Vref(i);
				shr_fordebug.p_shared_data->predicted_states[i].Rho = ltrl.Rho(i);
			}
			
			// --------------------------------------------
		}



		if(do_wait)
		{
			//		制御周期100ms
			while( stp.split()  < ltrl.tf)
			{ 
				Sleep(1);	
			}
		}

		stp.reset();



		ctrl_cnt++;

		// uが到達したら終了，3つ分くらい手前で終了すればまぁ大丈夫かな？？
		//if( car_state.u > map[map.Rows()-3][0] )break;
		if (n_lap == 2) break; //１周で終了
	}

	// 保存処理．ファイルクローズのため，この中カッコは消さない事
	{
		time_t now = time(NULL);
		tm* now_local = localtime(&now);

		char str[256];
		strftime(str, sizeof(str)-1, "%Y%m%d_%H%M%S", now_local);

		string date_str(str);
		string filename = string("mpcdata")
			+ "_" + date_str 
			+ "_n" + boost::lexical_cast<string>(ltrl.n)
			+ "_r" + boost::lexical_cast<string>(ltrl.r)
			+ "_q" + boost::lexical_cast<string>(ltrl.Qd(0))
			+ "_in" + (car.mode==CAR_DYN_CARSIM ? "SIM" : "DS")
			+ ".csv";
		ofstream data_out_stream(filename); 

		// ログデータをストリームに書き出し，クリア
		logger.print(data_out_stream);
		logger.clear();
	}


	// パラメータ探索時はここまでを繰り返す

	// ----------------------------------------
	// Close DataViewProcess
	HWND target;
	target = FindWindowA(NULL, "COMSMPCDataView");
	SendMessage( target, WM_SYSCOMMAND, SC_CLOSE, 0);

	// ---------------------------------------

	return 0;




}