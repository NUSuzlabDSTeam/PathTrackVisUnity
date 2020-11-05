#pragma once

/// COMSCOMプロジェクト用共有データ構造体定義ファイル
/// このファイルを編集した場合，c#側のcomscom.csも合わせて修正すること．

/** 
	処理系ごとの共有メモリの構成：
	基地局：
		COMSCOM
		COMS_SENSOR0  COMS_SENSOR1  COMS_SENSOR2
		COMS_CONTROL0 COMS_CONTROL1 COMS_CONTROL2

	各車両 (nは車両ID, 0,1,2）：
		COMSCOM
		COMS_SENSORn
		COMS_CONTROLn
		
	- COMSCOMの同期はTinyBee2Shareが行う
	- 各車両のCOMS_SENSORn, COMS_CONTROLnの基地局へのアップロードはWiFi2Shareが行う（未実装）
 **/

#define RENEW_DATA_STRUCT


#include <cstring>

#pragma pack(push, 8)

/// ################## 名大 COMS C#→他プロセス 共有データ構造体 ########################
/// <summary>
/// ひとつ分のLRFデータ共有用構造体
/// </summary>
/// 1024点までの計測情報と，各レイの角度情報を送る．
/// 角度は複数のLRF情報を気にせず送るために用意したが，
/// いらないかも．(or始点，終点角度だけ送ると節約に…)
struct LRF_DATA
{
	enum {
		MAX_NUM_OF_RAY = 1024,
	};

	//float position[3];                  ///< LRF取り付け位置（車体座標[m]）
	//float orientation;                  ///< LRF取り付け向き（上から見て反時計回り[rad]）
    int   num_of_ray;
    float range [MAX_NUM_OF_RAY];		///< 距離データ[m]
	float angle [MAX_NUM_OF_RAY];		///< 角度データ[deg] 前方が0deg
	//float origin[2];					///< 点群描画時に必要な原点
	//float points[2*MAX_NUM_OF_RAY];		///< 点群データ

    LRF_DATA(int x=0)
    {
		num_of_ray = x;
		//memset(position, 0, sizeof(position));
		//orientation = 0.0f;
		memset(range , 0, sizeof(range ));
		memset(angle , 0, sizeof(angle ));
		//memset(origin, 0, sizeof(origin));
		//memset(points, 0, sizeof(points));
    }
};

/// <summary>
/// タイムスタンプ等をCOMS足回り→他プロセスに通知する構造体    
/// </summary>
struct COMS_TIME
{
    /// 制御フレーム番号[step] (足回りプログラムがクロックを制御)
    int t_frame;
    /// 制御開始からの時間[msec] (足回りプログラムから書き込み)
    float time_elap;
    /// サンプリング時間(基準値)
    float time_Ts;
    /// サンプリング時間(実測値)
    float time_intvl;

    /// 時刻
    COMS_TIME(int x = 0)
    {
        t_frame = 0;
        time_elap = 0;
        time_Ts = 0;
        time_intvl = 0;
    }
};

/// <summary>
/// エンコーダや他センサ等，LRF以外のデータ共有用構造体
/// </summary>
struct SENSOR_DATA
{
    //int l_enc;       /// 左エンコーダカウント値
    //int r_enc;       /// 右エンコーダカウント値
	//int l_enc_diff;  /// 左エンコーダ差分(ラップ済み)
    //int r_enc_diff;  /// 右エンコーダ差分(ラップ済み)
	//float pot_accel;	/// アクセルペダルポテンショ(0:踏まない～1:max踏んだ)
	//float pot_brake;	/// ブレーキペダルポテンショ(0:踏まない～1:max踏んだ)
	//float pot_steer;	/// ステアリング  ポテンショ(-1:左～1:右)(方向は変わる可能性があります)	
	//float frc_accel;	/// アクセルペダル踏力
	//float frc_brake;	/// ブレーキペダル踏力
    float l_vel_mps;   /// 左タイヤ速度(m/s)
    float r_vel_mps;   /// 右タイヤ速度(m/s)
	float steer_angle;   ///< 操舵切れ角[rad]（ステアリングポテンショ電圧からの換算値）
    float tire_angle;	///< タイヤ角[rad]

	SENSOR_DATA(){
		l_vel_mps = 0.0f;
		r_vel_mps = 0.0f;
		steer_angle = 0.0f;
		tire_angle = 0.0f;
	}
};

#ifdef RENEW_DATA_STRUCT
struct ODOMETRY_DATA{
	float v_mps;		/// オドメトリによる車両前後速度(m/s)
    float w_radps;		/// オドメトリによる車両角速度(rad/s)
	float trip;			/// 制御開始時からの走行距離(v_mpsの数値積分)
	float lcl_x;		/// 制御開始時からのx．初期姿勢における前後方向距離．
    float lcl_y;		/// 制御開始時からのy．初期姿勢における左右方向距離．
	float angle;		/// 制御開始時の向きを0とした時の角度(w_radpsの数値積分)
};
#else
struct ODOMETRY_DATA{
	float v_mps;		/// オドメトリによる車両前後速度(m/s)
    float w_radps;		/// オドメトリによる車両角速度(rad/s)
	float trip;			/// 制御開始時からの走行距離(v_mpsの数値積分)
	float angle;		/// 制御開始時の向きを0とした時の角度(w_radpsの数値積分)
	float lcl_x;		/// 制御開始時からのx．初期姿勢における前後方向距離．
    float lcl_y;		/// 制御開始時からのy．初期姿勢における左右方向距離．
};
#endif 

#ifdef RENEW_DATA_STRUCT
/// <summary>
/// グローバル位置情報情報．GPSから取得した値．
/// </summary>
/// GPSで取得した緯度，経度など．絶対位置で． // added by Okuda, 2016 03 03
struct SEMIGLOBAL_DATA
{
    float y_diff;     /// distance from 35deg 10min in N-S direction
    float x_diff;    /// distance from 137deg + 5min in E-W direction
    float z;		    /// altitude
    float roll;              /// roll by GNSS
    float pitch;             /// pitch by GNSS
    float yaw;		        /// yaw by GNSS
    int state;       // state of GNSS                                    
};
#endif

struct GPS_DATA{
	/// レシーバから受信する情報
	float	time;			///< UTCタイムゾーンでの時刻 hhmmss.ss
	float	latitude;		///< 緯度 1234.56なら北緯12度34.56分
	float	longitude;		///< 経度 1234.56なら東経12度34.56分
	int		quality;		///< 品質
	int		numSats;		///< 計算に使われた衛星数
	float	dilution;		///< 精度
	float	altitude;		///< 標高（ジオイド面からの高さ）
	float	geoidHeight;	///< 楕円体からのジオイド面の高さ

	/// 換算して得られる情報
	float	latMeter;		///< 経度のメートル換算
	float	lonMeter;		///< 緯度のメートル換算
};

struct JOYSTICK_DATA{
	/// スティックの名称
	struct Axis{
		enum{ H1, V1, H2, V2, Num };
	};
	/// ボタンの名称
	struct Button{
		enum{ Y, X, B, A, L1, R1, L2, R2, Left, Right, Select, Start, Num };
	};

	/// スティック変位の範囲
	enum{
		AxisMin = -32768,
		AxisMax =  32767,
	};

	short	axis  [Axis  ::Num];
	short	button[Button::Num];

	JOYSTICK_DATA(){
		for(int i = 0; i < Axis::Num; i++)
			axis[i] = 0;
		for(int i = 0; i < Button::Num; i++)
			button[i] = false;
	}
};

/// ################## 自己位置同定プロセス→他プロセスへの構造体 ########################
/// <summary>
/// 自己位置同定結果を書き出す共有メモリ領域
/// </summary>

struct LOCALIZE_DATA{
	float g_x;		/// 自己位置同定プログラムによるグローバル位置x(m)
    float g_y;		/// 自己位置同定プログラムによるグローバル位置y(m)
	float g_yaw;	/// 自己位置同定プログラムによるグローバル位置yaw(rad)

#ifdef RENEW_DATA_STRUCT
	float g_z;		/// added by Okuda @ 2016_03_07 to keep consistency with comscom_nu.cs
#endif 
	float trip;		/// 制御開始時からの走行距離(m)
	float v_mps;	/// 走行速度(m/s)
	float w_radps;	/// 角速度(m/s)
};
		
#define COMS_LOCALIZE_NAME ("COMS_LOCALIZE")
#define COMS_LOCALIZE_MUTEX_NAME ("COMS_LOCALIZE_MUTEX")
struct COMS_LOCALIZE
{
	/// 時間情報(最新データが計算された際の)
	COMS_TIME time;
	/// 自己位置同定結果
	LOCALIZE_DATA state;
	/// パーティクル尤度など
	float Likelihood;

};

/// <summary>
/// Information of leading vehicle for following
/// </summary>
/// Information of leading car and relative info between ego and lead car
struct LEADINGCAR_DATA
{
    float range;         /// Distance to rear of leading vehicle (m)
    float range_rate;    /// Relative velocity (m/s)
    float speed;		    /// Driving speed of leading car (m/s)
    float lateral;       /// Lateral displacement (m)
    float direction;     /// Direction to leading vehicle (rad)

};


/**
	COMSのCSプログラムから，他のプロセスに向けた情報発信用構造体
 **/
#define COMS_SENSOR_NAME ("COMS_SENSOR")
#define COMS_SENSOR_MUTEX_NAME ("COMS_SENSOR_MUTEX")
struct COMS_SENSOR
{
    ///-------------------------- 時間管理部 -----------------------
    COMS_TIME		time;

    ///-------------------------- センシングデータ -----------------------
    LRF_DATA		lrf_front;
	LRF_DATA        lrf_rear;
	LRF_DATA        lrf_right;
    LRF_DATA        lrf_left;

	SENSOR_DATA		sense_data;

	ODOMETRY_DATA	odo_data;

    /// GPSによるローカライズ情報
    LOCALIZE_DATA gps_lcl_data;

    /// 自己位置情報
    LOCALIZE_DATA lcl_data;

    /// 追従データ
    LEADINGCAR_DATA lcar_data;

#ifdef RENEW_DATA_STRUCT
	/// semi global from GPS
	SEMIGLOBAL_DATA semiglobalpos_data;
#endif 

	///-------------------------- 操作入力 -----------------------
    //JOYSTICK_DATA   joy_data;

    static const char* GetLRFSuffix(int idx){
        switch (idx){
            case 0: return "front";
            case 1: return "rear";
            case 2: return "right";
            case 3: return "left";
        }
        return "";
    }
	LRF_DATA* GetLRF(int idx){
		switch (idx){
            case 0: return &lrf_front;
            case 1: return &lrf_rear ;
            case 2: return &lrf_right;
            case 3: return &lrf_left ;
        }
        return 0;
	}
    
    COMS_SENSOR(){}
};

/// 車両(COMS)への操作量共有用構造体
/// ################## 名大 他プロセス→COMS C# 共有データ構造体 ########################
/// <summary>
/// 名大その他プロセス → COMS制御プログラム(C#)のデータ共有用構造体
/// </summary>

/// <summary>
/// 制御量共有用構造体
/// </summary>
/// 
struct COMS_OUTPUT
{
	int odr_type;            ///< 指令タイプ。 0 : トルク、 1 : 速度+タイヤ角
    float vel_ref;

    float acc_ref;           ///< 指令加速度値(m/s2)：追加by奥田

    float tire_angle_ref;
    float accel_trq;         ///< アクセル量トルク(ひとまず正規化0～1) 0:無し 1:max
    float brake_trq;         ///< ブレーキ量トルク(ひとまず正規化0～1) 0:無し 1:max
    float steer_trq;         ///< ハンドル量トルク(ひとまず正規化-1～1) -1:左max  0:無し 1:右max
    int gearR;				///< リバースギア  0:ニュートラル 1:Rギア
	int gearD;				///< ドライブギア  0:ニュートラル 1:Dギア
	int winkerL;				///< 左ウィンカー  0:消灯 1:点灯
	int winkerR;				///< 右ウィンカー  0:消灯 1:点灯
};

/// <summary>
/// 軌道共有用構造体
/// </summary>
struct PATH_DATA
{
    int		num_of_points;
	float	x[1024];
	float	y[1024];
	float	yaw[1024];

	PATH_DATA(int n=0)
    {
        num_of_points = n;
		memset( (void*)x, 0, sizeof(x));
		memset( (void*)y, 0, sizeof(y));
		memset( (void*)yaw, 0, sizeof(yaw));
    }
};

/// <summary>
/// 同定した自己位置，通信で得たもの等，他車情報も含めた共有用構造体
/// </summary>
#define COMS_CONTROL_NAME ("COMS_CONTROL")
#define COMS_CONTROL_MUTEX_NAME ("COMS_CONTROL_MUTEX")
struct COMS_CONTROL
{
	// COMSに指定したい制御量
    COMS_OUTPUT coms_output;

	/// 目標軌道
	PATH_DATA	path_data;

	COMS_CONTROL(int x = 0)
    {
    }
};

//





#pragma pack(pop)
