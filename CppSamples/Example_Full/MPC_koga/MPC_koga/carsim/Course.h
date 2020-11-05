
#ifndef __COURSE_H__
#define __COURSE_H__

#pragma once

#include <vector>
#include <string>
//#include "CarManager.h"

namespace CarManagement{

	using namespace std;

	//class Car;
	//class CarManager;

	// コースを扱うクラス
	class Course
	{
	public:
		// Vec3構造体を内部定義
		struct CheckPoint
		{
			double pos[3]; // グローバル座標に対する位置
			double u; // その点までのコースに沿った距離
			double tangent[3]; // コースの接線方向(実際は次の点の方向から計算したもの)
			double normal[3]; // コースの平面内で，tangentに垂直方向(tangentとupvectorより計算したもの)
			double upvector[3]; // コースの面に垂直な方向(データとして与える)

			// チェックポイントの座標系からグローバルに対する姿勢roll,pitch,yawを計算
			// いまのところ合ってるかどうか不明
			void getRollPitchYaw(double *roll, double *pitch, double *yaw) const;
		};

		// 読み込んだコースファイル
		std::string coursefile;

		// コース長
		double length;

		// チェックポイントの数
		int num_of_checkpoints;
		// チェックポイントの配列
		CheckPoint *checkPoint;

		// CourseからCarManagerを参照するためのポインタ
		//static CarManager *cm;

		// コンストラクタ
		Course( std::string file);

		// デストラクタ
		~Course();

		// 周回数を返す関数
		int getCurrentLap(double u) const;
		// 周回数をカットし，この周回における走行位置を返す関数
		double getCurrentLapPosition(double u) const;

		// 一番近いチェックポイントのインデックスを返す関数
		int getNearestCheckPointIndex(double x, double y, double z) const;
		//int getNearestCheckPointIndex(const Car &car) const;
		// 1ステップ前の一番近かったインデックスを渡してそこから検索することで高速化を図るバージョン
		int getNearestCheckPointIndex(double x, double y, double z, int prev) const;
		//int getNearestCheckPointIndex(const Car &car, int prev) const;

		// 二番目に近いチェックポイントのインデックスを返す関数
		// 厳密に言うと，渡されたチェックポイントの前後のうちの近い方を返す関数
		int getSecondNearestCheckPointIndex(double x, double y, double z) const;
		//int getSecondNearestCheckPointIndex(const Car &car) const;
		// 二番目に近いチェックポイントのインデックスを返す関数(一番近いチェックポイントを渡すことで高速化)
		// 厳密に言うと，渡されたチェックポイントの前後のうちの近い方を返す関数
		int getSecondNearestCheckPointIndex(double x, double y, double z, int nearest) const;
		//int getSecondNearestCheckPointIndex(const Car &car, int nearest) const;

		// 超えている一番大きなチェックポイントのインデックスを返す関数
		int getPrevCheckPointIndex(double u, double v, double w) const;

		// 位置座標を求める
		double getX(double u, double v, double w) const;
		//double getX(const Car &car) const;
		double getY(double u, double v, double w) const;
		//double getY(const Car &car) const;
		double getZ(double u, double v, double w) const;
		//double getZ(const Car &car) const;

		// コースに沿った座標を求める
		// ただしuに関しては周回数まで含めた距離はわからない
		double getU(double x, double y, double z) const;
		//double getU(const Car &car) const;
		double getV(double x, double y, double z) const;
		//double getV(const Car &car) const;
		double getW(double x, double y, double z) const;
		//double getW(const Car &car) const;

		// 位置座標からコースに沿った座標を求める関数
		// ただし全ての点と現在位置を比較すると時間がかかりすぎるので，1ステップ前のu座標を渡すことで高速化
		void XYZ2UVW(double x, double y, double z, double *u, double *v, double *w) const;

		// コースに沿った座標から位置座標を求める関数
		void UVW2XYZ(double u, double v, double w, double *x, double *y, double *z) const;
	};


	// 便利系関数等

	// 二乗距離
	double dist2(double x1, double y1, double z1, double x2=0, double y2=0, double z2=0);

	// 距離(dist2より重い)
	double dist(double x1, double y1, double z1, double x2=0, double y2=0, double z2=0);

	// 正規化
	void normalize(double *x, double *y, double *z);

	// 外積(範囲チェック？何それおいしいの？)
	void outer_product(double a[], double b[], double res[]);

	// 簡易外積
	double crossprod2d(const double &ax, const double &ay, const double &bx, const double &by);

}


#endif // __COURSE_H__