#pragma warning(disable:4996)

//#include "../stdafx.h"

#include "Course.h"
//#include "Cars.h"
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <cstdlib>
#include <cmath>
#include <numeric>
#include <algorithm>

#include <windows.h>
#include "MyUtils2.h"

#include <boost/lexical_cast.hpp>

using namespace std;
using namespace boost;
using namespace myutil2;

namespace CarManagement{
	// Courseクラス

	// CourseクラスからCarManagerを参照するためのポインタを初期化
	//CarManager *Course::cm = NULL;

	Course::Course(string file)
	{
		INDEBUG( cout << "Loading the course file [" << file << "]..." );

		// ポインタ初期化
		checkPoint = NULL;

		// 入力ファイル
		coursefile = file;
		ifstream ifs(coursefile.c_str());

		if( !ifs.is_open())throw new std::exception();

		// チェックポイント数
		//string str;
		//ifs >> str;
		ifs >> num_of_checkpoints;
		// チェックポイント格納用配列を確保
		checkPoint = new CheckPoint[num_of_checkpoints];
		// ファイルからチェックポイントを読み取って代入
		for(int i=0; i<num_of_checkpoints; i++)
		{
			ifs >> checkPoint[i].pos[0] >> checkPoint[i].pos[1] >> checkPoint[i].pos[2]
			>> checkPoint[i].upvector[0] >> checkPoint[i].upvector[1] >> checkPoint[i].upvector[2];
		}

		// tangentとnormalとlengthとuを設定
		length = 0;
		for(int i=0; i<num_of_checkpoints; i++)
		{
			// インデックス
			int i_next = (i + 1) % num_of_checkpoints;
			int i_prev = (i - 1 + num_of_checkpoints) % num_of_checkpoints;

			// tangentの向きを求める
			checkPoint[i].tangent[0] = checkPoint[i_next].pos[0] - checkPoint[i].pos[0];
			checkPoint[i].tangent[1] = checkPoint[i_next].pos[1] - checkPoint[i].pos[1];
			checkPoint[i].tangent[2] = checkPoint[i_next].pos[2] - checkPoint[i].pos[2];

			// tangentを正規化
			normalize(&checkPoint[i].tangent[0], &checkPoint[i].tangent[1], &checkPoint[i].tangent[2]);

			// tangentとupvectorからnormalを計算
			outer_product(checkPoint[i].upvector, checkPoint[i].tangent, checkPoint[i].normal);
			// 正規化も
			normalize(&checkPoint[i].normal[0], &checkPoint[i].normal[1], &checkPoint[i].normal[2]);

			// tangentとnormalからupvuectorを計算しなおす
			outer_product(checkPoint[i].tangent, checkPoint[i].normal, checkPoint[i].upvector);

			// 距離 [i] - [i-1]
			double len_p = dist(checkPoint[i].pos[0], checkPoint[i].pos[1], checkPoint[i].pos[2],
				checkPoint[i_prev].pos[0], checkPoint[i_prev].pos[1], checkPoint[i_prev].pos[2]);

			// コース全長lengthを設定
			length += len_p;

			// 各チェックポイントのスタートからの距離uを計算
			if(i == 0) checkPoint[i].u = 0;
			else checkPoint[i].u = checkPoint[i-1].u + len_p;
		}

		INDEBUG( cout << "Complete!!" << endl );
	}

	Course::~Course()
	{
		if(checkPoint != NULL) SafeDeleteArray(checkPoint);
	}

	int Course::getCurrentLap(double u) const
	{
		// 周回数が-1000まではバグらないはず
		try{
			u += 1000 * length;
		}catch(...){
			OutputDebugStringW(L"hoge!");
		}
		return (int)(u / length) - 1000;
	}

	double Course::getCurrentLapPosition(double u) const
	{
		// 周回数をカット 周回数が-1000まではバグらないはず
		u = u - getCurrentLap(u) * length;
		return u;
	}

	// まびき探索or1ステップ前の位置周り探索で高速化の余地あり
	int Course::getNearestCheckPointIndex(double x, double y, double z) const
	{
		int ind = 0;
		double min_distance = 1.0e20, tmp;
		for(int i=0; i<num_of_checkpoints; i++)
		{
			// ここで二乗距離を計算
			tmp = dist2(x, y, z, checkPoint[i].pos[0], checkPoint[i].pos[1], checkPoint[i].pos[2]);
			if(min_distance > tmp) { ind = i; min_distance = tmp; }
		}
		return ind;
	}

	//int Course::getNearestCheckPointIndex(const Car &car) const
	//{
	//	return getNearestCheckPointIndex(car.getX(), car.getY(), car.getZ());
	//}

	// やっぱりだめなのでとりあえず元に戻した
	// 計算誤差を小さくするためにコースの点の打ち方を少し工夫する必要がある
	// 点が直線状に並ばないところでは点が等間隔で細かく打ってないと少しおかしくなる可能性があるので
	// とくに直線と曲線の切り替わり部分で切り替わる点の前後も等間隔になっていることを確認すべし
	int Course::getSecondNearestCheckPointIndex(double x, double y, double z, int nearest) const
	{
		double next, prev;
		int i_next, i_prev;
		i_next = (nearest + 1) % num_of_checkpoints;
		next = dist2(x, y, z, checkPoint[i_next].pos[0], checkPoint[i_next].pos[1], checkPoint[i_next].pos[2]);

		i_prev = (nearest - 1 + num_of_checkpoints) % num_of_checkpoints;
		prev = dist2(x, y, z, checkPoint[i_prev].pos[0], checkPoint[i_prev].pos[1], checkPoint[i_prev].pos[2]);

		return (next < prev) ? i_next : i_prev;
	}

	//int Course::getSecondNearestCheckPointIndex(const Car &car, int nearest) const
	//{
	//	return getSecondNearestCheckPointIndex(car.getX(), car.getY(), car.getZ(), nearest);
	//}
	int Course::getSecondNearestCheckPointIndex(double x, double y, double z) const
	{
		return getSecondNearestCheckPointIndex(x, y, z, getNearestCheckPointIndex(x, y, z));
	}
	//int Course::getSecondNearestCheckPointIndex(const Car &car) const
	//{
	//	return getSecondNearestCheckPointIndex(car.getX(), car.getY(), car.getZ());
	//}

	// 前回のインデックスから調べるようにすることで，より高速化は可能
	int Course::getPrevCheckPointIndex(double u, double v, double w) const
	{
		// 周回数をカット
		u = getCurrentLapPosition(u);

		// 最後のチェックポイントを超えていれば，最後のチェックポイントを返す
		if(checkPoint[num_of_checkpoints-1].u < u) return num_of_checkpoints-1;
		// u座標を超えない一番大きなチェックポイントのインデックスを求める(2分探索化した)
		int i_max = num_of_checkpoints - 1;
		int i_min = 0;
		int i_ave;
		while(i_min + 1 < i_max)
		{
			i_ave = (i_max + i_min) / 2;
			if(u < checkPoint[i_ave].u) i_max = i_ave;
			else i_min = i_ave;
		}
		return i_min;
	}


	double Course::getX(double u, double v, double w) const
	{
		double x, y, z;
		UVW2XYZ(u, v, w, &x, &y, &z);
		return x;
	}

	double Course::getY(double u, double v, double w) const
	{
		double x, y, z;
		UVW2XYZ(u, v, w, &x, &y, &z);
		return y;
	}

	double Course::getZ(double u, double v, double w) const
	{
		double x, y, z;
		UVW2XYZ(u, v, w, &x, &y, &z);
		return z;
	}

	double Course::getU(double x, double y, double z) const
	{
		double u, v, w;
		XYZ2UVW(x, y, z, &u, &v, &w);
		return u;
	}
	double Course::getV(double x, double y, double z) const
	{
		double u, v, w;
		XYZ2UVW(x, y, z, &u, &v, &w);
		return v;
	}
	double Course::getW(double x, double y, double z) const
	{
		double u, v, w;
		XYZ2UVW(x, y, z, &u, &v, &w);
		return w;
	}

	//double Course::getX(const Car &car) const { return getX(car.getU(), car.getV(), car.getW()); }
	//double Course::getY(const Car &car) const { return getY(car.getU(), car.getV(), car.getW()); }
	//double Course::getZ(const Car &car) const { return getZ(car.getU(), car.getV(), car.getW()); }

	//double Course::getU(const Car &car) const { return getU(car.getX(), car.getY(), car.getZ()); }
	//double Course::getV(const Car &car) const { return getV(car.getX(), car.getY(), car.getZ()); }
	//double Course::getW(const Car &car) const { return getW(car.getX(), car.getY(), car.getZ()); }

	void Course::UVW2XYZ(double u, double v, double w, double *x, double *y, double *z) const
	{
		// 周回数をカット
		u = getCurrentLapPosition(u);

		// u座標で前のインデックスを求める
		int i_prev = getPrevCheckPointIndex(u, v, w);

		// uの前のチェックポイントからの差分(v,wはチェックポイントの値が0なのでそのまま使える)
		double du = u - checkPoint[i_prev].u;

		// 座標のチェックポイントからの差分
		double dx = 0, dy = 0, dz = 0;

		dx += du * checkPoint[i_prev].tangent[0];
		dx += v * checkPoint[i_prev].normal[0];
		dx += w * checkPoint[i_prev].upvector[0];

		dy += du * checkPoint[i_prev].tangent[1];
		dy += v * checkPoint[i_prev].normal[1];
		dy += w * checkPoint[i_prev].upvector[1];

		dz += du * checkPoint[i_prev].tangent[2];
		dz += v * checkPoint[i_prev].normal[2];
		dz += w * checkPoint[i_prev].upvector[2];

		// 戻り値セット
		*x = checkPoint[i_prev].pos[0] + dx;
		*y = checkPoint[i_prev].pos[1] + dy;
		*z = checkPoint[i_prev].pos[2] + dz;
	}

	void Course::XYZ2UVW(double x, double y, double z, double *u, double *v, double *w) const
	{
		// 与えられた位置から最も近い2点をインデックス順にi_first,i_secondとする
		int i_first, i_second;
		i_first = getNearestCheckPointIndex(x, y, z);
		i_second = getSecondNearestCheckPointIndex(x, y, z, i_first);
		if(i_first > i_second) swap(i_first, i_second);

		// 基準とすべき座標系のインデックスをi_baseとする
		int i_base;
		if(i_first == 0 && i_second == num_of_checkpoints-1) i_base = num_of_checkpoints - 1;
		else i_base = i_first;

		// x,y,zのチェックポイントからの差分
		double dx, dy, dz;
		dx = x - checkPoint[i_base].pos[0];
		dy = y - checkPoint[i_base].pos[1];
		dz = z - checkPoint[i_base].pos[2];

		// u,v,wのチェックポイントからの差分()
		double du = 0, dv = 0, dw = 0;

		du += dx * checkPoint[i_base].tangent[0];
		du += dy * checkPoint[i_base].tangent[1];
		du += dz * checkPoint[i_base].tangent[2];

		dv += dx * checkPoint[i_base].normal[0];
		dv += dy * checkPoint[i_base].normal[1];
		dv += dz * checkPoint[i_base].normal[2];

		dw += dx * checkPoint[i_base].upvector[0];
		dw += dy * checkPoint[i_base].upvector[1];
		dw += dz * checkPoint[i_base].upvector[2];

		// 戻り値セット
		*u = checkPoint[i_base].u + du;
		*v = dv;
		*w = dw;

		if(_isnan(dv) || _isnan(dw))
		{
			printf("uvw coodinate computation error!!\n");
		}
	}


	void Course::CheckPoint::getRollPitchYaw(double *roll, double *pitch, double *yaw) const
	{
		// http://demura.net/9ode/511.html より
		//*roll = atan2(normal[2], upvector[2]);
		//*pitch = atan2(-tangent[2], sqrt(normal[2]*normal[2] + upvector[2]*upvector[2]));
		//*yaw = atan2(tangent[1], tangent[0]);

		// 左手系？
		*roll = atan2(upvector[1], upvector[2]);
		*pitch = atan2(-upvector[0], sqrt(upvector[1]*upvector[1] + upvector[2]*upvector[2]));
		*yaw = atan2(tangent[1],tangent[0]);
	}


	// 便利系関数等
	inline double dist2(double x1, double y1, double z1, double x2, double y2, double z2)
	{
		x1 = x1 - x2;
		y1 = y1 - y2;
		z1 = z1 - z2;
		return (x1*x1 + y1*y1 + z1*z1);
	}

	double dist(double x1, double y1, double z1, double x2, double y2, double z2)
	{
		return sqrt(dist2(x1, y1, z1, x2, y2, z2));
	}

	void normalize(double *x, double *y, double *z)
	{
		double d = dist(*x, *y, *z);
		*x /= d;
		*y /= d;
		*z /= d;
	}

	void outer_product(double a[], double b[], double res[])
	{
		res[0] = a[1] * b[2] - a[2] * b[1];
		res[1] = a[2] * b[0] - a[0] * b[2];
		res[2] = a[0] * b[1] - a[1] * b[0];
	}

	// 簡易外積
	double crossprod2d(const double &ax, const double &ay, const double &bx, const double &by)
	{
		double z = ax * by - ay * bx;
		return z;
	}

}