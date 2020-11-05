
#pragma once

#include "stdafx.h"
#include <queue>

//線形補完
double linear(double x1, double x2, double y1, double y2, double x){
	double y;
	if (x2==x1) y = y1;
	else y = ((y2 - y1)/(x2 - x1))*(x -x1) + y1;
	return y;
}

/// return a-b with wrapping
double diff_rad(double a, double b)
{
	double e = a - b;
	const double pi = 3.141592653579793238;
	if (e > pi) e = e - 2 * pi;
	if (e < -pi) e = e + 2 * pi;
	return e;
}

double limit(double a, double min, double max)
{
	if (a < min)return min;
	if (a > max)return max;
	return a;
}

double limit_warning(double a, double min, double max)
{
	if (a < min)
	{
		printf("Limit! %f reaches to %f\r\n", a, min);
		return min;
	}
	if (a > max)
	{
		printf("Limit! %f reaches to %f\r\n", a, max);
		return max;
	}
	return a;
}


////uからpを返す(b=1のときp_zeroを更新、0のときそのまま）
int get_p(double u, RTCLib::CSVLoader& map, int m, int b, double course_length, int& p_zero){
			
	if ( u > course_length )
	{ u = u - course_length;
	  m = 0;
	}

	//// Add skipping search 
	//// 2019 07 by Okuda
	//size_t Step = 10;
	//size_t skip = map.Rows() / Step;
	//for (int i = m; i < map.Rows(); i+=Step)
	//{
	//	if (u < map[i*Step][0]) break;
	//	m+=Step;
	//}

	for(int i = m ; i < map.Rows() ;i++)
	{ 
		if( u < map[i+1][0]) break;
		m++;
	}

	if(b==1) p_zero = m;
	

	return m;
}



//p、uからtheta_dを返す
double get_theta_d(double u, RTCLib::CSVLoader& map, int p){

	double theta_d;
	double pre_u = map[p][0];
	double next_u = map[p + 1][0];
	double pre_th = map[p][1];
	double next_th = map[p + 1][1];

	theta_d = linear(map[p][0], map[p + 1][0], map[p][1], map[p + 1][1], u);
	if (theta_d < -3.14159265357)theta_d += 2 * 3.14159265357;
	if (theta_d > +3.14159265357)theta_d -= 2 * 3.14159265357;
	return theta_d;
}


//p,uからrhoを返す
double get_rho(double u, RTCLib::CSVLoader& map, int p){

	double rho;	
	return rho = linear(map[p][0], map[p+1][0], map[p][2], map[p+1][2], u);
}


//p、uからvelを返す
double get_vref(double u, RTCLib::CSVLoader& map, int p){

	double v;
	return v = linear(map[p][0], map[p+1][0], map[p][3], map[p+1][3], u);
}


// rounding
double round(double x){
	if(x > 0.0) return floor(x+0.5);
	else return -1.0 * floor(fabs(x) + 0.5);
}



////正規乱数を計算
//void seiki(double *x,double *y){
//	double a,b,r,kaku;
//	a = rand()/((double)RAND_MAX+1.0);
//	b = rand()/((double)RAND_MAX+1.0);
//
//	r = sqrt(-2.0*log(1.0-a));
//
//	kaku = 6.2831853071795864769*b;
//
//	*x = r*cos(kaku);
//	*y = r*sin(kaku);
//
//	return;
//}

//double get_throttle(double vel, double acc)
//	{
//		double input;
//		
//		RTCLib::CSVLoader throttle("th_map.csv",0);
//		int i = floor(acc/0.1) + 9;
//		if(i < 0) i = 0;
//		if(i > throttle.Rows()) i = throttle.Rows(); 
//
//		int j = floor(vel/0.5) ;
//		if(j < 0) j = 0;
//		if(j > throttle.Cols()) j = throttle.Cols(); 
//
//		input = throttle[i][j];
//	
//
//		return input;
//	
//	}


// Save parameter to text file
void write_param(RTCLib::CSVLoader&	prm, string filename){

	vector<string> Header;
	vector<double> Value;

	string filename_dir = filename;

	ofstream ofs;
	ofs.open(filename_dir);


	for(int i=0; i<prm.Cols(); i++)
	{
		Header.push_back(prm.Header(i));
		Value.push_back(prm[0][i]);
		ofs << Header[i] << ":" << Value[i] << endl;
	}

	ofs.close();

}

class moving_average
{
private:
	moving_average() {};

	std::vector<double> history;
public:
	size_t size;
	size_t cursor;

	moving_average(int N)
		: history(N)
	{
		size = N;
		cursor = 0;
	}

	double input(double in)
	{
		if (history.size() < size - 1)
		{
			history.push_back(in);
			cursor++;
		}else{
			history[cursor%size] = in;
		}
		return get_average();
	}

	double get_average()
	{
		if (history.size() == 0)return 0;
		double sum = 0;
		for (size_t i = 0; i < history.size(); i++)
		{
			sum += history.at(i);
		}
		return sum / history.size();
	}



};


