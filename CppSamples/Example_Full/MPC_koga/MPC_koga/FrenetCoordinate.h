#pragma once

#include <math.h>
#include <memory>
#include <string>
#include <algorithm>

#include <CSVLoader.h>

/// CSV loader for double matrix
class SimpleCSVLoader
{
public:
	void LoadPathFromCSV(std::string fn)
	{
		std::ifstream ifs(fn, std::ios::in);

		LoadTagLine(ifs);

		LoadBodyLine(ifs);
	}

	void LoadTagLine(std::ifstream& ifs)
	{
		// load header line
		char buf[1024];
		ifs.getline(buf, 1023);
		if (tags.size() > 0)
		{
			tags.clear();
		}

		char* ctx = nullptr;
		char* delim = ",";
		char *next = strtok_s(buf, delim, &ctx);

		while (next) {
			std::string tmp(next);
			tags.push_back(tmp);

			next = strtok_s(NULL, delim, &ctx);
		}
	}

	void LoadBodyLine(std::ifstream& ifs)
	{
		// load line
		const size_t N = 4096;


		char buf[N];
		ifs.getline(buf, N - 1);
		while (!ifs.eof())
		{
			//if (strcmpi(buf, "loop")==0)
			//{
			//	isPathLooping = true;
			//	break;
			//}
			ParseLine(buf);
			ifs.getline(buf, N - 1);
		}

	}

	void ParseLine(char buf[])
	{
		std::vector<double>	data_line;

		char* ctx = nullptr;
		char* delim = ",";
		char *next = strtok_s(buf, delim, &ctx);

		while (next) {
			double tmp_data = 0;
			//try {
			tmp_data = atof(next);
			//}
			//catch (...) {
			//	if (strcmpi(next, "loop")==0)
			//	{
			//		isPathLooping = true;
			//		break;
			//	}
			//	throw;
			//}
			data_line.push_back(tmp_data);

			next = strtok_s(NULL, delim, &ctx);
		}
		if (data_line.size() >= 6)
			data.push_back(data_line);
	}

public:

	bool isPathLooping = false;
	std::vector<std::string> tags;
	std::vector<std::vector<double>> data;

};


struct FrenetPoint2D
{
	// global position
	double x;
	double y;

	FrenetPoint2D* next;

	// u 
	double du;	// distance to next
	double u;	// distance from start along path

	// vector to next
	double tx_next;
	double ty_next;
	double t_next_theta;

	// normal vector to next
	double nx_next;
	double ny_next;
	double n_next_theta;

	// tangent vector
	double tx;
	double ty;
	double t_theta;

	// normal vector
	double nx;
	double ny;
	double n_theta;

	// curvature
	double kappa;



	FrenetPoint2D()
	{
		x = y = 0;
		u = 0;

		tx_next = ty_next = 0;
		nx_next = ny_next = 0;
		tx = ty = t_theta = 0;
		nx = ny = n_theta = 0;
		kappa = 0;
	}

	void CalculateU(const FrenetPoint2D &prev)
	{
		du = (x - prev.x)*(x - prev.x)
			+ (y - prev.y)*(y - prev.y);
		du = std::sqrt(du);
		u = prev.u + du;
	}

	inline void normalize(double &x, double &y)
	{
		double l = sqrt(x * x + y * y);
		x = x / l;
		y = y / l;
	}

	void ConnectToNext(FrenetPoint2D &_next)
	{
		next = &_next;
		tx_next = _next.x - x;
		ty_next = _next.y - y;
		normalize(tx_next, ty_next);
		t_next_theta = atan2(ty_next, tx_next);

		nx_next = -ty_next;
		ny_next = tx_next;
		n_next_theta = atan2(ny_next, nx_next);

	}

	void OutputLaneTagsForDebug(std::ofstream &ofs)
	{
		ofs << "x,y,du,u,"
			<< "tx_next,ty_next,t_next_theta,";

		ofs << "nx_next,ny_next,n_next_theta,";
		ofs << "tx,ty,t_theta,";
		ofs << "nx,ny,n_theta,";
		ofs << "kappa" << std::endl;
	}

	void OutputLaneForDebug(std::ofstream& ofs)
	{
		ofs << x << "," << y << "," << du << "," << u << ","
			<< tx_next << "," << ty_next << "," << t_next_theta << ",";

		ofs << nx_next << "," << ny_next << "," << n_next_theta << ",";
		ofs << tx << "," << ty << "," << t_theta << ",";
		ofs << nx << "," << ny << "," << n_theta << ",";
		ofs << kappa << "," << std::endl;
	}

};

typedef size_t CacheToGlobal;
typedef size_t CacheToFrenet;

class FrenetCoordinate2D
{
	const double pi = 3.141592653579793238;
public:

	bool isLooping = false;
	SimpleCSVLoader csvLoader;

	std::vector< FrenetPoint2D> points;
	std::vector<double> u_cache;

	double length;
	size_t size;

	FrenetCoordinate2D()
	{
	}

	FrenetCoordinate2D(std::string fn, bool bLoop)
	{
		LoadPathFromCSV(fn, bLoop);
	}

	void LoadPathFromCSV(std::string fn, bool bLoop)
	{
		isLooping = bLoop;
		csvLoader.LoadPathFromCSV(fn);
		if (isLooping)
		{
			points.resize(csvLoader.data.size()+1);
		}
		else {
			points.resize(csvLoader.data.size());
		}

		SetXY();
		ComputeU();
		Connect();
	}

	inline void SetXY()
	{
		for (size_t i = 0; i < csvLoader.data.size(); i++)
		{
			points[i].x = csvLoader.data[i][0];
			points[i].y = csvLoader.data[i][1];
		}
		if (isLooping)
		{	// if looping, n+1 th point is the start point
			size_t n = csvLoader.data.size();
			points[n].x = csvLoader.data[0][0];
			points[n].y = csvLoader.data[0][1];
		}
		size = points.size();
	}

	inline void ComputeU()
	{
		u_cache.resize(points.size());
		points[0].u = 0;
		points[0].du = 0;
		u_cache[0] = 0;
		for (size_t i = 1; i < points.size(); i++)
		{
			points[i].CalculateU(points[i - 1]);
			u_cache[i] = points[i].u;
		}
		length = points.back().u;
	}

	inline void Connect()
	{
		for (size_t i = 0; i < points.size()-1; i++)
		{
			points[i].ConnectToNext(points[i + 1]);
		}
	}

	// get global position from Frenet coordinate
	CacheToGlobal GetGlobal(double u, double v, 
		double &x, double &y, CacheToGlobal* prev)
	{
		CacheToGlobal index = SearchU(u, prev);

		double diffu = u - points[index].u;
		double diffv = v;

		double diffx = diffu * points[index].tx_next 
					+ diffv * points[index].nx_next;
		double diffy = diffu * points[index].ty_next
					+ diffv * points[index].ny_next;

		x = points[index].x + diffx;
		y = points[index].y + diffy;

		return index;
	}

	// get global position from Frenet coordinate together with yaw angle
	CacheToGlobal GetGlobal(double u, double v, double theta_e, 
		double &x, double &y, double& yaw, CacheToGlobal* prev)
	{
		size_t index = GetGlobal(u, v, x, y, prev);

		yaw = points[index].n_next_theta + theta_e;
		if (theta_e > pi) theta_e -= 2 * pi;
		if (theta_e < -pi) theta_e += 2 * pi;

		return index;
	}

	CacheToGlobal SearchU(double u, CacheToGlobal* cache)
	{
		size_t index = 0;
		size_t n = u_cache.size();
		if (cache == nullptr)
		{	// find from all : binary search
			size_t front = 0;
			size_t back = n - 1;
			while (back - front > 3)
			{
				size_t mid = (front + back) / 2;
				if (u_cache[mid] < u)
				{
					front = mid;
				}
				else {
					back = mid;
				}
				index = mid;
			}
		}
		else {
			index = *cache;
		}

		if (index < 0)index = 0;
		if (index >= n) index = n - 1;
		// search around mid

		// backward search
		while (index > 0 && u < u_cache[index])
		{
			index--;
		}
		// forward search
		while (index < n-2 && u > u_cache[index+1])
		{
			index++;
		}
		CacheToGlobal ret;
		ret = index;
		return ret;
	}

	CacheToFrenet GetFrenet(double x, double y, double yaw,
		double &u, double &v, double &theta_e, CacheToFrenet cache)
	{
		size_t index = GetFrenet(x, y, u, v, cache);
		theta_e = yaw - points[index].t_next_theta;

		if (theta_e > pi) theta_e -= 2 * pi;
		if (theta_e < -pi) theta_e += 2 * pi;

		return index;
	}

	CacheToFrenet GetFrenet(double x, double y,
		double &u, double &v, CacheToFrenet cache)
	{
		// check whether positive or negative side of prev index
		size_t index = cache;

		// vector from base point to current point
		double dx = x - points[index].x;
		double dy = y - points[index].y;

		double dotTanAndDiff;
		dotTanAndDiff =
			points[index].tx_next*dx + points[index].ty_next*dy;

		// backward search
		while (dotTanAndDiff < 0 && index > 0)
		{	// compute inner dot with tangent
			if (dotTanAndDiff < 0) index--;
			dotTanAndDiff =
				points[index].tx_next*dx + points[index].ty_next*dy;
		}
		if (dotTanAndDiff < 0 && index == 0)
		{	// if path is not looping, negative u is allowed.
			if ( ! isLooping)
			{
				u = dotTanAndDiff;
				v = points[index].nx_next*dx + points[index].ny_next*dy;
				return index;
			}
			// if path is looping, the (x,y) is regarded as 
			// 1 lap before
			u = u + length;

		}
		// forward search
		while (dotTanAndDiff > 0 && index < size - 1)
		{
			// vector from base point to current point
			size_t next = (index + 1) % size;
			double dxnext = x - points[next].x;
			double dynext = y - points[next].y;
			
			double dotTanAndDiffNext;
			dotTanAndDiffNext =
				points[next].tx_next*dxnext + points[next].ty_next*dynext;

			// if du from next point, index is advanced.
			if (dotTanAndDiffNext >= 0)
			{
				index++;
				continue;
			}
			if (dotTanAndDiffNext < 0)
			{
				break;
			}
		}
		u = points[index].u + dotTanAndDiff;
		v = points[index].nx_next*dx + points[index].ny_next*dy;
		return index;
	}

	void OutputLaneForDebug(std::string testfn)
	{
		std::ofstream ofs(testfn);
		points[0].OutputLaneTagsForDebug(ofs);

		for (size_t i = 0; i < size; i++)
		{
			points[i].OutputLaneForDebug(ofs);
		}

	}

};

