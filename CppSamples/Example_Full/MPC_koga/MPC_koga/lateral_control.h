#pragma once

#include <Eigen/Core>
#include <Eigen/Sparse>
#include "optdata_st.h"

// lateral control MPC
class LateralControl
{
public:

	// ???
	double v;
	double pi;

	// 最適化パラメータと変数の実体
	// main内ではこれの参照を使う事
	opt_slt ltrl;

	LateralControl(){}

	// Create optimization parameters
	opt_slt& CreateParameter(RTCLib::CSVLoader &prm, int n)
	{
	
		ltrl.u=0;

double dim_y = 2;

#ifdef DELAY
	#ifdef BICYCLE
		double dim_x = 5;
		ltrl.Cd.resize(dim_y,dim_x);
		ltrl.Cd <<	1, 0, 0, 0,	0,		
					0, 0, 1, 0, 0;
	#else
		#ifndef RHO
			double dim_x = 3;
			ltrl.Cd.resize(dim_y,dim_x);
			ltrl.Cd <<	1, 0, 0,		
						0, 1, 0;
		#else
			double dim_x = 4;
			ltrl.Cd.resize(dim_y,dim_x);
			ltrl.Cd <<	1, 0, 0, 0,		
						0, 1, 0, 0;
		#endif
	#endif
#else
	#ifdef BICYCLE
		double dim_x = 4;
		double l1 = 1.0, l3 = 0.5, h1 = 0.3, h2 = 1.0;
		double L = (h1+h2)/h1*l1;
		ltrl.Cd.resize(dim_y,dim_x);
		//ltrl.Cd <<	1, 0, 0, 0,		
		//			0, 0, 1, 0;
		ltrl.Cd <<	1/L, 0, 1+l3/L, 0,		
					l1/L, 0, l1*(1+l3/L), 0;
	#else
		#ifndef RHO
			double dim_x = 2;
			ltrl.Cd.resize(dim_y,dim_x);
			ltrl.Cd <<	1, 0,		
						0, 1;
		#else
			double dim_x = 3;
			ltrl.Cd.resize(dim_y,dim_x);
			ltrl.Cd <<	1, 0, 0,		
						0, 1, 0;
		#endif
	#endif
#endif

#ifdef RHO
	double dim_w = 2;
#else
	double dim_w = 1;
#endif

		ltrl.Ad.resize(dim_x,dim_x);
		ltrl.Bd.resize(dim_x,1);
		ltrl.Wd.resize(dim_x,dim_w);
		ltrl.Qd.resize(dim_y,dim_y);
		ltrl.Sf.resize(dim_y,dim_y);
		


		ltrl.n = prm[n][prm.GetColOf("N")] ;     //ステップ数
		ltrl.r = prm[n][prm.GetColOf("R_lat")] ;//入力差分の重み
		ltrl.s = prm[n][prm.GetColOf("S_lat")] ;
		ltrl.tf = prm[n][prm.GetColOf("period")];  //ホライゾン長さ
		ltrl.Qd << prm[n][prm.GetColOf("Q_y_lat")] , 0, 0, prm[n][prm.GetColOf("Q_theta_lat")] ;
		ltrl.Sf << prm[n][prm.GetColOf("Sf_y_lat")] , 0, 0,prm[n][prm.GetColOf("Sf_theta_lat")] ;
		//double Rf = ltrl.n = prm[n][prm.GetColOf("Rf")] ;
		//double Sf = ltrl.n = prm[n][prm.GetColOf("Sf")] ;

		ltrl.X.resize(dim_x);
		ltrl.A.resize((ltrl.n + 1)*dim_x, dim_x);							 
		ltrl.B.resize( (ltrl.n + 1)* dim_x, ltrl.n);
		ltrl.W.resize( (ltrl.n + 1)* dim_x, ltrl.n* dim_w);
		ltrl.C.resize( (ltrl.n + 1)* dim_y, (ltrl.n + 1)* dim_x);

		ltrl.Q.resize( (ltrl.n + 1)* dim_y, (ltrl.n + 1)* dim_y);
		ltrl.D.resize(ltrl.n,ltrl.n);
		ltrl.R = ltrl.r * MatrixXd::Identity(ltrl.n,ltrl.n);    
		ltrl.S = ltrl.s *MatrixXd::Identity(ltrl.n,ltrl.n);
		ltrl.Rho = VectorXd :: Ones(ltrl.n * dim_w);
		ltrl.Vref= VectorXd :: Zero(ltrl.n);

		//RTCLib::CSVLoader R_mat("R_mat.csv",0);
		//
		//for(int i=0; i<ltrl.n; i++){
		//	for(int j=0; j<ltrl.n; j++){
		//	ltrl.R(i,j) = R_mat[i][j];
		//	}
		//}

		MatrixXd O;

	///////////////定数パラメータの設定////////////////////////////////
	//D行列拡大系//////////////////
	for(int l=0; l < ltrl.n; l++)
	{
		for (int m = 0; m < ltrl.n; m++){
			if (l == m ) {

				if(l!=0) ltrl.D(l,m) = -1;
				else ltrl.D(l,m) = 0;
			}
			else if(l-1 == m) ltrl.D(l,m) = 1;
			else ltrl.D(l,m) = 0;
		}
	}
		



	////////C行列拡大系///////////
	O = MatrixXd::Zero(dim_y,dim_x);
	for(int l=0; l < (ltrl.n+1)*dim_y; l+=dim_y)
	{
		for (int m = 0; m < (ltrl.n+1)*dim_x; m+=dim_x){
			if (l*dim_x == m*dim_y)
				ltrl.C.block(l,m,dim_y,dim_x) = ltrl.Cd;

			else{
				ltrl.C.block(l,m,dim_y,dim_x) = O;
			}
		}
	}




	//////Q行列拡大系///////////
	O = MatrixXd::Zero(2,2);
	for(int l=0; l < (ltrl.n+1)*2; l+=2)
	{
		for (int m = 0; m < (ltrl.n+1)*2; m+=2){
			if (l == m && m != ltrl.n*2){
				ltrl.Q.block(l,m,2,2) = ltrl.Qd;
			}else if ( l == ltrl.n*2 && m == ltrl.n*2){
				ltrl.Q.block(l,m,2,2) = ltrl.Sf;
			}else{
				ltrl.Q.block(l,m,2,2) = O;

			}
		}
	}

	//Qが時変のとき
	//O = MatrixXd::Zero(2,2);
	//for(int l=0; l < (ltrl.n+1)*2; l+=2)
	//{
	//	for (int m = 0; m < (ltrl.n+1)*2; m+=2){
	//		if (l == m && m != ltrl.n*2){
	//			ltrl.Qd << prm[l/2][prm.GetColOf("Q_y_lat")] , 0, 0, prm[l/2][prm.GetColOf("Q_theta_lat")];
	//			ltrl.Q.block(l,m,2,2) =ltrl.Qd;
	//		}else if ( l == ltrl.n*2 && m == ltrl.n*2){
	//			ltrl.Q.block(l,m,2,2) = ltrl.Sf;
	//		}else{
	//			ltrl.Q.block(l,m,2,2) = O;
	//		}
	//	}
	//}


	//ltrl.R(ltrl.n,ltrl.n)=Rf;
	//ltrl.S(ltrl.n,ltrl.n)=Sf;


		return ltrl;
	}



};
