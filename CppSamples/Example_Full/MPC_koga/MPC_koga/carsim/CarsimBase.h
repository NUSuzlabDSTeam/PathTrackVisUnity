/*!
 * CarsimBase class 
 * 
 * Copyright (c) 2013 by Hiroyuki Okuda, Suzlab. Nagoya Univ.
 */

///
/// CarsimBaseの使い方 How to use CarsimBase
///
/// 1. CarsimBaseを継承します。
/// class YourClass : public CarsimBase{}
///
/// 2. YourClass のコンストラクタでは
/// CarsimBaseのコンストラクタにsimfile名を渡します。
/// // constructor of your class
/// YourClass(string simfile_name) : CarsimBase(simfile_name) {
///
/// 3. Callback functionsを作ります。
/// 通常YourClassのstaticとしておくと便利です。
/// 
/// static void calc_callback(vs_real t, vs_ext_loc loc){...}
/// static vs_bool scan_callback(char *keyword, char *buffer){...}
/// static void setdef_callback(void){...}
/// static void echo_callback(vs_ext_loc loc){...}
///
/// (Tips) - これらの関数群内部では，メンバ関数コールの形にすると便利です．
/// example:
/// YourClass...
/// { ... 
///		static YourClass* instance;
///		void calc(vs_real, vs_ext_loc);
///	... }
/// static void calc_callback(vs_real t, vs_ext_loc loc){ 
///    YourClass::instance->calc(t, loc);
/// }
///
/// 4. Callback をregisterします．
///   A. 個別に登録する
///    	vs_install_calc_function (&calc_callback);
///		vs_install_echo_function (&echo_callback);
///		vs_install_scan_function (&scan_callback);
///		vs_install_setdef_function (&setdef_callback); 
///
///   B. CarsimBaseのヘルパ関数を使う
/// 	CarsimBase::register_callbacks( 
///     	&YourClass::calc_callback, 
///     	&YourClass::echo_callback, 
///     	&YourClass::setdef_callback, 
///     	&YourClass::scan_callback
///     );
/// 
/// 5. Callbackの中，あるいは外で，適切な処理を書きます．
/// 
/// * please reffer to sample program : CarsimNenpi.h and CarsimNenpi.cpp


#pragma once

#include <Windows.h>
#include <string>

#include "vs_deftypes.h" // VS types and definitions

class CarsimBase
{
public:
	CarsimBase(std::string fn);
	virtual ~CarsimBase(void);

	// open dll and load function pointers
	void open_dll(std::string simfile_name);

	// free dlls
	void free_dll();

	// run simulation all W/O stop
	virtual int run_all();

	virtual int run_init_step();
	virtual int run_integrate();
	virtual int run_terminate();

	virtual bool is_continuing();

	// typedef of function callback 
	typedef void (*calc_func_type) (vs_real time, vs_ext_loc loc) ;
	typedef void (*echo_func_type) (vs_ext_loc where);
	typedef void (*setdef_func_type) (void);
	typedef vs_bool (*scan_func_type) (char *, char *);
	typedef void (*free_func_type) (void);

	// register_static_callbacks
	virtual void register_callbacks(
		calc_func_type _calc, 
		echo_func_type _echo, 
		setdef_func_type _setdef, 
		scan_func_type _scan_
		);

protected:
	HMODULE vsDLL; // DLL with VS API
public:
	std::string dll_path;
	std::string simfile;

	// simulation time in carsim
	double carsim_t;

#include "vs_api.hpp"  // VS API functions
	// utility function from vs_get_api.c
	int vss_dll_problem (char *dll, char *func, int code);
	
};

