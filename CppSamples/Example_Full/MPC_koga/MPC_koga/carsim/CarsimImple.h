#pragma once

#pragma warning (disable : 4996)

#include "CarsimBase.h"
#include <map>

/// Here carsim information from carsim
/// 
/// 
class InfoFromCarsim
{
public:
	double XCG_TM;	// X in Glocal
	double YCG_TM;	// Y in Flobal
	double YAW;		// Yaw angle

	double VX;		// x speed
	double VY;		// y speed
	double AVZ;		// yaw rate

	double BETA;	// slip angle

	double TSTART;	// what for??
	double PBK_CON;	// brake pressure
	double MFUEL;	// consumed fuel mass
	double AV_ENG;	// engine rotation

	double GEARAUTO;// gear number in automatic shift control
};

/// Here carsim information to carsim
///
class InfoToCarsim
{
public:
	double IMP_STEER_SW;		// steering input
	double IMP_THROTTLE_ENGINE;	// throttele input
	double IMP_FBK_PDL;			// brake input
};


class CarsimImple : public CarsimBase
{
public:
	CarsimImple(std::string simfile_name);
	virtual ~CarsimImple(void);

	// override how to run
	int run_all();
	int run_integrate();

	// number of integrate in one control frame 
	int integrate_count;

	// register_static_callbacks
	void register_callbacks();

	// needs instance pointer static 
	static CarsimImple* instance;	// please set instance

	bool isValid;

	// use map instead
	//// define some static variables used for the controller
	//vs_real *sXcg, *sYcg, *sYaw, // pointers to vehicle  X, Y, and yaw
	//	*Vx, *Vy, *Vyaw, // x, y speeds and yaw rate
	//	*sTstart,	// pointer to start time
	//	*sImpStr,	// pointer to imported steering wheel angle 
	//	*sBrake,	// for braking
	//	*sBrakeOut, // oil pressure to brake
	//	*sThrottle,	// for throttle
	//	*sFuel,	// pointer to fuel; okuda
	//	*sEngRpm, // pointer to engine crank shaft rotation
	//	*sGiearAuto; // gear number of automatic transmittion
	////---------------------------------------------------------------

	std::map<std::string, vs_real*> input_variable_map;
	std::map<std::string, vs_real*> output_variable_map;

	//--------------------------------------------------------------- 
	//  set the variable pointers
	//---------------------------------------------------------------
	void set_def()
	{
		// use map instead
		// get pointers to vehicle X, Y, yaw, and imported steering wheel angle
		// that will be used in calculation made during the run
		/*sXcg      = vs_get_var_ptr("XCG_TM");
		sYcg      = vs_get_var_ptr("YCG_TM");
		sYaw      = vs_get_var_ptr("YAW");

		Vx      = vs_get_var_ptr("VX");
		Vy      = vs_get_var_ptr("VY");
		Vyaw      = vs_get_var_ptr("AVZ");

		sImpStr   = vs_get_var_ptr("IMP_STEER_SW");
		sThrottle = vs_get_var_ptr("IMP_THROTTLE_ENGINE");
		sTstart   = vs_get_var_ptr("TSTART");
		sBrake    = vs_get_var_ptr("IMP_FBK_PDL");
		sBrakeOut = vs_get_var_ptr("PBK_CON");
		sFuel     = vs_get_var_ptr("MFUEL");
		sEngRpm   = vs_get_var_ptr("AV_ENG");

		sGiearAuto = vs_get_var_ptr("GEARAUTO");*/

		// setting variables by map 
		set_def_by_map();
	}

	void set_def_by_map()
	{
		//input variable mapping
		for( auto itr = input_variable_map.begin(); 
			itr != input_variable_map.end();
			itr++)
		{
			std::string tmp = itr->first;
			char buf[256];
			strcpy(buf, tmp.c_str());
			vs_real* t = vs_get_var_ptr( const_cast<char*>(buf ) );
			itr->second = t;
		}
		//output variable mapping
		for( auto itr = output_variable_map.begin(); 
			itr != output_variable_map.end();
			itr++)
		{
			itr->second = vs_get_var_ptr( const_cast<char*>(itr->first.c_str()) );
		}

	}

	/* ----------------------------------------------------------------------------
	Set up variables for the model extension. For the steering controller,
	define new units and parameters and set default values of the parameters
	that will be used if nothing is specified at run time.
	---------------------------------------------------------------------------- */
	static void external_setdef (void)
	{
		CarsimImple* trg = CarsimImple::instance;
		// call member function
		trg->set_def();
		
	}

	// not in use now
	void calc(vs_real t, vs_ext_loc loc)
	{
		//double f = 0, x, y, str, thr, brk;

		switch (loc)
		{
			// initialization after reading parsfile but before init
		case VS_EXT_EQ_PRE_INIT:
			break;

		case VS_EXT_EQ_INIT: // initialization after reading parsfile
			break;

		case VS_EXT_EQ_INIT2: // initialization after calculating outputs
			break;

		case VS_EXT_EQ_IN: // calculations at the start of a time step
			//if (!sUseExternal) ; // no effect if sUseExternal is FALSE
			//else if (t <= *sTstart) *sImpStr = 0.0; // no steering at the start
			//else // steer proportional to the lateral error
			//*sImpStr = 0.0;//sGainStr*(sLatTrack - vs_road_l(sXprev, sYprev));
			//*sThrottle = 0.0;
			//*sBrake = 0.1*1000000.0;
			break;

		case VS_EXT_EQ_OUT: // calculate output variables at the end of a time step
			// calculate X and Y coordinates of preview point
			//trg->sXprev = *trg->sXcg + trg->sLfwd*cos(*trg->sYaw);
			//trg->sYprev = *trg->sYcg + trg->sLfwd*sin(*trg->sYaw);
			//f = (double)(*sFuel);
			//x = (double)(*sXcg);
			//y = (double)(*sYcg);

			//str = (double)*sImpStr;//sGainStr*(sLatTrack - vs_road_l(sXprev, sYprev));
			//thr = (double)*sThrottle;
			//brk = (double)*sBrakeOut;

			//// debug printing
			//{
			//	static int i= 0;
			//	if( (i++)%1000==0)printf("t,x,y,fuel = %.2f ,%.2f ,%.2f ,%.2fcc, %.2f, %.2f, %.2f\n",t,x, y, f*1000.0, str, thr, brk);
			//}
			break;

		case VS_EXT_EQ_END: // calculations done at end of run
			break;
		}

	}

	/* ----------------------------------------------------------------------------
	Perform calculations involving the model extensions. The function is called
	from four places as indicated with the argument where, which can have the 
	values: VS_EXT_EQ_PRE_INIT, VS_EXT_EQ_INIT, VS_EXT_EQ_IN, VS_EXT_EQ_OUT, 
	and VS_EXT_EQ_END (defined in vs_deftypes.h).
	---------------------------------------------------------------------------- */
	static void  external_calc (vs_real t, vs_ext_loc loc)
	{
		//CarsimImple* trg = CarsimImple::instance;
		//trg->calc(t, loc);
	}


	/* ----------------------------------------------------------------------------
	Write information into the current output echo file using the VS API 
	function vs_write_to_echo_file. This function is called four times when
	generating the echo file as indicated with the argument where, which can
	have the values: VS_EXT_ECHO_TOP, VS_EXT_ECHO_SYPARS, VS_EXT_ECHO_PARS, and
	VS_EXT_ECHO_END (defined in vs_deftypes.h).
	---------------------------------------------------------------------------- */
	static void external_echo (vs_ext_loc loc)
	{
		//CarsimImple* trg = CarsimImple::instance;

		static char buffer[200];

		switch (loc)
		{
		case VS_EXT_ECHO_TOP: // top of echo file
			//trg->vs_write_to_echo_file (
			//	"! This model was extended with custom C code to provide a point-follower\n"
			//	"! steer controller.\n\n");
			break;

		case VS_EXT_ECHO_SYPARS: // end of system parameter section
			//trg->vs_write_i_to_echo_file ("OPT_USE_EXT_STEER", trg->sUseExternal,
			//	"! Use steering controller defined with external C code");
			break;

		case VS_EXT_ECHO_PARS: // end of model parameter section
			break;

		case VS_EXT_ECHO_END: // end of echo file
			break;
		}
	}


	/* ----------------------------------------------------------------------------
	Scan a line read from the current input parsfile. Return TRUE if the keyword
	is recognized, FALSE if not.

	keyword -> string with current ALL CAPS keyword to be tested
	buffer  -> string with rest of line from parsfile.
	---------------------------------------------------------------------------- */
	static vs_bool external_scan (char *keyword, char *buffer)
	{
		// CarsimNenpi* trg = CarsimNenpi::instance;

		return FALSE;
	}








};

