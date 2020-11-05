
/*!
 * CarsimBase class 
 * 
 * Copyright (c) 2013 by Hiroyuki Okuda, Suzlab. Nagoya Univ.
 */
#include <Windows.h>
#include "CarsimBase.h"

#pragma warning ( push )
#pragma warning ( disable : 4996 )

CarsimBase::CarsimBase(std::string fn)
	:vsDLL(NULL), carsim_t(0)
{
	open_dll(fn);
}


CarsimBase::~CarsimBase(void)
{
	free_dll();
}

void CarsimBase::free_dll(void)
{
	FreeLibrary (vsDLL);
}

void CarsimBase::open_dll(std::string simfile_name)
{
	simfile = simfile_name;
	char  pathDLL[FILENAME_MAX];

	// load dll	
	if (vs_get_dll_path( (char*)simfile.c_str(), pathDLL)) return;

	dll_path = pathDLL;
	vsDLL = LoadLibraryA(pathDLL);

	// get standard sets of API functions 
	if (vs_get_api_basic(vsDLL, pathDLL)) return; // basic functions
	if (vs_get_api_install_external(vsDLL, pathDLL)) return; // install functions
	if (vs_get_api_extend(vsDLL, pathDLL)) return; // model extension functions
	if (vs_get_api_road(vsDLL, pathDLL)) return; // road-related functions


}

int CarsimBase::run_all()
{
	run_init_step();

	// Run. Each loop advances time one full step.
	while (is_continuing())
	{
		run_integrate();
	}

	return 0;
}

bool CarsimBase::is_continuing()
{
	return !vs_stop_run();
}


int CarsimBase::run_init_step()
{
	char *printMsg = vs_get_output_message(); // pointer to text from DLL
	// Initialize VS model
	carsim_t = vs_setdef_and_read(simfile.c_str(), NULL, NULL);
	if (vs_error_occurred ()) return 1;
	vs_initialize (carsim_t, NULL, NULL);
	//vss_print_message (printMsg);
	if (vs_error_occurred ()) return 1;
	return 0;
}

int CarsimBase::run_integrate()
{
	vs_integrate (&carsim_t, NULL);
	if (vs_error_occurred ()) return 1;
	//vs_bar_graph_update (&ibarg); // update bar graph?
	//vss_print_message (printMsg);
	return 0;
}

int CarsimBase::run_terminate()
{
	// Terminate
	vs_terminate (carsim_t, NULL);
	//vss_print_message (printMsg);
	vs_free_all ();
	return 0;
}


// register_static_callbacks
void CarsimBase::register_callbacks(
	calc_func_type _calc, 
	echo_func_type _echo, 
	setdef_func_type _setdef, 
	scan_func_type _scan
	)
{
	// install external C functions
	vs_install_calc_function (_calc);
	vs_install_echo_function (_echo);
	vs_install_scan_function (_scan);
	vs_install_setdef_function (_setdef);
}

//////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////


/* ----------------------------------------------------------------------------
Get solver path according keyword DLLFILE in the simfile. Return 0 if OK,
-1 if not.
---------------------------------------------------------------------------- */
int CarsimBase::vs_get_dll_path(char *simfile, char *pathDLL)
{
	FILE *fp;
	char tmpstr[FILENAME_MAX*2], *key, *rest;

	if ((fp = fopen(simfile, "r")) == NULL)
	{
		char simfile2[FILENAME_MAX * 2 + 20];
		sprintf(simfile2, "%s.sim", simfile);
		if ((fp = fopen(simfile2, "r")) == NULL)
		{
			sprintf(tmpstr,
				"\nThis program needs a simfile to obtain other file names. The file\n"
				"\"%s\" or \"%s\" either does not exist or could not be opened.", simfile, simfile2);
			MessageBoxA(NULL, tmpstr, "Error", MB_ICONERROR);
			return -1;
		}
	}
	while (fgets(tmpstr, 255, fp))
	{
		key = (char *)strtok(tmpstr, " \t\n");
		rest = (char *)strtok(NULL, "\n\t");
		if (!strcmp(key, "DLLFILE") && rest && rest[0])
		{
			strcpy (pathDLL, rest);
			fclose(fp);

			// Now see if the DLL exists
			if ((fp = fopen(pathDLL, "rb")) == NULL)
			{
				sprintf (tmpstr, 
					"\nThe simfile identified the DLL file \"%s\"\n"
					"with the keyword DLLFILE. This DLL file either does not exist or "
					"cannot be opened.", pathDLL);
				MessageBoxA (NULL, tmpstr, "Error", MB_ICONERROR);
				return -1;
			}
			fclose(fp);
			return 0;
		}
		else if (!strcmp(key, "END")) break;
	}

	fclose(fp);
	sprintf (tmpstr, "\nThis program needs a DLL to run, identified with the\n"
		"keyword DLLFILE. The simfile \"%s\" did\n"
		"not identify a DLL file.", simfile);
	MessageBoxA (NULL, tmpstr, "Error", MB_ICONERROR);
	return -1;
}

// Utility for error handling in getting DLL functions
int CarsimBase::vss_dll_problem (char *dll, char *func, int code)
{
	char tmpstr[FILENAME_MAX*2];
	if (code == -1)
		sprintf (tmpstr,  "The function %s was not given a valid DLL.\n"
		"\"%s\" did not load.", func, dll);
	else 
		sprintf (tmpstr,  "The function %s could not get the required VS API functions\n"
		"from DLL \"%s\".", func, dll);
	MessageBoxA (NULL, tmpstr, "Error", MB_ICONERROR);
	return code;
}

/* ---------------------------------------------------------------------------
Cast utility for c++
--------------------------------------------------------------------------- */
template< class T>
void Cast_By_Decltype(HMODULE dll, char* name, T& var)
{
	var = static_cast<T>( (void *)GetProcAddress(dll, name) );
}

#define ASIGN_VS_FUNCTION( x ) Cast_By_Decltype(dll, #x , x );

/* ----------------------------------------------------------------------------
Get basic VS API functions needed to run a simulation from a VS DLL.
---------------------------------------------------------------------------- */
int CarsimBase::vs_get_api_basic (HMODULE dll, char *dll_fname)
{
	if (dll == NULL) 
		return vss_dll_problem(dll_fname, "vs_get_api_basic", -1);

	// Get addresses for basic VS API functions
	// followings are modified by okuda... @ 2013 / 06 / 11

	//vs_bar_graph_update = (void *)GetProcAddress(dll, "vs_bar_graph_update");
	//vs_copy_io = (void *)GetProcAddress(dll, "vs_copy_io");
	//vs_error_occurred = (void *)GetProcAddress(dll, "vs_error_occurred");
	//vs_free_all = (void *)GetProcAddress(dll, "vs_free_all");
	//vs_get_error_message = (void *)GetProcAddress(dll, "vs_get_error_message");
	//vs_get_output_message = (void *)GetProcAddress(dll, "vs_get_output_message");
	//vs_get_tstep = (void *)GetProcAddress(dll, "vs_get_tstep");
	//vs_get_version_product = (void *)GetProcAddress(dll, "vs_get_version_product");
	//vs_get_version_vs = (void *)GetProcAddress(dll, "vs_get_version_vs");
	//vs_initialize = (void *)GetProcAddress(dll, "vs_initialize");
	//vs_integrate = (void *)GetProcAddress(dll, "vs_integrate");
	//vs_integrate_io = (void *)GetProcAddress(dll, "vs_integrate_io");
	//vs_opt_pause = (void *)GetProcAddress(dll, "vs_opt_pause");
	//vs_read_configuration = (void *)GetProcAddress(dll, "vs_read_configuration");
	//vs_setdef_and_read = (void *)GetProcAddress(dll, "vs_setdef_and_read");
	//vs_stop_run = (void *)GetProcAddress(dll, "vs_stop_run");
	//vs_terminate = (void *)GetProcAddress(dll, "vs_terminate");

	ASIGN_VS_FUNCTION( vs_bar_graph_update );
	ASIGN_VS_FUNCTION( vs_copy_io );
	ASIGN_VS_FUNCTION( vs_error_occurred );
	ASIGN_VS_FUNCTION( vs_free_all );
	ASIGN_VS_FUNCTION( vs_get_error_message );
	ASIGN_VS_FUNCTION( vs_get_output_message );
	ASIGN_VS_FUNCTION( vs_get_tstep );
	ASIGN_VS_FUNCTION( vs_get_version_product );
	ASIGN_VS_FUNCTION( vs_get_version_vs );
	ASIGN_VS_FUNCTION( vs_initialize );
	ASIGN_VS_FUNCTION( vs_integrate );
	ASIGN_VS_FUNCTION( vs_integrate_io );
	ASIGN_VS_FUNCTION( vs_opt_pause );
	ASIGN_VS_FUNCTION( vs_read_configuration );
	ASIGN_VS_FUNCTION( vs_setdef_and_read );
	ASIGN_VS_FUNCTION( vs_stop_run );
	ASIGN_VS_FUNCTION( vs_terminate );

	// Check to see if all functions were found
	if (!vs_bar_graph_update || !vs_copy_io || !vs_error_occurred ||
		!vs_free_all || !vs_get_error_message || !vs_get_output_message ||
		!vs_get_tstep || !vs_get_version_vs || !vs_get_version_product ||
		!vs_initialize || !vs_integrate || !vs_integrate_io || !vs_opt_pause ||
		!vs_read_configuration || !vs_setdef_and_read ||!vs_stop_run ||
		!vs_terminate)
	{
		return vss_dll_problem(dll_fname, "vs_get_api_basic", -2);
	}


	return 0;
}

/* ----------------------------------------------------------------------------
Get basic VS API functions needed to extend a model.
---------------------------------------------------------------------------- */
int CarsimBase::vs_get_api_extend (HMODULE dll, char *dll_fname)
{
	if (dll == NULL) 
		return vss_dll_problem(dll_fname, "vs_get_api_extend", -1);

	// Get addresses for VS API functions used to extend a model
	// followings are modified by okuda... @ 2013 / 06 / 11

	//vs_define_import = (void *)GetProcAddress(dll, "vs_define_import");
	//vs_define_output = (void *)GetProcAddress(dll, "vs_define_output");
	//vs_define_parameter = (void *)GetProcAddress(dll, "vs_define_parameter");
	//vs_define_units = (void *)GetProcAddress(dll, "vs_define_units");
	//vs_define_variable = (void *)GetProcAddress(dll, "vs_define_variable");
	//vs_get_var_ptr = (void *)GetProcAddress(dll, "vs_get_var_ptr");
	//vs_get_var_ptr_int = (void *)GetProcAddress(dll, "vs_get_var_ptr_int");
	//vs_set_units = (void *)GetProcAddress(dll, "vs_set_units");
	//vs_install_calc_func = (void *)GetProcAddress(dll, "vs_install_calc_func");
	//vs_printf = (void *)GetProcAddress(dll, "vs_printf");
	//vs_printf_error = (void *)GetProcAddress(dll, "vs_printf_error");
	//vs_set_sym_int = (void *)GetProcAddress(dll, "vs_set_sym_int");
	//vs_set_sym_real = (void *)GetProcAddress(dll, "vs_set_sym_real");
	//vs_set_sym_attribute = (void *)GetProcAddress(dll, "vs_set_sym_attribute");
	//vs_read_next_line = (void *)GetProcAddress(dll, "vs_read_next_line");
	//vs_write_to_echo_file = (void *)GetProcAddress(dll, "vs_write_to_echo_file");
	//vs_write_header_to_echo_file = (void *)GetProcAddress(dll, "vs_write_header_to_echo_file");
	//vs_write_f_to_echo_file = (void *)GetProcAddress(dll, "vs_write_f_to_echo_file");
	//vs_write_i_to_echo_file = (void *)GetProcAddress(dll, "vs_write_i_to_echo_file");
	//vs_get_sym_attribute = (void *)GetProcAddress(dll, "vs_get_sym_attribute");
	//vs_define_parameter_int = (void *)GetProcAddress(dll, "vs_define_parameter_int");

	
	ASIGN_VS_FUNCTION( vs_define_import );
	ASIGN_VS_FUNCTION( vs_define_output );
	ASIGN_VS_FUNCTION( vs_define_parameter  );
	ASIGN_VS_FUNCTION( vs_define_units  );
	ASIGN_VS_FUNCTION( vs_define_variable  );
	ASIGN_VS_FUNCTION( vs_get_var_ptr  );
	ASIGN_VS_FUNCTION( vs_get_var_ptr_int  );
	ASIGN_VS_FUNCTION( vs_set_units  );
	ASIGN_VS_FUNCTION( vs_install_calc_func  );
	ASIGN_VS_FUNCTION( vs_printf  );
	ASIGN_VS_FUNCTION( vs_printf_error  );
	ASIGN_VS_FUNCTION( vs_set_sym_int  );
	ASIGN_VS_FUNCTION( vs_set_sym_real  );
	ASIGN_VS_FUNCTION( vs_set_sym_attribute  );
	ASIGN_VS_FUNCTION( vs_read_next_line  );
	ASIGN_VS_FUNCTION( vs_write_to_echo_file  );
	ASIGN_VS_FUNCTION( vs_write_header_to_echo_file  );
	ASIGN_VS_FUNCTION( vs_write_f_to_echo_file  );
	ASIGN_VS_FUNCTION( vs_write_i_to_echo_file  );
	ASIGN_VS_FUNCTION( vs_get_sym_attribute  );
	ASIGN_VS_FUNCTION( vs_define_parameter_int  );

	// Check to see if all functions were found
	if (!vs_define_import || !vs_define_output || !vs_define_parameter ||
		!vs_define_units || !vs_define_variable || !vs_get_var_ptr ||
		!vs_get_var_ptr_int || !vs_set_units || !vs_install_calc_func ||
		!vs_printf || !vs_printf_error || !vs_set_sym_int || !vs_set_sym_real ||
		!vs_set_sym_attribute || !vs_read_next_line || !vs_write_to_echo_file ||
		!vs_write_header_to_echo_file || !vs_write_f_to_echo_file ||
		!vs_write_i_to_echo_file || !vs_get_sym_attribute || !vs_define_parameter_int)
	{
		return vss_dll_problem(dll_fname, "vs_get_api_extend", -2);
	}

	return 0;
}

/* ----------------------------------------------------------------------------
Get VS API functions to access road geometry.
---------------------------------------------------------------------------- */
int CarsimBase::vs_get_api_road (HMODULE dll, char *dll_fname)
{
	if (dll == NULL) 
		return vss_dll_problem(dll_fname, "vs_get_api_road", -1);

	// Get addresses for VS API functions used to extend a model (since June 2007)
	// followings are modified by okuda... @ 2013 / 06 / 11

	//vs_road_s = (void *)GetProcAddress(dll, "vs_road_s");
	//vs_road_l = (void *)GetProcAddress(dll, "vs_road_l");
	//vs_road_x = (void *)GetProcAddress(dll, "vs_road_x");
	//vs_road_y = (void *)GetProcAddress(dll, "vs_road_y");
	//vs_road_z = (void *)GetProcAddress(dll, "vs_road_z");
	//vs_road_yaw = (void *)GetProcAddress(dll, "vs_road_yaw");
	//vs_s_loop = (void *)GetProcAddress(dll, "vs_s_loop");
	//vs_get_dzds_dzdl = (void *)GetProcAddress(dll, "vs_get_dzds_dzdl");
	//vs_get_road_start_stop = (void *)GetProcAddress(dll, "vs_get_road_start_stop");
	//vs_get_road_xyz = (void *)GetProcAddress(dll, "vs_get_road_xyz");
	//vs_get_road_contact = (void *)GetProcAddress(dll, "vs_get_road_contact");
	//vs_target_l = (void *)GetProcAddress(dll, "vs_target_l");

	//// Get addresses for VS API functions used to extend a model (since June 2008)
	//vs_get_dzds_dzdl_i = (void *)GetProcAddress(dll, "vs_get_dzds_dzdl_i");
	//vs_get_road_contact_sl = (void *)GetProcAddress(dll, "vs_get_road_contact_sl");
	//vs_road_curv_i = (void *)GetProcAddress(dll, "vs_road_curv_i");
	//vs_road_l_i = (void *)GetProcAddress(dll, "vs_road_l_i");
	//vs_road_pitch_sl_i = (void *)GetProcAddress(dll, "vs_road_pitch_sl_i");
	//vs_road_roll_sl_i = (void *)GetProcAddress(dll, "vs_road_roll_sl_i");
	//vs_road_s_i = (void *)GetProcAddress(dll, "vs_road_s_i");
	//vs_road_x_i = (void *)GetProcAddress(dll, "vs_road_x_i");
	//vs_road_y_i = (void *)GetProcAddress(dll, "vs_road_y_i");
	//vs_road_yaw_i = (void *)GetProcAddress(dll, "vs_road_yaw_i");
	//vs_road_z_i = (void *)GetProcAddress(dll, "vs_road_z_i");
	//vs_road_z_sl_i = (void *)GetProcAddress(dll, "vs_road_z_sl_i");

	ASIGN_VS_FUNCTION( vs_road_s  );
	ASIGN_VS_FUNCTION( vs_road_l  );
	ASIGN_VS_FUNCTION( vs_road_x  );
	ASIGN_VS_FUNCTION( vs_road_y  );
	ASIGN_VS_FUNCTION( vs_road_z  );
	ASIGN_VS_FUNCTION( vs_road_yaw  );
	ASIGN_VS_FUNCTION( vs_s_loop  );
	ASIGN_VS_FUNCTION( vs_get_dzds_dzdl  );
	ASIGN_VS_FUNCTION( vs_get_road_start_stop  );
	ASIGN_VS_FUNCTION( vs_get_road_xyz  );
	ASIGN_VS_FUNCTION( vs_get_road_contact  );
	ASIGN_VS_FUNCTION( vs_target_l  );

	// Get addresses for VS API functions used to extend a model (since June 2008)
	ASIGN_VS_FUNCTION( vs_get_dzds_dzdl_i  );
	ASIGN_VS_FUNCTION( vs_get_road_contact_sl  );
	ASIGN_VS_FUNCTION( vs_road_curv_i  );
	ASIGN_VS_FUNCTION( vs_road_l_i  );
	ASIGN_VS_FUNCTION( vs_road_pitch_sl_i  );
	ASIGN_VS_FUNCTION( vs_road_roll_sl_i  );
	ASIGN_VS_FUNCTION( vs_road_s_i  );
	ASIGN_VS_FUNCTION( vs_road_x_i  );
	ASIGN_VS_FUNCTION( vs_road_y_i  );
	ASIGN_VS_FUNCTION( vs_road_yaw_i  );
	ASIGN_VS_FUNCTION( vs_road_z_i  );
	ASIGN_VS_FUNCTION( vs_road_z_sl_i  );

	// Check to see if all functions were found
	if (!vs_road_s || !vs_road_l || !vs_road_x || !vs_road_y || !vs_road_z || 
		!vs_road_yaw || !vs_s_loop || !vs_get_dzds_dzdl || !vs_get_road_start_stop || 
		!vs_get_road_xyz || !vs_get_road_contact || !vs_target_l ||
		!vs_get_dzds_dzdl_i || !vs_get_road_contact_sl || !vs_road_curv_i ||
		!vs_road_l_i || !vs_road_pitch_sl_i || !vs_road_roll_sl_i || !vs_road_s_i ||
		!vs_road_x_i || !vs_road_y_i || !vs_road_yaw_i || !vs_road_z_i ||
		!vs_road_z_sl_i
		)
	{
		return vss_dll_problem(dll_fname, "vs_get_api_road", -2);
	}

	return 0;
}

/* ----------------------------------------------------------------------------
Get VS API functions to install externally defined functions
---------------------------------------------------------------------------- */
int CarsimBase::vs_get_api_install_external (HMODULE dll, char *dll_fname)
{
	if (dll == NULL) 
		return vss_dll_problem(dll_fname, "vs_get_api_install_external", -1);

	// Get addresses for VS API functions used to extend a model (since June 2007)
	// followings are modified by okuda... @ 2013 / 06 / 11

	//vs_install_calc_function = (void *)GetProcAddress(dll, "vs_install_calc_function");
	//vs_install_echo_function = (void *)GetProcAddress(dll, "vs_install_echo_function");
	//vs_install_setdef_function = (void *)GetProcAddress(dll, "vs_install_setdef_function");
	//vs_install_scan_function = (void *)GetProcAddress(dll, "vs_install_scan_function");
	//vs_install_free_function = (void *)GetProcAddress(dll, "vs_install_free_function");
	//vs_run = (void *)GetProcAddress(dll, "vs_run");

	
	ASIGN_VS_FUNCTION( vs_install_calc_function  );
	ASIGN_VS_FUNCTION( vs_install_echo_function  );
	ASIGN_VS_FUNCTION( vs_install_setdef_function  );
	ASIGN_VS_FUNCTION( vs_install_scan_function  );
	ASIGN_VS_FUNCTION( vs_install_free_function  );
	ASIGN_VS_FUNCTION( vs_run  );


	// Check to see if all functions were found
	if (!vs_install_calc_function || !vs_install_echo_function || !vs_install_setdef_function || 
		!vs_install_scan_function || !vs_install_free_function || !vs_run )
	{
		return vss_dll_problem(dll_fname, "vs_get_api_install_external", -2);
	}

	return 0;
}


#pragma warning(pop)