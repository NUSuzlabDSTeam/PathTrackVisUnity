
#include <Windows.h>
#include "CarsimImple.h"

CarsimImple* CarsimImple::instance = NULL;

CarsimImple::CarsimImple( std::string simfile_name)
	: CarsimBase(simfile_name), isValid(false) // init base class with a directed simfile
{
	// this class is singleton...  multiple instances are not allowed
	if(CarsimImple::instance != NULL)
	{
		throw( std::exception("Multiple creation of CarsimNenpi is not allowed.") );
	}

	// set instance
	CarsimImple::instance = this;

	// register static callbacks
	try{
		register_callbacks();
		isValid = true;
	}catch(...){
		isValid = false;
	}
}


CarsimImple::~CarsimImple(void)
{
	instance = NULL;
}

// register_static_callbacks
void CarsimImple::register_callbacks()
{
	if( CarsimBase::vsDLL != NULL)
	{
	CarsimBase::register_callbacks( 
		&CarsimImple::external_calc, 
		&CarsimImple::external_echo, 
		&CarsimImple::external_setdef, 
		&CarsimImple::external_scan
	);
	}
}

int CarsimImple::run_integrate()
{
	for(int i=0;i<integrate_count;++i)
	{
		vs_integrate (&carsim_t, NULL);
		if (vs_error_occurred ()) return 1;
	}
	//vs_bar_graph_update (&ibarg); // update bar graph?
	//vss_print_message (printMsg);
	return 0;
}

int CarsimImple::run_all()
{
	// init carsim
	run_init_step();

	// Run. Each loop advances time one full step.
	while (is_continuing() )  // is_continuing returns false if the simulation finished
	{
		// state integration
		run_integrate();
	}

	// termination
	run_terminate();

	return 0;
}
