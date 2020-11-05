/* VS API functions in a VS DLL. This version has prototypes for all of the functions
   available in a VS solver for CarSim or TruckSim.

  Log:
  May 20, 09. M. Sayers. updated to include vs_run and new functions for CarSim 8.0.
  Jun 06, 08. M. Sayers. created, based on previous version that used a structure.
  */ 
// load DLL and get API functions
int vs_get_dll_path(char *simfile, char *pathDLL);
int vs_get_api_basic (HMODULE dll, char *dll_fname);
int vs_get_api_extend (HMODULE dll, char *dll_fname);
int vs_get_api_road (HMODULE dll, char *dll_fname);
int vs_get_api_install_external (HMODULE dll, char *dll_fname);

  
// Core functions (from vs_solver_api.c)
int     (*vs_bar_graph_update) (int *);
void    (*vs_copy_io) (vs_real *imports, vs_real *exports);
vs_bool (*vs_error_occurred) (void);
void    (*vs_free_all) (void);
char    *(*vs_get_error_message) (void);
char    *(*vs_get_output_message) (void);
vs_real (*vs_get_tstep) (void);
char    *(*vs_get_version_vs) (void);
char    *(*vs_get_version_product) (void);
void    (*vs_initialize) (vs_real t, 
                          void (*ext_calc) (vs_real, vs_ext_loc),
                          void (*ext_echo) (vs_ext_loc));
int    (*vs_integrate) (vs_real *t, 
                        void (*ext_eq_in) (vs_real, vs_ext_loc));
int    (*vs_integrate_io) (vs_real t, vs_real *imports, vs_real *exports);
vs_bool (*vs_opt_pause)(void);
void   (*vs_read_configuration) (const char *simfile, int *n_import,
                                 int *n_export, vs_real *tstart, vs_real *tstop,
                                 vs_real *tstep);
vs_real (*vs_setdef_and_read) (const char *simfile, void (*ext_setdef) (void),
                              int (*ext_scan) (char *, char *));
int    (*vs_stop_run) (void);
void   (*vs_terminate) (vs_real t, void (*ext_echo) (vs_ext_loc));
void   (*vs_terminate_run) (vs_real t);
int    (*vs_run) (char *simfile);

// VS API functions for extending models & adding equations
int     (*vs_define_import) (char *keyword, char *desc, vs_real *real, char *);
int     (*vs_define_output) (char *shortname, char *longname, vs_real *real, char *);
int     (*vs_define_parameter) (char *keyword, char *desc, vs_real *, char *);
void    (*vs_define_units) (char *desc, vs_real gain);
int     (*vs_define_variable) (char *keyword, char *desc, vs_real *);    
vs_real *(*vs_get_var_ptr) (char *keyword);                                        
int     *(*vs_get_var_ptr_int) (char *keyword);
void    (*vs_set_units) (char *var_keyword, char *units_keyword);
void    (*vs_install_calc_func) (char *name, void *func);
void    (*vs_printf) (const char *format, ...);
void    (*vs_printf_error) (const char *format, ...);
int     (*vs_set_sym_int) (int id, vs_sym_attr_type dataType, int value);
int     (*vs_set_sym_real) (int id, vs_sym_attr_type dataType, vs_real value);
int     (*vs_set_sym_attribute) (int id, vs_sym_attr_type type, const void *att);
void    (*vs_read_next_line) (char *buffer, int n);
void    (*vs_write_to_echo_file) (char *buffer);
void    (*vs_write_header_to_echo_file) (char *buffer);
void    (*vs_write_f_to_echo_file) (char *key, vs_real , char *doc);
void    (*vs_write_i_to_echo_file) (char *key, int , char *doc);
int     (*vs_get_sym_attribute) (int id, vs_sym_attr_type type, void **att);
int     (*vs_define_parameter_int) (char *keyword, char *desc, int *);

// functions for accessing the 3D road model in CarSim 7.01 and TruckSim 7.0
void    (*vs_get_dzds_dzdl) (vs_real s, vs_real l, vs_real *dzds, vs_real *dzdl);
void    (*vs_get_road_contact) (vs_real y, vs_real x, int inst, vs_real *z,
                                vs_real *dzdy, vs_real *dzdx, vs_real *mu);
void    (*vs_get_road_start_stop) (vs_real *start, vs_real *stop);
void    (*vs_get_road_xyz) (vs_real s, vs_real l, vs_real *x, vs_real *y, 
                            vs_real *z);
vs_real (*vs_road_l) (vs_real x, vs_real y);
vs_real (*vs_road_s) (vs_real x, vs_real y);
vs_real (*vs_road_x) (vs_real s);
vs_real (*vs_road_y) (vs_real s);
vs_real (*vs_road_z) (vs_real x, vs_real y);
vs_real (*vs_road_yaw) (vs_real sta, vs_real direction);
vs_real (*vs_s_loop) (vs_real s);
vs_real (*vs_target_l) (vs_real s);

// functions for accessing the 3D road model starting with CarSim 7.1
void    (*vs_get_dzds_dzdl_i) (vs_real s, vs_real l, vs_real *dzds,
                                       vs_real *dzdl, vs_real inst);
void    (*vs_get_road_contact_sl) (vs_real s, vs_real l, int inst, vs_real *z,
                             vs_real *dzds, vs_real *dzdl, vs_real *mu);
vs_real (*vs_road_curv_i) (vs_real s, vs_real inst);
vs_real (*vs_road_l_i) (vs_real x, vs_real y, vs_real inst);
vs_real (*vs_road_pitch_sl_i) (vs_real s, vs_real l, vs_real yaw, vs_real inst);
vs_real (*vs_road_roll_sl_i) (vs_real s, vs_real l, vs_real yaw, vs_real inst);
vs_real (*vs_road_s_i) (vs_real x, vs_real y, vs_real inst);
vs_real (*vs_road_x_i) (vs_real sy, vs_real inst);
vs_real (*vs_road_y_i) (vs_real sy, vs_real inst);
vs_real (*vs_road_yaw_i) (vs_real sta, vs_real directiony, vs_real inst);
vs_real (*vs_road_z_i) (vs_real x, vs_real yy, vs_real inst);
vs_real (*vs_road_z_sl_i) (vs_real s, vs_real l, vs_real inst);

// tire function for model used in CarSim and TruckSim
void    (*cs_tire) (vs_real *fx, vs_real *fy, vs_real *mx, vs_real *my, vs_real *mz,
         vs_real *ds_lag_long, vs_real *ds_lag_lat,
         vs_real fz, vs_real *s_lag_long, vs_real *s_lag_lat, vs_real gamma,
         vs_real mux, vs_real muy, vs_real mu_ref_x, vs_real mu_ref_y,
         vs_real vxwc, vs_real vyctc, vs_real *rre, vs_real w, 
         vs_real lrelax_long, vs_real lrelax_lat, vs_real vlow_alpha, 
         vs_real tstep, int opt_no_lag, vs_real rr_rsc, vs_real rr_0, vs_real rr_v,
         vs_real cro_fy, vs_real cro_mz, vs_real cro_mx,
         int itab, int inst, int opt_model); 

// installation functions tested Nov 30, 2008
void (*vs_install_calc_function) (void (*calc) (vs_real time, vs_ext_loc loc));
void (*vs_install_echo_function) (void (*echo) (vs_ext_loc loc));
void (*vs_install_setdef_function) (void (*setdef) (void));
//void (*vs_install_scan_function) (void (*scan) (char *, char *));
// modified 2013, Jun. 20 by Okuda, 
void (*vs_install_scan_function) (vs_bool (*scan) (char *, char *));
void (*vs_install_free_function) (void (*free) (void));
