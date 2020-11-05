/* Define data types and macros for VehicleSim-generated math models.

   Revision Log
   ------------   
   May 15, 09. M. Sayers. One more enum in type vs_sym_attr_type.
   Jun 10, 08. M. Sayers. VS_API_EXPORT macro.
   Dec 12, 07. M. Sayers. Added #include "vsm_project.h"
   Aug 16, 07. M. Sayers. Moved vs_func to calc.c.
   Jul 13, 07. M. Sayers. Added VS_KEYWORD_CALC.
   May 24, 07. M. Sayers. Added SYS_PAR_KEYWORD_CALC.
   Apr 13, 07. M. Sayers. Added VS_EXT_EQ_INIT2.
   Apr 12, 07. M. Sayers. Added SV_DERIVATIVE and "vs_target.h".
   Mar 22, 07. M. Sayers. Added VS_EXT_EQ_PRE_INIT.
   Mar 12, 07. M. Sayers. Released CarSim 7.0.
   Jan 07, 07. M. Sayers. Removed vs_target.h
   Dec 19, 06. M. Sayers. Added vs_units_status and status slot for vs_units.
   Dec 16, 06. M. Sayers. Added vs_ext_loc & OUTVAR_ECHO_DESC.
   Oct 30, 06. M. Sayers. Major reoranization for Alpha 6.
   Mar 16, 06. M. Sayers. Checked for VehicleSim Alpha 4 compatibility.
   Mar 02, 06. M. Sayers. Merged old 1d and 2d table structures into one.
*/

#ifndef _VS_DEFTYPES_H
  #define _VS_DEFTYPES_H

  #include <stdio.h>
  #include "vs_target.h"
  //#include "vsm_project.h"

  #define TRUE 1
  #define FALSE 0
  #define PI 3.141592653589793
  #define PI_HALF 1.5707963267948966

  // macro for declaring public functions in a DLL
  #if (defined(_WIN32) || defined(_WIN64)) && !defined(_LABVIEW_RT) 
    #define VS_API_EXPORT   __declspec(dllexport)
  #else
    #define VS_API_EXPORT
  #endif

// Floating point and boolean types
  typedef double vs_real;
  typedef int vs_bool;

  // locations used for calling functions external_echo and external_calc
  typedef enum 
    {
    VS_EXT_ECHO_TOP, VS_EXT_ECHO_SYPARS, VS_EXT_ECHO_PARS, VS_EXT_ECHO_END,
    VS_EXT_EQ_INIT, VS_EXT_EQ_IN, VS_EXT_EQ_OUT, VS_EXT_EQ_END, 
    VS_EXT_EQ_PRE_INIT, VS_EXT_EQ_INIT2 // Added for CarSim 7.01
    } vs_ext_loc;

  // Types of data that can be represented with vs_table.
  typedef enum 
    {
    VS_TAB_CONST, VS_TAB_COEF, VS_TAB_LIN, VS_TAB_LIN_LOOP, VS_TAB_LIN_FLAT, 
    VS_TAB_STEP, VS_SPLINE, VS_SPLINE_LOOP, VS_SPLINE_FLAT, VS_TAB_2D, 
    VS_TAB_2D_LOOP, VS_TAB_2D_STEP, VS_TAB_2D_FROM_ZERO, VS_TAB_2D_SPLINE, 
    VS_TAB_2D_VAR_WIDTH_STEP, VS_TAB_2D_VAR_WIDTH
    } vs_table_type;

  // Attributes that can be set for an internal symbolic structure (sym)
  typedef enum 
    {  // start with attributes for outputs & exports
    OUTVAR_SHORT_NAME, OUTVAR_LONG_NAME, OUTVAR_GEN_NAME, OUTVAR_BODY_NAME,
    OUTVAR_REAL, OUTVAR_VALUE, OUTVAR_UNITS, OUTVAR_ECHO_DESC, 
    IMP_UNITS, IMP_KEYWORD, IMP_DESC, IMP_REAL, // attributes for imports
    SV_UNITS, SV_KEYWORD, SV_DESC, SV_VALUE,    // attributes for state variables
    // next are attributes for indexed syms, with up to 5 dimensions
    ISYM_UNITS, ISYM_KEYWORD, ISYM_REAL, ISYM_INTEGER, ISYM_DIMENSIONS,
    ISYM_LIMIT_1, ISYM_LIMIT_2, ISYM_LIMIT_3, ISYM_LIMIT_4, ISYM_LIMIT_5, 
    ISYM_INDEX_1, ISYM_INDEX_2, ISYM_INDEX_3, ISYM_INDEX_4, ISYM_INDEX_5,
    SYS_PAR_UNITS, SYS_PAR_KEYWORD, SYS_PAR_DESC, SYS_PAR_REAL, SYS_PAR_INTEGER,
    SYS_PAR_VALUE, SYS_PAR_VISIBLE, // attributes for system parameters
    PAR_UNITS, PAR_KEYWORD, PAR_DESC, PAR_REAL, PAR_INTEGER, // regular pars
    PAR_VALUE, PAR_VISIBLE,
    ANI_SHORT_NAME, ANI_REAL, ANI_REAL_SCALED, // variables set for live ani
    SV_DERIVATIVE, // state variable derivative, added April 10, 2007
    SYS_PAR_KEYWORD_CALC, // added May 25, 2007. Keyword for calculated par
    IMP_VISIBLE // added May 14, 2009, to allow new imports to be inactive
    } vs_sym_attr_type;

  // Types of objects that can be accessed with a keyword.
  typedef enum 
    {
    VS_KEYWORD_NULL = 100, VS_KEYWORD_SYM, VS_KEYWORD_ISYM, VS_KEYWORD_FUNC0, 
    VS_KEYWORD_FUNC1, VS_KEYWORD_OUTVAR, VS_KEYWORD_WRT, VS_KEYWORD_EXP,
    VS_KEYWORD_ANI, VS_KEYWORD_IMP, VS_KEYWORD_TAB_1D, VS_KEYWORD_TAB_2D,
    VS_KEYWORD_TAB_GAIN, VS_KEYWORD_TAB_OFFSET, VS_KEYWORD_TAB_CONST,
    VS_KEYWORD_TAB_COEF, VS_KEYWORD_TAB_GROUP_1D, VS_KEYWORD_TAB_GROUP_2D,
    VS_KEYWORD_TAB_GROUP_GAIN, VS_KEYWORD_TAB_GROUP_OFFSET,
    VS_KEYWORD_TAB_GROUP_CONST, VS_KEYWORD_TAB_GROUP_COEF, VS_KEYWORD_CALC
    } vs_keyword_type;

  // Status of a vs_units structure
  typedef enum 
    {
    VS_UNITS_NATIVE, VS_UNITS_NEW, VS_UNITS_MODIFIED
    } vs_units_status;
    
  // Structure for units.
  typedef struct
    {
    char *desc, *keyword; // desc is case-sensitive, keyword is all-caps
    vs_real gain;         // gain: multiply to convert from internal to output
    vs_units_status status; // status of the structure
    } vs_units;

  /* Structure for generic table. Value produced using form:
       constant, or
       offset + x*coefficient, or
       offset + gain*F(x_row, itab, inst), or
       offset + gain*F(x_col, y_row, itab, inst).
     */
  //typedef struct vs_tab_group_tag vs_tab_group; // doesn't work with GNU

  typedef struct
    {
    vs_real ****c;       //Coefficients for each grid square for 2d splines
    } vs_2d_spline_coef;


  // Structure for table group.
  typedef struct
    {
    void *tabs;   // parent group: must coerce. Stupid GNU!
    int     visible;      // should table be printed in echo file?
    vs_table_type type;   // used for any table or constant
    vs_real constant, coefficient; // constants used to replace a table
    vs_real *x, *y;       // arrays used for any table
    vs_real *loop;        // range of table, used for looping
    vs_real gain, offset; // used to transform any table
    int     nx, *jx;      // primary size and last entry; used for any table
    vs_real *dydx;        // array used only for 1D table
    vs_real **fxy;        // 2D array used only for 2D tables
    int     ny, *jy, extra;   // 2nd size used only for 2D tables
    char *table_keyword, *carpet_keyword; // Keywords (if not "") for parsfiles
    char *gain_keyword, *offset_keyword, *constant_keyword, *coefficient_keyword;
    //vs_real *y_inverse; // array with reversed Y for inverse table lookup
    int *j_inverse; // instances for inverse tables
    void *lateral_tab;  //only used in variable width tables
                         //table to hold a 2d table with the position of the lanes
                         //(in roads, S vs L for the columns) 
    vs_2d_spline_coef *coef; // structure to hold the coefficients needed for
                             // bicubic interpolation. Used only in 2D_SPLINE
    } vs_table;

  // Structure for table group.
  typedef struct _vs_tab_group_
    {
    vs_table  **table; // array of table pointers
    int   ntab, ninst; // number of tables and instances per table
    vs_table_type type_default; // type of 2D table that is supported
    vs_units *units_row, *units_out; // Used for all tables
    vs_units *units_col;             // used for column variable in 2D tables
    char *table_keyword, *carpet_keyword; // Keywords
    char *gain_keyword, *offset_keyword;
    char *constant_keyword, *coefficient_keyword, *root_keyword;
    char *title;
    char *desc_row; // description of the primary independent variable
    char *desc_out; // description of the dependent variable from the table
    char *desc_col; // description of column variable in 2D tables
    int  (*get_itab) (void); // function to get table id if ntab > 1
    char *(*print_id) (int); // function to print table id info if ntab > 1
    char *(*print_label) (int); // function to print table label if ntab > 1
    int  *id; // pointer to table ID, used IF ntab > 1 and get_itab is NULL
    int inverse; // Does this table group support inverse lookup?
    void (*set_def) (void); // function to set default values
    //int index; // index into array of table groups
    } vs_tab_group;

  // Structure for deferred function call (no argument)
  typedef struct
    {
    char *keyword;     // keyword should be all-caps, no spaces.
    void (*func) (void); // pointer to function name
    } vs_func0;

  // Structure for deferred function call (1 argument)
  typedef struct
    {
    char *keyword;     // keyword should be all-caps, no spaces.
    void (*func) (char *); // pointer to function name
    } vs_func1;
    
#endif  // end block for _VS_DEFTYPES_H
