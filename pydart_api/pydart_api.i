%module pydart_api

%{
  #define SWIG_FILE_WITH_INIT
  #include "pydart_api.h"
%}

/* Include the NumPy typemaps library */
%include "numpy.i"

%init %{
  import_array();
%}

/* Typemap for the sum_list(double* input_array, int length) C/C++ routine */
%apply (double* IN_ARRAY1, int DIM1) {(double* inpose, int ndofs)};
%apply (double* IN_ARRAY1, int DIM1) {(double* inpose2, int ndofs2)};
%apply (double* IN_ARRAY1, int DIM1) {(double* inpose3, int ndofs3)};
%apply (double* IN_ARRAY1, int DIM1) {(double* intorque, int ndofs)};
%apply (double* ARGOUT_ARRAY1, int DIM1) {(double* outpose, int ndofs)};
%apply (double* ARGOUT_ARRAY1, int DIM1) {(double* outpose2, int ndofs2)};
%apply (double* ARGOUT_ARRAY1, int DIM1) {(double* outpose3, int ndofs3)};
%apply (double* ARGOUT_ARRAY1, int DIM1) {(double* outv, int len)};
%apply (double IN_ARRAY1[ANY]) {(double inv3[3])};
%apply (double IN_ARRAY1[ANY]) {(double inv3_2[3])};
%apply (double ARGOUT_ARRAY1[ANY]) {(double outv3[3])};
%apply (double ARGOUT_ARRAY1[ANY]) {(double outv6[6])};
%apply (double* INPLACE_ARRAY2, int DIM1, int DIM2) {(double* array2, int nrows, int ncols)};
%apply (double ARGOUT_ARRAY2[ANY][ANY]) {(double outv33[3][3])};
%apply (double ARGOUT_ARRAY2[ANY][ANY]) {(double outv44[4][4])};


%include "pydart_api.h"
