//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  lagr_calc.h
//
//  Code generation for function 'lagr_calc'
//


#ifndef LAGR_CALC_H
#define LAGR_CALC_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
void lagr_calc(const double X[6010], double xs[10], const double con[14424],
               const double lambda[14424], const double mu_diag[14424], double
               J_objs[601], double J_cons[601], double J_tots[601], double
               *J_obj, double *J_con, double *J_tot);

#endif

// End of code generation (lagr_calc.h)
