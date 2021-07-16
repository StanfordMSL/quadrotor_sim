//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  forward_pass.h
//
//  Code generation for function 'forward_pass'
//


#ifndef FORWARD_PASS_H
#define FORWARD_PASS_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
void forward_pass(double X[6010], double U[2400], const double l[2400], const
                  double L[24000], double La_p_tot, const double lambda[14424],
                  const double mu[14424], const double xs[10], double *La_c_con,
                  double *La_c_tot, double con[14424], double con_x[144240],
                  double con_u[57600]);

#endif

// End of code generation (forward_pass.h)
