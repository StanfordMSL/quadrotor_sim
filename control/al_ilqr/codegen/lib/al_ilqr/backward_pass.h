//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  backward_pass.h
//
//  Code generation for function 'backward_pass'
//


#ifndef BACKWARD_PASS_H
#define BACKWARD_PASS_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
void backward_pass(const double X[6010], const double U[2400], const double con
                   [14424], const double con_x[144240], const double con_u[57600],
                   const double lambda[14424], const double mu_diag[14424],
                   double xs[10], double l[2400], double L[24000], double delV
                   [1202]);

#endif

// End of code generation (backward_pass.h)
