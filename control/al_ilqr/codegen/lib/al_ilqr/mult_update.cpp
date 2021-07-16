//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  mult_update.cpp
//
//  Code generation for function 'mult_update'
//


// Include files
#include "mult_update.h"
#include "rt_nonfinite.h"
#include "rt_nonfinite.h"

// Function Definitions
void mult_update(double lambda[14424], double mu[14424], double phi, const
                 double con[14424])
{
  for (int k = 0; k < 601; k++) {
    for (int b_k = 0; b_k < 24; b_k++) {
      double d;
      double d1;
      int i;
      i = b_k + 24 * k;
      d = mu[i];
      d1 = lambda[i] + d * con[i];
      if ((0.0 > d1) || rtIsNaN(d1)) {
        d1 = 0.0;
      }

      lambda[i] = d1;
      mu[i] = phi * d;
    }
  }
}

// End of code generation (mult_update.cpp)
