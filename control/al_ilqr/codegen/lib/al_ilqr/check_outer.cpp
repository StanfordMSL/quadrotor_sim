//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  check_outer.cpp
//
//  Code generation for function 'check_outer'
//


// Include files
#include "check_outer.h"
#include "rt_nonfinite.h"

// Function Definitions
boolean_T check_outer(const double con[14424], double tol_motor, double tol_gate)
{
  int a;
  int i;
  int i2;
  int ix;
  int iy;
  int nz;
  boolean_T c_x[9616];
  boolean_T b_x[4808];
  boolean_T x[601];
  boolean_T exitg1;
  boolean_T flag;
  for (i2 = 0; i2 < 601; i2++) {
    for (iy = 0; iy < 8; iy++) {
      b_x[iy + (i2 << 3)] = (con[iy + 24 * i2] > tol_motor);
    }

    x[i2] = false;
  }

  i2 = 1;
  iy = -1;
  for (i = 0; i < 601; i++) {
    a = i2 + 7;
    ix = i2;
    i2 += 8;
    iy++;
    exitg1 = false;
    while ((!exitg1) && (ix <= a)) {
      if (!b_x[ix - 1]) {
        ix++;
      } else {
        x[iy] = true;
        exitg1 = true;
      }
    }
  }

  nz = x[0];
  for (i2 = 0; i2 < 600; i2++) {
    nz += x[i2 + 1];
  }

  for (i2 = 0; i2 < 601; i2++) {
    for (iy = 0; iy < 16; iy++) {
      c_x[iy + (i2 << 4)] = (con[(iy + 24 * i2) + 8] > tol_gate);
    }

    x[i2] = false;
  }

  i2 = 1;
  iy = -1;
  for (i = 0; i < 601; i++) {
    a = i2 + 15;
    ix = i2;
    i2 += 16;
    iy++;
    exitg1 = false;
    while ((!exitg1) && (ix <= a)) {
      if (!c_x[ix - 1]) {
        ix++;
      } else {
        x[iy] = true;
        exitg1 = true;
      }
    }
  }

  iy = x[0];
  for (i2 = 0; i2 < 600; i2++) {
    iy += x[i2 + 1];
  }

  if (static_cast<double>(nz) + static_cast<double>(iy) > 0.0) {
    flag = false;

    //  keep going
  } else {
    flag = true;

    //  stop
  }

  //      disp(['[check_outer]: Unsatisfied Constraints [motor/gate]: ',mat2str(con_status)]); 
  return flag;
}

// End of code generation (check_outer.cpp)
