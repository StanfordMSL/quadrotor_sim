//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  mldivide.cpp
//
//  Code generation for function 'mldivide'
//


// Include files
#include "mldivide.h"
#include "rt_nonfinite.h"
#include "xzgetrf.h"
#include <cstring>

// Function Definitions
namespace coder
{
  void mldivide(const double A[16], double B[40])
  {
    double b_A[16];
    double temp;
    int ipiv[4];
    int b_i;
    int i;
    int j;
    int jBcol;
    std::memcpy(&b_A[0], &A[0], 16U * sizeof(double));
    internal::reflapack::xzgetrf(b_A, ipiv, &jBcol);
    for (i = 0; i < 3; i++) {
      b_i = ipiv[i];
      if (b_i != i + 1) {
        for (j = 0; j < 10; j++) {
          int temp_tmp;
          jBcol = j << 2;
          temp_tmp = i + jBcol;
          temp = B[temp_tmp];
          jBcol = (b_i + jBcol) - 1;
          B[temp_tmp] = B[jBcol];
          B[jBcol] = temp;
        }
      }
    }

    for (j = 0; j < 10; j++) {
      jBcol = j << 2;
      if (B[jBcol] != 0.0) {
        for (i = 2; i < 5; i++) {
          b_i = (i + jBcol) - 1;
          B[b_i] -= B[jBcol] * b_A[i - 1];
        }
      }

      if (B[jBcol + 1] != 0.0) {
        for (i = 3; i < 5; i++) {
          b_i = (i + jBcol) - 1;
          B[b_i] -= B[jBcol + 1] * b_A[i + 3];
        }
      }

      if (B[jBcol + 2] != 0.0) {
        for (i = 4; i < 5; i++) {
          B[jBcol + 3] -= B[jBcol + 2] * b_A[11];
        }
      }
    }

    for (j = 0; j < 10; j++) {
      jBcol = j << 2;
      temp = B[jBcol + 3];
      if (temp != 0.0) {
        B[jBcol + 3] = temp / b_A[15];
        for (i = 0; i < 3; i++) {
          b_i = i + jBcol;
          B[b_i] -= B[jBcol + 3] * b_A[i + 12];
        }
      }

      temp = B[jBcol + 2];
      if (temp != 0.0) {
        B[jBcol + 2] = temp / b_A[10];
        for (i = 0; i < 2; i++) {
          b_i = i + jBcol;
          B[b_i] -= B[jBcol + 2] * b_A[i + 8];
        }
      }

      temp = B[jBcol + 1];
      if (temp != 0.0) {
        B[jBcol + 1] = temp / b_A[5];
        for (i = 0; i < 1; i++) {
          B[jBcol] -= B[jBcol + 1] * b_A[4];
        }
      }

      if (B[jBcol] != 0.0) {
        B[jBcol] /= b_A[0];
      }
    }
  }
}

// End of code generation (mldivide.cpp)
