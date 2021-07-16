//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  xzgetrf.cpp
//
//  Code generation for function 'xzgetrf'
//


// Include files
#include "xzgetrf.h"
#include "rt_nonfinite.h"
#include <cmath>

// Function Definitions
namespace coder
{
  namespace internal
  {
    namespace reflapack
    {
      void xzgetrf(double A[16], int ipiv[4], int *info)
      {
        int jA;
        int jy;
        ipiv[0] = 1;
        ipiv[1] = 2;
        ipiv[2] = 3;
        ipiv[3] = 4;
        *info = 0;
        for (int j = 0; j < 3; j++) {
          double smax;
          int b_tmp;
          int i;
          int ix;
          int jp1j;
          int k;
          int mmj_tmp;
          mmj_tmp = 2 - j;
          b_tmp = j * 5;
          jp1j = b_tmp + 2;
          jy = 4 - j;
          jA = 0;
          ix = b_tmp;
          smax = std::abs(A[b_tmp]);
          for (k = 2; k <= jy; k++) {
            double s;
            ix++;
            s = std::abs(A[ix]);
            if (s > smax) {
              jA = k - 1;
              smax = s;
            }
          }

          if (A[b_tmp + jA] != 0.0) {
            if (jA != 0) {
              jy = j + jA;
              ipiv[j] = jy + 1;
              smax = A[j];
              A[j] = A[jy];
              A[jy] = smax;
              smax = A[j + 4];
              A[j + 4] = A[jy + 4];
              A[jy + 4] = smax;
              smax = A[j + 8];
              A[j + 8] = A[jy + 8];
              A[jy + 8] = smax;
              smax = A[j + 12];
              A[j + 12] = A[jy + 12];
              A[jy + 12] = smax;
            }

            i = (b_tmp - j) + 4;
            for (jy = jp1j; jy <= i; jy++) {
              A[jy - 1] /= A[b_tmp];
            }
          } else {
            *info = j + 1;
          }

          jy = b_tmp + 4;
          jA = b_tmp;
          for (jp1j = 0; jp1j <= mmj_tmp; jp1j++) {
            smax = A[jy];
            if (A[jy] != 0.0) {
              ix = b_tmp + 1;
              i = jA + 6;
              k = (jA - j) + 8;
              for (int ijA = i; ijA <= k; ijA++) {
                A[ijA - 1] += A[ix] * -smax;
                ix++;
              }
            }

            jy += 4;
            jA += 4;
          }
        }

        if ((*info == 0) && (!(A[15] != 0.0))) {
          *info = 4;
        }
      }
    }
  }
}

// End of code generation (xzgetrf.cpp)
