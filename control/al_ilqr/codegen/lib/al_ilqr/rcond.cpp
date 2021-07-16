//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  rcond.cpp
//
//  Code generation for function 'rcond'
//


// Include files
#include "rcond.h"
#include "rt_nonfinite.h"
#include "xzgetrf.h"
#include "rt_nonfinite.h"
#include <cmath>
#include <cstring>

// Function Definitions
namespace coder
{
  double rcond(const double A[16])
  {
    double b_A[16];
    double x[4];
    double ainvnm;
    double normA;
    double result;
    double s;
    int ipiv[4];
    int iter;
    int j;
    int jump;
    int kase;
    boolean_T exitg1;
    result = 0.0;
    normA = 0.0;
    j = 0;
    exitg1 = false;
    while ((!exitg1) && (j < 4)) {
      kase = j << 2;
      s = ((std::abs(A[kase]) + std::abs(A[kase + 1])) + std::abs(A[kase + 2]))
        + std::abs(A[kase + 3]);
      if (rtIsNaN(s)) {
        normA = rtNaN;
        exitg1 = true;
      } else {
        if (s > normA) {
          normA = s;
        }

        j++;
      }
    }

    if (!(normA == 0.0)) {
      int exitg3;
      std::memcpy(&b_A[0], &A[0], 16U * sizeof(double));
      internal::reflapack::xzgetrf(b_A, ipiv, &kase);
      kase = 3;
      do {
        exitg3 = 0;
        if (kase + 1 > 0) {
          if (b_A[kase + (kase << 2)] == 0.0) {
            exitg3 = 1;
          } else {
            kase--;
          }
        } else {
          ainvnm = 0.0;
          iter = 2;
          kase = 1;
          jump = 1;
          j = 0;
          x[0] = 0.25;
          x[1] = 0.25;
          x[2] = 0.25;
          x[3] = 0.25;
          exitg3 = 2;
        }
      } while (exitg3 == 0);

      if (exitg3 != 1) {
        int exitg2;
        do {
          exitg2 = 0;
          if (kase == 1) {
            int i;
            for (i = 0; i < 3; i++) {
              x[i + 1] -= x[0] * b_A[i + 1];
            }

            for (i = 0; i < 2; i++) {
              x[i + 2] -= x[1] * b_A[i + 6];
            }

            for (i = 0; i < 1; i++) {
              x[3] -= x[2] * b_A[11];
            }

            x[3] /= b_A[15];
            for (i = 0; i < 3; i++) {
              x[2 - i] -= x[3] * b_A[14 - i];
            }

            x[2] /= b_A[10];
            for (i = 0; i < 2; i++) {
              x[1 - i] -= x[2] * b_A[9 - i];
            }

            x[1] /= b_A[5];
            for (i = 0; i < 1; i++) {
              x[0] -= x[1] * b_A[4];
            }

            x[0] /= b_A[0];
          } else {
            int i;
            x[0] /= b_A[0];
            s = x[1];
            for (i = 0; i < 1; i++) {
              s -= b_A[4] * x[0];
            }

            x[1] = s / b_A[5];
            s = x[2];
            for (i = 0; i < 2; i++) {
              s -= b_A[i + 8] * x[i];
            }

            x[2] = s / b_A[10];
            s = x[3];
            for (i = 0; i < 3; i++) {
              s -= b_A[i + 12] * x[i];
            }

            x[3] = s / b_A[15];
            s = x[2];
            for (i = 4; i >= 4; i--) {
              s -= b_A[11] * x[3];
            }

            x[2] = s;
            s = x[1];
            for (i = 4; i >= 3; i--) {
              s -= b_A[i + 3] * x[i - 1];
            }

            x[1] = s;
            s = x[0];
            for (i = 4; i >= 2; i--) {
              s -= b_A[i - 1] * x[i - 1];
            }

            x[0] = s;
          }

          if (jump == 1) {
            double absrexk;
            double ainvnm_tmp;
            double b_ainvnm_tmp;
            s = std::abs(x[0]);
            absrexk = std::abs(x[1]);
            ainvnm_tmp = std::abs(x[2]);
            b_ainvnm_tmp = std::abs(x[3]);
            ainvnm = ((s + absrexk) + ainvnm_tmp) + b_ainvnm_tmp;
            if (rtIsInf(ainvnm) || rtIsNaN(ainvnm)) {
              result = ainvnm;
              exitg2 = 1;
            } else {
              if (s > 2.2250738585072014E-308) {
                x[0] /= s;
              } else {
                x[0] = 1.0;
              }

              if (absrexk > 2.2250738585072014E-308) {
                x[1] /= absrexk;
              } else {
                x[1] = 1.0;
              }

              if (ainvnm_tmp > 2.2250738585072014E-308) {
                x[2] /= ainvnm_tmp;
              } else {
                x[2] = 1.0;
              }

              if (b_ainvnm_tmp > 2.2250738585072014E-308) {
                x[3] /= b_ainvnm_tmp;
              } else {
                x[3] = 1.0;
              }

              kase = 2;
              jump = 2;
            }
          } else if (jump == 2) {
            double absrexk;
            j = 0;
            s = std::abs(x[0]);
            absrexk = std::abs(x[1]);
            if (!(absrexk <= s)) {
              j = 1;
              s = absrexk;
            }

            absrexk = std::abs(x[2]);
            if (!(absrexk <= s)) {
              j = 2;
              s = absrexk;
            }

            if (!(std::abs(x[3]) <= s)) {
              j = 3;
            }

            iter = 2;
            x[0] = 0.0;
            x[1] = 0.0;
            x[2] = 0.0;
            x[3] = 0.0;
            x[j] = 1.0;
            kase = 1;
            jump = 3;
          } else if (jump == 3) {
            double absrexk;
            double ainvnm_tmp;
            double b_ainvnm_tmp;
            s = std::abs(x[0]);
            absrexk = std::abs(x[1]);
            ainvnm_tmp = std::abs(x[2]);
            b_ainvnm_tmp = std::abs(x[3]);
            ainvnm = ((s + absrexk) + ainvnm_tmp) + b_ainvnm_tmp;
            if (ainvnm <= x[0]) {
              x[0] = 1.0;
              x[1] = -1.3333333333333333;
              x[2] = 1.6666666666666665;
              x[3] = -2.0;
              kase = 1;
              jump = 5;
            } else {
              if (s > 2.2250738585072014E-308) {
                x[0] /= s;
              } else {
                x[0] = 1.0;
              }

              if (absrexk > 2.2250738585072014E-308) {
                x[1] /= absrexk;
              } else {
                x[1] = 1.0;
              }

              if (ainvnm_tmp > 2.2250738585072014E-308) {
                x[2] /= ainvnm_tmp;
              } else {
                x[2] = 1.0;
              }

              if (b_ainvnm_tmp > 2.2250738585072014E-308) {
                x[3] /= b_ainvnm_tmp;
              } else {
                x[3] = 1.0;
              }

              kase = 2;
              jump = 4;
            }
          } else if (jump == 4) {
            double absrexk;
            kase = j;
            j = 0;
            s = std::abs(x[0]);
            absrexk = std::abs(x[1]);
            if (!(absrexk <= s)) {
              j = 1;
              s = absrexk;
            }

            absrexk = std::abs(x[2]);
            if (!(absrexk <= s)) {
              j = 2;
              s = absrexk;
            }

            if (!(std::abs(x[3]) <= s)) {
              j = 3;
            }

            if ((std::abs(x[kase]) != std::abs(x[j])) && (iter <= 5)) {
              iter++;
              x[0] = 0.0;
              x[1] = 0.0;
              x[2] = 0.0;
              x[3] = 0.0;
              x[j] = 1.0;
              kase = 1;
              jump = 3;
            } else {
              x[0] = 1.0;
              x[1] = -1.3333333333333333;
              x[2] = 1.6666666666666665;
              x[3] = -2.0;
              kase = 1;
              jump = 5;
            }
          } else {
            if (jump == 5) {
              s = 2.0 * (((std::abs(x[0]) + std::abs(x[1])) + std::abs(x[2])) +
                         std::abs(x[3])) / 3.0 / 4.0;
              if (s > ainvnm) {
                ainvnm = s;
              }

              if (ainvnm != 0.0) {
                result = 1.0 / ainvnm / normA;
              }

              exitg2 = 1;
            }
          }
        } while (exitg2 == 0);
      }
    }

    return result;
  }
}

// End of code generation (rcond.cpp)
