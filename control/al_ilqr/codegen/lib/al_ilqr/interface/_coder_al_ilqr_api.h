/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * _coder_al_ilqr_api.h
 *
 * Code generation for function 'al_ilqr'
 *
 */

#ifndef _CODER_AL_ILQR_API_H
#define _CODER_AL_ILQR_API_H

/* Include files */
#include "emlrt.h"
#include "tmwtypes.h"
#include <string.h>

/* Type Definitions */
#ifndef typedef_struct0_T
#define typedef_struct0_T

typedef struct {
  real_T x[7813];
  real_T u_wr[2400];
  real_T t_fmu[2];
  real_T f_out[12020];
  real_T hz;
  real_T x_br[6010];
  real_T u_br[2400];
  real_T L[24000];
} struct0_T;

#endif                                 /*typedef_struct0_T*/

#ifndef typedef_struct1_T
#define typedef_struct1_T

typedef struct {
  real_T type;
  real_T x[26];
} struct1_T;

#endif                                 /*typedef_struct1_T*/

#ifndef typedef_struct2_T
#define typedef_struct2_T

typedef struct {
  real_T p_g[3];
  real_T p_gc[12];
  real_T q_star[4];
  real_T x_lim[2];
  real_T y_lim[2];
  real_T z_lim[2];
} struct2_T;

#endif                                 /*typedef_struct2_T*/

/* Variable Declarations */
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

#ifdef __cplusplus

extern "C" {

#endif

  /* Function Declarations */
  void al_ilqr(struct0_T *traj, struct1_T *obj, struct2_T *map);
  void al_ilqr_api(const mxArray * const prhs[3], const mxArray *plhs[1]);
  void al_ilqr_atexit(void);
  void al_ilqr_initialize(void);
  void al_ilqr_terminate(void);
  void al_ilqr_xil_shutdown(void);
  void al_ilqr_xil_terminate(void);

#ifdef __cplusplus

}
#endif
#endif

/* End of code generation (_coder_al_ilqr_api.h) */
