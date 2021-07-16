//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  al_ilqr_types.h
//
//  Code generation for function 'al_ilqr'
//


#ifndef AL_ILQR_TYPES_H
#define AL_ILQR_TYPES_H

// Include files
#include "rtwtypes.h"

// Type Definitions
struct struct0_T
{
  double x[7813];
  double u_wr[2400];
  double t_fmu[2];
  double f_out[12020];
  double hz;
  double x_br[6010];
  double u_br[2400];
  double L[24000];
};

struct struct1_T
{
  double type;
  double x[26];
};

struct struct2_T
{
  double p_g[3];
  double p_gc[12];
  double q_star[4];
  double x_lim[2];
  double y_lim[2];
  double z_lim[2];
};

#endif

// End of code generation (al_ilqr_types.h)
