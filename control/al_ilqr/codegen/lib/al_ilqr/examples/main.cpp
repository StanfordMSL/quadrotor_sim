//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  main.cpp
//
//  Code generation for function 'main'
//


//***********************************************************************
// This automatically generated example C++ main file shows how to call
// entry-point functions that MATLAB Coder generated. You must customize
// this file for your application. Do not modify this file directly.
// Instead, make a copy of this file, modify it, and integrate it into
// your development environment.
//
// This file initializes entry-point function arguments to a default
// size and value before calling the entry-point functions. It does
// not store or use any values returned from the entry-point functions.
// If necessary, it does pre-allocate memory for returned values.
// You can use this file as a starting point for a main function that
// you can deploy in your application.
//
// After you copy the file, and before you deploy it, you must make the
// following changes:
// * For variable-size function arguments, change the example sizes to
// the sizes that your application requires.
// * Change the example values of function arguments to the values that
// your application requires.
// * If the entry-point functions return values, store these values or
// otherwise use them as required by your application.
//
//***********************************************************************

// Include files
#include "main.h"
#include "al_ilqr.h"
#include "al_ilqr_terminate.h"
#include "al_ilqr_types.h"
#include "rt_nonfinite.h"
#include <cstring>

// Function Declarations
static void argInit_10x601_real_T(double result[6010]);
static void argInit_13x2_real_T(double result[26]);
static void argInit_13x601_real_T(double result[7813]);
static void argInit_1x2_real_T(double result[2]);
static void argInit_3x1_real_T(double result[3]);
static void argInit_3x4_real_T(double result[12]);
static void argInit_4x10x600_real_T(double result[24000]);
static void argInit_4x1_real_T(double result[4]);
static void argInit_4x5x601_real_T(double result[12020]);
static void argInit_4x600_real_T(double result[2400]);
static double argInit_real_T();
static void argInit_struct0_T(struct0_T *result);
static void argInit_struct1_T(struct1_T *result);
static void argInit_struct2_T(struct2_T *result);
static void main_al_ilqr();

// Function Definitions
static void argInit_10x601_real_T(double result[6010])
{
  // Loop over the array to initialize each element.
  for (int idx0 = 0; idx0 < 10; idx0++) {
    for (int idx1 = 0; idx1 < 601; idx1++) {
      // Set the value of the array element.
      // Change this value to the value that the application requires.
      result[idx0 + 10 * idx1] = argInit_real_T();
    }
  }
}

static void argInit_13x2_real_T(double result[26])
{
  // Loop over the array to initialize each element.
  for (int idx0 = 0; idx0 < 13; idx0++) {
    for (int idx1 = 0; idx1 < 2; idx1++) {
      // Set the value of the array element.
      // Change this value to the value that the application requires.
      result[idx0 + 13 * idx1] = argInit_real_T();
    }
  }
}

static void argInit_13x601_real_T(double result[7813])
{
  // Loop over the array to initialize each element.
  for (int idx0 = 0; idx0 < 13; idx0++) {
    for (int idx1 = 0; idx1 < 601; idx1++) {
      // Set the value of the array element.
      // Change this value to the value that the application requires.
      result[idx0 + 13 * idx1] = argInit_real_T();
    }
  }
}

static void argInit_1x2_real_T(double result[2])
{
  // Loop over the array to initialize each element.
  for (int idx1 = 0; idx1 < 2; idx1++) {
    // Set the value of the array element.
    // Change this value to the value that the application requires.
    result[idx1] = argInit_real_T();
  }
}

static void argInit_3x1_real_T(double result[3])
{
  // Loop over the array to initialize each element.
  for (int idx0 = 0; idx0 < 3; idx0++) {
    // Set the value of the array element.
    // Change this value to the value that the application requires.
    result[idx0] = argInit_real_T();
  }
}

static void argInit_3x4_real_T(double result[12])
{
  // Loop over the array to initialize each element.
  for (int idx0 = 0; idx0 < 3; idx0++) {
    for (int idx1 = 0; idx1 < 4; idx1++) {
      // Set the value of the array element.
      // Change this value to the value that the application requires.
      result[idx0 + 3 * idx1] = argInit_real_T();
    }
  }
}

static void argInit_4x10x600_real_T(double result[24000])
{
  // Loop over the array to initialize each element.
  for (int idx0 = 0; idx0 < 4; idx0++) {
    for (int idx1 = 0; idx1 < 10; idx1++) {
      for (int idx2 = 0; idx2 < 600; idx2++) {
        // Set the value of the array element.
        // Change this value to the value that the application requires.
        result[(idx0 + (idx1 << 2)) + 40 * idx2] = argInit_real_T();
      }
    }
  }
}

static void argInit_4x1_real_T(double result[4])
{
  // Loop over the array to initialize each element.
  for (int idx0 = 0; idx0 < 4; idx0++) {
    // Set the value of the array element.
    // Change this value to the value that the application requires.
    result[idx0] = argInit_real_T();
  }
}

static void argInit_4x5x601_real_T(double result[12020])
{
  // Loop over the array to initialize each element.
  for (int idx0 = 0; idx0 < 4; idx0++) {
    for (int idx1 = 0; idx1 < 5; idx1++) {
      for (int idx2 = 0; idx2 < 601; idx2++) {
        // Set the value of the array element.
        // Change this value to the value that the application requires.
        result[(idx0 + (idx1 << 2)) + 20 * idx2] = argInit_real_T();
      }
    }
  }
}

static void argInit_4x600_real_T(double result[2400])
{
  // Loop over the array to initialize each element.
  for (int idx0 = 0; idx0 < 4; idx0++) {
    for (int idx1 = 0; idx1 < 600; idx1++) {
      // Set the value of the array element.
      // Change this value to the value that the application requires.
      result[idx0 + (idx1 << 2)] = argInit_real_T();
    }
  }
}

static double argInit_real_T()
{
  return 0.0;
}

static void argInit_struct0_T(struct0_T *result)
{
  // Set the value of each structure field.
  // Change this value to the value that the application requires.
  argInit_4x600_real_T(result->u_wr);
  argInit_13x601_real_T(result->x);
  argInit_1x2_real_T(result->t_fmu);
  argInit_4x5x601_real_T(result->f_out);
  result->hz = argInit_real_T();
  argInit_10x601_real_T(result->x_br);
  argInit_4x10x600_real_T(result->L);
  std::memcpy(&result->u_br[0], &result->u_wr[0], 2400U * sizeof(double));
}

static void argInit_struct1_T(struct1_T *result)
{
  // Set the value of each structure field.
  // Change this value to the value that the application requires.
  result->type = argInit_real_T();
  argInit_13x2_real_T(result->x);
}

static void argInit_struct2_T(struct2_T *result)
{
  // Set the value of each structure field.
  // Change this value to the value that the application requires.
  argInit_1x2_real_T(result->x_lim);
  argInit_3x1_real_T(result->p_g);
  argInit_3x4_real_T(result->p_gc);
  argInit_4x1_real_T(result->q_star);
  result->y_lim[0] = result->x_lim[0];
  result->z_lim[0] = result->x_lim[0];
  result->y_lim[1] = result->x_lim[1];
  result->z_lim[1] = result->x_lim[1];
}

static void main_al_ilqr()
{
  static struct0_T traj;
  struct1_T r;
  struct2_T r1;

  // Initialize function 'al_ilqr' input arguments.
  // Initialize function input argument 'traj'.
  // Initialize function input argument 'obj'.
  // Initialize function input argument 'map'.
  // Call the entry-point 'al_ilqr'.
  argInit_struct0_T(&traj);
  argInit_struct1_T(&r);
  argInit_struct2_T(&r1);
  al_ilqr(&traj, &r, &r1);
}

int main(int, const char * const [])
{
  // The initialize function is being called automatically from your entry-point function. So, a call to initialize is not included here. 
  // Invoke the entry-point functions.
  // You can call entry-point functions multiple times.
  main_al_ilqr();

  // Terminate the application.
  // You do not need to do this more than one time.
  al_ilqr_terminate();
  return 0;
}

// End of code generation (main.cpp)
