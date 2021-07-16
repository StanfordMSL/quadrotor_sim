//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  xzgetrf.h
//
//  Code generation for function 'xzgetrf'
//


#ifndef XZGETRF_H
#define XZGETRF_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder
{
  namespace internal
  {
    namespace reflapack
    {
      void xzgetrf(double A[16], int ipiv[4], int *info);
    }
  }
}

#endif

// End of code generation (xzgetrf.h)
