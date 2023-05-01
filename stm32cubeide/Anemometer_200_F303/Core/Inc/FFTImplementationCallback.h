/*
 * File: FFTImplementationCallback.h
 *
 * MATLAB Coder version            : 5.5
 * C/C++ source code generated on  : 29-Apr-2023 16:59:07
 */

#ifndef FFTIMPLEMENTATIONCALLBACK_H
#define FFTIMPLEMENTATIONCALLBACK_H

/* Include Files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void c_FFTImplementationCallback_doH(const double x[200], creal_T y[200],
                                     const creal_T wwc[199],
                                     const double costabinv[257],
                                     const double sintabinv[257]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for FFTImplementationCallback.h
 *
 * [EOF]
 */
