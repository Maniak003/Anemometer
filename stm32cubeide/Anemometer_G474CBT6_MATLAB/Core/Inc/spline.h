/*
 * File: spline.h
 *
 * MATLAB Coder version            : 5.5
 * C/C++ source code generated on  : 27-May-2023 12:28:36
 */

#ifndef SPLINE_H
#define SPLINE_H

/* Include Files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void spline(const float x_data[], const int x_size[2], const float y[10],
            const float xx[29], float output[29]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for spline.h
 *
 * [EOF]
 */
