/*
 * File: spline.c
 *
 * MATLAB Coder version            : 5.5
 * C/C++ source code generated on  : 27-May-2023 12:28:36
 */

/* Include Files */
#include "spline.h"
#include "rt_nonfinite.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : const double x_data[]
 *                const int x_size[2]
 *                const double y[10]
 *                const double xx[29]
 *                double output[29]
 * Return Type  : void
 */
void spline(const float x_data[], const int x_size[2], const float y[10],
            const float xx[29], float output[29])
{
	float md_data[1034];
	float pp_coefs_data[36];
	float dzdxdx;
	float r;
  int high_i;
  int ix;
  int low_ip1;
  int md_size_idx_1;
  int nxm1;
  int pp_coefs_size_idx_1;
  int yoffset;
  boolean_T has_endslopes;
  has_endslopes = (x_size[1] + 2 == 10);
  if ((x_size[1] <= 2) || ((x_size[1] <= 3) && (!has_endslopes))) {
    if (x_size[1] <= 2) {
      pp_coefs_size_idx_1 = 2;
      pp_coefs_data[0] = (y[1] - y[0]) / (x_data[1] - x_data[0]);
      pp_coefs_data[1] = y[0];
      md_size_idx_1 = x_size[1];
      high_i = x_size[1];
      if (high_i - 1 >= 0) {
        memcpy(&md_data[0], &x_data[0], (unsigned int)high_i * sizeof(double));
      }
    } else {
      pp_coefs_size_idx_1 = 3;
      dzdxdx = x_data[1] - x_data[0];
      r = (y[1] - y[0]) / dzdxdx;
      pp_coefs_data[0] = ((y[2] - y[1]) / (x_data[2] - x_data[1]) - r) /
                         (x_data[2] - x_data[0]);
      pp_coefs_data[1] = r - pp_coefs_data[0] * dzdxdx;
      pp_coefs_data[2] = y[0];
      md_size_idx_1 = 2;
      md_data[0] = x_data[0];
      md_data[1] = x_data[2];
    }
  } else {
	  float dx_data[1033];
	  float s_data[10];
	  float dvdf_data[9];
	  float d31;
	  float dnnm2;
    signed char szs_idx_1;
    nxm1 = x_size[1] - 1;
    if (has_endslopes) {
      szs_idx_1 = 8;
      yoffset = 1;
    } else {
      szs_idx_1 = 10;
      yoffset = 0;
    }
    for (low_ip1 = 0; low_ip1 < nxm1; low_ip1++) {
      dzdxdx = x_data[low_ip1 + 1] - x_data[low_ip1];
      dx_data[low_ip1] = dzdxdx;
      high_i = yoffset + low_ip1;
      dvdf_data[low_ip1] = (y[high_i + 1] - y[high_i]) / dzdxdx;
    }
    for (low_ip1 = 2; low_ip1 <= nxm1; low_ip1++) {
      s_data[low_ip1 - 1] =
          3.0 * (dx_data[low_ip1 - 1] * dvdf_data[low_ip1 - 2] +
                 dx_data[low_ip1 - 2] * dvdf_data[low_ip1 - 1]);
    }
    if (has_endslopes) {
      d31 = 0.0;
      dnnm2 = 0.0;
      s_data[0] = y[0] * dx_data[1];
      s_data[x_size[1] - 1] = dx_data[x_size[1] - 3] * y[x_size[1] + 1];
    } else {
      d31 = x_data[2] - x_data[0];
      dnnm2 = x_data[x_size[1] - 1] - x_data[x_size[1] - 3];
      s_data[0] = ((dx_data[0] + 2.0 * d31) * dx_data[1] * dvdf_data[0] +
                   dx_data[0] * dx_data[0] * dvdf_data[1]) /
                  d31;
      dzdxdx = dx_data[x_size[1] - 2];
      s_data[x_size[1] - 1] = ((dzdxdx + 2.0 * dnnm2) * dx_data[x_size[1] - 3] *
                                   dvdf_data[x_size[1] - 2] +
                               dzdxdx * dzdxdx * dvdf_data[x_size[1] - 3]) /
                              dnnm2;
    }
    md_data[0] = dx_data[1];
    dzdxdx = dx_data[x_size[1] - 3];
    md_data[x_size[1] - 1] = dzdxdx;
    for (low_ip1 = 2; low_ip1 <= nxm1; low_ip1++) {
      md_data[low_ip1 - 1] =
          2.0 * (dx_data[low_ip1 - 1] + dx_data[low_ip1 - 2]);
    }
    r = dx_data[1] / md_data[0];
    md_data[1] -= r * d31;
    s_data[1] -= r * s_data[0];
    for (low_ip1 = 3; low_ip1 <= nxm1; low_ip1++) {
      r = dx_data[low_ip1 - 1] / md_data[low_ip1 - 2];
      md_data[low_ip1 - 1] -= r * dx_data[low_ip1 - 3];
      s_data[low_ip1 - 1] -= r * s_data[low_ip1 - 2];
    }
    r = dnnm2 / md_data[x_size[1] - 2];
    md_data[x_size[1] - 1] -= r * dzdxdx;
    s_data[x_size[1] - 1] -= r * s_data[x_size[1] - 2];
    s_data[x_size[1] - 1] /= md_data[x_size[1] - 1];
    for (low_ip1 = nxm1; low_ip1 >= 2; low_ip1--) {
      s_data[low_ip1 - 1] =
          (s_data[low_ip1 - 1] - dx_data[low_ip1 - 2] * s_data[low_ip1]) /
          md_data[low_ip1 - 1];
    }
    s_data[0] = (s_data[0] - d31 * s_data[1]) / md_data[0];
    nxm1 = x_size[1];
    pp_coefs_size_idx_1 = 4;
    for (high_i = 0; high_i <= nxm1 - 2; high_i++) {
      dzdxdx = dvdf_data[high_i];
      r = s_data[high_i];
      d31 = dx_data[high_i];
      dnnm2 = (dzdxdx - r) / d31;
      dzdxdx = (s_data[high_i + 1] - dzdxdx) / d31;
      pp_coefs_data[high_i] = (dzdxdx - dnnm2) / d31;
      pp_coefs_data[(szs_idx_1 + high_i) - 1] = 2.0 * dnnm2 - dzdxdx;
      pp_coefs_data[((szs_idx_1 - 1) << 1) + high_i] = r;
      pp_coefs_data[3 * (szs_idx_1 - 1) + high_i] = y[yoffset + high_i];
    }
    md_size_idx_1 = x_size[1];
    high_i = x_size[1];
    memcpy(&md_data[0], &x_data[0], (unsigned int)high_i * sizeof(float));
  }
  for (ix = 0; ix < 29; ix++) {
    if (rtIsNaN(xx[ix])) {
      r = rtNaN;
    } else {
      high_i = md_size_idx_1;
      yoffset = 1;
      low_ip1 = 2;
      while (high_i > low_ip1) {
        nxm1 = (yoffset >> 1) + (high_i >> 1);
        if (((yoffset & 1) == 1) && ((high_i & 1) == 1)) {
          nxm1++;
        }
        if (xx[ix] >= md_data[nxm1 - 1]) {
          yoffset = nxm1;
          low_ip1 = nxm1 + 1;
        } else {
          high_i = nxm1;
        }
      }
      dzdxdx = xx[ix] - md_data[yoffset - 1];
      r = pp_coefs_data[yoffset - 1];
      for (high_i = 2; high_i <= pp_coefs_size_idx_1; high_i++) {
        r = dzdxdx * r +
            pp_coefs_data[(yoffset + (high_i - 1) * (md_size_idx_1 - 1)) - 1];
      }
    }
    output[ix] = r;
  }
}

/*
 * File trailer for spline.c
 *
 * [EOF]
 */
