#include "arm_math.h"
#include "arm_const_structs.h"
#include "arm_rhilbert_f32.h"

void arm_rhilbert_f32(const arm_rfft_fast_instance_f32 * S, float32_t* p1, float32_t* p2) {
	uint16_t L, i, l;
	L = (S->fftLenRFFT);
	l = L / 2;
	
	arm_rfft_fast_f32(S, p1, p2, 0);
	for(i = 1; i < L; i++) {
		if( i < l ) {
			p2[i] = 2 * p2[i];
		}
		else {
			p2[i] = 0;
		}
	}
	memset(p1, 0, L);
	arm_rfft_fast_f32(S, p2, p1, 1);
}
