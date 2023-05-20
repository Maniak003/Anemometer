#include "arm_math.h"
#include "arm_const_structs.h"
#include "arm_chilbert_f32.h"

void arm_chilbert_f32(
	const arm_cfft_instance_f32* S,
	float32_t* p1) 
{
	uint32_t i, L;
	
	L = (S->fftLen);
	
	arm_cfft_f32(S, p1, 0, 1);
	for(i = 0; i <= (L*2)-1; i += 2){
		if(i == 0 || i == L) {}
		else if(i <= L-2) {
			p1[i] = 2 * p1[i];
			p1[i+1] = 2 * p1[i+1];
		}
		else {
			p1[i] = 0;
			p1[i+1] = 0;
		}
	}
	
	arm_cfft_f32(S, p1, 1, 1);
}
