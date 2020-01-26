#include <stdio.h>
#include <asm_multiplication.h>
#include <stdlib.h>
#include <arm_math.h>
	

void multiplication(float *f1_array, float *f2_array, int N){

	//----C multiplication function-----
	int i;
	float DSP_mult;
	float* result;
	result = (float *)malloc(N* sizeof(float));
	
	for(i=0; i<N ; i++) {
		result[i] = f1_array[i]*f2_array[i];
	}
	
	//-----------------------------------------------//
	//------- CMSIS-DSP Function --------------------//
	//-----------------------------------------------//
	
	//void 	arm_mult_f32 (const float32_t *pSrcA, const float32_t *pSrcB, float32_t *pDst, uint32_t blockSize)
	
	arm_mult_f32(f1_array, f2_array, &DSP_mult , N );
	
	//-----------------------------------------------//
	//------ Assembly multiplication function -------//
	//-----------------------------------------------//
	
	asm_multiplication(f1_array, f2_array, N);
	
	//-----------------------------------------------//
	//-----------------------------------------------//
	
	/* Print Results */
	
	printf("Pure C   :  Multiplication: \n");

	for(i = 0; i < N; i++)
		printf("%f ", result[i]);
	
	printf("\n");
	
	printf("Assembly : Multiplication: \n");
	for(i = 0; i < N; i++)
		printf("%f ", f1_array[i]);
		
			printf("\n");
	
	printf("DSP : Multiplication: \n");
	
	for(i = 0; i < N; i++)
		printf("%f ", (DSP_mult+i));
}