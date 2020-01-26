#include <stdio.h>
#include <std_dev.h>
#include <math.h>
void arm_std_f32(float *pSrc,int blocksize,float * Results);
void std_dev(float *f_array, int N){
	
	float variance;
	float sum;
	float mean;
	float std_dev;
	
	float a_std_dev;
	
	float DSP_std_dev;
	
	//----C standard deviation function-----
	/* LOOP TO FIND SUM*/
	int i;
	sum = 0;
	for(i=0 ; i<N ; i++) {
		sum += f_array[i];
	}
	
	//find mean
	mean = 0;
	mean = sum/N;
	
	//find variance
	variance = 0;
	for(i=0 ; i<N; i++){
		variance+= (f_array[i]-mean)*(f_array[i]-mean)/(N-1);
	}
	
	//find standard deviation
	std_dev = sqrt(variance);
	
	//-----------------------------
	
	/* Assembly variance function */
	asm_std_dev(f_array, N, &a_std_dev);
	
	/* CMSIS-DSP variance function */
	arm_std_f32(f_array, N, &DSP_std_dev);
	
	
		
	printf("Pure C   :  Standard Deviation = %f\n", std_dev);
	printf("Assembly :  Standard Deviation = %f\n", a_std_dev);
	printf("C: DSP   :  Standard Deviation = %f\n", DSP_std_dev);	
}


