
	AREA test, CODE, READONLY
	
	export asm_multiplication        ; label must be exported if it is to be used as a function in C
asm_multiplication  
	
	VSUB.F32 S2, S2, S2			    ; result = 0
	VLDR.F32 S0, [R0]
loop
	SUBS R2, R2, #1                 ; loop counter (N = N-1)
	BLT done                        ; loop has finished?
	VLDR.F32 S1, [R0]				; Matrix 1 index value move to S1
	VLDR.F32 S2, [R1]				; Matrix 2 index value move to S2
	VMUL.F32 S0, S2, S1	
	VSTR.F32 S0, [R0];							multiply S0 and S1 
	ADD R0, R0, #4					; go to the address of next index in first array
	ADD R1, R1, #4
	BGT continue                    ; branch loop
	
continue
	B loop                          ; loop
	
done
	              

	BX LR                           ; return
	
	END