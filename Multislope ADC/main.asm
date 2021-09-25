;
; Multi slope ADC with residual charge reading 
;
; Created: 19.07.2018 
; Author / Copyright : Ulrich Harms
;  free for private and educational use, if the copyright note is kept
;   
;
;  Hardware: 
;  Signal control  = PD2            active on high
;  Ref control     = PD3/4
;  MUX control     = PC3/4/5
;  Comparator for Peak overflow   =  optional PC2 
;  Zero comparator       = AN0/1  = D6/D7
;  optional comparator shift      = PD5  (adds to Signal at PD6)
;  residual charge ADC input        = AD1 
;  opt. ADC input (e.g. temp diode, average voltage) = AD0

.DEF  temp    = r16             ; scratch registers
.DEF  t2      = r17
.DEF  t3      = r22

.DEF  LoopCntL = r18            ; counter for loops
.DEF  LoopCntH = r19
.DEF  coutAL  = r12             ; count of run-up patterns, also used for sum
.DEF  coutAH  = r13
.DEF  coutBL  = r10             ; debug info during run-up / sum 
.DEF  coutBH  = r11
.DEF  coutCL  = r20             ; 
.DEF  coutCH  = r21

.DEF  t1negL  = r4              ; timestamps in rundown - phase, also some other uses
.DEF  t1negH  = r5
.DEF  t1posL  = r6
.DEF  t1posH  = r7
.DEF  t1slowL = r8
.DEF  t1slowH = r9
.DEF  reg0 = r0                 ; zero register
.DEF  nextmux = r23             ; next mux setting (port C)

.DEF w2       = r15             ; predefined value for fine delay
.DEF slow_length = r14 

; unused CPU registers: R24, R25,R3,R0,R1

#define ubuffer  XL          ; pointer for UART puffer (count up)  X = R 26 / R 27

#define  par_rustepsL   0x100        ; Variables in SRAM
#define  par_rustepsH   0x101
#define  par_muxchan   0x102    ; MUX on ADC board -> port C
#define  par_extgainA  0x103    ; external setting (e.g. Gain,ext. MUX) send via SPI
#define  par_extgainB  0x104    ; external setting (e.g. Gain,ext. MUX) send via SPI

#define F_CPU 16000000
#define BAUD         9600
#define UBRR_BAUD   ((F_CPU/(16*BAUD))-1)

#define K2_loops 120            ; number of loops for K2 adjust (max about 150 to stay inside 16 bit)
                                ; maximum about 100 if raw data are send out
#define K2_puleslength 3        ; pulse length for K2 adjust (usually 2-4)  
#define K1_long 25              ; longer pulse for K1 adjust (usually 20-30) , lmited by length of 2500 cycels to reach 20 ms loop
#define K1_short 5              ; short pulse for K1 adjust (usually 5)

#define adddel 20               ; added delay (9 cycles each to slow mode)
#define cyc_222 (F_CPU/50/(222+9*adddel))    
; number of runup loops for 1 PLC

; hardware specific definitions 

.equ  control_pos  = 16           ; pattern for  larger Reference (negativ on BB)  (PD4)
.equ  control_neg  = 8            ; pattern for  smaller (positive) (PD3)
.equ  control_Sig  = 4            ; pattern for  Signal             (PD2)

.equ  control_Sigposeg = control_Sig + control_neg  ; pattern for  Signal + small
.equ  control_Sigpos = control_Sig + control_pos    ; pattern for  Signal + large
.equ  control_SigSlow = control_Sigposeg + control_pos   ; pattern for  Signal + small + large 
.equ  control_slow = control_pos+control_neg       ; pattern for  small + large = slow 
.equ  control_hold  = 0                            ; pattern for off

; defines to check comparator bit in register temp
; adjust here if change from larger positive to larger negative or comparator polarity
#define skipCompPos  SBRS t3,ACO    
#define skipCompNeg  SBRc t3,ACO    
; define ports used for ADC control (4053), mux
#define portSW portd
#define portMUX portc            ; MUX port   , rest ist Eingang / ADC ohne Pull up !

; mux setting, including fixed part (currently 0)
#define mux0  8+16+32           ; Mux channel for 0 V = Nr. 8
#define mux7  16+32             ; mux channel for 7 V ref = Nr. 7
#define muxTemp  32             ; mux channel for diode (Temp) = Nr. 5


.equ  ADcontr  = (1 << aden) +  (1<< ADSC) + (1<<ADIF) + 6     ; ADC enable + start  + Flag (to clear) + clock / 64 (6 -> 125 kHz bei 8 MHz)
                                           ; include Interrupt flag to clear flag on start
.equ  ADMUXval = 1 + 64      ;ADC channel (1) + Ref. (64=VCC , 192 = internal , 0 = external)
                          

start:
    ldi  temp, 7+16+32    ; Data directions:  PB3 is SPI / ISP input , PB0 is unused, start as output 
	out  DDRB,temp
	ldi  temp, 32+16+8    ; port for MUX control = PC3/4/5
	out DDRC,temp         ; port C = ADC  and comparator in, MUX out
	ldi  temp, 2+4+8+16+32
	out DDRD,temp          ; port D ;  D0=Rx, D1 = TX , D2/D3/D4 = 4053,  D5 = level shift ; D6/7 = int_COMP 
	ldi temp,3
	sts DIDR1,temp         ; disable digital inputs for comparator pins
	; Stack is all ready at RAMEND for M48 !

	; ADC initialisierung:
	ldi temp,  ADMUXval         ; ADC channal + speed +  Ref. . for AVCC ref. (no link needed)
	sts ADMUX,temp
	ldi temp, ADcontr     ; ADC config mit start
	sts ADCSRA,temp 
	ldi temp, 1+2         ; Disable digital input for ADC inputs  0 and 1 
	sts DIDR0, temp

	ldi temp, 3           ; setup OCR1A for timinig of rundown loop (time till ADC starts)
	sts OCR1AH,temp
	ldi temp,0
	sts OCR1AL,temp

    clr reg0      ; register fixed to 0 !  (don't use r0, in case multiplications are used) 
	LDI XH,2      ; Start UART puffer at 0200
	LDI XL,0      ; Buffer empty
    // Baudrate setting (normal mode)
    ldi temp, HIGH(UBRR_BAUD)
    sts UBRR0H,temp
	ldi temp, LOW(UBRR_BAUD)
    sts UBRR0L,temp
	ldi temp, (1<<UCSZ01)|(1<<UCSZ00)|(1<<USBS0)
	sts  UCSR0C,temp
	ldi temp, (1<<RXEN0)|(1<<TXEN0)
	sts  UCSR0B,temp

	ldi temp, low(cyc_222)       ; number of Runup cycles
	sts par_rustepsL,temp
	ldi temp, high(cyc_222)    
	sts par_rustepsH,temp

	ldi nextmux, mux7
	out portMUX,nextmux
	sts par_muxchan,nextmux   ; starting point for mux 

	ldi temp,0
	rcall longdelay 

	ldi t2,'\n'   ; Data backwards ! Buffer is stack
	st x+,t2
	ldi t2,'C'
	st x+,t2
	ldi t2,'D'
	st x+,t2
	ldi t2,'A'
	st x+,t2
    rcall uart_sendall  
		
	in   t3,ACSR              ; comparator status
    skipCompPos                 ; test comparator for sign 
	st x+,t2
    rcall uart_sendall  

	                   ; initial reading may be too early after turn on to give good values !
	rcall  TestK2      ; alternative measure of K2 (slower but easier)
	rcall K1_measure
    rcall K1_measure      ; measure ratio of slow slope to smaller slope
    rcall K2_measure      ; measure µC internal ADC gain relative to slow slope
	rcall K2_measure      ; start with self cal as a self test    
   
	; rcall Drifttest  ; test for slope amplifier linearity, hum and drift rate
	rjmp mslopeA       ; multislope version A   to start 	


;***********************
control:     ; check for UART data in and  change control loop accordingly
             ; attention: no normal subroutine if action taken ! Will not return to caller if control word send !

   lds  temp,UCSR0A
   sbrs temp, RXC0   ; test for recieved data
   ret               ; no Data -> just return

   lds temp, udr0    ; read UART data
  
   cpi temp, 'A'
   brne control1
   pop temp         ; remove return address from stack
   pop temp
   rjmp mslopeA     ; Multisloope  AZ mode via MUX 

control1:
   cpi temp, 'D'
   brne control2
   pop temp
   pop temp
   rjmp dualslope   ; Dual slope  (for tests) 
control2:
   cpi temp, 'L'     
   brne control3
   pop temp
   pop temp
   rjmp Adjust_loop2  ; Scale factor measurements, fast version
control3:
  cpi temp, 'B'
   brne control4
   pop temp
   pop temp
   rjmp mslopeB    ; Multislope  2 Speed versions for linearity test

control4:
 cpi temp, 'K'
   brne control4b
   pop temp
   pop temp
   ldi nextmux, muxTemp    ; just for tests use side effect
   sts par_muxchan,nextmux
   rjmp Adjust_loop  ; Test for scale factors 
control4b:
 cpi temp, 'I'
   brne control5
   pop temp
   pop temp
   rjmp runup_inf    ; run-up only testmode 
control5:
  cpi temp, 'W'        
   brne control6
contl5:
   lds  temp,UCSR0A    ;  Wait for sync   --  does not work for some odd reason  
   nop
   sbrs temp, RXC0   ; test for recieved data , loop if no data
   rjmp contl5    ; wait for data
   lds temp, udr0    ; read UART data to clear flags
   ret       
            
control6:
  cpi temp, 'C'
   brne control8
   pop temp
   pop temp
   rjmp mslopec    ; Multislope  3 Readings: 0, ref, and Signal(MUX)

control8:
  cpi temp, 'S'    ; slow down: double integration time
   brne control9
   lds temp,par_rustepsL
   lds t2,par_rustepsH
   lsl temp    ; * 
   rol t2
   sts par_rustepsL,temp
   sts par_rustepsH,t2
   ret
control9:
  cpi temp, 'F'   ; fast mode = 1 PLC
   brne control7
   ldi temp, low(cyc_222)       ; number of Runup cycles
   sts par_rustepsL,temp
   ldi temp, high(cyc_222)    
   sts par_rustepsH,temp
   ret
   
control7:
   mov t2,temp          ; copy 
   andi temp,0xF0       ; upper bits = command with 4 bits parameter
   cpi temp, '0'        ; command 0,1,2,3,4,5,...
   brne loopend

   andi t2, 7           ; lower 3 bits 
   lsl  t2              ; shift 3 times to position of MUX control
   lsl  t2
   lsl  t2
   sts  par_muxchan,t2
   
loopend: 
ret
;  rjmp conrol       ; check for more data, just in case


;********************** general Subroutines:

uart_send:           ; Subroutine for UART send, fixed run time! 10 + RET + rcall = 17 cycles 
    tst ubuffer             ; check for buffer empty 
	breq  uart_ret1
	lds temp,UCSR0A         ; Bit UDRE0 is set when ready for new data
	BST temp,UDRE0          ; Bit -> T
	BRTC  uart_ret2
	ld  temp,-X             ; data from buffer
	STS UDR0,temp
	ret
uart_ret1:
    nop                     ; extra delay to get fixed run-time
	nop
	nop
	nop
uart_ret2:
    nop
	nop
    nop
	ret

uart_sendall:
   rcall uart_send
   tst   xl            ;  XL = 0  means empty buffer 
   brne uart_sendall
   ret


readAD_buf:              ;read ADC and copy data to buffer
	lds temp, ADCL       ; need to read ADCL first !   
	lds t2, ADCH
    st x+,t2             ; Store data
    st x+,temp           
	ret 

fullADC:       ; Start ADC, wait and read to buffer
    ldi temp, ADcontr     ; ADC config with start 
    sts ADCSRA,temp
readAD_wait:   ; wait for ADC to finish and read
    LDS temp, ADCSRA     
    andi temp, (1<<ADSC)   ; start flag to test ADC ready
	brne readAD_wait
	rjmp readAD_buf;       ; read AD to buffer and return

AD_wait:       ; wait for ADC to finish
    LDS temp, ADCSRA     
    andi temp, (1<<ADSC)   ; start flag to test ADC ready
	brne AD_wait
	ret

Delay2: nop                    ; Delay um 3*temp + 8  incl rcall und RET
Delay1: nop                    ; Delay um 3*temp + 7  incl rcall und RET
Delay0: dec temp               ; Delay um 3*temp + 6  incl rcall und RET
        brne Delay0
		ret

longdelay:                     ; delay by temp*777 cylces + about 10 cylces
      push t2
	  mov t2, temp
	  clr temp
ldel1:
       rcall delay0           ; delay of 262 cycles 
	   dec t2
	   brne ldel1
	  pop t2
	  ret

;********************** ADC specific subs:
runup_prepare:         ; prepare for runup and run down 
    clr coutAL         ; counter A for Ref. patterns during runup 
	clr coutAH
	clr coutBL         ; counter B for Ref. patterns 
	clr coutBH

Reset_counter:         ; Reset Timer 1 to prepare for wait for OC1A (e.g. rundown, adjustK )     
    ldi temp,0
	sts TCCR1B,temp                 ; stop counter
  	STS TCNT1H,temp              
    STS TCNT1L,temp                 
	ldi temp, 1 << OCF1A            ; clear flag for OC1A  
	out TIFR1,temp                  ; Clear flag by writing a 1 ! 
	RET

; Variable part of runup pattern, counting of pos phases 
variphase1:                  ; with compartor test 
    skipCompPos              ; test comparator for sign 
	ldi t2,control_Sigpos
variphase:                   ; variable phase for runup, especially useful for multi check version
	out portSW,t2            ; start variable part
	clr temp 
	bst t2, 3                ; bit 3 -> T  (output pos)
	bld temp,0               ; T -> bit 0 
	add  coutAL,temp         ; count phases 
	adc  coutAH,reg0
	; possibly add debug part (countB) here
	rcall uart_send          ; send uart data from buffer if ready 
	                         ; for higher speed could be moved just before UART_send
	ret                      ; T2 should still be output state 
	

; different versions of runup
;*******************************
runup_PV:    ; multislope runup with 4 comparator test per period (new -untested)
                               
    ldi t2,control_Sig              ; start input early, before initial preparations
	                            ; possibly add offset via reference already on
	out portSW,t2

	rcall runup_prepare         ; common init
	ldi t2,control_Sigpos 
	out portSW,t2                ; Start same as end
	in  t3,ACSR               ; get comparator status
 	lds ZL,par_rustepsL         ; number of cycles  - moved here to get min lengt for pulse
	lds ZH,par_rustepsH       
 
runupPV_loop:
                                ; T2 is old state 
    skipCompNeg               ; test comparator for sign 
	 ldi t2,control_Sigposeg             ; possible new direction
	rcall variphase             ; 1. variable
	
	in  t3,ACSR               ; get comparator status
	                            ; T2 is old state 
    skipCompNeg                 ; test comparator for sign 
     ldi t2,control_Sigposeg            ; possible new direction
	rcall variphase             ; 2nd variable phase
	in  t3,ACSR               ; get comparator status  ( for 3rd variable part early)
	nop                         ; compensate for rcall 
	nop	
	nop
	nop
	ldi t2,control_Sigposeg
	out portSW,t2                ; fixed phase, end of 1. variable phase
    nop 
    nop                         ; compensate for loop end, no need for fixed phase to be exactly the same 
                                ; T2 is old state 
    skipCompPos                 ; test comparator for sign 
	 ldi t2,control_Sigpos            ; possible new direction
	rcall variphase             ; start of 2 nd variable phase

	in   t3,ACSR              ; comparator status
    skipCompPos                 ; test comparator for sign 
	 ldi t2,control_Sigpos            ; possible new direction
	rcall variphase             ; 
	in   t3,ACSR              ; comparator status -- 1st test early !	
    nop
	nop
	sbiw ZH:ZL,1             ; subtract 1 from ZL/ZH - moved up quite a bit
	
	ldi t2,control_Sigpos      
	out portSW,t2             ; end of 2 nd variable phase   
	BRNE runupPV_loop 
	ret 

;****************************
runup_P:            ; runup phase for multislope with: 1 Comparator test per loop (2 PWM cases)
                    ; slow Version 222 Zyklen = 16+16+190   =  normal mode 
    ldi temp,control_Sig            ; start input early, before initial preparations:
	out portSW,temp
    lds ZL,par_rustepsL         ; number of cycles 
	lds ZH,par_rustepsH       
	nop    ; extra delay to get same lenght
	nop
    rcall runup_prepare         ; common init
    in   t3,ACSR              ; comparator status for first loop part
	
runupP_loop:
    ldi temp,3            ; Delay for fixed phase  
	rcall delay0          ; 3*temp+6 cycles
                          ; delay for neg phase min lenght 
  	ldi t2,control_Sigposeg             ; start positive to end negative and better matches rundown 
	out portSW,t2 
	ldi temp,2            ; Delay for fixed phase
	rcall delay1          ; 3*temp+7 cycles
                          ; extra delay from call in Varipahse

  
    skipCompPos                ; test register temp comparator for sign 
	ldi t2,control_Sigpos
	rcall variphase
    ldi  temp, 23+2*adddel     ; extra delay to set lenght of pattern (25 für 222 zyklen loop)
	rcall delay1       

    in   t3,ACSR            ; comparator status (a little after middel of Phase)
    push temp               ; wert merken (nicht elegant, aber zeit ist ja da)
    ldi  temp, 22+adddel    ; extra delay to set lenght of pattern
	rcall delay1       
	pop temp      

	ldi t2,control_Sigpos         ; neg  ref on , end of variable phase 
	out portSW,t2            ;  could more to after SBIW ... if needed
	                        ; add delay for min lenth at top, to get better start of rundown !
   	sbiw ZH:ZL,1            ; subtract 1 from ZL/ZH
	BRNE runupP_loop        ; loop
	ret                     ; end of runup 

;**************************************
runup_P1:            ; runup phase for multislope with: 1 Comparator test per loop (2 PWM cases) - fast version: 
                     ; 15+15+44 = 74 zyklen per loop
                     ; lower useful range 
    lds ZL,par_rustepsL             ; number of cycles = 3 times normal number
	lds ZH,par_rustepsH       
	lsl ZL
	rol ZH
    ldi temp,control_Sig            ; start input early, before initial preparations:
	out portSW,temp
 
	lds temp,par_rustepsL         
	add ZL,temp
	lds temp,par_rustepsH       
	adc ZH,temp
    
    rcall runup_prepare         ; common init
    in   t3,ACSR              ; comparator status for first loop part
	
runupP1_loop:
    nop                         ; delay for neg phase min lenght 
    nop                         ; delay, to get min lenght
	nop
	nop
    nop                         ; delay, to get min lenght
	nop
	nop
    nop                         ; delay, to get min lenght
	nop
	ldi t2,control_Sigposeg             ; start positive to end negative and better matches rundown 
	out portSW,t2               
	nop                         ; delay, to get min lenght
	nop
	nop
    nop                         ; delay, to get min lenght
	nop
	nop
    nop
	nop
    nop

  
    skipCompPos                ; test register temp comparator for sign 
	ldi t2,control_Sigpos
	rcall variphase

    in   t3,ACSR               ; comparator status
    push temp                ; wert merken (nicht elegant, aber zeit ist ja da)
    ldi  temp, 1+adddel      ; extra delay to set lenght of pattern
	rcall delay1       
	pop temp      

	ldi t2,control_Sigpos         ; neg  ref on , end of variable phase 
	out portSW,t2            ;  could more to after SBIW ... if needed
	                        ; add delay for min lenth at top, to get better start of rundown !
   	sbiw ZH:ZL,1            ; subtract 1 from ZL/ZH
	BRNE runupP1_loop        ; loop
	ret                     ; end of runup 
	

;******************************
; subroutine rundown : brings integrator back to zero and records data for residual charge
;                      usually needs runup_prepare to be run first (e.g. before runup)
;                      still need to wait for ADC sampling 
rundown:
    ldi t2,control_pos
	out portSW,t2               ; start of Rundown: start with larger Ref.; Input off
	  ; may need extra delay here  (min length for phase)
	ldi t2,control_neg       
	out portMUX, nextmux         ; change MUX for next conversion 
	LDI temp,1                 ; timer1 start (already 0 and OC1A flag cleared in runup prepare)
	STS TCCR1B,temp

Lrd1: in t3,ACSR             ; comparator status   - wait for comparator
    skipCompPos               
	rjmp Lrd1

	out portSW,t2              ; Rundown: smaller reference (P)
	lds t1negL,TCNT1L          ; timer value as time stamp 
	lds t1negH,TCNT1H
	ldi t2,control_slow            ; value slow slope to come next 

Lrd2: in t3,ACSR             ; comparator status
    skipCompNeg                ; comparator 
	rjmp Lrd2
	; nop                      ; add some overshoot to force longer slow time - already too long ! more like overshoot reduction 
	out portSW,t2              ; slow mode = both references polarity like N
    lds t1posL,TCNT1L          ; time-stamp (End of pos Phase)
	lds t1posH,TCNT1H
	ldi t2,control_hold             ; next step is stop
	nop                        ; some delay in case of ringing -> min length of slow phase
	nop

Lrd3: in t3,ACSR             ; comparator status
    skipCompPos              
	rjmp Lrd3

	out portSW,t2               ; stop after slow mode
    lds t1slowL,TCNT1L         ; time-stamp (End of slow Phase)
	lds t1slowH,TCNT1H

        ; wait till fixed time to allow ADC amplifier to settle and in case of drift
	    ; wait for OC1A flag with loop similar to wait for comparator -> same step size
Lrd4: in temp,TIFR1            ; Timer flags 
    SBRS temp,OCF1A            ; wait for OC1A flag set
	rjmp Lrd4
	 
	ldi temp,5
	rcall delay0            ; minimum delay after slow mode, should not get relevant anyway !

     ; ADC starts in sync with ADC clock! ideally cycle time should be a multiple of 64 cycles !
    ldi temp, ADcontr       ; ADC config mit start
	sts ADCSRA,temp         ; ADC sampling still takes some time (up to 2.5 ADC clock cycles) !
	ret

;*************************
rundown_data:                    ; save data from rundown to buffer
	mov temp,t1negL
	mov t2, t1negH
	add temp, t1slowL            ; Strong Phase: T1 neg+ T1slow-T1pos
	adc t2, t1slowH 
	sub temp, t1posL          
	sbc t2, t1posH             
	st x+,t2                     ; put result in UART buffer  H + L
	st x+,temp
	
	sub t1slowL, t1negL          ; weak phase 
	sbc t1slowH, t1negH          ; 
	st x+,t1slowH                ; put result in UART buffer
	st x+,t1slowL
	ret                       
	
;**************************
mslope0:                  ; ADC conversion mit UART send before and restart ADC
	rcall uart_sendall
	rcall Reset_counter   ; needed for rundown to have OC1A ready
	rcall rundown         ; includes restart of ADC  , extra rundown is needed, to reduce delay effekt !
	ldi temp,50           ; wait for sampling
	rcall delay0

mslope1:                  ; 1 conversion in multi slope mode
    ldi temp, 6           ; length of rundown(time till ADC starts) 6 x 16 µs = 96 µs 
	sts OCR1AH,temp       ; could be set shorter in final Version, but than more sensitive to fast DA ?
	ldi temp,0
	sts OCR1AL,temp
	
    rcall runup_P         ; includes runup prepare and send data
	rcall rundown 
mslope2:                  ; call point for just data collection:
	rcall readAD_buf      ; read ADC (just before runup)
	rcall readAD_wait     ; ADC right after rundown ;(for test purpose, data format easier)
      ldi temp, ADMUXval -1 ; MUX to auxiliary 
	  sts ADMUX,temp
	  rcall fullADC              ; start, w
	  ldi temp,  ADMUXval        ; ADC channal + speed +  Ref. . for AVCC ref. (no link needed)
	  sts ADMUX,temp

	ldi temp, ADcontr     ; ADC config with start for 2nd ADC reading
    sts ADCSRA,temp

	rcall rundown_data
	;st x+,coutBH          ; save coutB from runup = last output patterns for debug
	;st x+,coutBL
	st x+,coutAH          ; save coutA from runup 
	st x+,coutAL

    ldi temp, 60          ; extra wait for ADC Sampling needs 2-3 x 64/128 cycles 
	rcall delay0

  ret
	
;******************** measurement modes:
; routines called from control loop

runup_inf:                      ; infinite runup phase for initial test and HW adjust help         
    ldi temp,control_Sig            ; start input early, before initial preparations:
	out portSW,temp
	rcall runup_prepare         ; common init
runuI_loop:
    ldi t2,control_Sigposeg              ; Signal + P
	out portSW,t2
    ldi  temp, 3                ; min length
	rcall delay0  
	
    in   t3,ACSR              ; comparator status
    skipCompPos               ; test comparator for sign 
	ldi t2,control_Sigpos
	out portSW,t2              ; start new pattern soon
	ldi  temp, 20             ; delay to set Run-up data (2x 16 Bit)
                              ; Rundown Data (Zeiten für N und P)
                              ; ADC nach Wandlung
                              ; ADC vor Wandlung
                            
	rcall delay0       

	ldi temp,control_Sigpos        ; both refs on 
	out portSW,temp          ; change to second half !
	rcall control      
	rjmp runuI_loop


;****************************
mslopeA:                  ; run multi-slope mode with Auto Zero via MUX

    ldi nextmux,mux0      ; mux setting after next conversion 
	                      ; actual change in MUX is in rundown
	rcall mslope0         ; 1 st conversion:  Signal
	
    ldi temp,254          ; sync FF FE
    st  x+,temp
	ldi temp,255
    st  x+,temp           ; send out during next conversion 

	lds nextmux, par_muxchan;   

    rcall mslope0         ; 2 nd conversion:  0

	rcall control         ; check for UART command, possibly leave loop !
    rjmp mslopeA

;****************************
mslopeB:                      ; run multi-slope, 2 versions for tests 	
	rcall mslope1          ; 1 st conversion (runup_P)

	ldi temp,251           ; sync FF FC
    st  x+,temp
	ldi temp,255
    st  x+,temp           ; Data are send during next runup, one ADC is ready

    rcall runup_P1         ; include runup prepare and send data and adjusted number of cycles
	rcall rundown 
	rcall mslope2          ; data collecton 2nd conversion
    rcall control          ; Check UART
   rjmp mslopeB

;********************************
   mslopeC:               ; run multi-slope, 3 Wandlungen: 0 , 7 , Signal 	
    ldi nextmux,mux0      ; mux setting after next conversion 
	                      ; actual change in MUX is in rundown
	rcall mslope1         ; 1 st conversion:  Signal
	
    ldi temp,250          ; sync FF FA
    st  x+,temp
	ldi temp,255
    st  x+,temp           ; send out during next conversion 

	ldi nextmux, mux7;   
    rcall mslope1         ; 2 nd conversion:  0
	
	lds nextmux, par_muxchan;   
    rcall mslope1         ; 2 nd conversion:  0
		
	rcall control          ; Check UART
   rjmp mslopeC


;*************************
dualslope:            ; dual slope like conversion: no FB during signal integration
                      ; test mode not optimized for speed, measure and send separate
   ldi temp, 4        ; time for rundown (till ADC starts) 4 x 32 µs = 128 µs
   sts OCR1AH,temp
   ldi temp,0
   sts OCR1AL,temp
   
   rcall runup_prepare 
   rcall rundown      ; rundown to get zero , includes Start of ADC !
   ldi temp,60
   rcall delay0      ; wait for ADC sampling (ca. 9 µs)
   rcall runup_prepare 
   ldi temp, control_Sig
   out portSW, temp     ; Start integration 
   ldi temp, 220       ; wait  for runup and ADC conversion
   rcall delay0
   rcall readAD_buf    ; read ADC with data to UART buffer (ADC before conversion)
   
   rcall rundown
   rcall rundown_data

   rcall readAD_wait   ; read ADC with data to UART buffer ADC  (ADC after conversion)
   
   rcall fullADC   ;  Start ADC, wait and read to buffer 
   rcall fullADC   ;  Start ADC, wait and read to buffer 
   
   ldi temp,255        ; sync FF FF
   st  x+,temp
   st  x+,temp
   rcall uart_sendall   ; send data 
   rcall control
   rjmp dualslope

;************************
Adjust_loop:           ; loop with continuous self cal cycles
   rcall K1_measure      ; measure ratio of slow slope to smaller slope
   rcall testK2        ; alternative version of ADC gain check
   rcall control       ; test UART and possibly leave loop
   rjmp Adjust_loop

Adjust_loop2:           ; loop with continuous self cal cycles
   rcall K1_measure      ; measure ratio of slow slope to smaller slope
   rcall K1_measure      ; measure ratio of slow slope to smaller slope
   rcall K2_measure      ; measure µC internal ADC gain relative to slow slope
   rcall control       ; test UART and possibly leave loop
   rjmp Adjust_loop2


 
;  *************  measurements for ref ratio to calculate result:
;  measures ratio of fast to slow slope 
;  test with 15 is a kind of sanity check, can be removed later,
K1_measure:               
   ldi temp,9            ; 9*32 µs  Length of 1 test pattern (also sets upper limit to pos time)
   sts OCR1AH,temp       ;   
   ldi temp,150
   sts OCR1AL,temp       ; set timer limit for inner loop length (312 µs to get 20 ms total)
                         ; max time parameter for calls is about OCR1A / (3*slope ratio+10)
   
   ldi temp,K1_short     ;         parameter to test loop : length of negative phase 5 = relatively fast
   rcall K1_meas_cycle   ; test with 1 pattern, includes sending data
   ldi temp,K1_long
   rcall K1_meas_cycle    ; test slow
   ldi temp,K1_short             
   rcall K1_meas_cycle    ; test fast
   ldi temp,K1_long            
   rcall K1_meas_cycle    ; test slow
   ldi temp,K1_short     
   rcall K1_meas_cycle    ; test fast 
   ldi temp,K1_long
   rcall K1_meas_cycle     
   ret

;********************
K1_meas_cycle:  ; 1 pattern loop for slow/pos slope test, time for pos phase in Register temp
                ; positive for given time and slow till comparator + wait, repeat 64 times
				; also used in K2 functions for starting point
   mov LoopCntH,temp      ; remember length

   rcall runup_prepare    ; clear and prepare counter etc., also needed for rundown
   rcall rundown          ; rundown to get zero (no data)
   rcall readAD_wait      ; read ADC with data to UART buffer, for optional extra resolution
   
   rcall runup_prepare    ; clear sums 
   ldi LoopCntL,128        ; number of loops 64 loops to simplify calculation in AdjK2 (do not change !)
 
abgl_loop:
   rcall Reset_counter

   LDI temp,1                 ; timer1 start (already cleared and OC1A flag cleared !)
   STS TCCR1B,temp

   ldi t2,control_neg             ; smaller reference to start pulse
   out portSW,t2   
   mov temp,LoopCntH          ; length of pulse 
   rcall delay0 

   ldi t2,control_slow             
   out portSW,t2               ; go slow mode
   lds t1posL,TCNT1L          ; remember time (start of slow slope - should be constant)
   lds t1posH,TCNT1H
   ldi t2,control_hold             ; next step is stop
   
abgl1: 
   in t3,ACSR               ; comparator status
   skipCompPos              ; test comparator
   rjmp abgl1
   
   out portSW,t2               ; Rundown: stop after slow slope
   lds t1slowL,TCNT1L         ; remember time (end of slow phase)
   lds t1slowH,TCNT1H
   sub t1slowL,t1posL         ; subtract start of slow phase
   sbc t1slowH,t1posH         

   add coutAL,t1slowL         ; sum up time for slow slope
   adc coutAH,t1slowH
   adc coutBL,reg0              ; 3rd byte of sum , add 0 for carry

    ; wait for loop time (OC1A) 
abgl2: in temp,TIFR1          ; Timer flags 
    SBRS temp,OCF1A           ; check OC1A
	rjmp abgl2

   DEC LoopCntL
   BRNE abgl_loop            ; loop 
	       
   rcall fullADC    ; Start ADC, wail and data to UART buffer,  ADC value after test
   
   st x+,LoopCntH            ; length of pos phase = call parameter
   st x+,coutBL
   st x+,coutAH              ; sum of cycles, 3 bytes
   st x+,coutAL 
   ldi temp,253              ; sync FF FD - first to send out
   st  x+,temp
   ldi temp,255
   st  x+,temp
   
   rcall uart_sendall
   ret


;************ measurement of µC internal ADC scale (K2) 
;         with FB loop for single point near center of ADC range
;         should work well, but adjustment of delay length is tricky

K2_measure:
   ldi temp,4          ; waiting time, must be long enough for ADC conversion !
   sts OCR1AH,temp
   ldi temp,228        ; adjust loop length to get something like 1280 cycles (multiple of 64/128), > 900+settling delay
   sts OCR1AL,temp     ; set timer limit for total loop length (fixed)
 
   ldi temp, K2_puleslength   ; short pulse (units are 3 µC cycles)
   rcall K1_meas_cycle        ; test to get time for 0 net charge pattern time 
     ; divide number of counts by (4* Loop in AdjK1 = 128) = 512,  adjust if number is changed !
	 mov temp,coutAH
	 ldi t2,control_slow       ; fine delay via output state sequence
	 SBRS temp,0               ; check lowest bit to set fine step (2 cycles)
	 ldi t2,control_hold
	 mov w2,t2
	 LSR temp                ; divide by 2   , upper byte should be 0 for short pulse

	 subi temp, 3            ; subtract (extra fixed part (11 cycles) + half distance) / 4
	                         ; adjust for right timing here, fine adjust in loop with nop
	 mov slow_length, temp   ; short time for slow mode
	 st x+,temp              ; send out calculated delay
	 st x+,coutAH            ; send out measured delay - should be already there, but send anyway

	 
	           ; possibly extend resolution to 1 cycle resolution  with 2 more values if needed 
   rcall runup_prepare      
   rcall rundown            ; rundown to get zero (no data) - may not need it, because adjustcycleK1 ends good
  
   rcall runup_prepare      ; includes reset sums    
   ldi LoopCntL, K2_loops   ; number of loops for test 
   clr coutCL               
   clr coutCH
   ldi temp,4        
   mov t1slowH,temp         ; mark old ADC value as invalid

   rcall AD_wait            ; wait for ADC reading before loop

AdK2_loop:
   rcall Reset_counter     ; reset timer and OC1A flag
   
   LDI temp,1              ; timer1 start (already cleared and OC1A flag cleared !)
   STS TCCR1B,temp

   ldi t2,control_neg          ; ref (smaller Ref.)
   out portSW,t2   
   
   ldi temp, K2_puleslength   ; time for short phase, same as before
   rcall delay0 
   
   ldi t2,control_slow            
   out portSW,t2           ; go slow mode

   mov temp,slow_length    ; get delay (short value) 4 cycle units
ADK_del: nop               ; extra NOP to get 4 cycle delay loop to simplify calculation
   dec temp
   brne ADK_del               

   lds t1negL, adcL        ; read ADC value before this cycle  
   lds t1negH, adcH        
   mov temp, t1negH
                           ; fine adjust delay here with nop's
  
   cpi temp, 2             ; compare with center of ADC span, more slow phase gives lower ADC values 
   brcs ADK2_1             ; Jump if ADC < 0x0200 (Carry set)
   nop                     ; extra delay for loop FB  
   nop                     ; extra delay is number of nop - 1
   nop
   nop
   nop
   nop
   nop
   nop
   nop

ADK2_1:
   ldi t2,control_hold            
   out portSW,w2            ; 2 cycle steps in delay: W2 is control_hold or control_slow, depending on delay
   nop
   out portSW,t2            ; stop here if not already with W2 
    
   mov temp, t1slowH       ; high part of old ADC reading (has set length of last cycle)
   cpi temp, 4             ; check for invalid reading (at start)
   breq ADK2_3             ; no valid data, no entry 

   sub t1slowL,t1negL
   sbc t1slowH,t1negH        ; difference in ADC readings (old  - new)   
   cpi temp, 2               ; compare with center of ADC span (same as above !)
   brcs ADK2_2
   add coutAL,t1slowL        ; add result to sum A, longer delay
   adc coutAH,t1slowH
   inc coutCL
   rjmp ADK2_3 
ADK2_2:
   add coutBL,t1slowL        ; add result to sum B
   adc coutBH,t1slowH
   inc coutCH                ; no need to count other case, but want to send total number anyway 
   nop                       ; get same code length (just in case)
ADK2_3:
   mov t1slowL,t1negL        ; move new -> old 
   mov t1slowH,t1negH

   ldi temp,80           ; extra delay for settling of amplifier
   rcall delay0

  ldi temp, ADcontr     ; ADC config+start
  sts ADCSRA,temp

  st x+,t1negL         ; ADC value for debug purpose 
  st x+,t1negH        

  ; wait for loop time (OC1A) 	  
ADK2_L: in temp,TIFR1           ; Timer flags 
    SBRS temp,OCF1A             ; check OC1A
	rjmp ADK2_L

    DEC LoopCntL
	BRNE AdK2_loop       ; loop 
	          
   st x+,coutBH              ; Sum2
   st x+,coutBL
   st x+,coutAH              ; Sum1
   st x+,coutAL 
   st x+,coutCH              ; number for case B
   st x+,coutCL              ; number for case A
   ldi temp,252              ; sync FF FC  at end because of stack 
   st  x+,temp
   ldi temp,255
   st  x+,temp
   rcall uart_sendall

   rcall runup_prepare      
   rcall rundown             ; rundown after measurement for extended sanity check, debug 
   rcall rundown_data
   rcall uart_sendall
   rcall readAD_buf
   rcall uart_sendall
   ret

; **************** alternative version for K2 measurement (µC internal ADC scale)
; start with run-down, than do fixed step and record difference
; needs quite some overhead room, so that ADC does no go out of range and amplifier does not leave linear range.
; relatively slow , keep as a test
   TestK2:
   ldi temp,4               ; short waiting time, because of only short cycle
   sts OCR1AH,temp
   ldi temp,0
   sts OCR1AL,temp          ; set timer limit for total loop length (about 150-200µs ?)
 
   ldi temp,3             ; short pulse length pos
   rcall K1_meas_cycle    ; test fast to get starting value 
     ; divide number of counts by  (4* Loop in AdjK1 = 64) = 256  ,adjust if number is changed !
   mov temp,coutAH
   lsr temp               ; divide by 2 for 128 loops in K1_cal
   subi temp, 2           ; subtract extra fixed part to get suitable starting point

   mov coutCL, temp       ; time for slow mode
 tstK2_ol:                ; outer loop (measure one length)
	 
   ldi LoopCntL,60          ; number of loops, sum of 60 values still fits 16 bit.
   clr LoopCntH             ; count number of valid readings (after) 
   rcall runup_prepare      ; clear sums and counter

tstK2_loop:
   rcall reset_counter    
   rcall rundown            ; rundown to get starting point (Zero) and start ADC
   rcall AD_wait            ; wait for ADC to finish
   lds temp,adcl            ; sum up ADC before - should be always valid
   add coutAL, temp
   lds temp,adcH
   adc coutAH, temp

   rcall Reset_counter        ; reset counter and OC1A flag
   
   ldi t2,control_neg             ; ref positive 
   out portSW,t2   
   ldi temp, 3                ; time for short positive phase phase, same as at start
   rcall delay0 
   
   LDI temp,1                 ; timer1 start (already cleared and OC1A flag cleared !)
   STS TCCR1B,temp
   ldi t2,control_slow            
   out portSW,t2               ; go slow mode

   mov temp,coutCL            ; get delay (short value) 4 cycle units
tstK2_del: nop
   dec temp
   brne tstK2_del             ; 4 cycle delay loop to simplify calculation

   ldi t2,control_hold            
   out portSW,t2               ; stop  
   
  ; wait for loop time (OC1A) 	  
tstK2_L: in temp,TIFR1         ; Timer flags 
    SBRS temp,OCF1A            ; check OC1A
	rjmp tstK2_L

    ldi temp, ADcontr     ; ADC config + start
    sts ADCSRA,temp
	rcall AD_wait           ; wait for ADC to finish
	lds temp,adcl           ; sum up ADC before
	lds t2,adcH
	ldi t3,3              ; const to compare high byte 
	cpi temp,20
	cpc t2,reg0
	brcs tstK2LE     ; test for < 20
	cpi temp,240
	cpc t2,t3 
	brcc tstK2LE     ; test for > 1008
	inc LoopCntH     ; count valid readings
 tstK2LE:    
    add coutBL, temp  ; add values anyway ! (partial sum not useful anyway, as readings before don't match !)
    adc coutBH, t2

    DEC LoopCntL
	BRNE tstK2_loop       ; inner loop, sum with 1 delay setting 
	          
   rcall readAD_buf         ; for sanity check write last ADC value (in loop)
   st x+,coutBH             ; Sum2
   st x+,coutBL
   st x+,coutAH             ; Sum1
   st x+,coutAL 
   st x+,coutCL             ; length of slow phase
   st x+,LoopCntH           ; number of valid readings after test pattern
       
   ldi temp,249           ; sync FF FA  at end because of Stack 
   st  x+,temp
   ldi temp,255
   st  x+,temp
   rcall uart_sendall

   mov temp, coutBH
   cpi temp,10         ;finish if ADC near minimu -> too many invalid readings!
   brcs tk2_end     
   inc coutCL          ; increase length of slope phase
   breq tk2_end        ; limit length to 255 , usually coutBH limit will stop loop well before 
   rjmp tstk2_ol         

tk2_end:
ret


;***************************
; Test for linearity of the slope amplifier:
; ADC reading during drift phase, could be tricky with hum
; drift rate can vary (e.g. with 4053 unit, OPs) 
Drifttest:
  rcall runup_prepare    ; clear and prepare counter etc., also needed for rundown
  rcall rundown          ; rundown to get zero (no data)
  ldi temp,control_neg
  ldi t2,control_hold  
  out portSW,temp         ; pos step    very short pulse           
  out portSW,t2           ; stop  

  ldi temp,248
  st x+,temp
  ldi temp,255
  st x+,temp
  rcall uart_sendall     ; send marker

  ldi  temp,100
  mov coutAH,temp      ; max loop length for waiting, 256 ADC samples are some 28 ms

dtestL1:
  ldi temp, ADcontr    ; ADC config + start 
  sts ADCSRA,temp
  rcall AD_wait
  
  lds temp, adcL      ; read ADCL first
  lds t2, adcH        ; read ADCH to allow updates
    
  cpi temp, 0       ; test for valid: drift down(<> 255), drift up (<>0)
  brne dtest1a      ; leave loop if in ADC range
                    
  dec coutAL        ; cound down max delay
  brne dtestL1
  dec coutAH
  brne dtestL1

dtest1a: 
  clr coutAL         ; clear counter

  
dtest1:                 ; loop to send raw µC internal ADC data
  rcall fullADC
  rcall uart_sendall    ; send data and delay

  rcall fullADC
  rcall uart_sendall    ; send data and delay

  dec  coutAL
  brne dtest1         ; loop for data
  ret
