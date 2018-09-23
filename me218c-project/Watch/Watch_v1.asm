;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~;
;	PIC_network.asm
;	
;	Purpose: This code allows for a pic to communicate with our patented
;	Team Five Booo protocol. Each device on the network must have a unique 
;	address, defined as MY_ADDR. Whenever a device writes to that address
;	the PIC will read the message and light the appropriate LEDs
;	
;
;	Written by: Alex Yee	4/22/2015
;				Eric Fritz  4/23/2015
;
;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~;

    list P=PIC16F690
    #include "p16f690.inc"
    ;__config(_CP_OFF & _WDT_OFF & _PWRTE_ON & _HS_OSC & _BOR_ON & _MCLRE_OFF)
	__config(_CP_OFF & _WDT_OFF & _HS_OSC & _BOR_ON & _MCLRE_OFF)

    ERRORLEVEL  -302

#define MY_ADDR 0x40        ;Address could be 40,C0,80,00
#define ADDR_MASK 0xC0      ;b'11000000', location of the address bits on the message byte
#define DATA_MASK 0x38      ;b'00111000', we don't use this, but these bits are the data
#define ADC_SEGMENT	0x11
 
; Variable locations in memory
    CBLOCK  0x20
	WREG_TEMP,PCLATH_TEMP
    STATUS_TEMP,RC_DATA
	RESULTHI, RESULTLO
	CUR_TIME, RANGE_H,
	RANGE_L, IDX_TIME
    ENDC

; Start Code
    ORG     0
    GOTO    Main
    ORG     4
    GOTO    ISR

; Main code
Main:

; Initialize Variables
    MOVLW   0x00		    ;Initialize all the vars to 0
    MOVWF   RC_DATA
	MOVWF	WREG_TEMP
	MOVWF	PCLATH_TEMP
	MOVWF	STATUS_TEMP
	MOVWF	CUR_TIME
	MOVWF	RANGE_H
	MOVWF	RANGE_L
	MOVWF	IDX_TIME

; Set up the ports as needed
    ; Clear ANSEL
	BANKSEL ANSEL           ;Bank 2
    CLRF    ANSEL           ;Make all outputs digital
    
    ; Clear ports
	CLRF    PORTA           ;Init PORTA 
	CLRF    PORTB           ;Init PORTB 
	CLRF    PORTC           ;Init PORTC 

    ; Enable Interrupts on PB6 while we're in Bank 2
    BSF     IOCB,RB6

    ; Configure outputs (A4, B7, C0-C2)

    BANKSEL TRISC           ;Bank 1
    MOVLW   0xFF		    ;b'11111111'
	;MOVWF   TRISC 	        ;Set all C ports as inputs
	BCF		TRISB,7	        ;Set B7 as output for EUSART	
	BCF		TRISA,4	        ;Set A4 as output for clock	        

; Configure EUSART
    ; Set baud rate to 9600 (0.16% error if SYNC = 0, BRG16 = 0, and BRGH = 1: equ = F_OSC/(16(x+1)))
    MOVLW   0x81 	        ;Value of 129 b'10000001'					
    MOVWF   SPBRG	        ;This should set us to Baud=~9600
    ; Enable asynchronous serial port, enable transmission
    MOVLW   0x24	        ;Move b'00100100' TXEN=1,SYNC=0,BRGH=1 (high speed baud mode) 
    MOVWF   TXSTA	        ;
    MOVLW   0x00	        ;b'00000000' BRG16=0,SCKP=1: enables idle high
    MOVWF   BAUDCTL	        ;
    ; Enable reception
    BCF     STATUS,RP0      ;bank 0
    MOVLW   0x90	        ;b'10010000' SPEN=1;CREN=1
    MOVWF   RCSTA	        ;
	
    ; Enable interrupts
    BSF	    INTCON,GIE      ;Global interrupt enable
    BSF	    INTCON,PEIE	    ;Peripheral interrupt enable
    BSF     INTCON,RABIE    ;Port A/B Change interrupt enable

    BANKSEL PIE1            ;move to bank 1 real quick
    BSF     PIE1,RCIE	    ;Receiving interrupt enable
    MOVF    RCREG,W         ;clear the flag for good measure
    BCF     STATUS,RP0      ;and return to bank 0

;********************BEFORE *******************************
;	;This code block configures the ADC
;	;for polling, Vdd reference, Frc clock
;	;and AN4 input.
;	;
;	;Conversion start & polling for completion 
;	; are included.
;	;
;	BANKSEL ADCON1 ;
;	MOVLW 	0x50   ;B’01010000’ -- ADC read at Fosc/16
;	MOVWF ADCON1 ;
;	BANKSEL TRISC ;
;	BSF TRISC,0 ;Set RC0 to input
;	BANKSEL ANSEL ;
;	BSF ANSEL,4 ;Set RC0 to analog
;	BANKSEL ADCON0 ;
;	MOVLW 	0xD1	; B’11010001’ ;Right justify,
;	MOVWF ADCON0 ; Vref RA1, AN4 (PC0), turn On
;	;CALL SampleTime ;Acquisiton delay
;	MOVLW 	0xD1
;	MOVLW 	0xD1
;	MOVLW 	0xD1
;	MOVLW 	0xD1
;	MOVLW 	0xD1
;	MOVLW 	0xD1
;	MOVLW 	0xD1
;	MOVLW 	0xD1
;	MOVLW 	0xD1
;	MOVLW 	0xD1

	;This code block configures the ADC
	;for polling, Vdd reference, Frc clock
	;and AN0 input.
	;
	;Conversion start & polling for completion 
	; are included.
	;
	BANKSEL ADCON1 ;
	;MOVLW 0x70 ;B’01110000’ ;ADC Frc clock
	MOVLW 0x60
	MOVWF ADCON1 ;
	BANKSEL TRISA ;
	BSF TRISA,0 ;Set RA0 to input
	BANKSEL ANSEL ;
	BSF ANSEL,0 ;Set RA0 to analog
	BANKSEL ADCON0 ;
	MOVLW 0x01;	B’00000001’ ;Left justify,
	MOVWF ADCON0 ; Vdd Vref, AN0, On

Main_Loop:
    
; 	*****************Before CODE*******************
;    BSF ADCON0,GO ;Start conversion
;	BTFSC ADCON0,GO ;Is conversion done?
;	GOTO $-1 ;No, test again
;	BANKSEL ADRESH ;
;	MOVF ADRESH,W ;Read upper 2 bits
;	MOVWF RESULTHI ;store in GPR space
;;	BANKSEL ADRESL ;
;;	BANKSEL WDTCON ;
;	MOVF ADRESL,W ;Read lower 8 bits
;	MOVWF RESULTLO ;Store in GPR space
;
;	; Now do a test to see if we are above or below a threshold
;	MOVLW	0x0A;
;	;MOVF	RESULTHI,W
;	SUBWF	RESULTHI,W
;	BTFSS	STATUS,C
;	GOTO	TRANSMIT

	BSF ADCON0,GO ;Start conversion
	BTFSC ADCON0,GO ;Is conversion done?
	GOTO $-1 ;No, test again
	BANKSEL ADRESH ;
	MOVF ADRESH,W ;Read upper 2 bits
	BANKSEL RESULTLO
	MOVWF RESULTLO ;store in GPR space
;	BANKSEL ADRESL ;
;	MOVF ADRESL,W ;Read lower 8 bits
;	BANKSEL RESULTLO
;	MOVWF RESULTLO ;Store in GPR space

	; Now Check what time this corresponds to
	BANKSEL	RANGE_L	; First banksel
	MOVLW	0x00		; Reset RANGE to be 0x00
	MOVWF	RANGE_L
	MOVLW	ADC_SEGMENT		; Reset RANGE_H to be 0x15
	MOVWF	RANGE_H

ONE_OCLOCK:
	MOVLW	0x01		;1 o clock
	MOVWF	IDX_TIME	;First see if it's one o clock:
	MOVF	RANGE_L,W		;Move 0x00 into W
	SUBWF	RESULTLO,W ; If W<F, C=1;
	BTFSS	STATUS,C ; If W<F (C=1) skip
	GOTO	TWO_OCLOCK	;If W>F, check a new range
	MOVF	RANGE_H,W	;Move 0x10 into W
	SUBWF	RESULTLO,W ; If W>F, C=0;
	BTFSC	STATUS,C ; If W<F (C=1) skip
	GOTO	TWO_OCLOCK
	GOTO	SET_TIME	; it must be 1 o clock!

TWO_OCLOCK:
	MOVLW	ADC_SEGMENT		; we will add 0x15 to our ranges
	ADDWF	RANGE_L,RANGE_L
	ADDWF	RANGE_H,RANGE_H
	MOVLW	0x02		;2 o clock
	MOVWF	IDX_TIME	
	MOVF	RANGE_L,W		
	SUBWF	RESULTLO,W ; If W<F, C=1;
	BTFSS	STATUS,C ; If W<F (C=1) skip
	GOTO	THREE	;If W>F, check a new range
	MOVF	RANGE_H,W	;Move 0x15 into W
	SUBWF	RESULTLO,W ; If W>F, C=0;
	BTFSC	STATUS,C ; If W<F (C=1) skip
	GOTO	THREE
	GOTO	SET_TIME

THREE:
	MOVLW	ADC_SEGMENT		; we will add 0x15 to our ranges
	ADDWF	RANGE_L,RANGE_L
	ADDWF	RANGE_H,RANGE_H
	MOVLW	0x03		;2 o clock
	MOVWF	IDX_TIME	
	MOVF	RANGE_L,W		
	SUBWF	RESULTLO,W ; If W<F, C=1;
	BTFSS	STATUS,C ; If W<F (C=1) skip
	GOTO	FOUR	;If W>F, check a new range
	MOVF	RANGE_H,W	;Move 0x15 into W
	SUBWF	RESULTLO,W ; If W>F, C=0;
	BTFSC	STATUS,C ; If W<F (C=1) skip
	GOTO	FOUR
	GOTO	SET_TIME	

FOUR:
	MOVLW	ADC_SEGMENT		; we will add 0x15 to our ranges
	ADDWF	RANGE_L,RANGE_L
	ADDWF	RANGE_H,RANGE_H
	MOVLW	0x04		;2 o clock
	MOVWF	IDX_TIME	
	MOVF	RANGE_L,W		
	SUBWF	RESULTLO,W ; If W<F, C=1;
	BTFSS	STATUS,C ; If W<F (C=1) skip
	GOTO	FIVE	;If W>F, check a new range
	MOVF	RANGE_H,W	;Move 0x15 into W
	SUBWF	RESULTLO,W ; If W>F, C=0;
	BTFSC	STATUS,C ; If W<F (C=1) skip
	GOTO	FIVE
	GOTO	SET_TIME	

FIVE:
	MOVLW	ADC_SEGMENT		; we will add 0x15 to our ranges
	ADDWF	RANGE_L,RANGE_L
	ADDWF	RANGE_H,RANGE_H
	MOVLW	0x05		;2 o clock
	MOVWF	IDX_TIME	
	MOVF	RANGE_L,W		
	SUBWF	RESULTLO,W ; If W<F, C=1;
	BTFSS	STATUS,C ; If W<F (C=1) skip
	GOTO	SIX	;If W>F, check a new range
	MOVF	RANGE_H,W	;Move 0x15 into W
	SUBWF	RESULTLO,W ; If W>F, C=0;
	BTFSC	STATUS,C ; If W<F (C=1) skip
	GOTO	SIX
	GOTO	SET_TIME	

SIX:
	MOVLW	ADC_SEGMENT		; we will add 0x15 to our ranges
	ADDWF	RANGE_L,RANGE_L
	ADDWF	RANGE_H,RANGE_H
	MOVLW	0x06		;2 o clock
	MOVWF	IDX_TIME	
	MOVF	RANGE_L,W		
	SUBWF	RESULTLO,W ; If W<F, C=1;
	BTFSS	STATUS,C ; If W<F (C=1) skip
	GOTO	SEVEN	;If W>F, check a new range
	MOVF	RANGE_H,W	;Move 0x15 into W
	SUBWF	RESULTLO,W ; If W>F, C=0;
	BTFSC	STATUS,C ; If W<F (C=1) skip
	GOTO	SEVEN
	GOTO	SET_TIME	

SEVEN:
	MOVLW	ADC_SEGMENT		; we will add 0x15 to our ranges
	ADDWF	RANGE_L,RANGE_L
	ADDWF	RANGE_H,RANGE_H
	MOVLW	0x07		;2 o clock
	MOVWF	IDX_TIME	
	MOVF	RANGE_L,W		
	SUBWF	RESULTLO,W ; If W<F, C=1;
	BTFSS	STATUS,C ; If W<F (C=1) skip
	GOTO	EIGHT	;If W>F, check a new range
	MOVF	RANGE_H,W	;Move 0x15 into W
	SUBWF	RESULTLO,W ; If W>F, C=0;
	BTFSC	STATUS,C ; If W<F (C=1) skip
	GOTO	EIGHT
	GOTO	SET_TIME	

EIGHT:
	MOVLW	ADC_SEGMENT		; we will add 0x15 to our ranges
	ADDWF	RANGE_L,RANGE_L
	ADDWF	RANGE_H,RANGE_H
	MOVLW	0x08		;2 o clock
	MOVWF	IDX_TIME	
	MOVF	RANGE_L,W		
	SUBWF	RESULTLO,W ; If W<F, C=1;
	BTFSS	STATUS,C ; If W<F (C=1) skip
	GOTO	NINE	;If W>F, check a new range
	MOVF	RANGE_H,W	;Move 0x15 into W
	SUBWF	RESULTLO,W ; If W>F, C=0;
	BTFSC	STATUS,C ; If W<F (C=1) skip
	GOTO	NINE
	GOTO	SET_TIME	

NINE:
	MOVLW	ADC_SEGMENT		; we will add 0x15 to our ranges
	ADDWF	RANGE_L,RANGE_L
	ADDWF	RANGE_H,RANGE_H
	MOVLW	0x09		;2 o clock
	MOVWF	IDX_TIME	
	MOVF	RANGE_L,W		
	SUBWF	RESULTLO,W ; If W<F, C=1;
	BTFSS	STATUS,C ; If W<F (C=1) skip
	GOTO	TEN	;If W>F, check a new range
	MOVF	RANGE_H,W	;Move 0x15 into W
	SUBWF	RESULTLO,W ; If W>F, C=0;
	BTFSC	STATUS,C ; If W<F (C=1) skip
	GOTO	TEN
	GOTO	SET_TIME	

TEN:
	MOVLW	ADC_SEGMENT		; we will add 0x15 to our ranges
	ADDWF	RANGE_L,RANGE_L
	ADDWF	RANGE_H,RANGE_H
	MOVLW	0x0A		;2 o clock
	MOVWF	IDX_TIME	
	MOVF	RANGE_L,W		
	SUBWF	RESULTLO,W ; If W<F, C=1;
	BTFSS	STATUS,C ; If W<F (C=1) skip
	GOTO	ELEVEN	;If W>F, check a new range
	MOVF	RANGE_H,W	;Move 0x15 into W
	SUBWF	RESULTLO,W ; If W>F, C=0;
	BTFSC	STATUS,C ; If W<F (C=1) skip
	GOTO	ELEVEN
	GOTO	SET_TIME	

ELEVEN:
	MOVLW	ADC_SEGMENT		; we will add 0x15 to our ranges
	ADDWF	RANGE_L,RANGE_L
	ADDWF	RANGE_H,RANGE_H
	MOVLW	0x0B		;2 o clock
	MOVWF	IDX_TIME	
	MOVF	RANGE_L,W		
	SUBWF	RESULTLO,W ; If W<F, C=1;
	BTFSS	STATUS,C ; If W<F (C=1) skip
	GOTO	TWELVE	;If W>F, check a new range
	MOVF	RANGE_H,W	;Move 0x15 into W
	SUBWF	RESULTLO,W ; If W>F, C=0;
	BTFSC	STATUS,C ; If W<F (C=1) skip
	GOTO	TWELVE
	GOTO	SET_TIME	

TWELVE:
	MOVLW	ADC_SEGMENT		; we will add 0x15 to our ranges
	ADDWF	RANGE_L,RANGE_L
	ADDWF	RANGE_H,RANGE_H
	MOVLW	0x0C		;2 o clock
	MOVWF	IDX_TIME	
	MOVF	RANGE_L,W		
	SUBWF	RESULTLO,W ; If W<F, C=1;
	BTFSS	STATUS,C ; If W<F (C=1) skip
	GOTO	Main_Loop	;If W>F, check a new range
	MOVF	RANGE_H,W	;Move 0x15 into W
	SUBWF	RESULTLO,W ; If W>F, C=0;
	BTFSC	STATUS,C ; If W<F (C=1) skip
	GOTO	Main_Loop
	GOTO	SET_TIME	

;
;	MOVLW	0x0C		;12 o clock
;	MOVWF	IDX_TIME	;First see if it's one o clock:
;	MOVF	IDX_RANGE,W	
;	ADDLW	0x15	; Increments of 21 (12x21 = ~255)
;	MOVWF	IDX_RANGE ;Store current value of ranges we are checking
;	SUBWF	RESULTLO,W ; If W<F, C=1;
;	BTFSC	STATUS,C ; If W<F skip
;	GOTO	SET_TIME

	; Now do a test to see if we are above or below a threshold
;		MOVLW	0xFF;
;		;MOVF	RESULTHI,W
;		SUBWF	RESULTLO,W ; If W>F, C=0;
;		BTFSC	STATUS,C ; If W>F skip
;		GOTO	TRANSMIT

    GOTO    Main_Loop

SET_TIME:	
	MOVF	IDX_TIME,W
	MOVWF	CUR_TIME
	;GOTO	TRANSMIT
	GOTO	Main_Loop

TRANSMIT:
	MOVF   CUR_TIME,W         	;load inputs into w
	;MOVLW	0x0A
    MOVWF   TXREG           ;transmit the file
	GOTO	Main_Loop

ISR:  
PUSH: ; Pushing registers to memory
    MOVWF   WREG_TEMP       ;save our current W
    MOVF    STATUS,W        ;move STATUS to W
    CLRF    STATUS          ;bank 0
    MOVWF   STATUS_TEMP     ;save STATUS value
    MOVF    PCLATH,W        ;move PCLATH to W
    MOVWF   PCLATH_TEMP     ;save PCLATH value
    CLRF    PCLATH          ;page 0
	
RESPONSE:    
    ; Button pressed case
    BTFSS   INTCON,RABIF	;check to see if the interrupt flag is high
    GOTO    RC				;skip if not
    BCF     INTCON,RABIF    ;clear the flag
    BTFSS   PORTB,6         ;clears the mismatch, AND checks the bit
    GOTO    POP             ;if it was clear, skip. only a button press sends data
    MOVF    CUR_TIME,W         ;load inputs into w
    MOVWF   TXREG           ;transmit the file 
    GOTO    POP             ;exit the ISR

RC: 
    ; Transmission received case
    BTFSS   PIR1,RCIF       ;test receive interrupt flag
    GOTO    POP             ;if clear, exit ISR
    MOVF    RCREG,W
    MOVWF   RC_DATA         ;move received data to RC_DATA
    ANDLW   ADDR_MASK       ;Mask W with the address mask
    XORLW   MY_ADDR         ;XOR to see if the addresses are the same
    BTFSS   STATUS,Z        ;Check to see if the message was calling on us
    GOTO    POP             ;Addresses were not same

 ; Map output pin 0 to input pin 5
    BTFSS   RC_DATA,RC5     
    BCF	    PORTC,RC0
    BTFSC   RC_DATA,RC5
    BSF	    PORTC,RC0
    
    ; Map output pin 1 to input pin 4
    BTFSS   RC_DATA,RC4
    BCF	    PORTC,RC1
    BTFSC   RC_DATA,RC4
    BSF	    PORTC,RC1
    
    ; Map output pin 2 to input pin 3
    BTFSS   RC_DATA,RC3
    BCF	    PORTC,RC2
    BTFSC   RC_DATA,RC3
    BSF	    PORTC,RC2


POP: ; Popping values back into their rightful place
    CLRF    STATUS          ;bank 0
    MOVF    PCLATH_TEMP,W   ;store saved PCLATH value in W
    MOVWF   PCLATH          ;restore PCLATH
    MOVF    STATUS_TEMP,W   ;store saved STATUS value in W
    MOVWF   STATUS          ;restore STATUS
    SWAPF   WREG_TEMP,F     ;prepare W to be restored
    SWAPF   WREG_TEMP,W     ;restore W keeping status bits
    RETFIE
    

    END


;
;                  ."-,.__
;                  `.     `.  ,
;               .--'  .._,'"-' \
;              /    .'         `'
;              `.   /          ,'
;                \  '--.   ,-"'
;                 `"\   |  \
;                    -. \, |
;                     `--\.'      ___.
;                          \     |._, \
;                _.,        `.   <  <\                _
;              ,' /           `, `.   | \            ( `
;           ../, `.            `  |    |\`.           \ \_
;          ,' ,.. |            _._'    ||\|            )  '".
;         / ,'   \ \         ,'.-.`-._,'  |           .  _._`.
;       ,' /      \ \        `' / `--/   | \          / /   ..\
;     .'  /        \ .         |^___^ _ ,'` `        / /     `.`.
;     |  '          ..         `-...-"  |  `-'      / /        . \.
;     | /           |\__           |    |          / /          `. \
;    , /            \   .          |    |         / /             ` `
;   / /          ,.  `._ `-_       |    |  _   ,-' /               ` \
;  / .           \"`_/\ `-_ \_,.  /'    +-' `-'  _,        ..,-.    \`.
; .  '         .-|    ,`   `    '/       \__.---'     ,   .'   '     \ \
; ' /          `._    |     .' /          \..      ,_| |  `.  ,'`     |`
; |'      _.-""\  \    \ _,'  `            \ `.   `.   -.- / |   |     \\
; ||    ,'      `. `.   '       _,...._        \   /   _/ '  |   \     ||
; ||  ,'          `. ;.,.---' ,'       `.   `.. `-'  .   /_ .'    |_   ||
; || '              V      / /           \   | \   ,'   ,' '.    !  `. ||
; ||/            _,-------7 '             \  |  `-'    |         /    `||
;  \|          ,' .-   ,' ||               | .-.        `.      .'     ||
;  `'        ,'    `".'    |               |    `.        '. -.'       `'
;           /      ,'      |               |,'    \-.._,.'/'
;          (      /        .               .       \    .'/
;          `.    |         `.             /         :_,'.'
;           \ `...\   _     ,'-.        .'         /_.-'
;            `-.__ `,  `'   |  _.>----''.  _  __  /
;                 .'        /"'          |  "'   '_
;                /_ ,  , . \             '.  ,  '.__\
;                  /_./"'"\,'              `/ `-.|
