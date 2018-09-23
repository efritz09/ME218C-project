;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~;
;	PIC_WATCH.asm
;	
;	Purpose: This code uses the UART framework developed in lab 10 by Eric Fritz
;   and Alex Yee to sent a byte of data to the TIVA containing the channel to 
;   broadcast     
;	
;	Written by: Alex Yee	4/22/2015
;				Eric Fritz  4/23/2015
;               Vikram      5/11/2015
;               Denny       5/13/2015
;
;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~;
;CHANGELOG:
;DONE! - Clean up ISR
;DONE! - Clean up switch statement
;DONE! - Re-enable ISR for RX.. so we know if we are paired or not
;DONE! - Debug the ADC to time loop
; - Test with actual pot for values/direction/point of rollover
; - LED functionality

    list P=PIC16F690
    #include "p16f690.inc"
    ;__config(_CP_OFF & _WDT_OFF & _PWRTE_ON & _HS_OSC & _BOR_ON & _MCLRE_OFF)
    __config(_CP_OFF & _WDT_OFF & _HS_OSC & _BOR_ON & _MCLRE_OFF)  ;for use with PICkit

    ERRORLEVEL  -302

; ADC symbolic defines
#define ADC_SEGMENT         0x0F;0x11
#define CLOCK_START         0x01
#define	MAX_TIME			0x0C
; Button SM symbolic defines
#define DEBOUNCING          0x01
#define DEBOUNCE_TIME       0xFF
; LED output symbolic defines
#define UNPAIRED            0x00
#define PAIRED              0x01

; Variable locations in memory
    CBLOCK  0x20
    WREG_TEMP,PCLATH_TEMP,STATUS_TEMP       ;ISR buffers
    debounceState                           ;State of debouncing timer for button
    newRead,timeIndex,upperRange            ;ADC to time temp variables
    currentTime                             ;Current time watch is reading
    pairState                               ;State for LED's
    ENDC

;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~;

; Start Code
    ORG         0
    GOTO        Main
    ORG         4
    GOTO        ISR

; Main code
Main:
; Initialize Variables
    MOVLW       0x00            ;Initialize all the variables to 0
    MOVWF       WREG_TEMP
    MOVWF       PCLATH_TEMP
    MOVWF       STATUS_TEMP
    MOVWF       currentTime
    MOVWF       upperRange
    MOVWF       newRead
    MOVWF       timeIndex
    MOVWF       debounceState   ;Set initial state to debouncing
    INCF        debounceState,f
	MOVLW		0x10
	MOVWF       pairState		;Set the initial LED value

; Set up the ports as needed
    ; Clear ANSEL
    BANKSEL     ANSEL           ;Bank 2
    CLRF        ANSEL           ;Make all outputs digital
    CLRF        ANSELH          ;
    ; Clear ports
    CLRF        PORTA           ;Init PORTA
    CLRF        PORTB           ;Init PORTB
    CLRF        PORTC           ;Init PORTC
    ; Configure outputs A0 = Analog,A4 = CLK,B7 = TX)
	; C2,3,4 as LED outputs
    BSF         ANSEL,0         ;Set RA0 as analog 
    BANKSEL     TRISB           ;Bank 1
    BCF         TRISB,7         ;Set B7 as output for EUSART
    BCF         TRISA,4         ;Set A4 as output for clock
    BSF         TRISA,0         ;Set A0 to input for ADC
	BCF			TRISC,2			;set C2 as output
	BCF			TRISC,3			;set C3 as output
	BCF			TRISC,4			;set C4 as output

; Configure EUSART
    ; Set baud rate to 9600: equ = F_OSC/(16(x+1))
    ;(0.16% error SYNC = 0, BRG16 = 0, and BRGH = 1)
    MOVLW       0x81            ;Value of 129 b'10000001'					
    MOVWF       SPBRG           ;This should set us to Baud=~9600
    ; Enable asynchronous serial port, enable transmission
    MOVLW       0x24            ;Move b'00100100' TXEN=1,SYNC=0,BRGH=1 (high speed baud mode) 
    MOVWF       TXSTA           ;
    ; Enable reception
    BCF         STATUS,RP0      ;Bank 0
    MOVLW       0x90            ;b'10010000' SPEN=1;CREN=1
    MOVWF       RCSTA           ;

; Configure AD conversion
    ;Enable ADC for polling, Vdd reference, Frc clock
    BANKSEL     ADCON1          ;Bank1
    MOVLW       0x60            ;b'01100000'
    MOVWF       ADCON1          ;TODO.. check what is being set and comment
    BCF         STATUS,RP0      ;Bank0
    MOVLW       0x01            ;b’00000001’ ;Left justify,
    MOVWF       ADCON0          ;Vdd Vref, AN0, On
	
; Configure periodic timer for debouncing..look into one-shot mode
    MOVLW       0x31            ;b'00110001' clock prescaler 1:8 and enable timer
    MOVWF       T1CON           ;
    MOVLW       0x0A            ;b'00001010' set compare mode, CCP1 unaffected
    MOVWF       CCP1CON         ;
    MOVWF       DEBOUNCE_TIME   ;Set initial debounce
    MOVWF       CCPR1H          ;

; Enable interrupts
    ; Enable Interrupts on PB6
    BANKSEL     IOCB            ;Bank2
    BSF         IOCB,RB6        ;Enable Interrupt on Change B6 (button)
	
	BANKSEL		INTCON			;Bank 0
 	BSF	        INTCON,GIE      ;Global interrupt enable
    BSF	        INTCON,PEIE	    ;Peripheral interrupt enable
    BSF         INTCON,RABIE    ;Port A/B Change interrupt enable

    BANKSEL     PIE1            ;Bank1
    BSF         PIE1,RCIE       ;Enable RX interrupt
    BSF			PIE1,CCP1IE		;Enable Timer 1 compare interrupt
    BCF         STATUS,RP0      ;Bank 0
    ; Enable global, periphial, and button interrupts
   
	; Clear the interrupt flags for good measure
	MOVF        RCREG,W         ;UART RX
	BCF			PIR1,CCP1IF		;Timer 1 compare

	; set initial LED value

;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~;

Main_Loop:
    ;Continuously poll AD pin A0 and convert value to the "time" (1-12).. sent on button press
    ;TODO.. make cool LED pattern when pair button pressed

    BSF         ADCON0,GO       ;Start AD conversion
    BTFSC       ADCON0,GO       ;Is conversion done?
    GOTO        $-1             ;No, test again
    MOVF        ADRESH,W        ;Read 8 bit AD value with lower 2 bits truncated (noise)
    MOVWF       newRead         ;Store new AD read from watch
 	CALL        CheckTime       ;Check the tme on the watch
    GOTO        Main_Loop       ;

CheckTime: 
    ;Now Check what time this corresponds to 
    MOVLW       0x01            ;Start index at 1 (1-12 range for time)
    MOVWF       timeIndex       ;
    MOVLW       ADC_SEGMENT     ;Set upper ADC limit for 1 and save to running upper limit
    MOVWF       upperRange      ;
CheckTimeLoop:
    SUBWF       newRead,W       ;Subtract upper range from newRead
    BTFSS       STATUS,C        ;C is clear if our time is higher...set?
    GOTO        SetTime         ;
	MOVF		timeIndex,W		;Test if we are at 12 and set time if we are
	SUBLW		0x0C			;(due to rollover problem, 256 not divisible by 12)
	BTFSC		STATUS,Z		;
	GOTO		SetTime			;
    INCF        timeIndex,f     ;Set time to next hour
    MOVF        upperRange,W    ;Increment upper range by adding ADC_SEGMENT and keeping in W
    ADDLW       ADC_SEGMENT     ;
	MOVWF 		upperRange		;
    GOTO        CheckTimeLoop   ;Repeat check
SetTime:
    MOVF        timeIndex,w     ;Set the current time and return    
    MOVWF       currentTime     ;
    RETURN                      ;

;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~;

ISR:
PUSH: ;Pushing registers to memory
    MOVWF       WREG_TEMP       ;Save our current W
    MOVF        STATUS,W        ;Move STATUS to W
    CLRF        STATUS          ;Bank 0
    MOVWF       STATUS_TEMP     ;Save STATUS value
    MOVF        PCLATH,W        ;Move PCLATH to W
    MOVWF       PCLATH_TEMP     ;Save PCLATH value
    CLRF        PCLATH          ;Page 0

; Interrupt Response
    ;Identify which interrupt got us here
    BTFSC       PIR1,RCIF       ;Check if RX interrupt
    GOTO        RX_ISR          ;
    BTFSC       PIR1,CCP1IF     ;Check if Timer1 debouncer interrupt
    GOTO        TIMER_ISR       ;
    BTFSC       INTCON,RABIF    ;Check if transmit button interrupt
    GOTO        BUTTON_ISR      ;  
    GOTO        POP             ;Exit ISR

BUTTON_ISR: 
    ;Check debouncer and rising edge, then send current time to TIVA
    BCF         INTCON,RABIF    ;Clear the interrupt flag
    BTFSS       PORTB,6         ;Check for rising edge only (button press)
    GOTO        POP             ;
    ;BTFSC       debounceState,1 ;Check if we are currently debouncing
    ;GOTO        POP             ;
    MOVF        currentTime,W   ;Load time into W if not debouncing
	;MOVF		newRead,w
    MOVWF       TXREG           ;Transmit
    ;TODO: Start the debouncing timer.. will be turned off by compare
    ;Might have to reset timer or read value and add.. not sure
    ;Testcode.. might work
    ;MOVF        TMR1L,W         ;Add debouncing time to current time
    ;ADDLW       DEBOUNCE_TIME   ;and set compare value
    ;MOVWF       CCPR1H          ;
    ;INCF        debounceState,f ;Set debounce flag
    GOTO        POP             ;exit the ISR

TIMER_ISR:
    ;If state is debouncing, set to not debouncing
    ;otherwise do nothing
    BCF         PIR1,CCP1IF     ;Clear the flag
    BTFSC       debounceState,1 ;Check if we are debouncing
    GOTO        POP
    CLRF        debounceState   ;Clear debounce flag if it was set
    GOTO        POP             ;exit the ISR

RX_ISR:
    ;New data byte from TIVA
    ;Tells us if we are paired or unpaired
    MOVF        RCREG,W         ;Clear the flag by reading RCREG
    MOVWF       pairState       ;Save new data byte to buffer (still in w)
    ;TODO: Change LED's if state is different... add whatever functionality
	BTFSS		pairState,RC0
	BCF			PORTC,RC2
	BTFSC		pairState,RC0
	BSF			PORTC,RC2

	BTFSS		pairState,RC1
	BCF			PORTC,RC3
	BTFSC		pairState,RC1
	BSF			PORTC,RC3
	
	BTFSS		pairState,RC2
	BCF			PORTC,RC4
	BTFSC		pairState,RC2
	BSF			PORTC,RC4
	
	
POP: ;Popping values back into their rightful place
    CLRF        STATUS          ;Bank 0
    MOVF        PCLATH_TEMP,W   ;Store saved PCLATH value in W
    MOVWF       PCLATH          ;Restore PCLATH
    MOVF        STATUS_TEMP,W   ;Store saved STATUS value in W
    MOVWF       STATUS          ;Restore STATUS
    SWAPF       WREG_TEMP,F     ;Prepare W to be restored
    SWAPF       WREG_TEMP,W     ;Restore W keeping status bits
    RETFIE
    
 ;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~;   

Hang:
    GOTO        $               ;Hang if we get to the end of our code
    END

;
;                  ."-,.__
;                  `.     `.  ,
;               .--'  .._,'"-' \
;              /    .'  Boo    `'
;              `.   /   Team5  ,'
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
;           