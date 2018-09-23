;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~;
;   BalloonMonitor.asm
;   
;   Purpose: This code allows for a pic to communicate with the SPDL provided
;   Balloon Monitor. Written from the lab 10 template by Eric Fritz and Alex Yee.
;   
;   When loaded to a PIC16F690 it will query the status of the balloon at 4Hz
;   and store/check the received 8-byte packet over UART at 9600 baud for a valid 
;   "Balloon Intact" and "Balloon Popped" packet. The PIC then relays that status
;   to the TIVA vial a single pin that will be set (TTL) for an intact balloon
;   and cleared for a popped balloon.
;
;   Written by: Denny Delp  5/8/2015
;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~;

    list P=PIC16F690
    #include "p16f690.inc"
    __config(_CP_OFF & _WDT_OFF & _PWRTE_ON & _HS_OSC & _BOR_ON & _MCLRE_OFF)
    ;__config(_CP_OFF & _WDT_OFF & _HS_OSC & _BOR_ON & _MCLRE_OFF) ;for debugging
    ERRORLEVEL  -302

;Setup and GPIO symbolic defines
#define TIMER_COMPARE   0xFF    ;Periodic timer compare tick count
#define QUERY_MASK      0x01    ;Mask to flip the query bit
#define BALLOON_INTACT  0x02    ;for or= to set balloonStatus pin high
#define BALLOON_POPPED  0xFD    ;for and= to set balloonStatus pin low
;State machine symbolic defines
#define WAIT_FOR_0x02   0x00    ;dataState (wait for start byte)
#define COLLECT_DATA    0x01    ;dataState (colleting data packet)
#define INTACT_FLAG     0x01    ;dataEvent flag (IntactBalloonCheck event)
#define POPPED_FLAG     0x02    ;dataEvent flag (PoppedBalloonCheck event)
;Data packet symbolic defines
#define START_BYTE      0x02    ;First byte in packet
#define INTACT_SUM      0x5F    ;Correct Packet Data for intact balloon
#define POPPED_SUM      0x53    ;Correct Packet Data for popped balloon
#define END_OF_PACKET   0x05    ;Max number of data bytes (including checksum)
#define END_OF_DATA     0x04    ;Max number of data bytes (excluding checksum)

; Variable locations in memory
    CBLOCK 0x20
    WREG_TEMP,PCLATH_TEMP,STATUS_TEMP           ;ISR buffers
    dataAA,dataBB,dataCC,dataDD,dataEE,dataCK   ;Data packet storage
    dataEvent,dataBuffer,dataState,dataIndex    ;Data event/state handlers
    ENDC

;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~;
; Start code
    ORG         0
    GOTO        Main
    ORG         4
    GOTO        ISR

; Lookup Tables
IntactData:                     ;Correct data for an intact balloon
    ADDWF       PCL,f
    RETLW       0x48
    RETLW       0x75
    RETLW       0x6D
    RETLW       0x61
    RETLW       0x6E
PoppedData:                     ;Correct data for a popped balloon
    ADDWF       PCL,f
    RETLW       0x5A
    RETLW       0x6F
    RETLW       0x6D
    RETLW       0x62
    RETLW       0x69

;Main code
Main:
; Initialize variables
    MOVLW       0x00            ;Initialize variables to 0
    MOVWF       WREG_TEMP       
    MOVWF       PCLATH_TEMP     
    MOVWF       STATUS_TEMP     
    MOVWF       dataAA          
    MOVWF       dataBB          
    MOVWF       dataCC          
    MOVWF       dataDD          
    MOVWF       dataEE          
    MOVWF       dataCK          
    MOVWF       dataEvent       ;Clear any events
    MOVWF       dataBuffer      
    MOVWF       dataIndex           
    MOVWF       dataState       ;Set dataState to "wait for 0x02"

; Set up port pins
    ; Clear ANSEL
    BANKSEL     ANSEL           ;Bank2
    CLRF        ANSEL           ;All outputs digital
    CLRF        ANSELH          ;
    ; Clear Ports
    CLRF        PORTA           ;Init PORTA
    CLRF        PORTB           ;Init PORTB
    CLRF        PORTC           ;Init PORTC
    ; Configure outputs
    BANKSEL     TRISC           ;Bank1
    BCF         TRISC,0         ;Set C0 as output to Balloon Monitor
    BCF         TRISC,1         ;Set C1 as output to TIVA
    BCF         TRISB,7         ;Set B7 as output for EUSART
    BCF         TRISA,4         ;Set A4 as output for resonator

; Configure EUSART
    ; Set baud rate to 9600 
    ;(0.16% error if SYNC = 0, BRG16 = 0, BRGH = 1: equ = F_OSC/(16(x+1)))
    MOVLW       0x81            ;Value of 129 b'10000001'
    MOVWF       SPBRG           ;This should give us a baud ~9600
    ; Enable asynchronous serial port, enable transmission
    MOVLW       0x24            ;Move b'00100100' TXEN=1,SYNC=0,BRGH=1 (high speed baud mode) 
    MOVWF       TXSTA           ;
    ;  Enable reception
    BCF         STATUS,RP0      ;Bank0
    MOVLW       0x90            ;b'10010000' SPEN = 1,CREN=1
    MOVWF       RCSTA           ;

; Configure periodic timer
    MOVLW       0x31            ;b'00110001' clock prescaler 1:8 and enable timer
    MOVWF       T1CON           ;
    MOVLW       0x0A            ;b'00001010' set compare mode, CCP1 unaffected
    MOVWF       CCP1CON         ;
    MOVWF       TIMER_COMPARE   ;Compare value for timer
    MOVWF       CCPR1H          ;

; Enable interrupts
    ; Enable interrupts on PB6 (EUSART Rx) while we're in Bank 2
    BANKSEL     IOCB            ;Bank2
    BSF         IOCB,RB6        ;IOC - Interrupt on change
    BANKSEL     PIE1            ;Bank1
    BSF         PIE1,RCIE       ;EUSART RX Receive interrupt enable
    BSF         PIE1,CCP1IE     ;CCP1 interrupt enable      
    BCF         STATUS,RP0      ;Bank0
    BSF         INTCON,GIE      ;Global interrupt enable
    BSF         INTCON,PEIE     ;Periphrial interrupt enable
    MOVF        RCREG,W         ;Clear the UART RX flag for good measure
    BCF         PIR1,CCP1IF     ;Clear the timer compare flag for good measure

;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~;
;Event checker loop
MainLoop:
    ;Check for intact or popped test events
    BTFSC       dataEvent,0     ;If set, validate packet for intact balloon
    CALL        TestPacket      ;
    BTFSC       dataEvent,1     ;If set, validate packet for popped balloon
    CALL        TestPacket      ;
    GOTO        MainLoop       

;Packet validation test
TestPacket:
    CLRF        dataIndex       ;Set dataIndex to 0 for indirect addressing
TestLoop:
    ;Save current indexed data byte to data buffer
    MOVLW       dataAA          ;Bring first data byte address into W 
    ADDWF       dataIndex,W     ;Index to our current data byte
    MOVWF       FSR             ;Set FSR to current data byte address
    MOVF        INDF,W          ;Move the referenced data byte to W
    MOVWF       dataBuffer      ;Store it in dataBuffer
    ;Address correct byte at index and compare
    MOVF        dataIndex,W     ;
    BTFSC       dataEvent,0     ;Calls IntactData lookup table
    CALL        IntactData      ;
    BTFSC       dataEvent,1     ;Calls PoppedData lookup table
    CALL        PoppedData      ;
    XORWF       dataBuffer,W    ;
    BTFSS       STATUS,Z        ;Set if we have a match
    GOTO        TestFailed      ;
    ;Test if we are done checking packet
    MOVF        dataIndex,W     ;Check if we have validated all 5 data bytes
    XORLW       END_OF_DATA     ;
    BTFSC       STATUS,Z        ;Z set if entire packet validated
    GOTO        TestConfirmed   ;
    INCF        dataIndex       ;Increment dataIndex
    GOTO        TestLoop        ;Loop
TestFailed:
    CLRF        dataEvent       ;Clear event flag
    RETURN                      ;Return since packet doesn't match exactly
TestConfirmed:
    ;Set output pin based on test
    MOVF        PORTC,W         ;Read
    BTFSC       dataEvent,0     ;Set pin high
    IORLW       BALLOON_INTACT  ;Modify (C0 = 1)
    BTFSC       dataEvent,1     ;Set pin low
    ANDLW       BALLOON_POPPED  ;Modify (C0 = 0)
    MOVWF       PORTC           ;Write output
    CLRF        dataEvent       ;Clear event flag
    RETURN                      ;

;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~;
;Interrupt service responses
ISR:
PUSH: ; Pushing registers to memory
    MOVWF       WREG_TEMP       ;Save our current W
    MOVF        STATUS,W        ;Move STATUS to W
    CLRF        STATUS          ;Bank 0
    MOVWF       STATUS_TEMP     ;Save STATUS value
    MOVF        PCLATH,W        ;Move PCLATH to W
    MOVWF       PCLATH_TEMP     ;Save PCLATH value
    CLRF        PCLATH          ;Page 0

; Interrupt Response
    ; Identify which interrupt got us here
    BTFSC       PIR1,CCP1IF     ;Check if interrupt from Timer1 Compare
    GOTO        TIMER_ISR       ;
    BTFSC       PIR1,RCIF       ;Check if interrupt from EUSART Rx
    GOTO        DATA_ISR        ;
    GOTO        POP             ;Exit ISR

;ISR for periodic timer
TIMER_ISR:
    BCF         PIR1,CCP1IF     ;Clear the flag
    MOVLW       QUERY_MASK      ;Flip the query value
    XORWF       PORTC,W         ;Flip the query bit
    MOVW        PORTC           ;Write new query
    BTFSS       PORTC,0         ;Reset state to wait for 0x02 if we are lowering line
    CLRF        dataState       ;       
    MOVLW       TIMER_COMPARE   ;Reset compare value by adding TIMER_COMPARE
    ADDWF       CCPR1H,f        ;
    GOTO        POP             ;Exit ISR

;ISR for EUSART RX line (new byte recieved)
DATA_ISR:
    ;Check our state
    BTFSC       dataState,0     ;0x00 indicates wait for 0x02
    GOTO        CollectData     ;
WaitFor0x02:
    MOVF        RCREG,W         ;Clear the flag by reading RCREG
    XORLW       START_BYTE      ;Check if data matches start byte
    BTFSS       STATUS,Z        ;
    GOTO        POP             ;Exit ISR if no match   
    MOVLW       COLLECT_DATA    ;
    MOVWF       dataState       ;Set dataState to "Collect Data"
    MOVLW       0x00            ;
    MOVWF       dataIndex       ;Set dataIndex to 0
    ;TODO: Start timer for data timeout
    GOTO        POP
CollectData:
    ;Indirectly address the next data byte
    MOVLW       dataAA          ;Bring first data address into W 
    ADDWF       dataIndex,W     ;Index to our current data byte
    MOVWF       FSR             ;Set FSR to current data byte address
    MOVF        RCREG,W         ;Move the new data byte to W, clearing the flag
    MOVWF       INDF            ;Move the new data byte to the current data byte address
    MOVF        dataIndex,W     ;Check if we have saved entire packet
    XORLW       END_OF_PACKET   ;
    BTFSC       STATUS,Z        ;Z set if entire packet saved
    GOTO        CheckPacket     ;
    INCF        dataIndex       ;Increment dataIndex
    GOTO        POP             ;Exit ISR
CheckPacket:    ;Check the checksum to see if packet is valid
    CLRF        dataState       ;Change state back to wait for 0x02
    MOVLW       INTACT_SUM      ;Check for valid intact checksum
    XORWF       dataCK,W        ;
    BTFSC       STATUS,Z        ;
    GOTO        ValidIntactCK   ;
    MOVLW       POPPED_SUM      ;Check for valid popped checksum
    XORWF       dataCK,W        ;
    BTFSC       STATUS,Z        ;
    GOTO        ValidPoppedCK   ;
    GOTO        POP             ;Exit ISR
ValidIntactCK:
    MOVLW       INTACT_FLAG     ;Set intact check for packet event checker
    MOVWF       dataEvent       ;
    GOTO        POP             ;Exit ISR
ValidPoppedCK:
    MOVLW       POPPED_FLAG     ;Set popped check for packet event checker
    MOVWF       dataEvent       ;
    GOTO        POP             ;Exit ISR (just in case we all extra ISR lines or routines)

POP: ; Popping values back into their rightful place
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
