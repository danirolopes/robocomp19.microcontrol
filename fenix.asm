; INSTITUTO TECNOLOGICO DE AERONAUTICA
; ROBOCOMP19 - LHAMA - 2017

; PROF. DOUGLAS SOARES 
; GUILHERME OLIVEIRA
; LAURIVAL NETO
; VICTOR BREDER
    
; MICROCONTROLLER: PIC16F648A (4 MHz)
; DATASHEET: http://ww1.microchip.com/downloads/en/DeviceDoc/40044E.pdf
; BLUETOOTH SERIAL COMM: 9600 baud, 8 data bits, no parity, 1 stop bit
	    
; BOARD SETUP	    
; RA0 - Right motor
; RA1 - Left motor
; RB1/RX - Bluetooth TX

; COMPILER DIRECTIVES
list p = 16F648A
radix DEC
include <P16F648A.INC>
__config _LVP_OFF & _MCLRE_OFF & _BODEN_OFF & _LVP_OFF & _CP_OFF & _PWRTE_OFF & _WDT_OFF & _INTOSC_OSC_NOCLKOUT

; CONSTANTS
; PROGRAM MEMORY (ROM)
PICRESET    EQU	    0x00	; instruction executed after reset
PICINT	    EQU	    0x04	; instruction executed after interrupt
PICPROG	    EQU	    0x30	; start of program
; DATA MEMORY (RAM)
PICRAM	    EQU	    0x20	; start of general purpose registers in RAM
; ROBOT
TICK_DUR    EQU	    50		; duration of a tick (clock cycles)
PULSE_DUR   EQU	    64		; duration of a pulse (number of ticks)
	    
; VARIABLES (RAM)
	    ORG	    PICRAM
RECEIVED    RES	    1		; received byte from serial
MOTORS	    RES	    1		; current state of motors
				;   0x00 = both motors OFF
				;   0x01 = left motor OFF, right motor ON
				;   0x02 = left motor ON, right motor OFF
LOW_DUR	    RES	    1		; duration of LOW voltage in PWM pulse (0-63)
HIGH_DUR    RES	    1		; duration of HIGH voltage in PWM pulse (0-63)
PULSE_CUR   RES	    1		; remaining duration of current pulse
TICK_CUR    RES	    1		; remaining duration of current tick
    
; PROGRAM MEMORY START
	    ORG	    PICRESET
	    GOTO    start
	    ORG	    PICINT
	    GOTO    interrupt
	    ORG	    PICPROG
; PROGRAM
start:	    
	    ; reset state
	    CLRF    INTCON	; clear all interrupt flags
	    CLRF    PORTA	; set all pins to low
	    CLRF    PORTB	; set all pins to low
	    
	    ; setup input/output pins
	    BSF	    STATUS, RP0	; switch to memory bank1
	    CLRF    TRISA	; sets all pins in PORTA as output
	    MOVLW   0x02	; 0000 0010
	    MOVWF   TRISB	; sets RB1 as input and others as output in PORTB
	    BCF	    STATUS, RP0	; switch back to memory bank0
	    
	    ; setup serial communication
	    BSF	    STATUS, RP0	; switch to memory bank1
	    MOVLW   25		; 9600 baud (if 4 MHz, async mode, BRGH = 1)
	    MOVWF   SPBRG
	    BSF	    TXSTA, TXEN	; enable transmission
	    BSF	    TXSTA, BRGH ; high baud rate
	    BCF	    TXSTA, SYNC	; asynchronous mode
	    BCF	    STATUS, RP0	; switch back to memory bank0
	    BSF	    RCSTA, SPEN	; serial port enable
	    BSF	    RCSTA, CREN ; continuous receive
	    BCF	    PIR1, RCIF	; clear USART receive interrupt flag
	    BSF	    STATUS, RP0	; switch to memory bank1
	    BSF	    PIE1, RCIE	; USART receive interrupt enable
	    BCF	    STATUS, RP0	; switch back to memory bank0
	    BSF	    INTCON, PEIE; peripheral interrupt enable
	    BSF	    INTCON, GIE ; global interrupt enable
	    MOVLW   0x07
	    MOVWF   CMCON	; disable internal A/D converters
	    
	    ; variable initialization	    
	    CLRF    MOTORS	; start with motors turned off
	    MOVLW   0x32	; initial 50% duty cycle
	    MOVWF   LOW_DUR
	    MOVLW   0x31
	    MOVWF   HIGH_DUR

	    
; MAIN LOOP	    
; Generates PWM in desired motor, as set by MOTORS variable	    
main_loop:  
	    ; low part of the pulse
	    MOVFW   LOW_DUR
	    MOVWF   PULSE_CUR	; PULSE_CUR <- LOW_DUR
	    CLRF    PORTA	; turn off motors
low_loop:   CALL    delay_tick	; delay for LOW_DUR ticks
	    DECFSZ  PULSE_CUR
	    GOTO    low_loop
	    
	    ; high part of the pulse
	    MOVFW   HIGH_DUR
	    MOVWF   PULSE_CUR	; PULSE_CUR <- HIGH_DUR
	    MOVFW   MOTORS
	    MOVWF   PORTA	; PORTA <- MOTORS
high_loop:  CALL    delay_tick	; delay of HIGH_DUR ticks
	    DECFSZ  PULSE_CUR
	    GOTO    high_loop
	    
	    GOTO    main_loop	; repeat main loop forever
	    
    
; SUBROUTINE DELAY_TICK
; Delays the processor for TICK_DUR clock cycles (corresponds to a single tick)
delay_tick: MOVLW   TICK_DUR
	    MOVWF   TICK_CUR	; TICK_CUR <- TICK_DUR
tick_loop:  DECFSZ  TICK_CUR	; delay for TICK_DUR clock cycles
	    GOTO    tick_loop
	    RETURN

; INTERRUPTION HANDLER
; Reads received byte, stores it in RECEIVED and calls handle_comm
interrupt:  BTFSS   PIR1, RCIF	; check if USART received byte
	    RETFIE
	    
	    MOVLW   0x06	; 0000 01100
	    ANDWF   RCSTA, W	; check for overrun and framing error
	    BTFSS   STATUS, Z	; skip if no error
	    GOTO    int_error	; handle error
	    MOVFW   RCREG	; read received byte
	    ;XORLW   0xC0	; simulate inverted input for Proteus test
	    MOVWF   RECEIVED	; store received byte
	    CALL    command	; handle the received command
	    RETFIE
	    
int_error:
	    BCF	    RCSTA, CREN	; clear error flags by restarting async mode
	    BSF	    RCSTA, CREN
	    RETFIE

; SUBROUTINE HANDLE COMM
; Decodes received command, setting active motors and the PWM duty cycle
command:
	    ; reads duty cycle (6 least significant bits)
	    MOVFW   RECEIVED
	    ANDLW   0x3F	; 0011 1111
	    MOVWF   HIGH_DUR	; HIGH_DUR <- RECEIVED & 0x3F
	    SUBLW   PULSE_DUR
	    MOVWF   LOW_DUR	; LOW_DUR <- PULSE_DUR - HIGH_DUR
	    
	    ; reads active motor (2 most significant bits)
	    ; 10xx xxxx = left motor ON, right motor OFF
	    ; 01xx xxxx = left motor OFF, right motor ON
	    ; 11xx xxxx = left motor OFF, right motor OFF
	    ; 00xx xxxx = invalid command, ignore
	    
	    MOVFW   RECEIVED
	    ANDLW   0xC0	; 1100 0000
	    XORLW   0x80	; 1000 0000
	    BTFSC   STATUS, Z	
	    GOTO    left_motor
	    
	    MOVFW   RECEIVED
	    ANDLW   0xC0	; 1100 0000
	    XORLW   0x40	; 0100 0000
	    BTFSC   STATUS, Z
	    GOTO    right_motor
	    
	    MOVFW   RECEIVED
	    ANDLW   0xC0	; 1100 0000
	    XORLW   0xC0	; 1100 0000
	    BTFSC   STATUS, Z
	    CLRF    MOTORS	; deactivate both motors in main loop
	    
	    RETURN
	    
left_motor: MOVLW   0x02	; 0000 0010
	    MOVWF   MOTORS	; activate left motor in main loop
	    RETURN

right_motor:MOVLW   0x01	; 0000 0001
	    MOVWF   MOTORS	; activate right motor in main loop
	    RETURN
	    
; PROGRAM END
end
