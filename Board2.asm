;====================================================================
; PROJECT: Home Automation System - Board #2 (Curtain Control)
; AUTHOR:  [Hayri Baran Vural]
; STUDENT ID: [152120221044]
; DATE:    [30.12.2025]
;
; DESCRIPTION:
; This program controls a Curtain System using a Stepper Motor.
; Features:
; 1. Hybrid Control: Controlled by PC (UART) and Potentiometer.
;    - Smart logic prevents Potentiometer from overriding PC commands.
; 2. Reads Light Level via LDR (AN0) and closes curtain if too dark.
; 3. Drives a Stepper Motor (RB0-RB3) for Open/Close action.
; 4. Displays Status (Curtain %, Temp, Lux) on 2x16 LCD.
; 5. Implements UART Protocol to communicate with main application.
;====================================================================

    PROCESSOR 16F877A
    #include <xc.inc>

    ; --- Configuration Bits (4 MHz Crystal) ---
    CONFIG FOSC = XT        ; Oscillator Selection: XT
    CONFIG WDTE = OFF       ; Watchdog Timer: Disabled
    CONFIG PWRTE = ON       ; Power-up Timer: Enabled
    CONFIG BOREN = OFF      ; Brown-out Reset: Disabled
    CONFIG LVP = OFF        ; Low Voltage Programming: Disabled
    CONFIG CPD = OFF        ; Code Protection: Disabled
    CONFIG WRT = OFF        ; Write Protection: Disabled
    CONFIG CP = OFF         ; Code Protection: Disabled

;====================================================================
; VARIABLES (UDATA BANK0)
;====================================================================
    PSECT udata_bank0
    
    ; --- Curtain & Motor Variables ---
    DESIRED_CURTAIN:    DS 1    ; Target position % (0-100)
    DESIRED_CURTAIN_F:  DS 1    ; Fractional part (for protocol)
    CURRENT_CURTAIN:    DS 1    ; Current position %
    STEP_INDEX:         DS 1    ; Index for Stepper Motor Sequence
    SUB_STEP_CNT:       DS 1    ; Counter for motor speed/steps per unit
    
    ; --- Sensors & Control Variables ---
    LDR_VAL:            DS 1    ; Light Dependent Resistor value
    POT_VAL:            DS 1    ; Potentiometer value
    OLD_POT_VAL:        DS 1    ; Stores previous pot value (change detection)
    LIGHT_THRESHOLD:    DS 1    ; Threshold to trigger auto-close
    
    ; --- UART Communication ---
    RX_DATA:            DS 1    ; Received data
    TX_DATA:            DS 1    ; Data to send
    
    ; --- Dummy Environmental Data (for simulation) ---
    OUT_TEMP:           DS 1    
    OUT_TEMP_F:         DS 1    
    OUT_PRESS_H:        DS 1    
    OUT_PRESS_L:        DS 1    

    ; --- LCD & Math Helper Variables ---
    LCD_TEMP:           DS 1    ; Temp storage for LCD command
    LCD_REFRESH_CNT:    DS 1    ; Counter to slow down LCD refresh
    BCD_HUNDREDS:       DS 1    ; BCD Conversion
    BCD_TENS:           DS 1    
    BCD_ONES:           DS 1    
    MATH_L:             DS 1    ; Math low byte
    MATH_H:             DS 1    ; Math high byte
    TEMP_W:             DS 1    ; Temp W storage
    LOOP_VAR1:          DS 1    ; Loop counter
    LOOP_VAR2:          DS 1    ; Loop counter

;====================================================================
; RESET VECTOR
;====================================================================
    PSECT code, abs
    ORG 0x000
    CLRWDT
    GOTO MAIN

    ORG 0x004
    RETFIE

    ORG 0x020

; --- Stepper Motor Sequence Table (Half/Full Step logic) ---
GET_STEP_CODE:
    ANDLW   0x03            ; Mask index (0-3)
    ADDWF   PCL, F          ; Jump to sequence
    RETLW   0x01            ; Coil 1 (RB0)
    RETLW   0x02            ; Coil 2 (RB1)
    RETLW   0x04            ; Coil 3 (RB2)
    RETLW   0x08            ; Coil 4 (RB3)

;====================================================================
; MAIN PROGRAM
;====================================================================
MAIN:
    CLRWDT
    ; --- Port Configuration ---
    BANKSEL TRISB
    MOVLW   0xF0            ; RB0-RB3 Output (Motor)
    MOVWF   TRISB
    
    BANKSEL TRISA
    MOVLW   0x03            ; RA0, RA1 Input (LDR, POT)
    MOVWF   TRISA

    BANKSEL TRISD
    MOVLW   0x0F            ; RD4-RD7 Output (LCD Data)
    MOVWF   TRISD
    
    BANKSEL TRISE
    CLRF    TRISE           ; RE0-RE2 Output (LCD Control)
    
    ; --- UART Pin Configuration ---
    BANKSEL TRISC
    MOVLW   0x80            ; RC7 (RX) Input, RC6 (TX) Output
    MOVWF   TRISC
    
    BANKSEL PORTB
    CLRF    PORTB
    CLRF    PORTE
    CLRF    PORTD

    ; --- ADC Configuration ---
    BANKSEL ADCON1
    MOVLW   0x04            ; AN0, AN1 Analog, Others Digital
    MOVWF   ADCON1
    BANKSEL ADCON0
    MOVLW   0x41            ; ADC Enabled, Fosc/8
    MOVWF   ADCON0
    
    BANKSEL PORTA

    ; --- Initialization ---
    CLRF    STEP_INDEX
    MOVLW   10
    MOVWF   SUB_STEP_CNT
    CLRF    LCD_REFRESH_CNT
    CLRF    DESIRED_CURTAIN_F
    MOVLW   128             ; Set Light Threshold (approx mid-range)
    MOVWF   LIGHT_THRESHOLD
    
    ; Initialize Dummy Data for API testing
    MOVLW   25
    MOVWF   OUT_TEMP
    CLRF    OUT_TEMP_F
    MOVLW   0x03
    MOVWF   OUT_PRESS_H
    MOVLW   0xF5
    MOVWF   OUT_PRESS_L
    
    CALL    LCD_INIT        ; Initialize LCD Module
    CALL    UART_INIT       ; Initialize UART Module
    
    ; --- Initial Synchronization ---
    MOVLW   1
    CALL    READ_ADC        ; Read Potentiometer
    MOVWF   POT_VAL
    CALL    SCALE_TO_PERCENT; Convert to %
    MOVWF   DESIRED_CURTAIN
    MOVWF   CURRENT_CURTAIN
    MOVWF   OLD_POT_VAL     ; Store initial Pot value

;====================================================================
; MAIN CONTROL LOOP
;====================================================================
MAIN_LOOP:
    CLRWDT
    
    ; --- 1. SMART POTENTIOMETER CONTROL ---
    ; Logic: Only update Desired Curtain if Potentiometer MOVES.
    ; This allows PC commands (UART) to persist without being overwritten.
    MOVLW   1
    CALL    READ_ADC        ; Read Channel 1 (Pot)
    MOVWF   POT_VAL
    CALL    SCALE_TO_PERCENT
    MOVWF   TEMP_W          ; Store new percentage
    
    ; Detect Change: (New_Val - Old_Val)
    MOVF    OLD_POT_VAL, W
    SUBWF   TEMP_W, W
    BTFSC   STATUS, 2       ; If Zero flag set, No Change
    GOTO    SKIP_POT_UPDATE
    
    ; Change Detected -> Update Target
    MOVF    TEMP_W, W
    MOVWF   DESIRED_CURTAIN
    MOVWF   OLD_POT_VAL     ; Update history
    
SKIP_POT_UPDATE:

    ; --- 2. LDR (LIGHT SENSOR) CONTROL ---
    ; Logic: If Light < Threshold (Dark), Force Close Curtain.
    MOVLW   0
    CALL    READ_ADC        ; Read Channel 0 (LDR)
    MOVWF   LDR_VAL
    MOVF    LDR_VAL, W
    SUBWF   LIGHT_THRESHOLD, W
    BTFSC   STATUS, 0       ; If Threshold >= LDR (Dark)
    CALL    FORCE_CLOSE     ; Override target to 100% (Closed)

    ; --- 3. UART (PC CONTROL) ---
    ; Check for incoming commands from PC
    CALL    UART_TASK

    ; --- 4. LCD UPDATE ---
    INCF    LCD_REFRESH_CNT, F
    MOVLW   50              ; Refresh every 50 loops to prevent flicker
    SUBWF   LCD_REFRESH_CNT, W
    BTFSC   STATUS, 2
    CALL    UPDATE_LCD_SCREEN

    ; --- 5. STEPPER MOTOR CONTROL ---
    ; Compare Current Position vs Desired Position
    MOVF    CURRENT_CURTAIN, W
    SUBWF   DESIRED_CURTAIN, W
    BTFSC   STATUS, 2       ; If Equal, Stop Motor
    GOTO    RESET_SUB_STEP
    
    BTFSS   STATUS, 0       ; If Current > Desired -> Carry Clear -> Open
    GOTO    DO_STEP_OPEN
    GOTO    DO_STEP_CLOSE   ; If Current < Desired -> Carry Set -> Close

RESET_SUB_STEP:
    MOVLW   10              ; Reset Speed Counter
    MOVWF   SUB_STEP_CNT
    GOTO    MAIN_LOOP

FORCE_CLOSE:
    MOVLW   100             ; Set Target to 100% (Closed)
    MOVWF   DESIRED_CURTAIN
    RETURN

;====================================================================
; MOTOR MOVEMENT SUBROUTINES
;====================================================================
DO_STEP_CLOSE: 
    ; Rotate Motor in Closing Direction
    DECF    STEP_INDEX, F   ; Decrement Index
    MOVF    STEP_INDEX, W
    CALL    GET_STEP_CODE   ; Get Coil Pattern
    MOVWF   PORTB           ; Output to Motor
    CALL    DELAY_STEP      ; Wait
    DECFSZ  SUB_STEP_CNT, F ; Speed Control
    GOTO    MAIN_LOOP
    INCF    CURRENT_CURTAIN, F ; Increment Position %
    MOVLW   10
    MOVWF   SUB_STEP_CNT
    GOTO    MAIN_LOOP

DO_STEP_OPEN: 
    ; Rotate Motor in Opening Direction
    INCF    STEP_INDEX, F   ; Increment Index
    MOVF    STEP_INDEX, W
    CALL    GET_STEP_CODE   ; Get Coil Pattern
    MOVWF   PORTB           ; Output to Motor
    CALL    DELAY_STEP      ; Wait
    DECFSZ  SUB_STEP_CNT, F ; Speed Control
    GOTO    MAIN_LOOP
    DECF    CURRENT_CURTAIN, F ; Decrement Position %
    MOVLW   10
    MOVWF   SUB_STEP_CNT
    GOTO    MAIN_LOOP

;====================================================================
; UART MODULE (9600 BAUD @ 4MHZ)
;====================================================================
UART_INIT:
    BANKSEL SPBRG
    MOVLW   25              ; 9600 Baud at 4MHz
    MOVWF   SPBRG
    BANKSEL TXSTA
    MOVLW   0x24            ; TX Enable, High Speed
    MOVWF   TXSTA
    BANKSEL RCSTA
    MOVLW   0x90            ; Serial Enable, RX Enable
    MOVWF   RCSTA
    BANKSEL PORTA
    RETURN

UART_TASK:
    ; Error Handling
    BANKSEL RCSTA
    BTFSS   RCSTA, 1        ; Check OERR
    GOTO    CHECK_RX
    BCF     RCSTA, 4        ; Clear Error
    BSF     RCSTA, 4
CHECK_RX:
    BANKSEL PIR1
    BTFSS   PIR1, 5         ; Check for Data
    RETURN
    BANKSEL RCREG
    MOVF    RCREG, W        ; Read Data
    MOVWF   RX_DATA
    BANKSEL PORTA
    
    ; Protocol Parser
    BTFSC   RX_DATA, 7      ; Check MSB for SET Command
    GOTO    PROCESS_SET
    
    ; GET Command Handlers
    MOVF    RX_DATA, W
    XORLW   0x01
    BTFSC   STATUS, 2
    GOTO    SEND_DES_FRAC
    MOVF    RX_DATA, W
    XORLW   0x02
    BTFSC   STATUS, 2
    GOTO    SEND_DES_INT
    MOVF    RX_DATA, W
    XORLW   0x03
    BTFSC   STATUS, 2
    GOTO    SEND_TEMP_FRAC
    MOVF    RX_DATA, W
    XORLW   0x04
    BTFSC   STATUS, 2
    GOTO    SEND_TEMP_INT
    MOVF    RX_DATA, W
    XORLW   0x06
    BTFSC   STATUS, 2
    GOTO    SEND_PRESS_H
    MOVF    RX_DATA, W
    XORLW   0x10
    BTFSC   STATUS, 2
    GOTO    SEND_LIGHT_H
    RETURN

PROCESS_SET:
    BTFSS   RX_DATA, 6      ; Validate Protocol
    RETURN
    MOVF    RX_DATA, W
    ANDLW   0x3F            ; Extract Value
    MOVWF   DESIRED_CURTAIN ; Update Target
    ; Note: OLD_POT_VAL is NOT updated here.
    ; This ensures Potentiometer doesn't overwrite PC command
    ; unless physically moved.
    RETURN

; --- Transmission Handlers ---
SEND_DES_INT:
    MOVF    DESIRED_CURTAIN, W
    GOTO    DO_TX
SEND_DES_FRAC:
    MOVF    DESIRED_CURTAIN_F, W
    GOTO    DO_TX
SEND_TEMP_INT:
    MOVF    OUT_TEMP, W
    GOTO    DO_TX
SEND_TEMP_FRAC:
    MOVF    OUT_TEMP_F, W
    GOTO    DO_TX
SEND_PRESS_H:
    MOVF    OUT_PRESS_H, W
    GOTO    DO_TX
SEND_LIGHT_H:
    MOVF    LDR_VAL, W
    GOTO    DO_TX

DO_TX:
    MOVWF   TX_DATA
    BANKSEL TXSTA
WAIT_TX:
    BTFSS   TXSTA, 1        ; Buffer Empty?
    GOTO    WAIT_TX
    BANKSEL TXREG
    MOVF    TX_DATA, W
    MOVWF   TXREG           ; Send Data
    BANKSEL PORTA
    RETURN

;====================================================================
; LCD & HELPER FUNCTIONS
;====================================================================
UPDATE_LCD_SCREEN:
    CLRF    LCD_REFRESH_CNT
    
    MOVLW   0x80            ; Line 1
    CALL    LCD_CMD
    ; Printing Static Text and Values...
    MOVLW   '+'
    CALL    LCD_DATA
    MOVLW   '2'
    CALL    LCD_DATA
    MOVLW   '5'
    CALL    LCD_DATA
    MOVLW   '.'
    CALL    LCD_DATA
    MOVLW   '0'
    CALL    LCD_DATA
    MOVLW   0xDF            ; Degree Symbol
    CALL    LCD_DATA
    MOVLW   'C'
    CALL    LCD_DATA
    MOVLW   ' '
    CALL    LCD_DATA
    MOVLW   ' '
    CALL    LCD_DATA
    MOVLW   '1'
    CALL    LCD_DATA
    MOVLW   '0'
    CALL    LCD_DATA
    MOVLW   '1'
    CALL    LCD_DATA
    MOVLW   '3'
    CALL    LCD_DATA
    MOVLW   'h'
    CALL    LCD_DATA
    MOVLW   'P'
    CALL    LCD_DATA
    
    MOVLW   0xC0            ; Line 2
    CALL    LCD_CMD
    
    ; Display Lux Value
    MOVF    LDR_VAL, W
    MOVWF   TEMP_W
    CALL    BIN_TO_BCD
    MOVLW   '0'
    CALL    LCD_DATA
    MOVLW   '0'
    CALL    LCD_DATA
    MOVF    BCD_HUNDREDS, W
    ADDLW   '0'
    CALL    LCD_DATA
    MOVF    BCD_TENS, W
    ADDLW   '0'
    CALL    LCD_DATA
    MOVF    BCD_ONES, W
    ADDLW   '0'
    CALL    LCD_DATA
    MOVLW   'L'
    CALL    LCD_DATA
    MOVLW   'u'
    CALL    LCD_DATA
    MOVLW   'x'
    CALL    LCD_DATA
    MOVLW   ' '
    CALL    LCD_DATA
    
    ; Display Curtain Percentage
    MOVF    CURRENT_CURTAIN, W
    MOVWF   TEMP_W
    CALL    BIN_TO_BCD
    MOVF    BCD_HUNDREDS, W
    ADDLW   '0'
    CALL    LCD_DATA
    MOVF    BCD_TENS, W
    ADDLW   '0'
    CALL    LCD_DATA
    MOVF    BCD_ONES, W
    ADDLW   '0'
    CALL    LCD_DATA
    MOVLW   '.'
    CALL    LCD_DATA
    MOVLW   '0'
    CALL    LCD_DATA
    MOVLW   '%'
    CALL    LCD_DATA
    RETURN

LCD_INIT:
    ; Standard 4-bit Initialization Sequence
    CALL    DELAY_MS_20
    BCF     PORTE, 0
    BCF     PORTE, 1
    MOVLW   0x30
    MOVWF   PORTD
    BSF     PORTE, 2
    NOP
    BCF     PORTE, 2
    CALL    DELAY_MS_5
    BSF     PORTE, 2
    NOP
    BCF     PORTE, 2
    CALL    DELAY_100US
    BSF     PORTE, 2
    NOP
    BCF     PORTE, 2
    CALL    DELAY_100US
    MOVLW   0x20            ; Set to 4-bit mode
    MOVWF   PORTD
    BSF     PORTE, 2
    NOP
    BCF     PORTE, 2
    CALL    DELAY_MS_5
    MOVLW   0x28            ; Function Set
    CALL    LCD_CMD
    MOVLW   0x0C            ; Display ON, Cursor OFF
    CALL    LCD_CMD
    MOVLW   0x01            ; Clear Display
    CALL    LCD_CMD
    CALL    DELAY_MS_5
    MOVLW   0x06            ; Entry Mode Set
    CALL    LCD_CMD
    RETURN

LCD_CMD:
    MOVWF   LCD_TEMP
    BCF     PORTE, 0        ; RS = 0 (Command)
    GOTO    LCD_WRITE_NIBBLES
LCD_DATA:
    MOVWF   LCD_TEMP
    BSF     PORTE, 0        ; RS = 1 (Data)
    GOTO    LCD_WRITE_NIBBLES

LCD_WRITE_NIBBLES:
    ; Sends High Nibble then Low Nibble
    BCF     PORTE, 1        ; RW = 0
    MOVF    LCD_TEMP, W
    ANDLW   0xF0
    MOVWF   PORTD
    BSF     PORTE, 2        ; Pulse E
    NOP
    BCF     PORTE, 2
    SWAPF   LCD_TEMP, W
    ANDLW   0xF0
    MOVWF   PORTD
    BSF     PORTE, 2        ; Pulse E
    NOP
    BCF     PORTE, 2
    CALL    DELAY_100US
    RETURN

BIN_TO_BCD:
    ; Double-Dabble algorithm or simple subtraction
    CLRF    BCD_HUNDREDS
    CLRF    BCD_TENS
    MOVF    TEMP_W, W
    MOVWF   BCD_ONES
CHK_100:
    MOVLW   100
    SUBWF   BCD_ONES, W
    BTFSS   STATUS, 0
    GOTO    CHK_10
    MOVLW   100
    SUBWF   BCD_ONES, F
    INCF    BCD_HUNDREDS, F
    GOTO    CHK_100
CHK_10:
    MOVLW   10
    SUBWF   BCD_ONES, W
    BTFSS   STATUS, 0
    GOTO    BCD_END
    MOVLW   10
    SUBWF   BCD_ONES, F
    INCF    BCD_TENS, F
    GOTO    CHK_10
BCD_END:
    RETURN

READ_ADC:
    ; Reads ADC channel specified in W
    BANKSEL ADCON0
    MOVWF   TEMP_W
    BCF     ADCON0, 3       ; Select Channel Bits (CHS0-2)
    BCF     ADCON0, 4
    BCF     ADCON0, 5
    BTFSC   TEMP_W, 0
    BSF     ADCON0, 3
    NOP
    NOP
    BSF     ADCON0, 2       ; Start Conversion
WAIT_ADC:
    BTFSC   ADCON0, 2
    GOTO    WAIT_ADC
    MOVF    ADRESH, W       ; Read Result (Upper 8 bits)
    BANKSEL PORTA
    RETURN

SCALE_TO_PERCENT:
    ; Scales ADC (0-255) to Percent (0-100)
    MOVWF   MATH_L
    CLRF    MATH_H
    BCF     STATUS, 0
    RLF     MATH_L, F
    RLF     MATH_H, F
    CLRF    TEMP_W
DIV_L:
    MOVLW   5
    SUBWF   MATH_L, F
    BTFSC   STATUS, 0
    GOTO    NO_B
    MOVF    MATH_H, F
    BTFSC   STATUS, 2
    GOTO    RET_SCALE
    DECF    MATH_H, F
NO_B:
    INCF    TEMP_W, F
    GOTO    DIV_L
RET_SCALE:
    MOVLW   100
    SUBWF   TEMP_W, W
    BTFSC   STATUS, 0
    MOVLW   100
    BTFSC   STATUS, 0
    MOVWF   TEMP_W
    MOVF    TEMP_W, W
    RETURN

DELAY_STEP:
    ; Delay for Stepper Motor Speed
    MOVLW   10
    MOVWF   LOOP_VAR1
D_OUT:
    MOVLW   50
    MOVWF   LOOP_VAR2
D_IN:
    CLRWDT
    DECFSZ  LOOP_VAR2, F
    GOTO    D_IN
    DECFSZ  LOOP_VAR1, F
    GOTO    D_OUT
    RETURN

DELAY_MS_20:
    MOVLW   40
    MOVWF   LOOP_VAR1
    GOTO    DELAY_LOOP
DELAY_MS_5:
    MOVLW   10
    MOVWF   LOOP_VAR1
    GOTO    DELAY_LOOP

DELAY_LOOP:
    MOVLW   250
    MOVWF   LOOP_VAR2
INNER_DELAY:
    DECFSZ  LOOP_VAR2, F
    GOTO    INNER_DELAY
    DECFSZ  LOOP_VAR1, F
    GOTO    DELAY_LOOP
    RETURN

DELAY_100US:
    MOVLW   30
    MOVWF   LOOP_VAR1
INNER_100US:
    DECFSZ  LOOP_VAR1, F
    GOTO    INNER_100US
    RETURN

    END