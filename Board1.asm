;====================================================================
; PROJECT: Home Automation System - Board #1 (Air Conditioner)
; AUTHOR:  [Emirhan Ünal]
; STUDENT ID: [151120221064]
; DATE:    [30.12.2025]
;
; DESCRIPTION:
; This program controls an Air Conditioner system using PIC16F877A.
; Features:
; 1. Reads Ambient Temperature from LM35 (AN0).
; 2. Reads Fan Speed via Tachometer (RC0) using Timer1 counter.
; 3. Controls Cooler (RC2) and Heater (RC5) based on desired temp.
; 4. Implements a UART Communication Protocol (9600 Baud) to exchange
;    data with a PC application.
; 5. Drives a 7-Segment Display to show system status.
;====================================================================

    PROCESSOR 16F877A
    #include <xc.inc>

    ; --- Configuration Bits (4 MHz Crystal) ---
    CONFIG FOSC = XT        ; Oscillator Selection: XT (Crystal/Resonator)
    CONFIG WDTE = OFF       ; Watchdog Timer: Disabled
    CONFIG PWRTE = ON       ; Power-up Timer: Enabled
    CONFIG BOREN = OFF      ; Brown-out Reset: Disabled
    CONFIG LVP = OFF        ; Low Voltage Programming: Disabled
    CONFIG CPD = OFF        ; Data EEPROM Memory Code Protection: Disabled
    CONFIG WRT = OFF        ; Flash Program Memory Write Enable: Disabled
    CONFIG CP = OFF         ; Flash Program Memory Code Protection: Disabled

;====================================================================
; VARIABLES (UDATA BANK0)
;====================================================================
    PSECT udata_bank0
    
    DESIRED_TEMP:   DS 1    ; Target temperature set by Keypad or PC
    AMBIENT_TEMP:   DS 1    ; Current temperature read from LM35
    FAN_SPEED:      DS 1    ; Fan speed measured by Timer1 (Tachometer)

    ; --- Display Variables ---
    DISPLAY_VAL:    DS 1    ; Value to be displayed on 7-Seg
    ONES:           DS 1    ; BCD Digit (Ones)
    TENS:           DS 1    ; BCD Digit (Tens)
    HUNDREDS:       DS 1    ; BCD Digit (Hundreds)
    
    ; --- Keypad Variables ---
    INPUT_INT:      DS 1    ; Buffer for keypad number entry
    KEY_PRESSED:    DS 1    ; Stores the pressed key code
    TEMP_MATH:      DS 1    ; Temporary variable for math operations
    COL_VAR:        DS 1    ; Column scanning variable
    
    ; --- General Purpose ---
    LOOP_VAR:       DS 1    ; Loop counter for delays
    SEC_COUNTER:    DS 1    ; Counter for timing loops
    SEC_MULTIPLIER: DS 1    ; Multiplier for longer delays
    STATE_MODE:     DS 1    ; State machine for display (Temp, Desired, Fan)
    TEMP_W:         DS 1    ; Temporary W register storage
    ADC_RAW:        DS 1    ; Raw ADC result

    ; --- UART Communication ---
    RX_DATA:        DS 1    ; Received byte from UART
    TX_DATA:        DS 1    ; Byte to transmit via UART
    DESIRED_TEMP_F: DS 1    ; Fractional part (for protocol compatibility)
    AMBIENT_TEMP_F: DS 1    ; Fractional part (for protocol compatibility)

;====================================================================
; RESET VECTOR
;====================================================================
    PSECT code, delta=2
    GOTO MAIN

;====================================================================
; LOOKUP TABLE: 7-Segment Decoding (Common Cathode)
;====================================================================
GET_SEG_CODE:
    ANDLW   0x0F            ; Ensure index is 0-15
    MOVWF   TEMP_W
    MOVLW   high(TABLE_START)
    MOVWF   PCLATH
    MOVF    TEMP_W, W
    ADDWF   PCL, F

TABLE_START:
    RETLW   0x3F ; 0
    RETLW   0x06 ; 1
    RETLW   0x5B ; 2
    RETLW   0x4F ; 3
    RETLW   0x66 ; 4
    RETLW   0x6D ; 5
    RETLW   0x7D ; 6
    RETLW   0x07 ; 7
    RETLW   0x7F ; 8
    RETLW   0x6F ; 9
    RETLW   0x00 ; Blank
    RETURN

;====================================================================
; MAIN PROGRAM
;====================================================================
MAIN:
    ; --- Port Configuration ---
    BANKSEL ADCON1
    MOVLW   0x8E            ; Configure AN0 as Analog, others Digital
    MOVWF   ADCON1
    
    BANKSEL TRISA
    MOVLW   0x01            ; Set RA0 as Input (LM35)
    MOVWF   TRISA
    
    BANKSEL TRISB
    MOVLW   0xF0            ; Set RB4-RB7 as Input (Rows), RB0-RB3 as Output (Cols)
    MOVWF   TRISB
    
    BANKSEL OPTION_REG
    BCF     OPTION_REG, 7   ; Enable PortB Weak Pull-ups
    
    ; --- UART & Tachometer Pin Configuration ---
    ; RC7 (RX) = 1 (Input)
    ; RC6 (TX) = 0 (Output)
    ; RC0 (Tach)= 1 (Input) for Timer1 Counting
    BANKSEL TRISC
    MOVLW   0x81            ; Binary: 1000 0001
    MOVWF   TRISC
    
    BANKSEL TRISD
    CLRF    TRISD           ; Set PortD as Output (7-Segment Data)

    BANKSEL PORTA
    CLRF    PORTA
    CLRF    PORTB
    CLRF    PORTC
    CLRF    PORTD

    ; --- Module Initialization ---
    CALL    ADC_INIT        ; Initialize ADC Module
    CALL    TIMER1_INIT     ; Initialize Timer1 for Fan Speed
    CALL    UART_INIT       ; Initialize UART (9600 Baud)

    ; --- Initial Values ---
    MOVLW   30
    MOVWF   DESIRED_TEMP    ; Default Desired Temp: 30 C
    CLRF    DESIRED_TEMP_F
    CLRF    AMBIENT_TEMP_F
    
    CLRF    STATE_MODE
    CLRF    SEC_COUNTER
    CLRF    SEC_MULTIPLIER

;====================================================================
; MAIN CONTROL LOOP
;====================================================================
MAIN_LOOP:
    ; 1. Check Keypad for 'A' (Enter Setup Mode)
    CALL    SCAN_KEYPAD
    SUBLW   0x0A            ; 0x0A represents 'A' key
    BTFSC   STATUS, 2       ; If Zero flag is set, A was pressed
    GOTO    INPUT_MODE_START

    ; 2. Check UART for Incoming PC Commands
    CALL    UART_CHECK
    
    ; 3. Read Sensors and Update Control Logic
    CALL    READ_SENSORS    ; Read LM35
    CALL    CONTROL_LOGIC   ; Decide Cooler/Heater state

    ; 4. Update Display Multiplexing
    MOVF    STATE_MODE, W
    SUBLW   0
    BTFSC   STATUS, 2
    GOTO    SHOW_DESIRED    ; State 0: Show Desired Temp
    
    MOVF    STATE_MODE, W
    SUBLW   1
    BTFSC   STATUS, 2
    GOTO    SHOW_AMBIENT    ; State 1: Show Ambient Temp
    
    GOTO    SHOW_FAN        ; State 2: Show Fan Speed

SHOW_DESIRED:
    MOVF    DESIRED_TEMP, W
    MOVWF   DISPLAY_VAL
    GOTO    REFRESH_SCREEN

SHOW_AMBIENT:
    MOVF    AMBIENT_TEMP, W
    MOVWF   DISPLAY_VAL
    GOTO    REFRESH_SCREEN

SHOW_FAN:
    MOVF    FAN_SPEED, W
    MOVWF   DISPLAY_VAL

REFRESH_SCREEN:
    CALL    BIN_TO_BCD      ; Convert Binary to BCD for Display
    CALL    DISPLAY_MUX     ; Drive 7-Segment Displays
    
    ; --- Timing and State Switching ---
    INCFSZ  SEC_COUNTER, F
    GOTO    MAIN_LOOP
    INCF    SEC_MULTIPLIER, F
    MOVLW   5
    SUBWF   SEC_MULTIPLIER, W
    BTFSS   STATUS, 2
    GOTO    MAIN_LOOP
    CLRF    SEC_MULTIPLIER
    
    ; --- Calculate Fan Speed from Timer1 ---
    MOVF    TMR1L, W        ; Read Timer1 Low Byte (Count of pulses)
    MOVWF   FAN_SPEED       ; Store as Fan Speed
    CLRF    TMR1L           ; Reset Timer
    CLRF    TMR1H
    
    ; Switch Display State (0 -> 1 -> 2 -> 0)
    INCF    STATE_MODE, F
    MOVLW   3
    SUBWF   STATE_MODE, W
    BTFSC   STATUS, 2
    CLRF    STATE_MODE
    GOTO    MAIN_LOOP

;====================================================================
; KEYPAD INPUT MODE (Setting Temperature)
;====================================================================
INPUT_MODE_START:
    BCF     PORTC, 5        ; Turn OFF Heater (Safety)
    BCF     PORTC, 2        ; Turn OFF Cooler (Safety)
    CLRF    INPUT_INT       ; Clear input buffer
    
WAIT_RELEASE_A:
    CALL    SCAN_KEYPAD
    XORLW   0xFF            ; Check if key is released
    BTFSS   STATUS, 2
    GOTO    WAIT_RELEASE_A

INPUT_LOOP:
    ; Display current input value while typing
    MOVF    INPUT_INT, W
    MOVWF   DISPLAY_VAL
    CALL    BIN_TO_BCD
    CALL    DISPLAY_MUX
    
    CALL    UART_CHECK      ; Keep UART alive

    CALL    SCAN_KEYPAD
    MOVWF   KEY_PRESSED
    XORLW   0xFF
    BTFSC   STATUS, 2       ; No key pressed
    GOTO    INPUT_LOOP
    
    CALL    DELAY_MS        ; Debounce delay
    
    ; Check for '#' (Enter/Confirm)
    MOVF    KEY_PRESSED, W
    SUBLW   0x0F            ; 0x0F is '#'
    BTFSC   STATUS, 2
    GOTO    SAVE_AND_EXIT

    ; Check for Digits (0-9)
    MOVF    KEY_PRESSED, W
    SUBLW   9
    BTFSS   STATUS, 0       ; If Key > 9, ignore
    GOTO    WAIT_RELEASE_KEY
    
    ; Math: INPUT = (INPUT * 10) + KEY
    MOVF    INPUT_INT, W
    MOVWF   TEMP_MATH
    BCF     STATUS, 0
    RLF     INPUT_INT, F    ; *2
    MOVF    INPUT_INT, W
    MOVWF   TEMP_W
    BCF     STATUS, 0
    RLF     INPUT_INT, F    ; *4
    BCF     STATUS, 0
    RLF     INPUT_INT, F    ; *8
    MOVF    TEMP_W, W
    ADDWF   INPUT_INT, F    ; *8 + *2 = *10
    MOVF    KEY_PRESSED, W
    ADDWF   INPUT_INT, F    ; + Key
    
WAIT_RELEASE_KEY:
    ; Maintain display while waiting for key release
    MOVF    INPUT_INT, W
    MOVWF   DISPLAY_VAL
    CALL    BIN_TO_BCD
    CALL    DISPLAY_MUX
    CALL    SCAN_KEYPAD
    XORLW   0xFF
    BTFSS   STATUS, 2
    GOTO    WAIT_RELEASE_KEY
    GOTO    INPUT_LOOP

SAVE_AND_EXIT:
    ; Validate Input (Range 10 - 50)
    MOVLW   10
    SUBWF   INPUT_INT, W
    BTFSS   STATUS, 0       ; If Input < 10
    GOTO    EXIT_NO_SAVE
    MOVF    INPUT_INT, W
    SUBLW   50
    BTFSS   STATUS, 0       ; If Input > 50
    GOTO    EXIT_NO_SAVE
    
    MOVF    INPUT_INT, W    ; Valid input
    MOVWF   DESIRED_TEMP    ; Update Desired Temp
EXIT_NO_SAVE:
    GOTO    MAIN_LOOP

;====================================================================
; HELPER SUBROUTINES
;====================================================================
SCAN_KEYPAD:
    ; Scans 4x4 Keypad by grounding one column at a time
    
    ; Column 1 (1, 4, 7, *)
    BSF     PORTB, 0
    BSF     PORTB, 1
    BSF     PORTB, 2
    BSF     PORTB, 3
    
    BCF     PORTB, 0        ; Activate Col 1
    NOP
    NOP
    BTFSS   PORTB, 4        ; Check Row 1
    RETLW   1
    BTFSS   PORTB, 5        ; Check Row 2
    RETLW   4
    BTFSS   PORTB, 6        ; Check Row 3
    RETLW   7
    BTFSS   PORTB, 7        ; Check Row 4
    RETLW   0x0E            ; '*' Key
    BSF     PORTB, 0        ; Deactivate Col 1
    
    ; Column 2 (2, 5, 8, 0)
    BCF     PORTB, 1
    NOP
    NOP
    BTFSS   PORTB, 4
    RETLW   2
    BTFSS   PORTB, 5
    RETLW   5
    BTFSS   PORTB, 6
    RETLW   8
    BTFSS   PORTB, 7
    RETLW   0
    BSF     PORTB, 1
    
    ; Column 3 (3, 6, 9, #)
    BCF     PORTB, 2
    NOP
    NOP
    BTFSS   PORTB, 4
    RETLW   3
    BTFSS   PORTB, 5
    RETLW   6
    BTFSS   PORTB, 6
    RETLW   9
    BTFSS   PORTB, 7
    RETLW   0x0F            ; '#' Key
    BSF     PORTB, 2
    
    ; Column 4 (A, B, C, D)
    BCF     PORTB, 3
    NOP
    NOP
    BTFSS   PORTB, 4
    RETLW   0x0A            ; 'A' Key
    BSF     PORTB, 3
    RETLW   0xFF            ; No key pressed

DISPLAY_MUX:
    ; Multiplexes the 4 digits of the 7-Segment Display
    
    ; Digit 1 (Thousands/Mode) - Usually 0 or Mode
    MOVLW   0
    CALL    GET_SEG_CODE
    MOVWF   PORTD
    BSF     PORTA, 2        ; Select Digit 1
    CALL    DELAY_MS
    BCF     PORTA, 2
    
    ; Digit 2 (Hundreds)
    MOVF    HUNDREDS, W
    CALL    GET_SEG_CODE
    MOVWF   PORTD
    BSF     PORTA, 3        ; Select Digit 2
    CALL    DELAY_MS
    BCF     PORTA, 3
    
    ; Digit 3 (Tens)
    MOVF    TENS, W
    CALL    GET_SEG_CODE
    MOVWF   PORTD
    BSF     PORTA, 1        ; Select Digit 3
    CALL    DELAY_MS
    BCF     PORTA, 1
    
    ; Digit 4 (Ones)
    MOVF    ONES, W
    CALL    GET_SEG_CODE
    MOVWF   PORTD
    BSF     PORTA, 5        ; Select Digit 4
    CALL    DELAY_MS
    BCF     PORTA, 5
    RETURN

BIN_TO_BCD:
    ; Converts Binary (DISPLAY_VAL) to BCD (Hundreds, Tens, Ones)
    CLRF    HUNDREDS
    CLRF    TENS
    MOVF    DISPLAY_VAL, W
    MOVWF   ONES
CHECK_100:
    MOVLW   100
    SUBWF   ONES, W
    BTFSS   STATUS, 0
    GOTO    CHECK_10
    MOVLW   100
    SUBWF   ONES, F
    INCF    HUNDREDS, F
    GOTO    CHECK_100
CHECK_10:
    MOVLW   10
    SUBWF   ONES, W
    BTFSS   STATUS, 0
    GOTO    BCD_END
    MOVLW   10
    SUBWF   ONES, F
    INCF    TENS, F
    GOTO    CHECK_10
BCD_END:
    RETURN

DELAY_MS:
    ; Simple blocking delay
    MOVLW   200
    MOVWF   LOOP_VAR
D_LOOP:
    DECFSZ  LOOP_VAR, F
    GOTO    D_LOOP
    RETURN

READ_SENSORS:
    ; Reads LM35 sensor connected to AN0
    BANKSEL ADCON0
    BSF     ADCON0, 2       ; Start Conversion (GO/DONE=1)
WAIT_ADC:
    BTFSC   ADCON0, 2       ; Wait for conversion to finish
    GOTO    WAIT_ADC
    BANKSEL ADRESL
    MOVF    ADRESL, W       ; Read Lower 8 bits (approx. temp)
    BANKSEL PORTA
    MOVWF   ADC_RAW
    BCF     STATUS, 0
    RRF     ADC_RAW, W      ; Divide by 2 for scaling
    MOVWF   AMBIENT_TEMP
    MOVLW   5
    MOVWF   AMBIENT_TEMP_F  ; Fixed fractional part for demo
    RETURN

CONTROL_LOGIC:
    ; Simple ON/OFF Control (Hysteresis can be added)
    MOVF    AMBIENT_TEMP, W
    SUBWF   DESIRED_TEMP, W
    BTFSS   STATUS, 0       ; If Desired < Ambient -> Cooling Needed
    GOTO    COOLER_ON
    
    ; Heating Needed
    BSF     PORTC, 5        ; Turn ON Heater
    BCF     PORTC, 2        ; Turn OFF Cooler
    CLRF    FAN_SPEED       ; Fan is OFF
    RETURN
COOLER_ON:
    BCF     PORTC, 5        ; Turn OFF Heater
    BSF     PORTC, 2        ; Turn ON Cooler
    ; Fan Speed is determined by external Tachometer
    RETURN

ADC_INIT:
    ; Configure ADC module
    BANKSEL ADCON0
    MOVLW   0x41            ; ADC Enabled, Fosc/8
    MOVWF   ADCON0
    BANKSEL PORTA
    RETURN

TIMER1_INIT:
    ; Configure Timer1 to count external pulses on RC0
    BANKSEL T1CON
    MOVLW   0x07            ; External Clock (RC0), Timer1 ON, No Prescaler
    MOVWF   T1CON
    BANKSEL PORTA
    RETURN

;====================================================================
; UART MODULE (9600 BAUD @ 4MHZ)
;====================================================================
UART_INIT:
    BANKSEL SPBRG
    MOVLW   25              ; SPBRG = 25 for 9600 Baud at 4MHz
    MOVWF   SPBRG
    BANKSEL TXSTA
    MOVLW   0x24            ; TXEN=1 (Transmit Enable), BRGH=1 (High Speed)
    MOVWF   TXSTA
    BANKSEL RCSTA
    MOVLW   0x90            ; SPEN=1 (Serial Port Enable), CREN=1 (Receive Enable)
    MOVWF   RCSTA
    BANKSEL PORTA
    RETURN

UART_CHECK:
    ; Checks for UART Errors and Data
    BANKSEL RCSTA
    BTFSS   RCSTA, 1        ; Check for OERR (Overrun Error)
    GOTO    READ_DATA
    BCF     RCSTA, 4        ; Reset CREN to clear error
    BSF     RCSTA, 4
READ_DATA:
    BANKSEL PIR1
    BTFSS   PIR1, 5         ; Check RCIF (Receive Interrupt Flag)
    RETURN
    
    ; Read Data
    BANKSEL RCREG
    MOVF    RCREG, W
    MOVWF   RX_DATA
    BANKSEL PORTA
    
    ; Protocol Parser
    BTFSC   RX_DATA, 7      ; Check MSB. If 1, it is a SET command.
    GOTO    CMD_SET
    
    ; GET Command Handlers
    MOVF    RX_DATA, W
    XORLW   0x04
    BTFSC   STATUS, 2
    GOTO    TX_AMB_INT
    MOVF    RX_DATA, W
    XORLW   0x03
    BTFSC   STATUS, 2
    GOTO    TX_AMB_FRAC
    MOVF    RX_DATA, W
    XORLW   0x02
    BTFSC   STATUS, 2
    GOTO    TX_DES_INT
    MOVF    RX_DATA, W
    XORLW   0x01
    BTFSC   STATUS, 2
    GOTO    TX_DES_FRAC
    MOVF    RX_DATA, W
    XORLW   0x05
    BTFSC   STATUS, 2
    GOTO    TX_FAN
    RETURN

CMD_SET:
    ; Protocol: 11xxxxxx -> Set Desired Temperature
    MOVF    RX_DATA, W
    ANDLW   0x3F            ; Mask upper 2 bits
    MOVWF   DESIRED_TEMP    ; Update Desired Temperature
    RETURN

; --- Transmission Handlers ---
TX_AMB_INT:
    MOVF    AMBIENT_TEMP, W
    GOTO    DO_TX
TX_AMB_FRAC:
    MOVF    AMBIENT_TEMP_F, W
    GOTO    DO_TX
TX_DES_INT:
    MOVF    DESIRED_TEMP, W
    GOTO    DO_TX
TX_DES_FRAC:
    MOVF    DESIRED_TEMP_F, W
    GOTO    DO_TX
TX_FAN:
    MOVF    FAN_SPEED, W
    GOTO    DO_TX

DO_TX:
    ; Sends the value in W register via UART
    MOVWF   TX_DATA
    BANKSEL TXSTA
WAIT_TX:
    BTFSS   TXSTA, 1        ; Wait for TRMT (Transmit Shift Register Empty)
    GOTO    WAIT_TX
    BANKSEL TXREG
    MOVF    TX_DATA, W
    MOVWF   TXREG           ; Write data to transmit buffer
    BANKSEL PORTA
    RETURN

    END