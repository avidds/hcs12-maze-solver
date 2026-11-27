;*****************************************************************
;* COE538 Project: LCD Updating & Motor Control Base             *
;* Target: MC9S12C32 (Absolute Assembly)                         *
;*****************************************************************

; export symbols
              XDEF  Entry, _Startup           ; Export entry symbols
              ABSENTRY Entry                  ; Mark application entry point

; Include derivative-specific definitions
              INCLUDE 'derivative.inc'

ROMStart      EQU   $4000                     ; Absolute address for code/constant data

;-----------------------------------------------------------------
; Equates Section
;-----------------------------------------------------------------
; LCD pins & commands (4-bit mode, 2-line display)
LCD_DAT       EQU   PORTB                     ; LCD data port (PB7..PB0)
LCD_CNTR      EQU   PTJ                       ; LCD control port (PJ7=E, PJ6=RS)
LCD_E         EQU   $80                       ; LCD Enable pin mask (Bit 7)
LCD_RS        EQU   $40                       ; LCD Register Select mask (Bit 6)

LCD_CLEAR     EQU   $01                       ; Clear screen & home cursor
LCD_4BIT_2LN  EQU   $28                       ; 4-bit bus, 2-line display
LCD_DISP_ON   EQU   $0C                       ; Display on, cursor off
LCD_ENTRY_INC EQU   $06                       ; Auto-increment cursor right
LCD_LINE1     EQU   $80                       ; Start of Row 1
LCD_LINE2     EQU   $C0                       ; Start of Row 2
LCD_SEC_LINE  EQU   64                        ; Starting addr. of 2nd line (decimal, for LCD_POS_CRSR)
DISP_INT      EQU   5                         ; Update LCD every 5 TOF ticks (~217ms @ 23Hz)
NULL          EQU   $00                       ; Null terminator

; Motor bit masks (PA1..0 = direction, PT5..4 = enable)
PORT_DIR_BIT  EQU   %00000001                 ; PA0: Port Dir (0 = FWD, 1 = REV)
STBD_DIR_BIT  EQU   %00000010                 ; PA1: Starboard Dir (0 = FWD, 1 = REV)
BOTH_DIR_BITS EQU   %00000011                 ; Both motor direction pins

PORT_SPD_BIT  EQU   %00010000                 ; PT4: Port Speed/On (1 = ON, 0 = OFF)
STBD_SPD_BIT  EQU   %00100000                 ; PT5: Starboard Speed/On (1 = ON, 0 = OFF)
BOTH_SPD_BITS EQU   %00110000                 ; Both motor enable pins

; Robot states (indices into 'tab' lookup table)
START         EQU   0
FWD           EQU   1
ALL_STP       EQU   2
LEFT_TRN      EQU   3
RIGHT_TRN     EQU   4
REV_TRN       EQU   5
LEFT_ALN      EQU   6
RIGHT_ALN     EQU   7

;-----------------------------------------------------------------
; Variable / Data Section (RAM)
;-----------------------------------------------------------------
              ORG   RAMStart

TOF_COUNTER   DS.B  1                         ; 23Hz hardware timer overflow counter
CRNT_STATE    DS.B  1                         ; Current state index (0..7)
LINE_STATUS   DS.B  1                         ; Interpreted sensor readout: 000(E/F)ABCD

TEN_THOUS     DS.B  1                         ; BCD/ASCII 10,000s digit (10s of Volts)
THOUSANDS     DS.B  1                         ; BCD/ASCII 1,000s digit (1s of Volts)
HUNDREDS      DS.B  1                         ; BCD/ASCII 100s digit (0.1s of Volts)
TENS          DS.B  1                         ; BCD/ASCII 10s digit
UNITS         DS.B  1                         ; BCD/ASCII 1s digit
NO_BLANK      DS.B  1                         ; Zero-suppression flag for BCD2ASC
BCD_SPARE     DS.B  2                         ; Padding buffer for int2BCD routine

LAST_DISP_TOF DS.B  1                         ; Tracks last TOF count we updated the LCD at
ALIVE_FLAG    DS.B  1                         ; Flips between 0 and 1 to blink the '*' indicator

;-----------------------------------------------------------------
; Code Section (ROM)
;-----------------------------------------------------------------
              ORG   ROMStart

Entry:
_Startup:
              LDS   #RAMEnd+1                 ; Initialize stack pointer to top of RAM

              JSR   INIT_MOTORS               ; Set up motor direction/speed pins (start OFF)
              JSR   initAD                    ; Configure A/D and digital bumper pins
              JSR   initLCD                   ; Configure LCD in 4-bit, 2-line mode
              JSR   INIT_LCD_UI               ; Print static labels on Row 1 and Row 2

              CLR   TOF_COUNTER               ; Start timer count at 0
              CLR   LINE_STATUS               ; Clear line status to known state
              MOVB  #ALL_STP,CRNT_STATE       ; Default to stopped state on reset

              JSR   ENABLE_TOF                ; Start 23Hz Timer Overflow Interrupt
              CLI                             ; Enable maskable global interrupts

mainLoop:
              JSR   UPDT_DISPL                ; Refresh battery voltage, heartbeat, state, and L:XX

              ; TODO: call guider sensor read + FSM dispatcher here
              ; Available motor calls can do:
              ;   MTR_STOP, MTR_FWD, MTR_REV
              ;   MTR_STEER_L, MTR_STEER_R, MTR_PIVOT_L, MTR_PIVOT_R
              ;   STARON/OFF, PORTON/OFF, STARFWD/REV, PORTFWD/REV

              BRA   mainLoop                  ; Repeat main control loop

;-----------------------------------------------------------------
; Constant Strings & Lookup Tables
;-----------------------------------------------------------------
msg1          DC.B  "Battery volt ",0         ; 13 chars + null terminator
msg2          DC.B  "State ",0                ; 6 chars + null terminator
msg_line      DC.B  " L:",0                   ; Label for LINE_STATUS hex byte
HEX_TABLE     FCC   '0123456789ABCDEF'        ; Table for converting hex nibbles to ASCII

; Each table entry is exactly 8 bytes (7 characters + null terminator)
tab           DC.B  "START  ",0               ; 0
              DC.B  "FWD    ",0               ; 1
              DC.B  "ALL_STP",0               ; 2
              DC.B  "LFT_TRN",0               ; 3
              DC.B  "RGT_TRN",0               ; 4
              DC.B  "REV_TRN",0               ; 5
              DC.B  "LFT_ALN",0               ; 6
              DC.B  "RGT_ALN",0               ; 7

;-----------------------------------------------------------------
; Motor Control Subroutines
; Only touches PA1..PA0 and PT5..PT4 so guider bits PA5..PA2 stay safe
;-----------------------------------------------------------------
INIT_MOTORS   BSET  DDRA,BOTH_DIR_BITS        ; PA1 (Starboard Dir), PA0 (Port Dir) as outputs
              BSET  DDRT,BOTH_SPD_BITS        ; PT5 (Starboard Spd), PT4 (Port Spd) as outputs
              BCLR  PTT,BOTH_SPD_BITS         ; Make sure both motors start OFF
              BCLR  PORTA,BOTH_DIR_BITS       ; Default both directions to forward
              RTS

; Starboard Motor Speed Control (PT5: 1 = ON, 0 = OFF)
STARON        BSET  PTT,STBD_SPD_BIT          ; Set bit 5 (PT5 = 1)
              RTS

STAROFF       BCLR  PTT,STBD_SPD_BIT          ; Clear bit 5 (PT5 = 0)
              RTS

; Port Motor Speed Control (PT4: 1 = ON, 0 = OFF)
PORTON        BSET  PTT,PORT_SPD_BIT          ; Set bit 4 (PT4 = 1)
              RTS

PORTOFF       BCLR  PTT,PORT_SPD_BIT          ; Clear bit 4 (PT4 = 0)
              RTS

; Starboard Motor Direction Control (PA1: 0 = FWD, 1 = REV)
STARFWD       BCLR  PORTA,STBD_DIR_BIT        ; Clear bit 1 (PA1 = 0)
              RTS

STARREV       BSET  PORTA,STBD_DIR_BIT        ; Set bit 1 (PA1 = 1)
              RTS

; Port Motor Direction Control (PA0: 0 = FWD, 1 = REV)
PORTFWD       BCLR  PORTA,PORT_DIR_BIT        ; Clear bit 0 (PA0 = 0)
              RTS

PORTREV       BSET  PORTA,PORT_DIR_BIT        ; Set bit 0 (PA0 = 1)
              RTS

; Combined Drive & Turning Helpers
MTR_STOP      BCLR  PTT,BOTH_SPD_BITS         ; Both motors OFF (PT5=0, PT4=0)
              RTS

MTR_FWD       BCLR  PORTA,BOTH_DIR_BITS       ; Both motors forward (PA1=0, PA0=0)
              BSET  PTT,BOTH_SPD_BITS         ; Both motors ON (PT5=1, PT4=1)
              RTS

MTR_REV       BSET  PORTA,BOTH_DIR_BITS       ; Both motors reverse (PA1=1, PA0=1)
              BSET  PTT,BOTH_SPD_BITS         ; Both motors ON (PT5=1, PT4=1)
              RTS

; Soft left steer for line tracking (cut left wheel, keep right wheel driving)
MTR_STEER_L   BCLR  PORTA,BOTH_DIR_BITS       ; Both set to FWD
              BCLR  PTT,PORT_SPD_BIT          ; Port OFF
              BSET  PTT,STBD_SPD_BIT          ; Starboard ON
              RTS

; Soft right steer for line tracking (cut right wheel, keep left wheel driving)
MTR_STEER_R   BCLR  PORTA,BOTH_DIR_BITS       ; Both set to FWD
              BSET  PTT,PORT_SPD_BIT          ; Port ON
              BCLR  PTT,STBD_SPD_BIT          ; Starboard OFF
              RTS

; In-place CCW pivot for 90 deg left turns
MTR_PIVOT_L   BSET  PORTA,PORT_DIR_BIT        ; Port REV (PA0=1)
              BCLR  PORTA,STBD_DIR_BIT        ; Starboard FWD (PA1=0)
              BSET  PTT,BOTH_SPD_BITS         ; Both motors ON
              RTS

; In-place CW pivot for 90 deg right / 180 deg reverse turns
MTR_PIVOT_R   BCLR  PORTA,PORT_DIR_BIT        ; Port FWD (PA0=0)
              BSET  PORTA,STBD_DIR_BIT        ; Starboard REV (PA1=1)
              BSET  PTT,BOTH_SPD_BITS         ; Both motors ON
              RTS

;-----------------------------------------------------------------
; Display Update Subroutines (Battery Voltage, Heartbeat, State, Line)
;-----------------------------------------------------------------
; Print static text once at startup so UPDT_DISPL only overwrites the values
INIT_LCD_UI   CLR   LAST_DISP_TOF
              CLR   ALIVE_FLAG
              JSR   clrLCD                    ; Clear LCD screen
              LDX   #msg1                     ; Print static label on Row 1
              JSR   putsLCD
              LDAA  #LCD_LINE2                ; Move cursor to start of Row 2
              JSR   cmd2LCD
              LDX   #msg2                     ; Print static label on Row 2
              JSR   putsLCD
              LDAA  #$CD                      ; Row 2, col 13: print " L:"
              JSR   cmd2LCD
              LDX   #msg_line
              JSR   putsLCD
              RTS

; Throttled display refresh so LCD writes don't bottleneck the main loop
UPDT_DISPL    LDAA  TOF_COUNTER               ; Check if DISP_INT ticks have passed
              SUBA  LAST_DISP_TOF             ; Unsigned sub handles 255->0 wraparound
              CMPA  #DISP_INT
              BLO   UPDT_EXIT                 ; Not time yet, skip update
              LDAA  TOF_COUNTER
              STAA  LAST_DISP_TOF

              ; Read battery voltage on AN0 (single channel so we don't mess with AN1)
              MOVB  #$80,ATDCTL5              ; Right-justified, unsigned, single ch = AN0
              BRCLR ATDSTAT0,$80,*            ; Wait for conversion sequence to finish
              LDAA  ATDDR0L                   ; Read battery sense channel (AN0)
              LDAB  #39                       ; Fixed-point scale: 1000*V = 39*Nad + 600
              MUL
              ADDD  #600
              JSR   int2BCD
              JSR   BCD2ASC

              LDAA  THOUSANDS                 ; Make sure 1s digit shows '0' if V < 1.0V
              CMPA  #' '
              BNE   UPDT_BATT_PRNT
              LDAA  #'0'
              STAA  THOUSANDS

UPDT_BATT_PRNT
              LDAA  #$8D                      ; Position cursor on Row 1 after "Battery volt "
              JSR   cmd2LCD
              LDAA  TEN_THOUS                 ; Print 10s of Volts (or space)
              JSR   putcLCD
              LDAA  THOUSANDS                 ; Print 1s of Volts
              JSR   putcLCD
              LDAA  #'.'                      ; Print decimal point
              JSR   putcLCD
              LDAA  HUNDREDS                  ; Print 0.1s of Volts
              JSR   putcLCD

              ; Blink '*' at Row 1, col 18 ($92) as an alive indicator
              LDAA  #$92
              JSR   cmd2LCD
              LDAA  ALIVE_FLAG
              EORA  #$01                      ; Toggle bit 0 each refresh
              STAA  ALIVE_FLAG
              BEQ   ALIVE_OFF
              LDAA  #'*'
              BRA   ALIVE_PRNT
ALIVE_OFF     LDAA  #' '
ALIVE_PRNT    JSR   putcLCD

              ; Print current state string on Row 2 after "State "
              LDAA  #$C6
              JSR   cmd2LCD
              LDAB  CRNT_STATE                ; Load current state index (0..7)
              LSLB                            ; Multiply index by 8 (2^3)
              LSLB
              LSLB
              LDX   #tab                      ; Load base address of state string table
              ABX                             ; Add byte offset (B) to base pointer (X)
              JSR   putsLCD                   ; Print 7-char state label

              ; Print LINE_STATUS in hex at Row 2, col 16 ($D0)
              LDAA  #$D0
              JSR   cmd2LCD
              LDAA  LINE_STATUS
              JSR   BIN2ASC                   ; Returns MS hex char in A, LS in B
              PSHB                            ; Save LS char since putcLCD uses A
              JSR   putcLCD                   ; Print upper hex digit
              PULA
              JSR   putcLCD                   ; Print lower hex digit

UPDT_EXIT     RTS

;-----------------------------------------------------------------
; Utility Subroutines (LCD, Delay, A/D, BCD/Hex, and Timer)
;-----------------------------------------------------------------
initLCD       BSET  DDRB,%11111111            ; PB7..PB0 as outputs
              BSET  DDRJ,%11000000            ; PJ7(E), PJ6(RS) as outputs
              LDY   #2000                     ; Wait 100 ms for power-up
              JSR   del_50us
              LDAA  #LCD_4BIT_2LN             ; 4-bit bus, 2-line display
              JSR   cmd2LCD
              LDAA  #LCD_DISP_ON              ; Display on, cursor off
              JSR   cmd2LCD
              LDAA  #LCD_ENTRY_INC            ; Auto-increment cursor right
              JSR   cmd2LCD
              RTS

clrLCD        LDAA  #LCD_CLEAR                ; Clear screen & home cursor
              JSR   cmd2LCD
              LDY   #40                       ; Wait 2 ms
              JSR   del_50us
              RTS

LCD_POS_CRSR  ORAA  #%10000000                ; Set the high bit of the control word
              JSR   cmd2LCD                   ; and set the cursor address
              RTS

cmd2LCD       BCLR  LCD_CNTR,LCD_RS           ; RS=0 for Instruction Register
              JSR   dataMov
              RTS

putcLCD       BSET  LCD_CNTR,LCD_RS           ; RS=1 for Data Register
              JSR   dataMov
              RTS

putsLCD       LDAA  1,X+                      ; Fetch byte and increment X pointer
              BEQ   donePS                    ; Stop at null byte (0)
              JSR   putcLCD
              BRA   putsLCD
donePS        RTS

dataMov       BSET  LCD_CNTR,LCD_E            ; Latch upper nibble
              STAA  LCD_DAT
              BCLR  LCD_CNTR,LCD_E
              LSLA                            ; Shift lower nibble into upper 4 bits
              LSLA
              LSLA
              LSLA
              BSET  LCD_CNTR,LCD_E            ; Latch lower nibble
              STAA  LCD_DAT
              BCLR  LCD_CNTR,LCD_E
              LDY   #1                        ; 50 us command execution delay
              JSR   del_50us
              RTS

del_50us      PSHX                            ; Save X register
eloop         LDX   #300                      ; 50 us inner loop count (1 NOP per loop)
iloop         NOP
              DBNE  X,iloop
              DBNE  Y,eloop
              PULX                            ; Restore X register
              RTS

; A/D setup merged for battery (AN0), guider (AN1), and bumpers (AN02/AN03)
initAD        MOVB  #$C0,ATDCTL2              ; Power up A/D, fast flag clear
              LDY   #1                        ; Set Y=1 for 50 us stabilization delay
              JSR   del_50us
              MOVB  #$20,ATDCTL3              ; 4 conversions per sequence (matches guider setup)
              MOVB  #$97,ATDCTL4              ; 8-bit resolution, prescaler=48 (500kHz ADC clk)
              BSET  ATDDIEN,$0C               ; AN03 (Rear), AN02 (Front) as digital inputs
              RTS

int2BCD       XGDX                            ; Transfer 16-bit binary value to X
              CLR   TEN_THOUS                 ; Zero out all BCD digit bytes
              CLR   THOUSANDS
              CLR   HUNDREDS
              CLR   TENS
              CLR   UNITS
              CLR   BCD_SPARE
              CLR   BCD_SPARE+1
              CPX   #0
              BEQ   CON_EXIT

              XGDX                            ; Extract UNITS digit
              LDX   #10
              IDIV
              STAB  UNITS
              CPX   #0
              BEQ   CON_EXIT

              XGDX                            ; Extract TENS digit
              LDX   #10
              IDIV
              STAB  TENS
              CPX   #0
              BEQ   CON_EXIT

              XGDX                            ; Extract HUNDREDS digit
              LDX   #10
              IDIV
              STAB  HUNDREDS
              CPX   #0
              BEQ   CON_EXIT

              XGDX                            ; Extract THOUSANDS digit
              LDX   #10
              IDIV
              STAB  THOUSANDS
              CPX   #0
              BEQ   CON_EXIT

              XGDX                            ; Extract TEN_THOUS digit
              LDX   #10
              IDIV
              STAB  TEN_THOUS
CON_EXIT      RTS

BCD2ASC       CLR   NO_BLANK                  ; Clear leading zero flag

C_TTHOU       LDAA  TEN_THOUS
              ORAA  NO_BLANK
              BNE   NOT_BLANK1
              LDAA  #' '
              STAA  TEN_THOUS
              BRA   C_THOU
NOT_BLANK1    LDAA  TEN_THOUS
              ORAA  #$30
              STAA  TEN_THOUS
              MOVB  #$01,NO_BLANK

C_THOU        LDAA  THOUSANDS
              ORAA  NO_BLANK
              BNE   NOT_BLANK2
              LDAA  #' '
              STAA  THOUSANDS
              BRA   C_HUNS
NOT_BLANK2    LDAA  THOUSANDS
              ORAA  #$30
              STAA  THOUSANDS
              MOVB  #$01,NO_BLANK

C_HUNS        LDAA  HUNDREDS
              ORAA  NO_BLANK
              BNE   NOT_BLANK3
              LDAA  #' '
              STAA  HUNDREDS
              BRA   C_TENS
NOT_BLANK3    LDAA  HUNDREDS
              ORAA  #$30
              STAA  HUNDREDS
              MOVB  #$01,NO_BLANK

C_TENS        LDAA  TENS
              ORAA  NO_BLANK
              BNE   NOT_BLANK4
              LDAA  #' '
              STAA  TENS
              BRA   C_UNITS
NOT_BLANK4    LDAA  TENS
              ORAA  #$30
              STAA  TENS

C_UNITS       LDAA  UNITS                     ; Always convert units digit to ASCII
              ORAA  #$30
              STAA  UNITS
              RTS

; Converts an 8-bit value in ACCA to 2 ASCII hex chars in D (A=MSN, B=LSN)
BIN2ASC       PSHA                            ; Save a copy of the input number on the stack
              TAB                             ; and copy it into ACCB
              ANDB  #%00001111                ; Strip off the upper nibble of ACCB
              CLRA                            ; D now contains 000n where n is the LSnibble
              ADDD  #HEX_TABLE                ; Set up for indexed load
              XGDX
              LDAA  0,X                       ; Get the LSnibble character

              PULB                            ; Retrieve the input number into ACCB
              PSHA                            ; and push the LSnibble character in its place
              LSRB                            ; Shift upper nibble into lower 4 bits
              LSRB
              LSRB
              LSRB
              CLRA                            ; D now contains 000n where n is the MSnibble
              ADDD  #HEX_TABLE                ; Set up for indexed load
              XGDX
              LDAA  0,X                       ; Get the MSnibble character into ACCA
              PULB                            ; Retrieve the LSnibble character into ACCB
              RTS

ENABLE_TOF    LDAA  #%10000000
              STAA  TSCR1                     ; Enable hardware timer counter
              STAA  TFLG2                     ; Clear pending TOF flag
              LDAA  #%10000100                ; Enable TOF interrupt, prescaler = 16
              STAA  TSCR2
              RTS

TOF_ISR       INC   TOF_COUNTER               ; Increment 23Hz tick counter
              LDAA  #%10000000                ; Clear TOF interrupt flag
              STAA  TFLG2
              RTI

;-----------------------------------------------------------------
; Interrupt Vectors
;-----------------------------------------------------------------
              ORG   $FFFE
              DC.W  Entry                     ; Reset Vector
              ORG   $FFDE
              DC.W  TOF_ISR                   ; Timer Overflow Interrupt Vector