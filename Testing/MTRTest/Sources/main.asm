; export symbols
            XDEF Entry, _Startup            ; export 'Entry' symbol
            ABSENTRY Entry        ; for absolute assembly: mark this as application entry point



; Include derivative-specific definitions 
		INCLUDE 'derivative.inc' 

*****************************************************************
*            Timer Alams                                        *
*****************************************************************
;definitions
OneSec          EQU     23              ; 1 second delay (at 23Hz)
TwoSec          EQU     46              ; 2 second delay (at 23Hz)

;LCD Pins & Definitions
LCD_DAT         EQU   PORTB                     ; LCD data port (PB7..PB0)
LCD_CNTR        EQU   PTJ                       ; LCD control port (PJ7=E, PJ6=RS)
LCD_E           EQU   $80                       ; LCD Enable pin mask (Bit 7)
LCD_RS          EQU   $40                       ; LCD Register Select mask (Bit 6)

LCD_CLEAR       EQU   $01                       ; Clear screen & home cursor
LCD_4BIT_2LN    EQU   $28                       ; 4-bit bus, 2-line display
LCD_DISP_ON     EQU   $0C                       ; Display on, cursor off
LCD_ENTRY_INC   EQU   $06                       ; Auto-increment cursor right
LCD_LINE1       EQU   $80                       ; Start of Row 1
LCD_LINE2       EQU   $C0                       ; Start of Row 2
LCD_SEC_LINE    EQU   64                        ; Starting addr. of 2nd line (decimal, for LCD_POS_CRSR)
DISP_INT        EQU   5                         ; Update LCD every 5 TOF ticks (~217ms @ 23Hz)
NULL            EQU   $00                       ; Null terminator

;Motor bit masks
PORT_DIR_BIT    EQU   %00000001                 ; PA0: Port Dir (0 = FWD, 1 = REV)
STBD_DIR_BIT    EQU   %00000010                 ; PA1: Starboard Dir (0 = FWD, 1 = REV)
BOTH_DIR_BITS   EQU   %00000011                 ; Both motor direction pins

PORT_SPD_BIT    EQU   %00010000                 ; PT4: Port Speed/On (1 = ON, 0 = OFF)
STBD_SPD_BIT    EQU   %00100000                 ; PT5: Starboard Speed/On (1 = ON, 0 = OFF)
BOTH_SPD_BITS   EQU   %00110000                 ; Both motor enable pins

;State definitions
START           EQU     0
FWD             EQU     1
ALL_STP         EQU     2
LEFT_TRN        EQU     3
RIGHT_TRN       EQU     4
TRN_180         EQU     5
DETECT_T        EQU     6
LOST            EQU     7

RIGHT_TRN_INT   EQU     20
LEFT_TRN_INT    EQU     23 
TRN_180_INT     EQU     35  
;Line States                            Based on sensor readout
STRAIGHT        EQU     $0A             ;On a straight line.Sensors A and C are activated.
TJUNCTION       EQU     $07             ;Forwards T Junction. C, B, D sensors activated
TLEFT           EQU     $0E             ;Left facing T junction, A,C,B sensors activated
TRIGHT          EQU     $0B             ;Right facing T Junction. A,C,D sensors activated
SENSA           EQU     $08             ;Sensor A mask
SENSB           EQU     $04             ;Sensor B mask
SENSC           EQU     $02             ;Sensor C mask
SENSD           EQU     $01             ;Sensor D mask

;Sensor level threhsold
BOW_THRESHOLD_H      EQU     $99               ;Threshold in N. On a line: 3V = 153 = $99
BOW_THRESHOLD_L      EQU     $5C               ;Off a line:  1.8V = 92 = $5C

PORT_THRESHOLD_H     EQU     $99               ;Threshold in N. On a line: 3V = 153 = $99
PORT_THRESHOLD_L     EQU     $5C

MID_THRESHOLD_H      EQU     $99               ;Threshold in N. On a line: 3V = 153 = $99
MID_THRESHOLD_L      EQU     $5C

STBD_THRESHOLD_H     EQU     $99               ;Threshold in N. On a line: 3V = 153 = $99
STBD_THRESHOLD_L     EQU     $5C

LINE_MID_VALUE       EQU     $99               ;Threshold in N. On a line: 3V = 153 = $99
LINE_DELTA           EQU     $5C

;-----------------------------------------------------------------
; Variable / Data Section (RAM)
;-----------------------------------------------------------------
                    ORG     $3850           ; Where our TOF counter register lives
TOF_COUNTER         dc.b    0               ; The timer, incremented at 23Hz
CRNT_STATE          dc.b    3               ; Current state register
PREV_STATE          dc.b    3               ; Previous state register
T_RIGHT_TRN         ds.b    1               ; RIGHT_TRN Time
T_LEFT_TRN          ds.b    1               ; LEFT_TRN Time
T_TRN_180           ds.b    1               ; 180 turning time
SENSOR_LINE         FCB     $01             ; Storage for guider sensor readings
SENSOR_BOW          FCB     $23             ; Initialized to test values
SENSOR_PORT         FCB     $45
SENSOR_MID          FCB     $67
SENSOR_STBD         FCB     $89
LINE_STATUS         FCB     $11             ; Interpreted Sensor readout. 0 is off line. 1 is on line.                                        
LINESENSOR_OUTPUT   RMB     2                 ;Main Output Variable
                                            ;Appendation of LINE_STATUS & Output of E&F
                                            ; XXXXXXXX + 000(E/F)ABCD
                                            ; E/F Value
SENSOR_NUM          RMB     1               ; The currently selected sensor
TEMP                RMB     1               ; Temporary location
TEN_THOUS           ds.b    1               ; 10,000 digit
THOUSANDS           ds.b    1               ; 1,000 digit
HUNDREDS            ds.b    1               ; 100 digit
TENS                ds.b    1               ; 10 digit
UNITS               ds.b    1               ; 1 digit
NO_BLANK            ds.b    1               ; Used in ’leading zero’ blanking by BCD2ASC
BCD_SPARE           ds.b    2               ; Padding buffer for int2BCD routine
LAST_DISP_TOF       ds.b    1               ; Tracks last TOF count we updated the LCD at
ALIVE_FLAG          ds.b    1               ; Flips between 0 and 1 to blink the '*' indicator

;code section
            ORG     $4000           ; Where the code starts
Entry:
_Startup:
*****************************************************************
*           Initialization (startup)                            *
*****************************************************************
            LDS     #$4000          ;Initialize stack pointer
            
            JSR     INIT_MOTORS
            JSR     initAD
            

            CLR     TOF_COUNTER
            MOVB    #RIGHT_TRN, CRNT_STATE
            MOVB    #$FF  , PREV_STATE
            JSR     ENABLE_TOF      ;Initialize TOF
            CLI                     ;Enable interrupts    
            
*****************************************************************
*           Main Loop                                           *
*****************************************************************            
           
            BRSET  PORTAD0, mPORTAD0_PTAD2, *              ;If front bmp is off: remain in start state            
MAIN        LDAA    CRNT_STATE      ;Get current state
            JSR     DISPATCHER      ;run the dispatcher
            BRA     MAIN            ;loop        
            
                    
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
              DC.B  "TRN_180",0               ; 5
              DC.B  "DCT_INT",0               ; 6
              DC.B  "LOST   ",0               ; 7  
                              
;Subroutines Section   

********************************************************************
*Dispatcher                                                        *
********************************************************************
;Compares CRNT_STATE to state values to see which state we're in.
DISPATCHER   CMPA   #START                    
             BNE    NOT_START
             JSR    START_ST
             BRA    DISP_EXIT
             
NOT_START    CMPA   #FWD
             BNE    NOT_FWD
             JSR    FWD_ST
             BRA    DISP_EXIT
             
NOT_FWD      CMPA   #LEFT_TRN
             BNE    NOT_LEFT_TRN
             JSR    LEFT_TRN_ST
             BRA    DISP_EXIT

NOT_LEFT_TRN CMPA   #RIGHT_TRN
             BNE    NOT_R_TRN
             JSR    RIGHT_TRN_ST
             BRA    DISP_EXIT

NOT_R_TRN    CMPA   #TRN_180
             BNE    NOT_TRN_180
             JSR    TRN_180_ST
             BRA    DISP_EXIT

NOT_TRN_180  CMPA   #DETECT_T
             BNE    NOT_DETECT_T
             JSR    DETECT_T_ST
             BRA    DISP_EXIT
             
NOT_DETECT_T CMPA   #LOST
             BNE    NOT_LOST
             JSR    LOST_ST
             BRA    DISP_EXIT

NOT_LOST     CMPA   #ALL_STP
             BNE    NOT_ALL_STP
             JSR    ALL_STP_ST
             BRA    DISP_EXIT        
;Stop code for testing
;NOT_FWD      CMPA   #DETECT_T
;             BNE    NOT_LOST
;             JSR    DETECT_T_ST
;             BRA    DISP_EXIT
             

             
NOT_ALL_STP SWI                      ;If it is none of the states: there is an error.
DISP_EXIT    RTS
********************************************************************
*States                                                            *
******************************************************************** 

********************************************************************
;Start State. Waits for front bumper to be bumped before moving forwards.

START_ST    BRSET  PORTAD0, mPORTAD0_PTAD2, NO_FWD          ;If front bmp is off: remain in start state
                                                           
            LDAB   LINESENSOR_OUTPUT + 1                    ;Load the thresholded linesensor output
            ANDB   #$0F                                     ;Isolate lower 4 bits (ABCD)
            CMPB   #STRAIGHT                                ;Compare to straight state
            BNE    NO_FWD                                   ;remain in start state if not in straight state.
            
            MOVB   CRNT_STATE, PREV_STATE                   ;Set state to FWD                                  
            MOVB   #FWD,CRNT_STATE                         
            BRA    START_EXIT
            
NO_FWD      NOP                                      ;Else
START_EXIT  RTS


********************************************************************
;Following Line state
;Check for bumper, check for intersections -> transition to next state
;Else: Follow the line based on sensor readings.
;Line is on the left: E/F = 0: turn left
;Line is on the right: E/f = 1: turn right

FWD_ST          LDAA   CRNT_STATE
                CMPA   PREV_STATE

                BEQ    FWD_SKIP                                   ;Skips init if previous state was the same state            
                JSR    INIT_FWD                
            
FWD_SKIP        MOVB   CRNT_STATE, PREV_STATE                    ;Set previous state as this state
                
                BRSET  PORTAD0, mPORTAD0_PTAD2, FWD_DETECT_INT    ;If front bmp is off: goto checking for intersections. Else activate stop state.

                
FWD_FRONT_BMP   MOVB   #TRN_180, CRNT_STATE                         ;Bumper activated state transition               
                BRA    FWD_EXIT

FWD_DETECT_INT  LDD    LINESENSOR_OUTPUT
                ANDB   #$0F                                       ;Checking for intersections
                CMPB   #TJUNCTION
                BEQ    FWD_DETECTED
                CMPB   #TLEFT
                BEQ    FWD_DETECTED
                CMPB   #TRIGHT
                BEQ    FWD_DETECTED
                BRA    FWD_CONTROL
                
FWD_DETECTED    MOVB   #DETECT_T, CRNT_STATE                     ;Intersection detected state transition
                BRA    FWD_EXIT                                

; No intersection, we must be following line still.                
; Using Bang-bang control for line-following
FWD_CONTROL     LDAB   LINESENSOR_OUTPUT+1
                
                BITB   #SENSB            ;Check if there is line Far Left
                BNE    FWD_LINE_LEFT
                
                BITB   #SENSD            ;Check if there is line far right
                BNE    FWD_LINE_RIGHT
                
                BITB   #%00100000        ;Check if in deadzone
                BEQ    FWD_LINE_CNTR     ;Just go forwards           
                                
                   
                BITB   #%00010000        ;Check E/F bit for high vs low
                BNE    FWD_LINE_RIGHT    
            
FWD_LINE_LEFT   JSR    MTR_STEER_L
                BRA    FWD_EXIT

FWD_LINE_RIGHT  JSR    MTR_STEER_R
                BRA    FWD_EXIT          

FWD_LINE_CNTR   JSR    MTR_FWD
                

FWD_EXIT        RTS

                 
********************************************************************
; Intersection detected
DETECT_T_ST     LDAA   CRNT_STATE
                CMPA   PREV_STATE
                BEQ    DTECT_T_SKIP           ;Skip initialization if in the same state as previous
                
                JSR    INIT_DETECT_T               
                

DTECT_T_SKIP    MOVB   CRNT_STATE, PREV_STATE  ;Set previous state as current state before any transitions
                LDAA   LINESENSOR_OUTPUT + 1   ;Get sensor output to find which intersection we have
                         
                                                 
                ANDA   #$0F                    ;Checking for intersections
                CMPA   #TJUNCTION
                BEQ    DTECT_T
                CMPA   #TLEFT
                BEQ    DTECT_TL
                CMPA   #TRIGHT
                BEQ    DTECT_TR
                BRA    DTECT_ERROR

;Always taking right-most path                
DTECT_T         NOP
DTECT_TR        MOVB   #RIGHT_TRN, CRNT_STATE
                BRA    DTECT_T_EXIT
                
DTECT_TL        MOVB   #FWD,CRNT_STATE
                BRA    DTECT_T_EXIT       

DTECT_ERROR     MOVB   #ALL_STP, CRNT_STATE          

DTECT_T_EXIT   RTS  
                
********************************************************************
;All stop state
;Stop everything until front bumper is pushed.
ALL_STP_ST      MOVB   CRNT_STATE, PREV_STATE
                JSR    MTR_STOP
                BRSET  PORTAD0, mPORTAD0_PTAD2, NO_START       ;If front bmp is off; remain in all stop
                
                MOVB   #START, CRNT_STATE
                BRA    ALL_STP_EXIT

NO_START        NOP               
ALL_STP_EXIT    RTS

********************************************************************
; 90 turn Right
RIGHT_TRN_ST    LDAA   CRNT_STATE                        ;Skip init for consecutive calls
                CMPA   PREV_STATE                
                BEQ    RIGHT_TRN_SKIP
                
                JSR    INIT_RIGHT_TRN                    ;Init turning & setup timer flag.
                
RIGHT_TRN_SKIP  LDAA   TOF_COUNTER                       ;Check if we're done turning
                SUBA   T_RIGHT_TRN                       ;Subtract interval
                BMI    R_NOT_DONE                        ;Branch if negative
                MOVB   #FWD, CRNT_STATE                                        

R_NOT_DONE
RIGHT_TRN_EXIT  RTS

********************************************************************

; 90 turn Left
LEFT_TRN_ST     LDAA   CRNT_STATE                        ;Skip init for consecutive calls
                CMPA   PREV_STATE                
                BEQ    LEFT_TRN_SKIP
                
                JSR    INIT_LEFT_TRN                    ;Init turning & setup timer flag.
                
LEFT_TRN_SKIP   LDAA   TOF_COUNTER                      ;Check if we're done turning
                SUBA   T_LEFT_TRN                       ;Subtract interval
                BMI    L_NOT_DONE                       ;branch if negative (time is less than interval)
                MOVB   #FWD, CRNT_STATE                                        

L_NOT_DONE
LEFT_TRN_EXIT  RTS

********************************************************************
; 180 turn Right
TRN_180_ST      LDAA   CRNT_STATE                        ;Skip init for consecutive calls
                CMPA   PREV_STATE                
                BEQ    TRN_180_SKIP
                
                JSR    INIT_TRN_180                    ;Init turning & setup timer flag.
                
TRN_180_SKIP    LDAA   TOF_COUNTER                       ;Check if we're done turning
                SUBA   T_TRN_180                         ;Subtract interval
                BMI    TRN_180_EXIT                      ;Branch if negative
                MOVB   #FWD, CRNT_STATE                                        

TRN_180_EXIT  RTS

********************************************************************
; Lost search routine
LOST_ST         RTS

*********************************************************************
*Init functions                                                    *
******************************************************************** 
INIT_SENS       BCLR    DDRAD, $FF      ;Make PORTAD an input (DDRAD @ $0272)
                BSET    DDRA,  $FF      ;Make portA an output
                BSET    DDRB,  $FF      ;Make portB an output
                BSET    DDRJ,  $C0      ;Make pins 7,6 OF PTJ outputs
                RTS
                
INIT_MOTORS   BSET  DDRA,BOTH_DIR_BITS        ; PA1 (Starboard Dir), PA0 (Port Dir) as outputs
              BSET  DDRT,BOTH_SPD_BITS        ; PT5 (Starboard Spd), PT4 (Port Spd) as outputs
              BCLR  PTT,BOTH_SPD_BITS         ; Make sure both motors start OFF
              BCLR  PORTA,BOTH_DIR_BITS       ; Default both directions to forward
              RTS
              
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
              
; A/D setup merged for battery (AN0), guider (AN1), and bumpers (AN02/AN03)
initAD        MOVB  #$C0,ATDCTL2              ; Power up A/D, fast flag clear
              LDY   #1                        ; Set Y=1 for 50 us stabilization delay
              JSR   del_50us
              MOVB  #$20,ATDCTL3              ; 4 conversions per sequence (matches guider setup)
              MOVB  #$97,ATDCTL4              ; 8-bit resolution, prescaler=48 (500kHz ADC clk)
              BSET  ATDDIEN,$0C               ; AN03 (Rear), AN02 (Front) as digital inputs
              RTS                                            
; Initialize FWD
INIT_FWD       JSR   MTR_FWD                    ;Start moving forwards.
               MOVB  #FWD, PREV_STATE
               RTS

; Initialize Detected intersection state
INIT_DETECT_T  JSR   MTR_STOP                    ;Stop all motors.
               MOVB  #DETECT_T, PREV_STATE
               RTS
               

INIT_RIGHT_TRN JSR   MTR_PIVOT_R                ;Start turning right
               LDAA  TOF_COUNTER                ;Setup timer flag
               ADDA  #RIGHT_TRN_INT
               STAA  T_RIGHT_TRN
               MOVB  #RIGHT_TRN, PREV_STATE
               RTS

INIT_LEFT_TRN  JSR   MTR_PIVOT_L                ;Start turning right
               LDAA  TOF_COUNTER                ;Setup timer flag
               ADDA  #LEFT_TRN_INT
               STAA  T_LEFT_TRN
               MOVB  #LEFT_TRN, PREV_STATE
               RTS

INIT_TRN_180   JSR   MTR_PIVOT_R               ;Start turning
               LDAA  TOF_COUNTER                ;Setup timer flag
               ADDA  #TRN_180_INT
               STAA  T_TRN_180
               MOVB  #TRN_180, PREV_STATE                   
               RTS

*****************************************************************
*Linesensor Subroutines                                         *      
*****************************************************************
LINESENSE       JSR     G_LEDS_ON       ; Enable the guider LEDs
                JSR     READ_SENSORS    ; Read the 5 guider sensors
                JSR     G_LEDS_OFF      ; Disable the guider LEDs
                JSR     GET_LINE_STATUS     ; Interpet the sensors into one of 6 states
                RTS 
               
G_LEDS_ON       BSET    PORTA,%00100000 ; Set bit 5
                RTS               
               
G_LEDS_OFF      BCLR    PORTA,%00100000 ; Clear bit 5
                RTS 
                
READ_SENSORS    CLR     SENSOR_NUM      ; Select sensor number 0
                LDX     #SENSOR_LINE    ; Point at the start of the sensor array

RS_MAIN_LOOP    LDAA    SENSOR_NUM      ; Select the correct sensor input
                JSR     SELECT_SENSOR   ;  on the hardware
                LDY     #400            ; 20 ms delay to allow the
                JSR     del_50us        ;  sensor to stabilize

                LDAA    #%10000001      ; Start A/D conversion on AN1
                STAA    ATDCTL5
                BRCLR   ATDSTAT0,$80,*  ; Repeat until A/D signals done

                LDAA    ATDDR0L         ; A/D conversion is complete in ATDDR0L
                STAA    0,X             ;  so copy it to the sensor register
                CPX     #SENSOR_STBD    ; If this is the last reading
                BEQ     RS_EXIT         ; Then exit

                INC     SENSOR_NUM      ; Else, increment the sensor number
                INX                     ;  and the pointer into the sensor array
                BRA     RS_MAIN_LOOP    ;  and do it again

RS_EXIT         RTS

SELECT_SENSOR   PSHA                    ; Save the sensor number for the moment

                LDAA    PORTA           ; Clear the sensor selection bits to zeros
                ANDA    #%11100011      ;
                STAA    TEMP            ; and save it into TEMP

                PULA                    ; Get the sensor number
                ASLA                    ; Shift the selection number left, twice
                ASLA                    ;
                ANDA    #%00011100      ; Clear irrelevant bit positions

                ORAA    TEMP            ; OR it into the sensor bit positions
                STAA    PORTA           ; Update the hardware
                RTS                              
;-------------------------------------------------------------------------------
;               Interpret Line Sensor Output. 
; Checks each sensor readout against threshold HI & LOW to determine status.
; Clobbers Accumulator A & B
; FORMAT XXXXXXXX000(E/F)ABCD. Sensor EF is passed through upper 4 bits.

;BOW SENSOR (A)
GET_LINE_STATUS LDAA    SENSOR_BOW                ;Load sensor value
                CMPA    #BOW_THRESHOLD_H                ;Compare to upper threshold
                BHS     SENS_BOW_HI               ;If above threashold, set bit 'A'
                
                CMPA    #BOW_THRESHOLD_L                ;Compare to lower threshold
                BLS     SENS_BOW_LO               ;If below threshold, clear bit 'A'
                
                ;Else reading is noise and it stays the same.
                BRA STATUS_PORT
                 
SENS_BOW_HI     BSET    LINE_STATUS, %00001000     ;Set bit 'A'
                BRA     STATUS_PORT

SENS_BOW_LO     BCLR    LINE_STATUS, %00001000     ;Clear bit 'A'
                BRA     STATUS_PORT

;PORT SENSOR (B)
STATUS_PORT     LDAA    SENSOR_PORT                ;Load sensor value
                CMPA    #PORT_THRESHOLD_H                ;Compare to upper threshold
                BHS     SENS_PORT_HI                ;If above threashold, set bit 'B'
                
                CMPA    #PORT_THRESHOLD_L                ;Compare to lower threshold
                BLS     SENS_PORT_LO               ;If below threshold, clear bit 'B'
                
                ;Else reading is noise and it stays the same.
                BRA     STATUS_MID
                 
SENS_PORT_HI    BSET    LINE_STATUS, %00000100     ;Set bit 'B'
                BRA     STATUS_MID

SENS_PORT_LO    BCLR    LINE_STATUS, %00000100     ;Clear bit 'B'
                BRA     STATUS_MID
                
;MIDDLE SENSOR(C)                
STATUS_MID      LDAA    SENSOR_MID                ;Load sensor value
                CMPA    #MID_THRESHOLD_H                ;Compare to upper threshold
                BHS     SENS_MID_HI                ;If above threashold, set bit 'C'
                
                CMPA    #MID_THRESHOLD_L                ;Compare to lower threshold
                BLS     SENS_MID_LO               ;If below threshold, clear bit 'C'
                
                ;Else reading is noise and it stays the same.
                BRA     STATUS_STBD
                 
SENS_MID_HI     BSET    LINE_STATUS, %00000010     ;Set bit 'C'
                BRA     STATUS_STBD

SENS_MID_LO     BCLR    LINE_STATUS, %00000010     ;Clear bit 'C'
                BRA     STATUS_STBD

;STARBORD SENSOR (D)                
STATUS_STBD     LDAA    SENSOR_STBD                ;Load sensor value
                CMPA    #STBD_THRESHOLD_H                ;Compare to upper threshold
                BHS     SENS_STBD_HI                ;If above threashold, set bit 'D'
                
                CMPA    #STBD_THRESHOLD_L                ;Compare to lower threshold
                BLS     SENS_STBD_LO               ;If below threshold, clear bit 'D'
                
                ;Else reading is noise and it stays the same.
                BRA     STATUS_EF
                 
SENS_STBD_HI     BSET    LINE_STATUS, %00000001     ;Set bit 'D'
                 BRA     STATUS_EF

SENS_STBD_LO     BCLR    LINE_STATUS, %00000001     ;Clear bit 'D'
                 BRA     STATUS_EF                

;LIne tracking sensor (EF)                
STATUS_EF        LDAA    SENSOR_LINE                ; A = current E/F reading
                 LDAB    LINE_MID_VALUE
                 ADDB    LINE_DELTA                 ; B = upper threshold (mid + delta)
                 CBA                                ; Computes A - B (reading - upper), B unchanged
                 BHS     EF_HI                      ; reading >= upper: set bit

                 LDAB    LINE_MID_VALUE
                 SUBB    LINE_DELTA                 ; B = lower threshold (mid - delta)
                 CBA                                ; Computes A - B (reading - lower)
                 BLS     EF_LO                      ; reading <= lower: clear bit
                 
                 BSET    LINE_STATUS,  %00100000     ; Inside the band: Set deadzone bit.
                 BRA     PASSTHROUGH
                                 
EF_HI            BSET    LINE_STATUS, %00010000     ; Set bit E/F: we are left of line                     
                 BCLR    LINE_STATUS, %00100000     ; Clear deadzone bit
                 BRA     PASSTHROUGH
EF_LO            BCLR    LINE_STATUS, %00010000     ; Clear bit E/F : we are right of line
                 BCLR    LINE_STATUS, %00100000     ; Clear deadzone bit
                 BRA     PASSTHROUGH

PASSTHROUGH      LDAA    SENSOR_LINE
                 LDAB    LINE_STATUS
                 STD     LINESENSOR_OUTPUT
                 RTS
*****************************************************************
*Motor Control Subroutines                                      *      
*****************************************************************
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
*****************************************************************
*UI Subroutines                                                 *      
*****************************************************************
; Throttled display refresh so LCD writes don't bottleneck the main loop
UPDT_DISPL    LDAA  TOF_COUNTER               ; Check if DISP_INT ticks have passed
              SUBA  LAST_DISP_TOF             ; Unsigned sub handles 255->0 wraparound
              CMPA  #DISP_INT
              LBLO  UPDT_EXIT                 ; Not time yet, skip update
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

*****************************************************************
*LCD Utilities                                                  *      
*****************************************************************
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
*****************************************************************
*Timer Subroutines                                              *      
*****************************************************************
                                     
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

                    
                       
********************************************************************
* Interrupt Vectors                                                *
********************************************************************
            ORG $FFFE
            FDB Entry               ; Reset Vector
  
            ORG   $FFDE
            DC.W  TOF_ISR                   ; Timer Overflow Interrupt Vector