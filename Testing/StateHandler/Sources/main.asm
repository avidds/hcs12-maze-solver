;*****************************************************************
;* Lab 4 Assignment 3              Timer Alarms                  *
;*****************************************************************

; export symbols
            XDEF Entry, _Startup            ; export 'Entry' symbol
            ABSENTRY Entry        ; for absolute assembly: mark this as application entry point



; Include derivative-specific definitions 
		INCLUDE 'derivative.inc' 

*****************************************************************
*            Timer Alams                                        *
*****************************************************************
;definitions
OneSec      EQU     23              ; 1 second delay (at 23Hz)
TwoSec      EQU     46              ; 2 second delay (at 23Hz)

LCD_DAT     EQU     PORTB           ; LCD data port, bits - PB7,...,PB0
LCD_CNTR    EQU     PTJ             ; LCD control port, bits - PJ7(E),PJ6(RS)
LCD_E       EQU     $80             ; LCD E-signal pin
LCD_RS      EQU     $40             ; LCD RS-signal pin
FWD_INT     EQU     69              ; 3 second delay (at 23Hz)
REV_INT     EQU     69              ; 3 second delay (at 23Hz)
FWD_TRN_INT EQU     46              ; 2 second delay (at 23Hz)
REV_TRN_INT EQU     46              ; 2 second delay (at 23Hz)

;State definitions
START       EQU     0
STOP        EQU     1
FLINE       EQU     2
DETECT_T    EQU     3

;Line States                            Based on sensor readout
STRAIGHT        EQU     $0A             ;On a straight line.Sensors A and C are activated.
TJUNCTION       EQU     $07             ;Forwards T Junction. C, B, D sensors activated
TLEFT           EQU     $0E             ;Left facing T junction, A,C,B sensors activated
TRIGHT          EQU     $0B             ;Right facing T Junction. A,C,D sensors activated



;variable/data section
            ORG     $3850           ; Where our TOF counter register lives
TOF_COUNTER dc.b    0               ; The timer, incremented at 23Hz
CRNT_STATE  dc.b    3               ; Current state register
PREV_STATE  dc.b    3               ; Previous state register

TEN_THOUS   ds.b    1               ; 10,000 digit
THOUSANDS   ds.b    1               ; 1,000 digit
HUNDREDS    ds.b    1               ; 100 digit
TENS        ds.b    1               ; 10 digit
UNITS       ds.b    1               ; 1 digit
NO_BLANK    ds.b    1               ; Used in ’leading zero’ blanking by BCD2ASC
;code section
            ORG     $4000           ; Where the code starts
Entry:
_Startup:
*****************************************************************
*           Initialization (startup)                            *
*****************************************************************

            CLI                     ;Enable interrupts
            LDS     #$4000          ;Initialize stack pointer
            
            BSET    DDRA, %00000011 ;Motor direction controls enable
            BSET    DDRT, %00110000 ;Motor speed controls enable
            
            JSR     initAD          ;initialize ATD converter
            
            JSR     initLCD         ;initialize LCD
            JSR     clrLCD          ;clear LCD & home the cursor
            
            LDX     #msg1           ;display msg
            JSR     putsLCD       
            
            LDAA    #$C0            ;move cursor to second row
            JSR     cmd2LCD
            
            LDX     #msg2           ;display msg2
            JSR     putsLCD       
            
            JSR     ENABLE_TOF      ;Initialize TOF
            
*****************************************************************
*           Main Loop                                           *
*****************************************************************            
           
MAIN        JSR     UPDT_DISPL      ;Update display
            LDAA    CRNT_STATE      ;Get current state
            JSR     DISPATCHER      ;run the dispatcher
            BRA     MAIN            ;loop
            
             
;Data section
msg1        dc.b  "battery volt ",0
msg2        dc.b  "State ",0
tab         dc.b  "START  ",0
            dc.b  "STOP   ",0
            dc.b  "FLINE  ",0
            dc.b  "DET_T  ",0
        
         
       
;Subroutines Section   

********************************************************************
*Dispatcher                                                        *
********************************************************************
;Compares CRNT_STATE to state values to see which state we're in.
DISPATCHER   CMPA   #START                    
             BNE    NOT_START
             JSR    START_ST
             BRA    DISP_EXIT
             
NOT_START    CMPA   #FLINE
             BNE    NOT_FLINE
             JSR    FLINE_ST
             BRA    DISP_EXIT
             
NOT_FLINE    CMPA   #DETECT_T
             BNE    NOT_DET_T
             JSR    DETECT_T_ST
             BRA    DISP_EXIT
             
NOT_DET_T    CMPA   #STOP
             BNE    NOT_STOP
             JSR    STOP_ST
             BRA    DISP_EXIT
                     
             
NOT_STOP     SWI                      ;If it is none of the states: there is an error.
DISP_EXIT    RTS
********************************************************************
*States                                                            *
******************************************************************** 

********************************************************************
;Start State. Waits for front bumper to be bumped before moving forwards.

START_ST    BRCLR  PORTAD0, mPORTAD0_PTAD2, NO_FLINE        ;If front bmp is off: remain in start state
            
            LDAB   LINESENSOR_OUTPUT + 1                    ;Load the thresholded linesensor output
            ANDB   #$0F                                     ;Isolate lower 4 bits (ABCD)
            CMPB   #STRAIGHT                                ;Compare to straight state
            BNE    NO_FLINE                                 ;remain in start state if not in straight state.
                                                                
            MOVB   #FLINE,CRNT_STATE                         ;Set state to FLINE
            BRA    START_EXIT
            
NO_FLINE    NOP                                      ;Else
START_EXIT  RTS


********************************************************************
;Following Line state
;Check for bumper, check for intersections -> transition to next state
;Else: Follow the line based on sensor readings.
;Line is on the left: E/F = 0: turn left
;Line is on the right: E/f = 1: turn right

FLINE           LDD    LINESENSOR_OUTPUT
                BRSET  PORTAD0, mPORTAD0_PTAD2, FL_DETECT_INT    ;If front bmp is off: goto checking for intersections. Else activate stop state.

FL_FRONT_BMP    MOVB   #STOP, CRNT_STATE                         ;Bumper activated state transition
                JSR    MC_STOP 
                BRA    FLINE_EXIT

FL_DETECT_INT   ANDB   #$0F                                      ;Checking for intersections
                CMPB   #TJUNCTION
                BEQ    FL_DETECTED
                CMPB   #TLEFT
                BEQ    FL_DETECTED
                CMPB   #TRIGHT
                BEQ    FL_DETECTED
                BRA    FL_CONTROL
                
FL_DETECTED     MOVB   #DETECT_T, CRNT_STATE                     ;Intersection detected state transition
                JSR    MC_STOP
                BRA    FLINE_EXIT                                
                
; Using Bang-bang control for line-following
FL_CONTROL      LDAB   LINESENSOR_OUTPUT+1
                   
                BITB   #%00010000        ;Check E/F bit for high vs low
                BNE    FL_LINE_RIGHT      ;If it's 0, line is on our left.
            
FL_LINE_LEFT    JSR    MC_SLIGHT_LEFT
                BRA    FLINE_EXIT

FL_LINE_RIGHT   JSR    MC_SLIGHT_RIGHT
                BRA    FLINE_EXIT          


FLINE_EXIT      RTS
            
; Using proportional Control 
                 
********************************************************************



********************************************************************
;All stop state
;Stop everything until front bumper is pushed.
STOP_ST       BRSET  PORTAD0, mPORTAD0_PTAD2, NO_START       ;If front bmp is off; remain in all stop
              BCLR   PTT,%00110000                           ;Init the START state (motors off)
              MOVB   #START, CRNT_STATE                      ;Set state to START
              BRA    ALL_STP_EXIT

NO_START      NOP
ALL_STP_EXIT  RTS


;Intersection detected state
DETECT_T_ST



********************************************************************
*Motor Controls                                                    *
******************************************************************** 


********************************************************************
;Drive forewards
INIT_FWD              BCLR  PORTA, %00000011              ;Set forwards direction for both motors
                      BSET  PTT,%00110000                 ;Turn on drive motors
                      LDAA  TOF_COUNTER                   ;Create the FWD time flag
                      ADDA  #FWD_INT
                      STAA  T_FWD
                      RTS

********************************************************************
;Drie backwrds
INIT_REV              BSET  PORTA, %00000011              ;Set reverse direction for both motors
                      BSET  PTT,%00110000                 ;Turn on drive motors
                      LDAA  TOF_COUNTER                   ;Create the REV time flag
                      ADDA  #REV_INT
                      STAA  T_REV
                      RTS
                      
********************************************************************
;Stop
INIT_ALL_STP          BCLR  PTT,%00110000                 ;Turn off motors
                      RTS
                      
********************************************************************
;Forwards turn
INIT_FWD_TRN          BSET  PORTA, %00000010              ;Set reverse direction for right motor (starboard)
                      LDAA  TOF_COUNTER                   ;Create the FWD_TRN time flag
                      ADDA  #FWD_TRN_INT
                      STAA  T_FWD_TRN
                      RTS
                      
********************************************************************    
;Reverse turn                  
INIT_REV_TRN          BCLR  PORTA, %00000010              ;Set forwards direction for right motor (starboard)
                      LDAA  TOF_COUNTER                   ;Create the FWD time flag
                      ADDA  #REV_TRN_INT
                      STAA  T_REV_TRN
                      RTS
                      
********************************************************************
*LCD Functions                                                     *
******************************************************************** 
            
********************************************************************
; Initialize the LCD                                               
; Nukes ACCA

initLCD                BSET DDRB, #%11111111              ; Configure all 8 bits as output
                       BSET DDRJ, #%11000000              ; Configure pin 7 & 4 as outputs
                         
                       LDY #2000                          ;                                   
                       JSR  del_50us                      ; wait 0.1s for LCD boot
                       
                       BCLR LCD_CNTR,LCD_RS               ; Select instruction register for LCD
                       LDAA #$38                          ; Activate 8 bit mode. Set 2 line display.
                       JSR  dataMov                       ; Send to LCD
                       LDAA #$0C                          ; Set display on w/ no cursor and no blinking
                       JSR  dataMov                       ; Send to LCD
                       LDAA #$06                          ; Set to move cursor L to R 
                       JSR  dataMov                       ; Send to LCD
                       RTS
                       
********************************************************************
; Clears entire display of LCD

clrLCD                 LDAA #$01                          ; clear cursor & return to home command
                       JSR  cmd2LCD                       
                       LDY  #40                           ; Delay 2ms for command to complete.
                       JSR del_50us
                       RTS                           

********************************************************************
; Generate delays with increments of 50us. Y * 50us of delay       
; Uses register X & Y. Preserves X's value by using stack          
;                                                                  
; Timer math: ECLK = 24MHz = ~41.67nS/clk.                         
;             50us / 41.67ns/clk = 1200 clks                       


del_50us               PSHX                      ; 2 eclks
; set X value for internal loop. 
; Total external delay of (10 + 1190 eclks) * Y = 50us * Y
eloop                  LDX #35                   ; 2 eclks
; Total internal delay of 34 e clks. 35 x 34 eclks within = 1190 for iloop section.
iloop                  PSHA                      ; Del 2 CLK
                       PULA                      ; Del 3 CLK
                       PSHA                      ; Del 2 CLK
                       PULA                      ; Del 3 CLK
                       PSHA                      ; Del 2 CLK
                       PULA                      ; Del 3 CLK
                       PSHA                      ; Del 2 CLK
                       PULA                      ; Del 3 CLK
                       PSHA                      ; Del 2 CLK
                       PULA                      ; Del 3 CLK
                       nop                       ; Del 1 CLK
                       dbne x,iloop              ; 3 eclks 
                       
                       PSHA                      ; DEL 2 CLK
                       PULA                      ; Del 3 CLK
                       dbne y,eloop              ; 3 eclks
; Reset value of X to its original value                        
                       PULX                      ; 3 eclks
                       RTS                       ; 5 eclks
                       
; Accounting for overhead: PSHX + PULX + RTS + LDY = 12 eclks ~ negligible 415ns.

********************************************************************
; Sends command to instruction register of LCD

cmd2LCD                BCLR LCD_CNTR, LCD_RS              ; Select LCD instruction register
                       JSR  dataMov                       ; Send data to LCD
                       RTS
********************************************************************
; Display a string to the LCD, stopping at a null byte ($00)       
; Assumes storage in regX                                          

putsLCD                LDAA 1, X+                         ; Get a character from string stored in X
                       BEQ  done                          ; exit if at null ($00)
                       JSR  putcLCD                       ; print current character
                       JSR  putsLCD                       ; repeat

done                   RTS

********************************************************************
; Display a character to the LCD. Assumes usage of AccA            

putcLCD                BSET LCD_CNTR, LCD_RS                 ; Select data register
                       JSR dataMov                        ; Send data
                       RTS 

********************************************************************
; General purpose function to send 8bit sequences to LCD           
; Bits are sent from ACCA                                          
;                                                                  
; Nukes ACCA of its contents.                                      

dataMov                BSET LCD_CNTR, LCD_E               ; Set enable high
                       STAA LCD_DAT                       ; Send upper 8 bits of A to LCD
                       BCLR LCD_CNTR, LCD_E              ; Set enable low to complete write
                                                    
                       LDY #1                             ; 50us delay for LCD to process data
                       JSR del_50us
                       RTS
*****************************************************************
*Update display                                                 *
***************************************************************** 
UPDT_DISPL             MOVB    #$90, ATDCTL5   ; Read ATD Converter. R-just, uns., sing. conv., mult., ch=0, start
                       BRCLR   ATDSTAT0, $80, *          ;Wait for ATD status bit
                       
                       LDAA    ATDDR0L            ;Display battery voltage
                       LDAB    #39                ;Convert from bit value to Voltage
                       MUL
                       ADDD    #600
                       
                       JSR     int2BCD            ;Convert to BCD 
                       JSR     BCD2ASC            ;Convert to ASCII to be output on screen
                       LDAA    #$8D               ;move LCD cursor to the 1st row, end of msg1
                       JSR     cmd2LCD 
                        
                       LDAA    TEN_THOUS          ;Output Battery voltage value to LCD
                       JSR     putcLCD 
                       LDAA    THOUSANDS
                       JSR     putcLCD
                       LDAA    #'.'
                       JSR     putcLCD
                       LDAA    HUNDREDS
                       JSR     putcLCD 
                       LDAA    TENS
                       JSR     putcLCD
                       LDAA    UNITS
                       JSR     putcLCD
                       
                       LDAA    #$C6               ;Move cursor to 2nd row, end of msg 2
                       JSR     cmd2LCD
                       
                       LDAB    CRNT_STATE         ;Display current state
                       LSLB                       ;Multiply B by 8, since each string is 8 bytes. 
                       LSLB
                       LSLB
                       LDX     #tab               ;get index of first message
                       ABX                        ;Add offset B to get corresponding message. (tab + CRNT_STATE)
                       JSR     putsLCD
                       
                       RTS
                       
*****************************************************************
*            Timer Subroutines                                  *
*****************************************************************   
                     
ENABLE_TOF             LDAA    #%10000000
                       STAA    TSCR1           ; Enable TCNT
                       STAA    TFLG2           ; Clear TOF
                       LDAA    #%10000100      ; Enable TOI and select prescale factor equal to 16
                       STAA    TSCR2
                       RTS
*****************************************************************
TOF_ISR                INC     TOF_COUNTER     ; Increment TOF counter
                       LDAA    #%10000000      ; Clear flag
                       STAA    TFLG2           ;  
                       RTI
                       
*****************************************************************
*            AD Converter                                       *
*****************************************************************
initAD                 MOVB #$C0,ATDCTL2                  ;ATDCTL2 power up AD, select fast flag clear
                       JSR del_50us                       ;wait for 50 us
                       MOVB #$00,ATDCTL3                  ;8 conversions in a sequence. Non-FIFO
                       MOVB #$85,ATDCTL4                  ;res=8, conv-clks=2, prescal=12
                       BSET ATDDIEN,$0C                   ;configure pins AN03,AN02 as digital inputs
                       RTS


*****************************************************************
*Unit Conversions                                               *
*****************************************************************                       

; Convert from 16bit int to BCD upto ten thousanth digit via       
; successive integer division.                                     
; Routine taken from lab manual                                    
; Empties BCD Buffer.                                              
; Converts number in D to number in BCD buffer                     
; Empties register X and D.                                                                                  
int2BCD                XGDX                               ; Save number into X
                       LDAA #$00                          ; Clear BCD_BUFFER
                       STAA TEN_THOUS
                       STAA THOUSANDS
                       STAA HUNDREDS
                       STAA TENS
                       STAA UNITS
                
                       
                       CPX  #0                            ;Check X for a zero input
                       BEQ  CON_EXIT                      ;If so exit.
                       
                       XGDX                               ;Send number back to D as a dividend
                       LDX  #10                           ;divide by 10
                       IDIV                               ;Quotient in X, remainder in D
                       STAB UNITS                         ;Store the remainder
                       CPX  #0                            ;Check if quotient is 0
                       BEQ  CON_EXIT                      ;if so exit
                       
                       XGDX                               ;Move quotient into D.
                       LDX  #10                           ;Divide by 10
                       IDIV                               ;Quotient in X, Remainder in D
                       STAB TENS                          ;Store the remainder
                       CPX  #0                            ;Check if quotient is 0
                       BEQ  CON_EXIT                      ;if so exit
                       
                       XGDX                               ;Move quotient into D.
                       LDX  #10                           ;Divide by 10
                       IDIV                               ;Quotient in X, Remainder in D
                       STAB HUNDREDS                      ;Store the remainder
                       CPX  #0                            ;Check if quotient is 0
                       BEQ  CON_EXIT                      ;if so exit
                       
                       XGDX                               ;Move quotient into D.
                       LDX  #10                           ;Divide by 10
                       IDIV                               ;Quotient in X, Remainder in D
                       STAB THOUSANDS                     ;Store the remainder
                       CPX  #0                            ;Check if quotient is 0
                       BEQ  CON_EXIT                      ;if so exit
                       
                       XGDX                               ;Move quotient into D.
                       LDX  #10                           ;Divide by 10
                       IDIV                               ;Quotient in X, Remainder in D
                       STAB TEN_THOUS                 ;Store the remainder                     
                      
CON_EXIT               RTS

**************************************************************************
; Convert BCD to ascii code. Checks for blank characters, then     
; converts numbers to ascii by adding $30. Possible because numbers 
; in BCD are garunteed to be 0-9. Result stored in BCD buffer      
BCD2ASC                LDAA #0                            ;Initialize blanking flag
                       STAA NO_BLANK
                       
C_TTHOU                LDAA TEN_THOUS                     ;Get the ten thousands digit
                       ORAA NO_BLANK                      ;Or operation with A to generate flags (i.e, number is smaller than 100000)
                       BNE  NOT_BLANK1                    ;Branch if it was not 0
                       
ISBLANK1               LDAA #' '                          ;It's a blank 
                       STAA TEN_THOUS                     ;Store a blank ASCII character into TEN_THOUS
                       BRA  C_THOU                        ;Check thousanths next.

NOT_BLANK1             LDAA TEN_THOUS                     ;Get ten thousanth's digit
                       ORAA #$30                          ;Convert to ACSII
                       STAA TEN_THOUS                     ;Store result
                       LDAA #$1                           ;Set flag that we've had non-blank digit
                       STAA NO_BLANK
 
                       
C_THOU                 LDAA THOUSANDS                     ;Get the ten thousands digit
                       ORAA NO_BLANK                      ;Or operation with A to generate flags (i.e, number is smaller than 100000)
                       BNE  NOT_BLANK2                    ;Branch if it was not 0
                       
ISBLANK2               LDAA #' '                          ;It's a blank 
                       STAA THOUSANDS                     ;Store a blank ASCII character into THOUSANDS
                       BRA  C_HUNS                        ;Check thousanths next.

NOT_BLANK2             LDAA THOUSANDS                     ;Get ten thousanth's digit
                       ORAA #$30                          ;Convert to ACSII
                       STAA THOUSANDS                     ;Store result
                       LDAA #$1                           ;Set flag that we've had non-blank digit
                       STAA NO_BLANK

C_HUNS                 LDAA HUNDREDS                      ;Get the hundreds digit
                       ORAA NO_BLANK                      ;Or operation with A to generate flags (i.e, number is smaller than 100)
                       BNE  NOT_BLANK3                    ;Branch if it was not 0
                       
ISBLANK3               LDAA #' '                          ;It's a blank 
                       STAA HUNDREDS                      ;Store a blank ASCII character into HUNDREDS
                       BRA  C_TENS                        ;Check tens next.

NOT_BLANK3             LDAA HUNDREDS                      ;Get hundreths digit
                       ORAA #$30                          ;Convert to ACSII
                       STAA HUNDREDS                      ;Store result
                       LDAA #$1                           ;Set flag that we've had non-blank digit
                       STAA NO_BLANK
                 
C_TENS                 LDAA TENS                          ;Get the Tens digit
                       ORAA NO_BLANK                      ;Or operation with A to generate flags (i.e, number is smaller than 10)
                       BNE  NOT_BLANK4                    ;Branch if it was not 0
                       
ISBLANK4               LDAA #' '                          ;It's a blank 
                       STAA TENS                          ;Store a blank ASCII character into TENS
                       BRA  C_UNITS                       ;Check ones next.

NOT_BLANK4             LDAA TENS                          ;Get tens digit
                       ORAA #$30                          ;Convert to ACSII
                       STAA TENS                          ;Store result
               
C_UNITS                LDAA UNITS                         ;Get ones digits
                       ORAA #$30                          ;Convert to ASCII (it will be atleast a '0')
                       STAA UNITS                         ;Store result
                       
                       RTS                                ;DONE                      
                       
********************************************************************
* Interrupt Vectors                                                *
********************************************************************
            ORG $FFFE
            FDB Entry               ; Reset Vector
            
            ORG $FFDE               ; Timer interrupt vector
            DC.W TOF_ISR