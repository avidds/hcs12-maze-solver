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

;State definitions
START         EQU     0
FWD           EQU     1
ALL_STP       EQU     2
LEFT_TRN      EQU     3
RIGHT_TRN     EQU     4
TRN_180       EQU     5
DETECT_T      EQU     6
LOST          EQU     7

RIGHT_TRN_INT EQU     23
LEFT_TRN_INT  EQU     23 
TRN_180_INT   EQU     46  
;Line States                            Based on sensor readout
STRAIGHT        EQU     $0A             ;On a straight line.Sensors A and C are activated.
TJUNCTION       EQU     $07             ;Forwards T Junction. C, B, D sensors activated
TLEFT           EQU     $0E             ;Left facing T junction, A,C,B sensors activated
TRIGHT          EQU     $0B             ;Right facing T Junction. A,C,D sensors activated
SENSA           EQU     $08             ;Sensor A mask
SENSB           EQU     $04             ;Sensor B mask
SENSC           EQU     $02             ;Sensor C mask
SENSD           EQU     $01             ;Sensor D mask


;variable/data section
            ORG     $3850           ; Where our TOF counter register lives
TOF_COUNTER dc.b    0               ; The timer, incremented at 23Hz
CRNT_STATE  dc.b    3               ; Current state register
PREV_STATE  dc.b    3               ; Previous state register
T_RIGHT_TRN ds.b    1               ; RIGHT_TRN Time
T_LEFT_TRN  ds.b    1               ; LEFT_TRN Time
T_TRN_180   ds.b    1               ; 180 turning time
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
            
            MOVB    #START, CRNT_STATE
            MOVB    #$FF  , PREV_STATE
            JSR     initAD          ;initialize ATD converter
            JSR     ENABLE_TOF      ;Initialize TOF
            MOVB    
            
*****************************************************************
*           Main Loop                                           *
*****************************************************************            
           
MAIN        JSR     UPDT_DISPL      ;Update display
            JSR     LINESENSE       ;Get linesensor output
            
            BRSET  PORTAD0,mPORTAD0_PTAD3,  NO_REV_BMP     ;If reverse bmp is on, stop. Else go to NO_REV_BMP            
            MOVB   #ALL_STP, CRNT_STATE                    ;Set state to all stop
            
            
NO_REV_BMP  LDAA    CRNT_STATE      ;Get current state
            JSR     DISPATCHER      ;run the dispatcher
            BRA     MAIN            ;loop
                    
       
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
                JSR    MTR_STP
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

*********************************************************************
*Init functions                                                    *
******************************************************************** 
; Initialize FWD
INIT_FWD       JSR   MTR_FWD                    ;Start moving forwards.
               MOVB  #FWD, PREV_STATE
               RTS

; Initialize Detected intersection state
INIT_DETECT_T  JSR   MTR_STP                    ;Stop all motors.
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
  