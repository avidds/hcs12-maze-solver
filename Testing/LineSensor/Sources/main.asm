;*******************************************************************
;*                            lab 7                                *
;*******************************************************************

; export symbols
        XDEF Entry, _Startup            ; export 'Entry' symbol
        ABSENTRY Entry          ; for absolute assembly: mark this as application entry point


; Include derivative-specific definitions
      INCLUDE 'derivative.inc'


;-------------------------------------------------------------------
;
;               'Read Guider' Demo Routine
;
; Reads the eebot guider sensors and displays the values
;  on the Liquid Crystal Display.

; Peter Hiscocks
; Version 2


; The guider hardware was modified with the addition of a 74HC138 decoder that
;  drives the individual LEDs, so that only the LED associated with a given
;  sensor is ON when that sensor is being read.
;  This requires that the software be modified to enable the decoder with bit PA5
;  in PORTA.
; The CdS cells are very slow in responding to changes in light, so a 20
;  millisecond delay is inserted between selecting a particular sensor and
;  reading its value.

; Overview:
; ---------
; The guider consists of four absolute brightness sensors and one
;  differential brightness pair of sensors. They are arranged at the nose of
;  the robot in the following pattern (viewed from above):
;
;                       A
;                    B  C  D
;                      E-F
;
; The sensors are cadmium sulphide (CdS) photoresistive cells, for which the
; resistance increases with decreasing light level. The absolute cells
; A,B,C and D are driven from a constant current source, and the voltage
; across the cell measured via the HCS12 A/D converter channel AN1. Thus
; the sensor reading increases as the sensor becomes darker (over a black
; line, for example).

; The differential sensor E-F is a voltage divider with the two CdS cells E
; and F separated 0.75 inches, which is the width of electrical tape. It is
; intended to be used to track the edges of the electrical tape 'line' once
; the absolute cells have 'found' a black line. Cell E is at the top of the
; divider, so as the reading from this sensor increases, cell E is becoming
; lighter, ie, cell E is straying onto the white background.
; Simultaneously, cell F is becoming darker as it moves over the black
; tape, and its resistance is increasing, aiding the same effect. The
; differential action should ignore ambient light.

; The program reads the sensor values, hopefully without disturbing any
; other settings on the robot. The values are displayed in hexadecimal on
; the LCD. On the LCD display, the pattern is as described in the routine
; 'DISPLAY_SENSORS'.
; The 4 absolute sensors should show readings equivalent to approximately 2
; volts when over a light surface and 4 volts when covered by a finger. The
; range from light background to black tape background is typically 1.5 volts
; over a light background to 2.4 volts over black tape.

; Using the program:
; ------------------
; Connect the eebot chassis to an HCS12 computer board as usual. Load
; 'read-guider' profram into the microcomputer. Run the routine 'MAIN'. The
; display should show the five sensor readings. Placing a finger over
; one of the sensors to block its illumination should cause the reading to
; increase significantly. Be extremely careful not to bend the sensors or
; LED illuminators when doing this.

; equates section
;
; The A/D converter automatically puts the 4 results in these registers.
; ATDDR0L EQU $0091   ; A/D Result Register 0
; ATDDR1L EQU $0093   ; A/D Result Register 1
; ATDDR2L EQU $0095   ; A/D Result Register 2
; ATDDR3L EQU $0097   ; A/D Result Register 3


; PORTA Register
;-------------------------------
; This register selects which sensor is routed to AN1 of the A/D converter

; PORTA EQU $0000 ; PORTA Register
;         7  6  5  4  3  2  1  0
;       --- --- --- --- --- --- --- ---
;      |   |   |   |   |   |   |   |   |
;       --- --- --- --- --- --- --- ---
;        ^   ^   ^   ^   ^   ^   ^   ^
;        |   |   |   |   |   |   |   |
;        |   |   |   |   |   |   |   +--- Port Motor Direction (0 = FWD)
;        |   |   |   |   |   |   |
;        |   |   |   |   |   |   +--- Starboard Motor Direction (0 = FWD)
;        |   |   |   |   |   |
;        |   |   |   |   +---+---+--- Sensor Select
;        |   |   |   |      000  Sensor Line
;        |   |   |   |      001  Sensor Bow
;        |   |   |   |      010  Sensor Port
;        |   |   |   |      011  Sensor Mid
;        |   |   |   |      100  Sensor Starboard
;        |   |   |   |
;        |   |   |   +--- Sensor LED enable (1 = ON)
;        |   |   |
;        +---+---+--- not used


; Liquid Crystal Display Equates
;-------------------------------
CLEAR_HOME      EQU     $01             ; Clear the display and home the cursor
INTERFACE       EQU     $38             ; 8 bit interface, two line display
CURSOR_OFF      EQU     $0C             ; Display on, cursor off
SHIFT_OFF       EQU     $06             ; Address increments, no character shift
LCD_SEC_LINE    EQU     64              ; Starting addr. of 2nd line of LCD (note decimal value!)

; LCD Addresses
LCD_CNTR        EQU     PTJ             ; LCD Control Register: E = PJ7, RS = PJ6
LCD_DAT         EQU     PORTB           ; LCD Data Register: D7 = PB7, ... , D0 = PB0
LCD_E           EQU     $80             ; LCD E-signal pin
LCD_RS          EQU     $40             ; LCD RS-signal pin

; Other codes
NULL            EQU     00              ; The string 'null terminator'
CR              EQU     $0D             ; 'Carriage Return' character
SPACE           EQU     ' '             ; The 'space' character 

*************************************************************************************
* To be copied to main file                                                         *
*************************************************************************************

;Line States                            Based on sensor readout
STRAIGHT        EQU     $0A             ;On a straight line.Sensors A and C are activated.
TJUNCTION       EQU     $07             ;Forwards T Junction. C, B, D sensors activated
TLEFT           EQU     $0E             ;Left facing T junction, A,C,B sensors activated
TRIGHT          EQU     $0B             ;Right facing T Junction. A,C,D sensors activated

;Sensor level threhsold
THRESHOLD_H     EQU     $99               ;Threshold in N. On a line: 3V = 153 = $99
THRESHOLD_L     EQU     $5C               ;Off a line:  1.8V = 92 = $5C

                                          ;Threshold in N, for E-F sensors w/ hysterysis of +- 0.3V
TRAC_THRESH_H   EQU     $8F               ;2.8V= 143 = $8F
TRAC_THRESH_L   EQU     $70               ;2.2V = 112 = $70
*************************************************************************************


; variable/data section

                ORG     $3800
;-------------------------------
; Storage Registers (9S12C32 RAM space: $3800 ... $3FFF)

*************************************************************************************
*To be copied to main file                                                          *
*************************************************************************************
SENSOR_LINE     FCB     $01             ; Storage for guider sensor readings
SENSOR_BOW      FCB     $23             ; Initialized to test values
SENSOR_PORT     FCB     $45
SENSOR_MID      FCB     $67
SENSOR_STBD     FCB     $89

LINE_STATUS     FCB     $11           ; Interpreted Sensor readout. 0 is off line. 1 is on line.
                                        ; 000(E/F)ABCD

LINESENSOR_OUTPUT RMB   2               ;Main Output Variable
                                        ;Appendation of LINE_STATUS & Output of E&F
                                        ; XXXXXXXX + 000(E/F)ABCD
                                        ; E/F Value

SENSOR_NUM      RMB     1               ; The currently selected sensor
TEMP            RMB     1               ; Temporary location
***********************************************************************************

TOP_LINE        RMB     20              ; Top line of display
                FCB     NULL            ;  terminated by null

BOT_LINE        RMB     20              ; Bottom line of display
                FCB     NULL            ;  terminated by null

CLEAR_LINE      FCC     '                  '
                FCB     NULL            ;  terminated by null



; code section

                ORG     $4000           ; Start of program text (FLASH memory)
;-------------------------------------------------------------------------------
;               Initialization

Entry:
_Startup:

                LDS     #$4000          ; Initialize the stack pointer
                CLI                     ; Enable interrupts
*************************************************************************************
*To be copied to main file                                                          *
*************************************************************************************
                JSR     INIT            ; Initialize ports
                JSR     openADC         ; Initialize the ATD
                
                
                JSR     openLCD         ; Initialize the LCD
                JSR     CLR_LCD_BUF     ; Write 'space' characters to the LCD buffer
                BCLR    LINE_STATUS, $FF ; Clear line status to known state.
                

;-------------------------------------------------------------------------------
;               Display Sensors

LINESENSE       JSR     G_LEDS_ON       ; Enable the guider LEDs
                JSR     READ_SENSORS    ; Read the 5 guider sensors
                JSR     G_LEDS_OFF      ; Disable the guider LEDs
                JSR     GET_LINE_STATUS     ; Interpet the sensors into one of 6 states
               ;; RTS
                SWI










; subroutine section
********************************************************************************
*                         Line Sensor Subroutines                              *
********************************************************************************

;-------------------------------------------------------------------------------
;               Initialize ports

INIT            BCLR    DDRAD, $FF      ;Make PORTAD an input (DDRAD @ $0272)
                BSET    DDRA,  $FF      ;Make portA an output
                BSET    DDRB,  $FF      ;Make portB an output
                BSET    DDRJ,  $C0      ;Make pins 7,6 OF PTJ outputs
                RTS

;-------------------------------------------------------------------------------
;               Initialize the ADC

openADC         MOVB    #$80,ATDCTL2    ; Turn on ADC (ATDCTL2 @ $0082)
                LDY     #1              ; Wait for 50 us for ADC to be ready
                JSR     del_50us        ;   - " -
                MOVB    #$20,ATDCTL3    ; 4 conversions on channel AN1 (ATDCTL3 @ $0083)
                MOVB    #$97,ATDCTL4    ; 8-bit resolution, prescaler=48 (ATDCTL4 @ $0084)
                RTS


;-------------------------------------------------------------------------------
;               Guider LEDs ON

; This routine enables the guider LEDs so that readings of the sensor
;  correspond to the 'illuminated' situation.

; Passed:   Nothing
; Returns:  Nothing
; Side:     PORTA bit 5 is changed

G_LEDS_ON       BSET    PORTA,%00100000 ; Set bit 5
                RTS

;-------------------------------------------------------------------------------
;               Guider LEDs OFF

; This routine disables the guider LEDs. Readings of the sensor
;  correspond to the 'ambient lighting' situation.

; Passed:   Nothing
; Returns:  Nothing
; Side:     PORTA bit 5 is changed

G_LEDS_OFF      BCLR    PORTA,%00100000 ; Clear bit 5
                RTS
;-------------------------------------------------------------------------------
;               Interpret Line Sensor Output. 
; Checks each sensor readout against threshold HI & LOW to determine status.
; Clobbers Accumulator A & B
; FORMAT XXXXXXXX000(E/F)ABCD. Sensor EF is passed through upper 4 bits.

;BOW SENSOR (A)
GET_LINE_STATUS LDAA    SENSOR_BOW                ;Load sensor value
                CMPA    #THRESHOLD_H                ;Compare to upper threshold
                BHS     SENS_BOW_HI               ;If above threashold, set bit 'A'
                
                CMPA    #THRESHOLD_L                ;Compare to lower threshold
                BLS     SENS_BOW_LO               ;If below threshold, clear bit 'A'
                
                ;Else reading is noise and it stays the same.
                BRA STATUS_PORT
                 
SENS_BOW_HI     BSET    LINE_STATUS, %00001000     ;Set bit 'A'
                BRA     STATUS_PORT

SENS_BOW_LO     BCLR    LINE_STATUS, %00001000     ;Clear bit 'A'
                BRA     STATUS_PORT

;PORT SENSOR (B)
STATUS_PORT     LDAA    SENSOR_PORT                ;Load sensor value
                CMPA    #THRESHOLD_H                ;Compare to upper threshold
                BHS     SENS_PORT_HI                ;If above threashold, set bit 'B'
                
                CMPA    #THRESHOLD_L                ;Compare to lower threshold
                BLS     SENS_PORT_LO               ;If below threshold, clear bit 'B'
                
                ;Else reading is noise and it stays the same.
                BRA     STATUS_MID
                 
SENS_PORT_HI    BSET    LINE_STATUS, %00000100     ;Set bit 'B'
                BRA     STATUS_MID

SENS_PORT_LO    BCLR    LINE_STATUS, %00000100     ;Clear bit 'B'
                BRA     STATUS_MID
                
;MIDDLE SENSOR(C)                
STATUS_MID      LDAA    SENSOR_MID                ;Load sensor value
                CMPA    #THRESHOLD_H                ;Compare to upper threshold
                BHS     SENS_MID_HI                ;If above threashold, set bit 'C'
                
                CMPA    #THRESHOLD_L                ;Compare to lower threshold
                BLS     SENS_MID_LO               ;If below threshold, clear bit 'C'
                
                ;Else reading is noise and it stays the same.
                BRA     STATUS_STBD
                 
SENS_MID_HI     BSET    LINE_STATUS, %00000010     ;Set bit 'C'
                BRA     STATUS_STBD

SENS_MID_LO     BCLR    LINE_STATUS, %00000010     ;Clear bit 'C'
                BRA     STATUS_STBD

;STARBORD SENSOR (D)                
STATUS_STBD     LDAA    SENSOR_STBD                ;Load sensor value
                CMPA    #THRESHOLD_H                ;Compare to upper threshold
                BHS     SENS_STBD_HI                ;If above threashold, set bit 'D'
                
                CMPA    #THRESHOLD_L                ;Compare to lower threshold
                BLS     SENS_STBD_LO               ;If below threshold, clear bit 'D'
                
                ;Else reading is noise and it stays the same.
                BRA     STATUS_EF
                 
SENS_STBD_HI     BSET    LINE_STATUS, %00000001     ;Set bit 'D'
                 BRA     STATUS_EF

SENS_STBD_LO     BCLR    LINE_STATUS, %00000001     ;Clear bit 'D'
                 BRA     STATUS_EF                

;LIne tracking sensor (EF)                
STATUS_EF        LDAA    SENSOR_LINE                ;Load sensor value
                 CMPA    #TRAC_THRESH_H
                 BHS     EF_HI
                 
                 CMPA    #TRAC_THRESH_L
                 BLS     EF_LO

                 BRA     PASSTHROUGH
                    
EF_HI            BSET    LINE_STATUS, %00010000      ;Set bit E/F: We are left of line
                 BRA     PASSTHROUGH
EF_LO            BCLR    LINE_STATUS, %00010000      ;Clear bit E/F: We are right of line 
                 BRA     PASSTHROUGH
                 
PASSTHROUGH      LDAA    SENSOR_LINE                ;Load the EF sensor value into upper bits
                 LDAB    LINE_STATUS                ;Load the parsed sensor values into lower bits
                 STD     LINESENSOR_OUTPUT          ;store in output 
                 RTS                                 

;-------------------------------------------------------------------------------
;               Read Sensors

; This routine reads the eebot guider sensors and puts the results in RAM
;  registers.

; Note: Do not confuse the analog multiplexer on the Guider board with the
;  multiplexer in the HCS12. The guider board mux must be set to the
;  appropriate channel using the SELECT_SENSOR routine. The HCS12 always
;  reads the selected sensor on the HCS12 A/D channel AN1.

; The A/D conversion mode used in this routine is to read the A/D channel
;  AN1 four times into HCS12 data registers ATDDR0,1,2,3. The only result
;  used in this routine is the value from AN1, read from ATDDR0. However,
;  other routines may wish to use the results in ATDDR1, 2 and 3.
; Consequently, Scan=0, Mult=0 and Channel=001 for the ATDCTL5 control word.

; Passed:       None
; Returns:      Sensor readings in:
;               SENSOR_LINE (0) (Sensor E/F)
;               SENSOR_BOW  (1) (Sensor A)
;               SENSOR_PORT (2) (Sensor B)
;               SENSOR_MID  (3) (Sensor C)
;               SENSOR_STBD (4) (Sensor D)
; Note:
;  The sensor number is shown in brackets

; Algorithm:
;       Initialize the sensor number to 0
;       Initialize a pointer into the RAM at the start of the Sensor Array storage
; Loop  Store %10000001 to the ATDCTL5 (to select AN1 and start a conversion)
;       Repeat
;           Read ATDSTAT0
;       Until Bit SCF of ATDSTAT0 = 1 (at which time the conversion is complete)
;       Store the contents of ATDDR0L at the pointer
;       If the pointer is at the last entry in Sensor Array, then
;           Exit
;       Else
;           Increment the sensor number
;           Increment the pointer
;       Loop again.

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

;-------------------------------------------------------------------------------
;               Select Sensor

; This routine selects the sensor number passed in ACCA. The motor direction
;  bits 0, 1, the guider sensor select bit 5 and the unused bits 6,7 in the
;  same machine register PORTA are not affected.
; Bits PA2,PA3,PA4 are connected to a 74HC4051 analog mux on the guider board,
;  which selects the guider sensor to be connected to AN1.

; Passed: Sensor Number in ACCA
; Returns: Nothing
; Side Effects: ACCA is changed

; Algorithm:
; First, copy the contents of PORTA into a temporary location TEMP and clear
;        the sensor bits 2,3,4 in the TEMP to zeros by ANDing it with the mask
;        11100011. The zeros in the mask clear the corresponding bits in the
;        TEMP. The 1's have no effect.
; Next, move the sensor selection number left two positions to align it
;        with the correct bit positions for sensor selection.
; Clear all the bits around the (shifted) sensor number by ANDing it with
;  the mask 00011100. The zeros in the mask clear everything except
;        the sensor number.
; Now we can combine the sensor number with the TEMP using logical OR.
;  The effect is that only bits 2,3,4 are changed in the TEMP, and these
;  bits now correspond to the sensor number.
; Finally, save the TEMP to the hardware.

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



*************************************************************************************
















********************************************************************************
*                              LCD Subroutines                                 *
********************************************************************************

;-------------------------------------------------------------------------------
;               Clear LCD Buffer

; This routine writes 'space' characters (ascii 20) into the LCD display
;  buffer in order to prepare it for the building of a new display buffer.
; This needs only to be done once at the start of the program. Thereafter the
;  display routine should maintain the buffer properly.

CLR_LCD_BUF     LDX     #CLEAR_LINE
                LDY     #TOP_LINE
                JSR     STRCPY

CLB_SECOND      LDX     #CLEAR_LINE
                LDY     #BOT_LINE
                JSR     STRCPY

CLB_EXIT        RTS
;-------------------------------------------------------------------------------
;               String Copy

; Copies a null-terminated string (including the null) from one location to
;  another

; Passed: X contains starting address of null-terminated string
;         Y contains first address of destination

STRCPY          PSHX                    ; Protect the registers used
                PSHY
                PSHA
STRCPY_LOOP     LDAA    0,X             ; Get a source character
                STAA    0,Y             ; Copy it to the destination
                BEQ     STRCPY_EXIT     ; If it was the null, then exit
                INX                     ; Else increment the pointers
                INY
                BRA     STRCPY_LOOP     ; and do it again
STRCPY_EXIT     PULA                    ; Restore the registers
                PULY
                PULX
                RTS


;-------------------------------------------------------------------------------
;               Display Sensor Readings

; Passed: Sensor values in RAM locations SENSOR_LINE through SENSOR_STBD.
; Returns: Nothing
; Side: Everything

; This routine writes the sensor values to the LCD. It uses the 'shadow buffer' approach.
;  The display buffer is built by the display controller routine and then copied in its
;  entirety to the actual LCD display. Although simpler approaches will work in this
;  application, we take that approach to make the code more re-useable.
; It's important that the display controller not write over other information on the
;  LCD, so writing the LCD has to be centralized with a controller routine like this one.
; In a more complex program with additional things to display on the LCD, this routine
;  would be extended to read other variables and place them on the LCD. It might even
;  read some 'display select' variable to determine what should be on the LCD.

; For the purposes of this routine, we'll put the sensor values on the LCD
;  in such a way that they (sort of) mimic the position of the sensors, so
;  the display looks like this:
;   01234567890123456789
;   ___FF______________
;   PP_MM_SS_LL________

; Where FF is the front sensor, PP is port, MM is mid, SS is starboard and
;  LL is the line sensor.

; The corresponding addresses in the LCD buffer are defined in the following
;  equates (In all cases, the display position is the MSDigit).

DP_FRONT_SENSOR EQU     TOP_LINE+3
DP_PORT_SENSOR  EQU     BOT_LINE+0
DP_MID_SENSOR   EQU     BOT_LINE+3
DP_STBD_SENSOR  EQU     BOT_LINE+6
DP_LINE_SENSOR  EQU     BOT_LINE+9

DISPLAY_SENSORS LDAA    SENSOR_BOW      ; Get the FRONT sensor value
                JSR     BIN2ASC         ; Convert to ascii string in D
                LDX     #DP_FRONT_SENSOR ; Point to the LCD buffer position
                STD     0,X             ;  and write the 2 ascii digits there

                LDAA    SENSOR_PORT     ; Repeat for the PORT value
                JSR     BIN2ASC
                LDX     #DP_PORT_SENSOR
                STD     0,X

                LDAA    SENSOR_MID      ; Repeat for the MID value
                JSR     BIN2ASC
                LDX     #DP_MID_SENSOR
                STD     0,X

                LDAA    SENSOR_STBD     ; Repeat for the STARBOARD value
                JSR     BIN2ASC
                LDX     #DP_STBD_SENSOR
                STD     0,X

                LDAA    SENSOR_LINE     ; Repeat for the LINE value
                JSR     BIN2ASC
                LDX     #DP_LINE_SENSOR
                STD     0,X

                LDAA    #CLEAR_HOME     ; Clear the display and home the cursor
                JSR     cmd2LCD         ;

                LDY     #40             ; Wait 2 ms until "clear display" command is complete
                JSR     del_50us

                LDX     #TOP_LINE       ; Now copy the buffer top line to the LCD
                JSR     putsLCD

                LDAA    #LCD_SEC_LINE   ; Position the LCD cursor on the second line
                JSR     LCD_POS_CRSR

                LDX     #BOT_LINE       ; Copy the buffer bottom line to the LCD
                JSR     putsLCD
                RTS
;-------------------------------------------------------------------------------
;               Binary to ASCII

; Converts an 8 bit binary value in ACCA to the equivalent ASCII character 2
;  character string in accumulator D
; Uses a table-driven method rather than various tricks.

; Passed: Binary value in ACCA
; Returns: ASCII Character string in D
; Side Fx: ACCB is destroyed

HEX_TABLE       FCC     '0123456789ABCDEF' ; Table for converting values

BIN2ASC         PSHA                    ; Save a copy of the input number on the stack
                TAB                     ;  and copy it into ACCB
                ANDB    #%00001111      ; Strip off the upper nibble of ACCB
                CLRA                    ; D now contains 000n where n is the LSnibble
                ADDD    #HEX_TABLE      ; Set up for indexed load
                XGDX
                LDAA    0,X             ; Get the LSnibble character

                PULB                    ; Retrieve the input number into ACCB
                PSHA                    ;  and push the LSnibble character in its place
                RORB                    ; Move the upper nibble of the input number
                RORB                    ;  into the lower nibble position.
                RORB
                RORB
                ANDB    #%00001111      ; Strip off the upper nibble
                CLRA                    ; D now contains 000n where n is the MSnibble
                ADDD    #HEX_TABLE      ; Set up for indexed load
                XGDX
                LDAA    0,X             ; Get the MSnibble character into ACCA
                PULB                    ; Retrieve the LSnibble character into ACCB
                RTS

;-------------------------------------------------------------------------------
;     Routines to control the Liquid Crystal Display

;-------------------------------------------------------------------------------
;               Initialize the LCD

openLCD         LDY     #2000           ; Wait 100 ms for LCD to be ready
                JSR     del_50us        ;   "
                LDAA    #INTERFACE      ; Set 8-bit data, 2-line display, 5x8 font
                JSR     cmd2LCD         ;   "
                LDAA    #CURSOR_OFF     ; Display on, cursor off, blinking off
                JSR     cmd2LCD         ;   "
                LDAA    #SHIFT_OFF      ; Move cursor right (address increments, no char. shift)
                JSR     cmd2LCD         ;   "
                LDAA    #CLEAR_HOME     ; Clear the display and home the cursor
                JSR     cmd2LCD         ;   "
                LDY     #40             ; Wait 2 ms until "clear display" command is complete
                JSR     del_50us        ;   "
                RTS

;-------------------------------------------------------------------------------
;               Send a command in accumulator A to the LCD

cmd2LCD         BCLR    LCD_CNTR,LCD_RS ; Select the LCD Instruction register
                JSR     dataMov         ; Send data to IR or DR of the LCD
                RTS

;-------------------------------------------------------------------------------
;               Send a character in accumulator A to LCD

putcLCD         BSET    LCD_CNTR,LCD_RS ; Select the LCD Data register
                JSR     dataMov         ; Send data to IR or DR of the LCD
                RTS

;-------------------------------------------------------------------------------
;               Send a NULL-terminated string pointed to by X

putsLCD         LDAA    1,X+            ; Get one character from the string
                BEQ     donePS          ; Reach NULL character?
                JSR     putcLCD
                BRA     putsLCD
donePS          RTS
;-------------------------------------------------------------------------------
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
;-------------------------------------------------------------------------------
;               Position the Cursor

; This routine positions the display cursor in preparation for the writing
;  of a character or string.
; For a 20x2 display:
; The first line of the display runs from 0 .. 19.
; The second line runs from 64 .. 83.

; The control instruction to position the cursor has the format
;               1aaaaaaa
; where aaaaaaa is a 7 bit address.

; Passed:   7 bit cursor Address in ACCA
; Returns:  Nothing
; Side Effects: None

LCD_POS_CRSR    ORAA    #%10000000      ; Set the high bit of the control word
                JSR     cmd2LCD         ;  and set the cursor address
                RTS

;-------------------------------------------------------------------------------
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
;-------------------------------------------------------------------------------
;               Interrupt Vectors

                ORG     $FFFE
                DC.W    Entry           ; Reset Vector                     