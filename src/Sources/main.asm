;*******************************************************************
;* Robot Roaming Program (9S32C)                                   *
;*******************************************************************



; export symbols
            XDEF Entry, _Startup          ; export 'Entry' symbol
            ABSENTRY Entry                ; for absolute assembly: mark this as application entry point

;*******************************************************************
;*          Include derivative-specific definitions                *
;*******************************************************************
                INCLUDE 'derivative.inc' 

;*******************************************************************
;*          Definitions/equates section                            *
;*******************************************************************

; Guider equates
CLEAR_HOME      EQU   $01                     ; Clear the display and home the cursor
INTERFACE       EQU   $38                     ; 8 bit interface, two line display
CURSOR_OFF      EQU   $0C                     ; Display on, cursor off
SHIFT_OFF       EQU   $06                     ; Address increments, no character shift
LCD_SEC_LINE    EQU   64                      ; Starting addr. of 2nd line of LCD (note decimal value!)

; LCD Addresses
LCD_DAT         EQU   PORTB                   ; LCD data port, bits - PB7,...,PB0
LCD_CNTR        EQU   PTJ                     ; LCD control port, bits - PJ7(E),PJ6(RS)
LCD_E           EQU   $80                     ; LCD E-signal pin
LCD_RS          EQU   $40                     ; LCD RS-signal pin

; Robot Movements
FWD_INT         EQU   69                      ; 3 second delay (at 23Hz)
REV_INT         EQU   69                      ; 3 second delay (at 23Hz)
FWD_TRN_INT     EQU   46                      ; 2 second delay (at 23Hz)
REV_TRN_INT     EQU   46                      ; 2 second delay (at 23Hz)
START           EQU   0
FWD             EQU   1
LEFTT           EQU   2
ALL_STP         EQU   3
RIGHTT          EQU   4
REV_TRN         EQU   5

; Other codes
NULL            EQU 00                        ; The string 'null terminator'
CR              EQU $0D                       ; 'Carriage Return' character
SPACE           EQU ' '                       ; The 'space' character

;*******************************************************************
;*          Variable/data section                                  *
;*******************************************************************

                ORG   $3850                   ; (9S12C32 RAM space: $3800 ... $3FFF)
; Guider Data                
SENSOR_LINE     FCB   $01                     ; Storage for guider sensor readings
SENSOR_BOW      FCB   $23                     ; Initialized to test values
SENSOR_PORT     FCB   $45
SENSOR_MID      FCB   $67
SENSOR_STBD     FCB   $89

SENSOR_NUM      RMB   1                       ; The currently selected sensor

TOP_LINE        RMB   20                      ; Top line of display
                FCB   NULL       	            ;  terminated by null

BOT_LINE        RMB   20                      ; Bottom line of display
                FCB   NULL                    ;  terminated by null

CLEAR_LINE      FCC   '                    '
                FCB   NULL                    ;  terminated by null
                
TEMP            RMB   1                       ; Temporary location

; Movement Data
TOF_COUNTER     dc.b  0                       ; The timer, incremented at 23Hz
CRNT_STATE      dc.b  3                       ; Current state register
T_FWD           ds.b  1                       ; FWD time
T_REV           ds.b  1                       ; REV time
T_FWD_TRN       ds.b  1                       ; FWD_TURN time
T_REV_TRN       ds.b  1                       ; REV_TURN time
TEN_THOUS       ds.b  1                       ; 10,000 digit
THOUSANDS       ds.b  1                       ;  1,000 digit
HUNDREDS        ds.b  1                       ;    100 digit
TENS            ds.b  1                       ;     10 digit
UNITS           ds.b  1                       ;      1 digit
NO_BLANK        ds.b  1                       ; Used in ’leading zero’ blanking by BCD2ASC
BCD_SPARE       RMB   10                      ; Extra space for decimal point and string terminator
;*****************************************************************
;*          Code Section                                         *
;*****************************************************************

            ORG   $4000                   ; Where the code starts --------------------
Entry:                                    ;                                           |
_Startup:                                 ;                                           |
            CLI                           ; Enable interrupts                         |
            LDS   #$4000                  ; Initialize the stack pointer
                                          ;                                           I
            BSET  DDRA,%00000011          ; STAR_DIR, PORT_DIR                        N
            BSET  DDRT,%00110000          ; STAR_SPEED, PORT_SPEED                    I
                                          ;                                           T
            ;JSR   initAD                  ; Initialize ATD converter                  I
                                          ;                                           A
            ;JSR   initLCD                 ; Initialize the LCD                        L
            ;JSR   clrLCD                  ; Clear LCD & home cursor                   I
                                          ;                                           Z
            ;LDX   #msg1                   ; Display msg1                              A
            ;JSR   putsLCD                 ;     "                                     T
                                          ;                                           I
            ;LDAA  #$C0                    ; Move LCD cursor to the 2nd row            O
            ;JSR   cmd2LCD                 ;                                           N

            ;LDX   #msg2                   ; Display msg2                              |
            ;JSR   putsLCD                 ;     "                                     |
                                          ;                                           |
            ;JSR   ENABLE_TOF              ; Jump to TOF initialization ---------------

MAIN        ;JSR   UPDT_DISPL              ; ----------------------------------------- M
            LDAA  CRNT_STATE              ;                                           A
            JSR   DISPATCHER              ;                                           I
            BRA   MAIN                    ; ----------------------------------------- N

;*****************************************************************
;*          Data Section                                         *
;*****************************************************************
msg1        dc.b  "Battery volt ",0
msg2        dc.b  "State ",0
tab         dc.b  "START  ",0
            dc.b  "FWD    ",0
            dc.b  "left   ",0
            dc.b  "ALL_STP",0
            dc.b  "right  ",0
            dc.b  "REV_TRN",0

;*****************************************************************
;*          Dispatcher                                           *
;*****************************************************************                
                
DISPATCHER      CMPA  #START                  ; If it’s the START state,Compares content of acc a to location M, set Z = 1 if result is $00 
                BNE   NOT_START               ; Branches if Z = 0,Branches to NOT_START if Z = 0
                JSR   START_ST                ;  then call START_ST routine              
                BRA   DISP_EXIT               ;  and exit                                
                                                                                     
NOT_START       CMPA  #FWD                    ; Else if it’s the FORWARD state           
                BNE   NOT_FORWARD             ;                                          
                JSR   FWD_ST                  ;  then call the FORWARD routine           
                JMP   DISP_EXIT               ;  and exit                                
                                                                                    
NOT_FORWARD     CMPA  #LEFTT                  ; Else if it’s the REVERSE state           
                BNE   NOT_LEFT                ;                                          
                JSR   LEFT_TRN_ST             ;  then call the REVERSE routine           
                JMP   DISP_EXIT               ;  and exit                                
                                                                                        
NOT_LEFT        CMPA  #ALL_STP                ; Else if it’s the ALL_STOP state          
                BNE   NOT_ALL_STOP            ;                                          
                JSR   ALL_STP_ST              ;  then call the ALL_STOP routine          
                JMP   DISP_EXIT               ;  and  exit                                
                                                                                    
NOT_ALL_STOP    CMPA  #RIGHTT                 ; Else if it’s the FORWARD_TURN state      
                BNE   NOT_RIGHT               ;                                          
                JSR   RIGHT_TRN_ST            ;  then call the FORWARD_TURN routine      
                JMP   DISP_EXIT               ;  and  exit                               
                                                                                        
                                                                                    
NOT_RIGHT       SWI                           ; Else the CRNT_ST is not defined, so stop 
DISP_EXIT       RTS                           ; Exit from the state dispatcher
                
;*****************************************************************
;*          Start State                                          *
;*****************************************************************                 
                
START_ST        BRCLR PORTAD0,$04,NO_FWD      ; If /FWD_BUMP
                JSR   INIT_FWD                ; Initialize the FORWARD state

                MOVB  #FWD,CRNT_STATE         ; Go into the FORWARD state
                BRA   START_EXIT              
NO_FWD          NOP                           ; Else
START_EXIT      RTS                           ;  return to the MAIN routine

;*****************************************************************
;*          Forward State                                        *
;*****************************************************************

FWD_ST          BRSET PORTAD0,$08,NO_REAR_BUMP; If REAR_BUMP, then we should stop
                JSR   START_ST                ; so initialize the START state
                MOVB  #START,CRNT_STATE       ; and change state to START
                JMP   FWD_EXIT                ; and return
              
NO_REAR_BUMP    LDAA  SENSOR_STBD             ; Load contents of Sensor Line into AccA 
                CMPA  #$C0                    ;
                BHI   STAR_STOP               ;

                LDAA  SENSOR_LINE             ; Load contents of Sensor Line into AccA 
                CMPA  #$B0                    ;
                BHI   RIGHT                   ;

                LDAA  SENSOR_LINE             ; Load contents of Sensor Line into AccA 
                CMPA  #$40                    ;
                BLO   LEFT                    ;



RIGHT           JSR   INIT_RIGHT_TURN
                MOVB  #RIGHTT,CRNT_STATE      ; and go to that state
                JMP   FWD_EXIT     

LEFT            JSR   INIT_LEFT_TURN
                MOVB  #LEFTT,CRNT_STATE       ; and go to that state
                JMP   FWD_EXIT     

STAR_STOP       JSR   INIT_ALL_STP
                MOVB  #ALL_STP,CRNT_STATE     ; and go to that state
                JMP   FWD_EXIT     



NO_FWD_TRN      NOP                           ; Else
FWD_EXIT        RTS                           ;  return to the MAIN routine

;*****************************************************************
;*          All Stop State                                       *
;*****************************************************************

ALL_STP_ST      BRSET PORTAD0,$04,NO_START    ; If FWD_BUMP
                BCLR  PTT,%00110000           ;  initialize the START state (both motors off)
                MOVB  #START,CRNT_STATE       ;  set the state to START
                BRA   ALL_STP_EXIT            ;  and return
NO_START        NOP                           ; Else
ALL_STP_EXIT    RTS                           ;  return to the MAIN routine

;*****************************************************************
;*          Right turn State                                     *
;*****************************************************************

RIGHT_TRN_ST    LDAA  SENSOR_LINE             ; If Tc>Tfwdturn then
                CMPA  #$B0                    ;  the robot should go FWD
                BHI   STAY_RIGHT
                
                JSR   INIT_FWD                ;  initialize the FWD state
                MOVB  #FWD,CRNT_STATE         ;  set state to FWD
                BRA   RIGHT_EXIT              ;  and return

STAY_RIGHT      NOP                           ; Else
RIGHT_EXIT      RTS                           ;  return to the MAIN routine

;*****************************************************************
;*          Left turn State                                      *
;*****************************************************************

LEFT_TRN_ST     LDAA  SENSOR_LINE             ; If Tc>Tfwdturn then
                CMPA  #$40                    ;  the robot should go FWD
                BlO   STAY_LEFT               ;  so
                
                JSR   INIT_FWD                ;  initialize the FWD state
                MOVB  #FWD,CRNT_STATE         ;  set state to FWD
                BRA   LEFT_EXIT               ;  and return

STAY_LEFT       NOP                           ; Else
LEFT_EXIT       RTS                           ;  return to the MAIN routine

;*****************************************************************
;*          Initilize forward                                    *
;*****************************************************************

INIT_FWD        BCLR  PORTA,%00000011         ; Set FWD direction for both motors
                BSET  PTT,%00110000           ; Turn on the drive motors
                RTS

;*****************************************************************
;*          Initilize RIGHT turn                                 *
;*****************************************************************

INIT_RIGHT_TURN BSET  PORTA,%00000010         ; Set REV dir. for STARBOARD (right) motor
                RTS

;*****************************************************************
;*          Initilize LEFT turn                                  *
;*****************************************************************

INIT_LEFT_TURN  BSET  PORTA,%00000001         ; Set REV dir. for STARBOARD (right) motor
                RTS

;*****************************************************************
;*          Initilize All Stop                                   *
;*****************************************************************

INIT_ALL_STP    BCLR  PTT,%00110000           ; Turn off the drive motors
                RTS

;*****************************************************************
;*          utility subroutines                                   *
;*****************************************************************

;*****************************************************************
;*Initialization of the LCD: 4-bit data width, 2-line display,   *
;*turn on display, cursor and blinking off. Shift cursor right   *
;*****************************************************************

initLCD     BSET  DDRB,%11110000          ; *configure pins PB7,...,PB0 for output
            BSET  DDRJ,%11000000          ; *configure pins PJ7,PJ6 for output
            LDY   #2000                   ; wait for LCD to be ready
            JSR   del_50us                ;    -"-
            LDAA  #$28                    ; set 4-bit data, 2-line display
            JSR   cmd2LCD                 ;    -"-
            LDAA  #$0C                    ; display on, cursor off, blinking off
            JSR   cmd2LCD                 ;    -"-
            LDAA  #$06                    ; move cursor right after entering a character
            JSR   cmd2LCD                 ;    -"-
            RTS

;*****************************************************************
;*          Clear display and home cursor                        *
;*                                                               *
;*****************************************************************

clrLCD      LDAA  #$01                    ; clear cursor and return to home position
            JSR   cmd2LCD                 ;    -"-
            LDY   #40                     ; wait until "clear cursor" command is complete
            JSR   del_50us                ;    -"-
            RTS

;*****************************************************************
;*          ([Y] x 50us)-delay. subroutine E-clk=41,67ns         *   
;*                                                               *    
;*****************************************************************

del_50us    PSHX                          ; (2 E-clk) Protect the X register
eloop       LDX   #300                    ; (2 E-clk) Initialize the inner loop counter
iloop       NOP                           ; (1 E-clk) No operation
            DBNE  X,iloop                 ; (3 E-clk) If the inner cntr not 0, loop again
            DBNE  Y,eloop                 ; (3 E-clk) If the outer cntr not 0, loop again
            PULX                          ; (3 E-clk) Restore the X register
            RTS                           ; (5 E-clk) Else return
            
;*****************************************************************
;*This function sends a command in accumulator A to the LCD      *
;*                                                               *
;*****************************************************************

cmd2LCD:    BCLR  LCD_CNTR,LCD_RS         ; select the LCD Instruction Register (IR)
            JSR   dataMov                 ; send data to IR
            RTS
                             
                             
;*****************************************************************
;*This function outputs a NULL-terminated string pointed to by X *
;*                                                               *
;*****************************************************************

putsLCD     LDAA  1,X+                    ; get one character from the string
            BEQ   donePS                  ; reach NULL character?
            JSR   putcLCD
            BRA   putsLCD
donePS      RTS

;*****************************************************************
;*This function outputs the character in accumulator in A to LCD *
;*                                                               *
;*****************************************************************

putcLCD     BSET  LCD_CNTR,LCD_RS         ; select the LCD Data register (DR)
            JSR   dataMov                 ; send data to DR
            RTS

;*****************************************************************
;*This function sends data to the LCD IR or DR depening on RS    *
;*                                                               *
;*****************************************************************

dataMov     BSET  LCD_CNTR,LCD_E          ; pull the LCD E-sigal high
            STAA  LCD_DAT                 ; send the upper 4 bits of data to LCD
            BCLR  LCD_CNTR,LCD_E          ; pull the LCD E-signal low to complete the write oper.
            
            LSLA                          ; match the lower 4 bits with the LCD data pins
            LSLA                          ;    -"-
            LSLA                          ;    -"-
            LSLA                          ;    -"-
            
            BSET  LCD_CNTR,LCD_E          ; pull the LCD E signal high
            STAA  LCD_DAT                 ; send the lower 4 bits of data to LCD
            BCLR  LCD_CNTR,LCD_E          ; pull the LCD E-signal low to complete the write oper.
            
            LDY   #1                      ; adding this delay will complete the internal
            JSR   del_50us                ; operation for most instructions
            RTS
            
;*****************************************************************
;*                                                               *
;*                                                               *
;*****************************************************************

initAD      MOVB  #$C0,ATDCTL2            ; power up AD, select fast flag clear                        Move byte of data #$C0 (1100 0000) to ATDCTL2 
            JSR   del_50us                ; wait for 50 us
            MOVB  #$00,ATDCTL3            ; 8 conversions in a sequence                                Move byte of data #$00 (0000 0000) to ATDCTL3
            MOVB  #$85,ATDCTL4            ; res=8, conv-clks=2, prescal=12                             Move byte of data #$85 (1000 0101) to ATDCTL4
            BSET  ATDDIEN,$0C             ; configure pins AN03,AN02 as digital inputs                 Mask ATDDIEN with  $0C (0000 1100), 2nd and 3rd bit, or w/ 1 
            RTS

;*****************************************************************
;*                                                               *
;*                                                               *
;*****************************************************************

int2BCD     XGDX                          ; Save the binary number into .X                             Exchanges Content of AccD and Reg X
            LDAA  #0                      ; Clear the BCD_BUFFER                                       Load 0 into A accumulator
            STAA  TEN_THOUS               ;                                                            Stores 0 in TEN_THOUS
            STAA  THOUSANDS               ;                                                               "        THOUSANDS
            STAA  HUNDREDS                ;                                                               "        HUNDREDS
            STAA  TENS                    ;                                                               "        TENS
            STAA  UNITS                   ;                                                               "        UNITS
            STAA  BCD_SPARE               ;                                                               "        BCD_SPARE
            STAA  BCD_SPARE+1             ;                                                               "        BCD_SPARE+1

            CPX   #0                      ; Check for a zero input                                     Compares content of X w/ 16 bit value $0000, will set Z if res. is $0000  
            BEQ   CON_EXIT                ; and if so, exit                                            Tests if Z = 1, jumps to CON_EXIT which RTS

            XGDX                          ; Not zero, get the binary number back to .D as dividend     Exchanges Content of AccD and Reg X
            LDX   #10                     ; Setup 10 (Decimal!) as the divisor                         Load 10 (decimal) into X
            IDIV                          ; Divide: Quotient is now in .X, remainder in .D             <- does that
            ANDB  #$0F                    ; Clear high nibble of remainder                             masks top of AccB to zero
            STAB  UNITS                   ; and store it.                                              Stores B into UNITS
            CPX   #0                      ; If quotient is zero,                                       If what left in X is zero
            BEQ   CON_EXIT                ; then exit                                                  Branch to this label

            XGDX                          ; else swap first quotient back into .D                      
            LDX   #10                     ; and setup for another divide by 10                               
            IDIV                                                                                             
            ANDB  #$0F                                                                                       
            STAB  TENS                                                                                       
            CPX   #0                                                                                         
            BEQ   CON_EXIT                                                                                   

            XGDX                          ; Swap quotient back into .D
            LDX   #10                     ; and setup for another divide by 10
            IDIV
            ANDB  #$0F
            STAB  HUNDREDS
            CPX   #0
            BEQ   CON_EXIT

            XGDX                          ; Swap quotient back into .D
            LDX   #10                     ; and setup for another divide by 10
            IDIV
            ANDB  #$0F
            STAB  THOUSANDS
            CPX   #0
            BEQ   CON_EXIT

            XGDX                          ; Swap quotient back into .D
            LDX   #10                     ; and setup for another divide by 10
            IDIV
            ANDB  #$0F
            STAB  TEN_THOUS

CON_EXIT RTS                              ; We're done the conversion

;*****************************************************************
;*                                                               *
;*                                                               *
;*****************************************************************

BCD2ASC     LDAA  #0                      ; Initialize the blanking flag                               Load 0 to AccA 
            STAA  NO_BLANK                ;                                                            Store AccA into NO_BLANK 

C_TTHOU     LDAA  TEN_THOUS               ; Check the 'ten_thousands' digit                            Load TEN_THOUS to AccA 
            ANDA  #$0F                    ; Clear the high nibble                                      masks top of AccA to zero
            ORAA  NO_BLANK                ;                                                            OR AccA w/ NO_BLANK, Z is set if result is $00
            BNE   NOT_BLANK1              ;                                                            Branches to Not_Blank1 if Z=0

ISBLANK1    LDAA  #' '                    ; It's blank
            STAA  TEN_THOUS               ; so store a space
            BRA   C_THOU                  ; and check the 'thousands' digit

NOT_BLANK1  LDAA  TEN_THOUS               ; Get the 'ten_thousands' digit
            ORAA  #$30                    ; Convert to ascii
            STAA  TEN_THOUS
            LDAA  #$1                     ; Signal that we have seen a 'non-blank' digit
            STAA  NO_BLANK

C_THOU      LDAA  THOUSANDS               ; Check the thousands digit for blankness
            ANDA  #$0F                    ; Clear the high nibble
            ORAA  NO_BLANK                ; If it's blank and 'no-blank' is still zero
            BNE   NOT_BLANK2              ;                                                            Branches to Not_Blank2 if Z=0

ISBLANK2    LDAA  #' '                    ; Thousands digit is blank
            STAA  THOUSANDS               ; so store a space
            BRA   C_HUNS                  ; and check the hundreds digit

NOT_BLANK2  LDAA  THOUSANDS               ; (similar to 'ten_thousands case)
            ORAA  #$30
            STAA  THOUSANDS
            LDAA  #$1
            STAA  NO_BLANK

C_HUNS      LDAA  HUNDREDS                ;  Check the hundreds digit for blankness
            ANDA  #$0F                    ; Clear the high nibble
            ORAA  NO_BLANK                ; If it's blank and 'no-blank' is still zero
            BNE   NOT_BLANK3

ISBLANK3    LDAA  #' '                    ; Hundreds digit is blank
            STAA  HUNDREDS                ; so store a space
            BRA   C_TENS                  ; and check the tens digit

NOT_BLANK3  LDAA  HUNDREDS                ; (similar to 'ten_thousands case)
            ORAA  #$30
            STAA  HUNDREDS
            LDAA  #$1
            STAA  NO_BLANK
           
C_TENS      LDAA  TENS                    ; Check the tens digit for blankness
            ANDA  #$0F                    ; Clear the high nibble
            ORAA  NO_BLANK                ; If it's blank and 'no-blank' is still zero
            BNE   NOT_BLANK4

ISBLANK4    LDAA  #' '                    ; Tens digit is blank
            STAA  TENS                    ; so store a space
            BRA   C_UNITS                 ; and check the units digit

NOT_BLANK4  LDAA  TENS                    ; (similar to 'ten_thousands case)
            ORAA  #$30
            STAA  TENS
            LDAA  #$1
            STAA  NO_BLANK

C_UNITS     LDAA  UNITS                   ; No blank check necessary, convert to ascii.
            ANDA  #$0F
            ORAA  #$30
            STAA  UNITS

            RTS                           ; We're done
            
;*****************************************************************
;*                                                               *
;*                                                               *
;*****************************************************************                             
                 
ENABLE_TOF  LDAA  #%10000000
            STAA  TSCR1                   ; Enable TCNT (Timer counter register) to count by setting the 7th bit (timer enable bit) to 1 (allows timer to function normally)
            STAA  TFLG2                   ; Must first initialize the TOF to 1 (only bit 7 important) as the function is called when the TOF: 1 -> 0 (when TCNT rolls over 7th is set to 1) 
            LDAA  #%10000100              ; Enable TOI (Timer overflow interupt) 7th bit to 1 (interupt requested when TOF flag is set) and select prescale factor equal to 16
            STAA  TSCR2                   ; Stored in the Timer control system register 2 which controls the operation of the timer counter 
            RTS
                
;*****************************************************************
;*                                                               *
;*                                                               *
;*****************************************************************                    
                 
TOF_ISR     INC   TOF_COUNTER             ; add 1 to value at TOF_COUNTER memory location
            LDAA  #%10000000              ; TFLG2 is cleared by writing 1 to it
            STAA  TFLG2                   ; ""
            RTI                           ; Return from interupt, restores all the CPU registers that have been stored in the stack by the CPU during the interrupt
            
;*******************************************************************
;* Update Display (Battery Voltage + Current State)                *
;*******************************************************************

UPDT_DISPL   MOVB  #$90,ATDCTL5           ; R-just., uns., sing. conv., mult., ch=0, start
             BRCLR ATDSTAT0,$80,*         ; Wait until the conver. seq. is complete
             LDAA  ATDDR0L                ; Load the ch0 result - battery volt - into A
                
                                          ; Display the battery voltage
             LDAB  #39                    ; *AccB = 39
             MUL                          ; AccD = 1st result x 39
             ADDD  #600                   ; *AccD = 1st result x 39 + 600
             
             JSR   int2BCD
             JSR   BCD2ASC           

             LDAA  #$8D                   ; move LCD cursor to the 1st row, end of msg1                From lab diagram moves near end of row 1
             JSR   cmd2LCD                ;     "
             
             LDAA  TEN_THOUS              ; output the TEN_THOUS ASCII character
             JSR   putcLCD                ;     "
             LDAA  THOUSANDS              ; output the THOUSANDS ASCII character
             JSR   putcLCD                ;     "
             LDAA  #'.'                   ; output a period character in ASCII
             JSR   putcLCD
             LDAA  HUNDREDS               ; output the HUNDREDS  ASCII character
             JSR   putcLCD                ;     "
             
;-------------------------
             LDAA  #$C6                   ; Move LCD cursor to the 2nd row, end of msg2
             JSR   cmd2LCD                ;

             LDAB  CRNT_STATE             ; Display current state
             LSLB                         ;     "
             LSLB                         ;     "
             LSLB                         ;     "
             LDX   #tab                   ;     "
             ABX                          ;     "
             JSR   putsLCD                ;     "
             RTS

;*****************************************************************
;*          Interrupt Vectors                                    *
;*****************************************************************
            
            ORG   $FFFE                   ; Reset Vector, first instruction to be executed after power is turned on or a reset 
            DC.W  Entry                   ; define words to be located at a specific address
                                          
            ORG   $FFDE                   ; Enhanced capture timer overflow
            DC.W  TOF_ISR                 ; Timer Overflow Interrupt Vector                 
                 