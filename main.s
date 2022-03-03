; Archivo: main.s
; Dispositivo: PIC16F887
; Autor: Cristian Catú
; Compilador: pic-as (v.30), MPLABX V5.40
;
; Programa: Contador con 2 displays de 7 segmentos
; Hardware: contador 7 segmentos
;
; Creado: 21 de feb, 2022
; Última modificación: 21 de feb, 2022
    
PROCESSOR 16F887
    
; PIC16F887 Configuration Bit Settings

; Assembly source line config statements

; CONFIG1
  CONFIG  FOSC = INTRC_NOCLKOUT ; Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
  CONFIG  WDTE = OFF            ; Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
  CONFIG  PWRTE = OFF            ; Power-up Timer Enable bit (PWRT enabled)
  CONFIG  MCLRE = OFF           ; RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
  CONFIG  CP = OFF              ; Code Protection bit (Program memory code protection is disabled)
  CONFIG  CPD = OFF             ; Data Code Protection bit (Data memory code protection is disabled)
  CONFIG  BOREN = OFF           ; Brown Out Reset Selection bits (BOR disabled)
  CONFIG  IESO = OFF            ; Internal External Switchover bit (Internal/External Switchover mode is disabled)
  CONFIG  FCMEN = OFF           ; Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
  CONFIG  LVP = OFF              ; Low Voltage Programming Enable bit (RB3/PGM pin has PGM function, low voltage programming enabled)

; CONFIG2
  CONFIG  BOR4V = BOR40V        ; Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
  CONFIG  WRT = OFF             ; Flash Program Memory Self Write Enable bits (Write protection off)

// config statements should precede project file includes.
#include <xc.inc>
  
; ------- VARIABLES EN MEMORIA --------
PSECT udata_shr		    ; Memoria compartida
    W_TEMP:		DS 1
    STATUS_TEMP:	DS 1
    segundos:           DS 1
    display:            DS 2
    nibbles:            DS 2
    banderas:           DS 1
    centenas:           DS 1
    decenas:            DS 1
    unidades:           DS 1
    segundos1:          DS 1
  
PSECT resVect, class=CODE, abs, delta=2
; ----------------vector reset-----------------
ORG 00h ;posición 0000h para el reset
resetVec:
    PAGESEL main
    goto main

PSECT intVect, class=CODE, abs, delta=2
ORG 04h			    ; posición 0004h para interrupciones
;-------------- VECTOR INTERRUPCIONES ------------------
PUSH:
    MOVWF   W_TEMP	    ; Guardamos W
    SWAPF   STATUS, W
    MOVWF   STATUS_TEMP	    ; Guardamos STATUS
    
ISR:
    BTFSC   T0IF	    ; Interrupcion de TMR0?
    CALL    INT_TMR0
    BTFSC   TMR1IF	    ; Interrupcion de TMR1?
    CALL    INT_TMR1
    BTFSC   TMR2IF	    ; Interrupcion de TMR2?
    CALL    INT_TMR2
POP:
    SWAPF   STATUS_TEMP, W  
    MOVWF   STATUS	    ; Recuperamos el valor de reg STATUS
    SWAPF   W_TEMP, F	    
    SWAPF   W_TEMP, W	    ; Recuperamos valor de W
    RETFIE		    ; Regresamos a ciclo principal

; ------------------------ tabla -------------------------------
PSECT code, delta=2, abs
ORG 100h ;posición para el código
tabla:
    CLRF    PCLATH		; Limpiamos registro PCLATH
    BSF	    PCLATH, 0		; Posicionamos el PC en dirección 02xxh
    ADDWF   PCL			; Apuntamos el PC a caracter en ASCII de CONT
    RETLW   00111111B			; ASCII char 0
    RETLW   00000110B			; ASCII char 1
    RETLW   01011011B			; ASCII char 2
    RETLW   01001111B			; ASCII char 3
    RETLW   01100110B           	; ASCII char 4
    RETLW   01101101B			; ASCII char 5
    RETLW   01111101B			; ASCII char 6
    RETLW   00000111B			; ASCII char 7
    RETLW   01111111B			; ASCII char 8
    RETLW   01101111B	                ; ASCII char 9
    RETLW   01110111B			; ASCII char 10
    RETLW   01111100B			; ASCII char 11
    RETLW   00111001B			; ASCII char 12
    RETLW   01011110B			; ASCII char 13
    RETLW   01111001B			; ASCII char 14
    RETLW   01110001B			; ASCII char 15 

; ------ SUBRUTINAS DE INTERRUPCIONES ------    
INT_TMR0: ; cada 2ms
    BANKSEL TMR0	    ; cambiamos de banco
    MOVLW   252
    MOVWF   TMR0	    ; 50ms retardo
    BCF	    T0IF	    ; limpiamos bandera de interrupción
    
    BANKSEL PORTD
    CLRF PORTD
    BTFSC banderas, 0
    GOTO DISPLAY_1
    GOTO DISPLAY_0
    
DISPLAY_0:
    MOVF display, W
    MOVWF PORTC
    BSF PORTD, 1
    BSF banderas, 0
    RETURN
    
DISPLAY_1:
    MOVF display+1, W
    MOVWF PORTC
    BSF PORTD, 0
    BCF banderas, 0
    RETURN
    
INT_TMR1: ; cada segundo
    BANKSEL TMR1H
    MOVLW   0x0B	    ; Literal a guardar en TMR1H
    MOVWF   TMR1H	    ; Guardamos literal en TMR1H
    MOVLW   0xDC	    ; Literal a guardar en TMR1L
    MOVWF   TMR1L	    ; Guardamos literal en TMR1L
    BCF	    TMR1IF	    ; Limpiamos bandera de int. TMR1
    INCF segundos
    
    MOVF segundos, W       ; se pone el valor1 a la variable valor
    MOVWF segundos1
    CLRF decenas
    CLRF unidades
    CALL OBT_CENTENAS    ; se obtienen las decenas
    MOVLW 100
    ADDWF segundos1, F
    CALL OBT_DECENAS     ; se obtienen las decenas
    MOVLW 10
    ADDWF segundos1, F
    CALL OBT_UNIDADES    ; se obtienen las unidades
    
    RETURN
    
INT_TMR2: ;cada 500ms
    BCF	    TMR2IF	    ; Limpiamos bandera de interrupcion de TMR1
    INCF    PORTB	    ; Incremento en PORTB
    RETURN
; ------------------ configuración ----------------------
main:
    CALL CONFIG_IO
    CALL CONFIG_RELOJ
    CALL CONFIG_TMR0
    CALL CONFIG_TMR1
    CALL CONFIG_TMR2
    CALL CONFIG_INT
LOOP:
    CALL OBT_NIBBLE
    CALL SET_DISPLAY
    GOTO LOOP

CONFIG_IO:                    ; se configuran las entradas y salidas respectivas
    BANKSEL ANSELH
    CLRF ANSEL
    CLRF ANSELH
    
    BANKSEL TRISA
    CLRF TRISA
    BCF TRISB, 0
    CLRF TRISC
    CLRF TRISD
    
    BANKSEL PORTA
    CLRF PORTA
    BSF RB0
    CLRF PORTC
    CLRF PORTD
    RETURN

CONFIG_RELOJ:
    BANKSEL OSCCON		; cambiamos a banco 1
    BSF	    OSCCON, 0		; SCS -> 1, Usamos reloj interno
    BCF	    OSCCON, 6
    BSF	    OSCCON, 5
    BSF	    OSCCON, 4		; IRCF<2:0> -> 011 500kHz
    RETURN    

CONFIG_TMR0:
    BANKSEL OPTION_REG	    ; cambiamos de banco
    BCF	    T0CS	    ; TMR0 como temporizador
    BCF	    PSA		    ; prescaler a TMR0
    BSF	    PS2
    BCF	    PS1
    BSF	    PS0		    ; PS<2:0> -> 111 prescaler 1 : 64
    
    BANKSEL TMR0	    ; cambiamos de banco
    MOVLW   252
    MOVWF   TMR0	    ; 50ms retardo
    BCF	    T0IF	    ; limpiamos bandera de interrupción
    RETURN 
    
CONFIG_TMR1:
    BANKSEL T1CON	    ; Cambiamos a banco 00
    BCF	    TMR1CS	    ; Reloj interno
    BCF	    T1OSCEN	    ; Apagamos LP
    BCF	    T1CKPS1	    ; Prescaler 1:2
    BSF	    T1CKPS0
    BCF	    TMR1GE	    ; TMR1 siempre contando
    BSF	    TMR1ON	    ; Encendemos TMR1
    
    BANKSEL TMR1H           ; TMR1 a 500ms
    MOVLW   0x0B
    MOVWF   TMR1H	    ; Guardamos literal en TMR1H
    MOVLW   0xDC
    MOVWF   TMR1L	    ; Guardamos literal en TMR1L   
    RETURN
    
CONFIG_TMR2:
    BANKSEL PR2		    ; Cambiamos a banco 01
    MOVLW   243		    ; Valor para interrupciones cada 500ms
    MOVWF   PR2		    ; Cargamos litaral a PR2
    
    BANKSEL T2CON	    ; Cambiamos a banco 00
    BSF	    T2CKPS1	    ; Prescaler 1:16
    BSF	    T2CKPS0
    
    BSF	    TOUTPS3	    ;Postscaler 1:16
    BSF	    TOUTPS2
    BSF	    TOUTPS1
    BSF	    TOUTPS0
    
    BSF	    TMR2ON	    ; Encendemos TMR2
    RETURN

CONFIG_INT:
    BANKSEL PIE1	    ; Cambiamos a banco 01
    BSF	    TMR1IE	    ; Habilitamos int. TMR1
    BSF	    TMR2IE	    ; Habilitamos int. TMR2
    
    BANKSEL INTCON	    ; Cambiamos a banco 00
    BSF	    PEIE	    ; Habilitamos int. perifericos
    BSF	    GIE		    ; Habilitamos int. globales
    BSF	    T0IE	    ; Habilitamos interrupcion TMR0
    BCF	    T0IF	    ; Limpiamos bandera de TMR0
    BCF	    TMR1IF	    ; Limpiamos bandera de TMR1
    BCF	    TMR2IF	    ; Limpiamos bandera de TMR2
    RETURN

OBT_NIBBLE:
    MOVLW 0x0F
    ANDWF segundos, W
    MOVWF nibbles
    
    MOVLW 0xF0
    ANDWF segundos, W
    MOVWF nibbles+1
    SWAPF nibbles+1, F
    RETURN
    
SET_DISPLAY:
    MOVF unidades, W
    CALL tabla
    MOVWF display
    
    MOVF decenas, W
    CALL tabla
    MOVWF display+1
    RETURN

OBT_CENTENAS:                 
    MOVLW   100
    SUBWF   segundos1, W      ; se va restando el valor de 100 hasta que el valor sea negativo
    MOVWF segundos1
    BANKSEL STATUS
    BTFSS   STATUS, 0
    RETURN                ; la unica manera de salir es cuando la resta es negativa, es decir C = 0
    INCF centenas, F      ; cada vez que restamos cien se incrementa el valor de centenas
    GOTO OBT_CENTENAS    
    
OBT_DECENAS:  
    MOVLW 10
    SUBWF segundos1, W       ;se va restando el valor de 10 hasta que el valor sea negativo
    MOVWF segundos1
    BANKSEL STATUS
    BTFSS STATUS, 0
    RETURN               ; la unica manera de salir es cuando la resta es negativa, es decir C = 0
    INCF decenas, F      ; cada vez que restamos 10 se incrementa el valor de decenas
    GOTO OBT_DECENAS

OBT_UNIDADES:
    MOVF segundos1, W        ; el valor restante son las unidades, así que solo se le atribuye el valor a la variable unidades
    MOVWF unidades
    RETURN    
    
END

