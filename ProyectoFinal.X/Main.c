#include <xc.h>
#include "LibLCDXC9.h"
#define _XTAL_FREQ 1000000
#pragma config FOSC=INTOSC_EC
#pragma config WDT=OFF
#pragma config PBADEN=OFF
#pragma config LVP=OFF //Libera pin RB5


void Transmitir(unsigned char mensaje);
unsigned int conversion(unsigned char);
void cambio(unsigned int numero);
char Mostrar=0;
char x=0;
char Bloqueo=0;
int resultado=0;
int contadorB=0;
int contadorT=0;
int BPM=0;
char cent=0;
char dec =0;
char uni=0;

void main(void) {

    TXSTA=0b00100100; // Habilitar transmisión, Habilita alta velocidad
    RCSTA=0b10010000; // Encender modulo, Habilitar Recepcion
    BAUDCON=0b00001000; //Divisor de frecuencia 16 bits
    SPBRG=25; //9600 bps a 1MHz
    //Interrupcion LED

    TRISB=0;
    LATB=0;
    TRISD=0;
    LATD=0;
    
    TRISE=0;
    LATD=0;
    
    T0CON=1; //LED activación
    TMR0=3036;//LED tope 
    TMR0IF=0;//Bandera en 0
    TMR0IE=1;//Habilitar localmente
    
    ADCON0=0b00000001; //activa el modulo adc
    ADCON1=14;   // activa los pines an0 
    ADCON2=0b10000000;  //ajustar a la derecha, utilizamos 10 bits de resolucion
    
    
    
    GIE=1;//Global
    TMR0ON=1;//Encender timmer 0 LED
    __delay_ms(1000);
    ConfiguraLCD(8); //Configura teclado
    InicializaLCD();
     BorraLCD();
    MensajeLCD_Var("Bienvenido BPM");
    __delay_ms(1500);
    BorraLCD();
    MensajeLCD_Var("Tomando muestra");
    while(1){
        
        
        resultado=conversion(0);
        
        Transmitir(resultado);
         
        if(resultado<440){
            
            Bloqueo=0;
            
        }
        
        if(resultado>440 && Bloqueo==0){
        
            contadorB=contadorB+1;
            Bloqueo=1;
            
        }
        
        if(contadorT>10000){
            BPM=contadorB*6;
            contadorB=0;
            contadorT=0;
            BorraLCD();
            EscribeLCD_c('B');
            EscribeLCD_c('P');
            EscribeLCD_c('M');
            EscribeLCD_c(':');
            EscribeLCD_c(' ');
            EscribeLCD_n8(BPM, 2);
       
        }
        
       contadorT=contadorT+1;
        __delay_ms(10);
       
    }
}

void __interrupt() ISR(void){
    if(TMR0IF==1){
        TMR0=3036;//Hacer precarga
        TMR0IF=0;//Se reinicia bandera
        LATB0=LATB0^1;//Cambio estado
    }
    
    
}

unsigned int conversion(unsigned char canal){ //POR DONDE SALE
    ADCON0= (ADCON0 & 0b00000011) | (canal<<2);
    GO=1;
    while(GO==1);
    return ADRES;  //ADRESH PARA RESOLUCION DE 8 BITS, ADRES 10 BITS    
}

void cambio(unsigned int numero){
    if(numero>=100){
        cent=numero/100;
        dec = (numero%100)/10;
        uni = ((numero%100)%10);
        
        Transmitir(cent+ '0');
        Transmitir(dec + '0');
        Transmitir(uni + '0');
        
    }else if(numero>=10){
        dec= (numero/10);
        uni=(numero%10);
        
        Transmitir(dec + '0');
        Transmitir(uni + '0');
        
    }else{
        Transmitir(numero + '0');
    }
    
}

void Transmitir(unsigned char mensaje){
    while(TRMT==0);
    TXREG=mensaje;

}