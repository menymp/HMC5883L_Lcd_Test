#include <18F4550.h>
#fuses XT,NOWDT,NOPROTECT,NOLVP
//#endif

#use Delay(Clock=48000000)
//#use rs232(baud=9600, UART1, ERRORS)
//#use rs232(baud=9600,xmit=PIN_C6,rcv=PIN_C7)//Config. del puerto serie
#use i2c(Master, sda=PIN_B0, scl=PIN_B1)

#build(reset=0x1000)
#build(interrupt=0x1008)
#org 0,0x0FFF void bootloader(){}

     #define LCD_RS_PIN      PIN_E0
     #define LCD_RW_PIN      PIN_E1  //Definicion de pines
     #define LCD_ENABLE_PIN  PIN_E2  //para LCD en puerto E control                                    ////     #define LCD_RW_PIN      PIN_E2                                    ////
     #define LCD_DATA4       PIN_D4 ////
     #define LCD_DATA5       PIN_D5 ////   PUERTO D PARA DATOS
     #define LCD_DATA6       PIN_D6 ////
     #define LCD_DATA7       PIN_D7  
#include <lcd.c>
#include <math.h>

// i2c direcciones del esclavo
#define HMC5883L_WRT_ADDR  0x3C //registros de escritura del sensor hmc, leer hoja de datos
#define HMC5883L_READ_ADDR 0x3D //registro de lectura del hmc5883

// direcciones de los registros a usar 
#define HMC5883L_CFG_A_REG 0x00 //configuracion de los registros de la lectura
#define HMC5883L_CFG_B_REG 0x01 
#define HMC5883L_MODE_REG  0x02 
#define HMC5883L_X_MSB_REG 0x03 

//------------------------------ 
// rutina de escritura de bajo nivel 
//------------------------------ 
void hmc5883l_write_reg(int8 reg, int8 data) //funcion para colocar el sistema en modo lectura
{ 
i2c_start();                  //inicializa la comunicacion i2c
i2c_write(HMC5883L_WRT_ADDR); //manda el registro de escritura al sensor
i2c_write(reg);               //una vez puesto en escritura manda el valor del registro al que se desea acceder
i2c_write(data);              //manda el dato
i2c_stop();                   //detiene la comunicacion
} 

//------------------------------ 
int8 hmc5883l_read_reg(int8 reg) //funcion para leer continuamente
{ 
int8 retval; 

i2c_start(); 
i2c_write(HMC5883L_WRT_ADDR); 
i2c_write(reg); 
i2c_start(); 
i2c_write(HMC5883L_READ_ADDR); 
retval = i2c_read(0); 
i2c_stop(); 

return(retval); 
} 

//------------------------------ 
typedef struct //estructura que retiene los tipos de datos para los ejes x,y,z de modo que sea mas ordenada la adquisicion de los datos
{ 
int16 x; 
int16 y; 
int16 z; 
}hmc5883l_result; //header de la estructura para los campos magneticos

// This global structure holds the values read 
// from the HMC5883L x,y,z registers. 
hmc5883l_result compass = {0,0,0}; 

//------------------------------ 
void hmc5883l_read_data(void) 
{ 
//el sensor trabaja con registros de 8 bits altos y bajos 
//dando como resultado un total de 16 bits
//registros del campo magnetico eje x
int8 x_lsb; 
int8 x_msb; 
//registros del campo magnetico eje y
int8 y_lsb; 
int8 y_msb; 
//registros del campo magnetico eje z
int8 z_lsb; 
int8 z_msb; 
//adquisicion de los datos
i2c_start(); //se inicia la comunicacion
i2c_write(HMC5883L_WRT_ADDR);  //se pone en  modo escritura
i2c_write(HMC5883L_X_MSB_REG); //se escribe el registro 
i2c_start();                   //se inicializa la comunicacion
i2c_write(HMC5883L_READ_ADDR); //se envia la solicitud de lectura

x_msb = i2c_read(); //lee el nibble bajo
x_lsb = i2c_read(); //lee el nible alto

y_msb = i2c_read(); 
y_lsb = i2c_read(); 

z_msb = i2c_read(); 
z_lsb = i2c_read(0);   // do a NACK on last read 

i2c_stop(); //detiene la comunicacion
  
// se convolucionan los registros bajo y alto en uno solo de 16 bits
compass.x = make16(x_msb, x_lsb); //se asignan a la estructura
compass.y = make16(y_msb, y_lsb); 
compass.z = make16(z_msb, z_lsb); 
} 


//========================================== 
void main() 
{ 
 delay_ms(500);  // un delay inicial para dar tiempo
 lcd_init(); 
 //printf(lcd_putc,"\n\rInicio:\n\r"); 
 delay_ms(500);
 //lcd_gotoxy(1,2);
// inicializacion del sensor 
// segun el fabricante estas configuraciones se emplean para sostener lecturas
// continuas de modo que solo se configura una vez como esclavo y este enviara los 
// valores continuamente
hmc5883l_write_reg(HMC5883L_CFG_A_REG, 0x70); 
hmc5883l_write_reg(HMC5883L_CFG_B_REG, 0xA0); 
hmc5883l_write_reg(HMC5883L_MODE_REG, 0x00); 

// lectura en tiempo real de los valores 
// se recomientda una espera de 70ms aproximadamente entre medicion para el sensor
// segun el fabricante
//int iniciar=0;

while(1) 
  { 
   float angulo=0.0;
    lcd_gotoxy(1,1);
   delay_ms(100);  //espera
   hmc5883l_read_data();    //se leen los datos
   int16 valx=compass.x;
   int16 valz=compass.z;
   
   angulo=atan2(valx,valz);
   angulo=angulo*57.2958;
   //printf(lcd_putc,"\n\r%f\n\r", angulo); //impresion de los valores por puerto serial
   printf(lcd_putc,"\n\r%lx,%lx\n\r"compass.x,compass.y);
  }    
    
}
