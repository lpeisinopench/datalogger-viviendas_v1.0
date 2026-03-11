#include <OneWire.h>                
#include <DallasTemperature.h>
#include <SD.h>
#include <Wire.h>
#include <RTClib.h>
#include <avr/wdt.h>

//PINES DIGITALES
//*********MEDICION****************
// Define to which pin of the Arduino the 1-Wire bus is connected:
/////*******NOTA*******/////
/////*******NO se puede usar el pin digital 10 para el sensor DS18B20 con protocolo onw_wire, si es que se utiliza el bus SPI*******/////
/////*******En este caso el bus SPI es utilizado por la targeta de memoria*******/////
#define ONE_WIRE_BUS 0 

// Create a new instance of the oneWire class to communicate with any OneWire device:
OneWire oneWire(ONE_WIRE_BUS);

// Pass the oneWire reference to DallasTemperature library:
DallasTemperature sensors(&oneWire);

#define NUM_SENSORS 18              // the number of DS18B20 sensors on the 1-Wire bus.

//*********COMUNICACIONES****************
File logFile; //archivo donde grabará el log

int ledpin = 4;       //Pin digital de comunicaciones

//*********CONTROL****************
RTC_DS3231 rtc; //control del tiempo 

//Pines digitales a los cuales va conectado el multiplexor
const int muxS0 = 8;
const int muxS1 = 7;
const int muxS2 = 6;
const int muxS3 = 5;

//PINES ANALOGICOS
//*********MEDICION****************
int potPin0 = A3; // Selecciona la entrada para la termocupla K
//const int muxSIG = A3; // pin analogico de la entrada del multiplexor EQUIVALENTE A potPin0

//*********COMUNICACIONES****************
//PINES ANALOGICOS


//VARIABLES

const int numTermocuplas = 12; //cantidad de termocuplas conectadas al multiplexor, en total pueden ser 16

//inicializacion de los vectores de varialbes para las termocuplas
int average[numTermocuplas];
float v2[numTermocuplas];
float v3[numTermocuplas];
   
//medicion de la temperatura con termocupla tipo T
const float voltage = 4970; // El voltaje real en el punto 5Vcc de tu placa Arduino unidades de mV
const float bias = 0; // En grados centigrados (°C), para ajustar el cero.
//const float bias = 39.4; // En grados centigrados (°C), para ajustar el cero.

//Rational function curve fits of NIST ITS-90 thermocouple data are compared to polynomial fits.
//T (°C) a partir de V (mV)
//T = T0 + ((V-VO)*(p1+(V-VO)*(p2+(V-VO)*(p3+(V-VO)*p4)))) / (1+(V-VO)*(q1+(V-VO)*(q2+(V-VO)*q3))) 
const float T0 = 1.3500000E+02;   
const float V0 = 5.9588600E+00;   
const float p1 = 2.0325591E+01;   
const float p2 = 3.3013079E+00;   
const float p3 = 1.2638462E-01;   
const float p4 = -8.2883695E-04;  
const float q1 =  1.7595577E-01;  
const float q2 = 7.9740521E-03;
const float q3 = 0.0;
//Sensitivity (uV/°C)= 49,2

float temp[numTermocuplas]; //Declaramos la variable Temperatura, de las termocuplas
float temp2[NUM_SENSORS]; //Declaramos la variable Temperatura, de los sensores DS18B20

float deltaV[numTermocuplas];
float delta_T[numTermocuplas];

float tmedia; //Variable para calcular media
float tmedia2; //Variable para calcular media
int cnt; //temperatura media y contador

//Variables del multiplexor
int SetMuxChannel(byte channel)
{
   digitalWrite(muxS0, bitRead(channel, 0));
   digitalWrite(muxS1, bitRead(channel, 1));
   digitalWrite(muxS2, bitRead(channel, 2));
   digitalWrite(muxS3, bitRead(channel, 3));
}

//Direcciones de los sensores DS18B20
//DeviceAddress 
const DeviceAddress sensor_address[NUM_SENSORS] = {
{0x28, 0x38, 0x6 , 0x39, 0x65, 0x20, 0x1, 0x1A},
{0x28, 0x96, 0xBF, 0x1B, 0x65, 0x20, 0x1, 0x9E},
{0x28, 0xB7, 0xAC, 0xFD, 0x64, 0x20, 0x1, 0xC6},
{0x28, 0xD7, 0x61, 0x7 , 0x65, 0x20, 0x1, 0xA},
{0x28, 0x72, 0xBE, 0x33, 0x65, 0x20, 0x1, 0xCA},
{0x28, 0x52, 0xC , 0xFD, 0x64, 0x20, 0x1, 0x37},
{0x28, 0x68, 0x3A, 0x2D, 0x65, 0x20, 0x1, 0xA7},
{0x28, 0x1B, 0xE9, 0x1C, 0x65, 0x20, 0x1, 0xB1},
{0x28, 0xD7, 0xD6, 0x23, 0x65, 0x20, 0x1, 0x6A},
{0x28, 0x56, 0x1A, 0x1B, 0x65, 0x20, 0x1, 0xE0},
{0x28, 0xA0, 0x43, 0x3E, 0x65, 0x20, 0x1, 0xB7},
{0x28, 0x16, 0x8A, 0x1A, 0x65, 0x20, 0x1, 0xAD},
{0x28, 0x33, 0xA8, 0xF8, 0x64, 0x20, 0x1, 0x6E},
{0x28, 0xB4, 0x26, 0x16, 0x65, 0x20, 0x1, 0x9F},
{0x28, 0xAB, 0x69, 0x3B, 0x65, 0x20, 0x1, 0x8B},
{0x28, 0x71, 0x4A, 0x2F, 0x65, 0x20, 0x1, 0x0},
{0x28, 0x20, 0x22, 0xFA, 0x64, 0x20, 0x1, 0x1D},
{0x28, 0x7B, 0x9B, 0x3 , 0x65, 0x20, 0x1, 0x3E},
};

//DeviceAddress 
int offset[NUM_SENSORS] = {
//918,  616, 860, 559, 642, 709, 482, 417, 488, 595, 695, 704, 455, 167, 834, 629, 522, 458};
668, 426, 550, 559, 642, 549, 482, 417, 358, 595, 695, 704, 455, 167, 834, 629, 522, 458};


void setup(){
  wdt_disable();
   
   pinMode(ledpin, OUTPUT);    // sets the digital pin 4 as output

   pinMode(muxS0, OUTPUT);  //set de output digital pins for multiplexer
   pinMode(muxS1, OUTPUT);
   pinMode(muxS2, OUTPUT);
   pinMode(muxS3, OUTPUT);


  for (int i = 0; i <= 10; i++) {
  digitalWrite(ledpin, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(100);                       // wait for a second
  digitalWrite(ledpin, LOW);    // turn the LED off by making the voltage LOW
  delay(100);                       // wait for a second
  }


//inicializacion del modulo RTC de control del tiempo
  digitalWrite(ledpin, HIGH);   // turn the LED on (HIGH is the voltage level)
  rtc.begin();
  delay(2000);                       // wait for a second
  digitalWrite(ledpin, LOW);    // turn the LED off by making the voltage LOW
  delay(2000);                       // wait for a second
  
//inicializacion de los sensores de temperatur DS18B20
  digitalWrite(ledpin, HIGH);   // turn the LED on (HIGH is the voltage level)
  sensors.begin();
  delay(2000);                       // wait for a second
  digitalWrite(ledpin, LOW);    // turn the LED off by making the voltage LOW
  delay(2000);                       // wait for a second



 // Serial.begin(9600);

//inicializacion de las varialbes de las termocuplas
for (int j = 0; j < numTermocuplas; j++)
   {
average[j] = 0 ;
v2[j] = 0 ;
v3[j] = 0 ;
temp[j] = 0;
deltaV[j] = 0;
delta_T[j] = 0;
  }
 

//inicializacion de la tarjeta de memoria

  digitalWrite(ledpin, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(2000);                       // wait for a second
  digitalWrite(ledpin, LOW);    // turn the LED off by making the voltage LOW
  delay(2000);                       // wait for a second

  if (!SD.begin(9))
  {
  digitalWrite(ledpin, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(60000);                       // wait for one minute
  digitalWrite(ledpin, LOW);    // turn the LED off by making the voltage LOW
  return;
  }
  //parpadeo del led
  for (int i = 0; i <= 10; i++) {
  digitalWrite(ledpin, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(100);                       // wait for a second
  digitalWrite(ledpin, LOW);    // turn the LED off by making the voltage LOW
  delay(100);                       // wait for a second
  }

logFile = SD.open("datalog.dat", FILE_WRITE);
   if (logFile) {
    logFile.println("#reiniciando");
    logFile.close();

  digitalWrite(ledpin, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);                       // wait for a second
  digitalWrite(ledpin, LOW);    // turn the LED off by making the voltage LOW

  }
  else {
    digitalWrite(ledpin, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(100);                       // wait for a second
  digitalWrite(ledpin, LOW);    // turn the LED off by making the voltage LOW
  }

  delay(4000);                       // wait for 4 second
    wdt_enable(WDTO_8S);
    }

void logValue(DateTime date)
{
   logFile.print(date.year(), DEC);
   logFile.print('/');
   logFile.print(date.month(), DEC);
   logFile.print('/');
   logFile.print(date.day(), DEC);
   logFile.print("\t");
   logFile.print(date.hour(), DEC);
   logFile.print(':');
   logFile.print(date.minute(), DEC);
   logFile.print(':');
   logFile.print(date.second(), DEC);
}

void loop(){
 wdt_reset();
///************ESCRITURA EN LA TARJETA DE MEMORIA************///           
  delay(1000);
 // Abrir archivo y escribir valor
logFile = SD.open("datalog.dat", FILE_WRITE);
   if (logFile) {
      DateTime now = rtc.now();
      logValue(now);
      logFile.close();

  digitalWrite(ledpin, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);                       // wait for a second
  digitalWrite(ledpin, LOW);    // turn the LED off by making the voltage LOW
}
  else {
    digitalWrite(ledpin, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(60000);                       // wait for one minute
  digitalWrite(ledpin, LOW);    // turn the LED off by making the voltage LOW
  }

//bucle para leer cada entrada del multiplexor -- en otras palabras, para leer cada una de las termocuplas
   for (byte i = 0; i < numTermocuplas; i++)
   {
      SetMuxChannel(i);
 //     byte muxValue = analogRead(muxSIG);
//      Serial.print(muxValue);
 //     Serial.print("\t");
   

delay(2000);  //para estabilizar la medicion
wdt_reset();
   //MEDICION Y CALCULO DE LA TEMPERATURA
   //Termocupla tipo T en arreglo de termocupla diferencial
   
   tmedia=0; //Inicializamos variable
   tmedia2=0;
   // medimos la temperatura 10 veces y la almacenamos en tmedia
    for ( cnt=0; cnt<20; cnt++){              //cnt==10000
        delay(250);
        tmedia = tmedia + analogRead(potPin0);
   }
    wdt_reset();

   
 // calculamos la media de las medidas
    tmedia = tmedia / cnt;
 
    average[i] = tmedia;


///************T TERMOCUPLA************///
   //Conversion de V a mV :)
    //float v2 = (voltage*float(tmedia))/1024.0f;    //e0
    v2[i] = (voltage*float(average[i]))/1024.0f;    //e0
    v3[i] = v2[i]/1205.2f ;     // v3 es el voltaje de la termocupla en milivoltios GANANCIA DEL AMPLIFICADOR DE INSTRUMENTACIÖN = 1205.2
                                     // conversion de volt a milovolt 

//T = T0 + ((V-VO)*(p1+(V-VO)*(p2+(V-VO)*(p3+(V-VO)*p4)))) / (1+(V-VO)*(q1+(V-VO)*(q2+(V-VO)*q3))) 
  deltaV[i] = v3[i] -V0;
  temp[i] = T0 + (deltaV[i]*(p1+deltaV[i]*(p2+deltaV[i]*(p3+deltaV[i]*p4)))) / (1+deltaV[i]*(q1+deltaV[i]*(q2+deltaV[i]*q3))) ;

  //ajuste de offset/bias de la termocupla
  //delta_T[i] = temp[i] - bias;
  if(i==11){
  delta_T[i] = temp[i];
  }
  else {
  delta_T[i] = temp[i] - delta_T[11]; //delta_T[11] corresponde a una termocupla diferencial que sus puntas estan unidas (isotérmicas)
  }

  
//**if (c > 10){                
///************ESCRITURA EN LA TARJETA DE MEMORIA************///           
  delay(500);
wdt_reset();
 // Abrir archivo y escribir valor
logFile = SD.open("datalog.dat", FILE_WRITE);
   if (logFile) {
 //     DateTime now = rtc.now();
//   logFile.print("\t");
//   logFile.print(temp2[i]);
   logFile.print("\t");
   logFile.print(delta_T[i]);  
   logFile.close();

  digitalWrite(ledpin, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);                       // wait for a second
  digitalWrite(ledpin, LOW);    // turn the LED off by making the voltage LOW
  }
  else {
    digitalWrite(ledpin, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(60000);                       // wait for a second
  digitalWrite(ledpin, LOW);    // turn the LED off by making the voltage LOW
  }

  //Serial.print(v2[i]);
  //Serial.print("\t");
  //Serial.print(v3[i]);
  //Serial.print("\t");
  //Serial.print(readIndex);
  //Serial.print("\t");
  //Serial.print(temp2[i]);
  //Serial.print("\t");
  //Serial.print(delta_T[i]);
  //Serial.print("\t");

//**delay(1000);

//**}
   }

     //parpadeo del led
  for (int i = 0; i <= 10; i++) {
  digitalWrite(ledpin, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(100);                       // wait for a second
  digitalWrite(ledpin, LOW);    // turn the LED off by making the voltage LOW
  delay(100);                       // wait for a second
  }

//**if (c > 10){                

  
 // digitalWrite(ledpin, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(2000);
  wdt_reset();
  
// Send the command for all devices on the bus to perform a temperature conversion:
  sensors.requestTemperatures();

  for (int k = 0; k < NUM_SENSORS; k++) {
    delay(500);
   temp2[k] = sensors.getTempC(sensor_address[k]);
   temp2[k] = temp2[k] + (0.001* offset[k]);
 
///************ESCRITURA EN LA TARJETA DE MEMORIA************///           
//*  delay(1000);
 // Abrir archivo y escribir valor
logFile = SD.open("datalog.dat", FILE_WRITE);
   if (logFile) {
 //     DateTime now = rtc.now();
   logFile.print("\t");
   logFile.print(temp2[k]);
   logFile.close();

  digitalWrite(ledpin, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);                       // wait for a second
  digitalWrite(ledpin, LOW);    // turn the LED off by making the voltage LOW
 wdt_reset();
  }
  else {
    digitalWrite(ledpin, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(60000);                       // wait for a second
  digitalWrite(ledpin, LOW);    // turn the LED off by making the voltage LOW
 }
}
//**}
      
///************ESCRITURA EN LA TARJETA DE MEMORIA************///           
  delay(1000);
 // Abrir archivo y escribir valor
logFile = SD.open("datalog.dat", FILE_WRITE);
   if (logFile) {
   logFile.println();
   logFile.close();

  digitalWrite(ledpin, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);                       // wait for a second
  digitalWrite(ledpin, LOW);    // turn the LED off by making the voltage LOW

  }
  else {
    digitalWrite(ledpin, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(60000);                       // wait for a second
  digitalWrite(ledpin, LOW);    // turn the LED off by making the voltage LOW
  }

 //  Serial.println();

    //parpadeo del led
  for (int i = 0; i <= 20; i++) {
  digitalWrite(ledpin, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(100);                       // wait for a second
  digitalWrite(ledpin, LOW);    // turn the LED off by making the voltage LOW
  delay(100);                       // wait for a second
  }
}       
