#include <Arduino.h>
#include <Wire.h>
#include "ClosedCube_HDC1080.h"
#include <Adafruit_MLX90614.h>
#include <XBee.h>
#include <SoftwareSerial.h>
#include "LowPower.h"

ClosedCube_HDC1080 hdc1080;
Adafruit_MLX90614 termometroIR = Adafruit_MLX90614();
XBee xbee = XBee();

uint8_t text[100] = {};
XBeeAddress64 addr64 = XBeeAddress64(0x000000000, 0x000000000);
ZBTxRequest zbTx = ZBTxRequest(addr64, text, sizeof(text));
ZBTxStatusResponse txStatus = ZBTxStatusResponse();

float Temp_dosel=0;
float Temp_amb=0;
float HumR=0;
int ARead_A0=0; 
int ARead_A1=0; 
int ARead_A2=0; 
int ARead_A3=0;
float V_s=4.47; 
double AnVol0=0;
double AnVol1=0;
double AnVol2=0;
double AnVol3=0;
// funciones 
void printSerialNumber();
void envia(String D);
void reinit();
void delay_min(unsigned int min);

void setup() {
  Serial.begin(9600);
  xbee.setSerial(Serial);
  termometroIR.begin();
  hdc1080.begin(0x40);
  // Serial.println("ClosedCube HDC1080 Arduino Test");
  // Serial.print("Manufacturer ID=0x");
  // Serial.println(hdc1080.readManufacturerId(), HEX); 
  // Serial.print("Device ID=0x");
  // Serial.println(hdc1080.readDeviceId(), HEX);
  pinMode(LED_BUILTIN, OUTPUT);
  //printSerialNumber();
}
// the loop function runs over and over again forever




void loop() {
  digitalWrite(LED_BUILTIN, HIGH);   

  delay(100);
  Temp_amb=hdc1080.readTemperature();
  delay(100);
  HumR=hdc1080.readHumidity();
  delay(100);
  Temp_dosel = termometroIR.readObjectTempC();
  Serial.print("T dosel =");
  Serial.println(Temp_dosel);
  Serial.print("T=");
  Serial.print(Temp_amb);  
  Serial.print("C, RH=");
  Serial.print(HumR);
  Serial.println("% ");

  ARead_A0=analogRead(A0); 
  ARead_A2=analogRead(A2);  
  ARead_A1=analogRead(A1);  
  ARead_A3=analogRead(A3);

  AnVol0 = (ARead_A0*V_s) / 1023;
  AnVol1 = (ARead_A1*V_s) / 1023;
  AnVol2 = (ARead_A2*V_s) / 1023;
  AnVol3 = (ARead_A3*V_s) / 1023;

  Serial.print("SENSORS:");Serial.println(String(AnVol0)+","+String(AnVol1)+","+String(AnVol2)+","+String(AnVol3)); 
  String dataStringXbee ="SENSORS:"+String(ARead_A0)+";"+String(ARead_A1)+";"+String(ARead_A2)+";"+String(ARead_A3)+";"+ String(HumR)+";"+ String(Temp_dosel)+";"+String(Temp_amb)+";End"; 
  envia(dataStringXbee);
  Serial.println("------------------------------------------------");
  digitalWrite(LED_BUILTIN, LOW); 
  delay_min(15);
  //setup();
  reinit();
}

void reinit(){
  delay(20);
  Serial.begin(9600);
  xbee.setSerial(Serial);
  termometroIR.begin();
  hdc1080.begin(0x40);
}

void delay_min(unsigned int min){
  while(min>0){
    int sec=60;
    while(sec>0){ 
    delay(1000);
    //LowPower.powerDown(SLEEP_1S, ADC_OFF, BOD_OFF);
    sec--;
    }
    min--;
  }
}




void printSerialNumber() {

Serial.print("Device Serial Number=");
HDC1080_SerialNumber sernum = hdc1080.readSerialNumber();
char format[12];
sprintf(format, "%02X-%04X-%04X", sernum.serialFirst, sernum.serialMid, sernum.serialLast);
Serial.println(format);
}

void envia(String D)
{
  for(int i=0;i<=100;i++)
  {
    text[i]=D[i];
  }
 xbee.send(zbTx);
}