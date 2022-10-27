/*************************************************** 
  This is a library for the Adafruit PT100/P1000 RTD Sensor w/MAX31865

  Designed specifically to work with the Adafruit RTD Sensor
  ----> https://www.adafruit.com/products/3328

  This sensor uses SPI to communicate, 4 pins are required to  
  interface
  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/
// Implemented on Arduino Nano
//
#include <Adafruit_MAX31865.h>

// Use software SPI: CS, DI, DO, CLK
Adafruit_MAX31865 thermo = Adafruit_MAX31865(9, 10, 11, 12);
// use hardware SPI, just pass in the CS pin
//Adafruit_MAX31865 thermo = Adafruit_MAX31865(10);

// The value of the Rref resistor. Use 430.0 for PT100 and 4300.0 for PT1000
#define RREF      4300.0
// The 'nominal' 0-degrees-C resistance of the sensor
// 100.0 for PT100, 1000.0 for PT1000
#define RNOMINAL  1000.0
#define PIN_ENABLE 7 //Heating enabled when this pin is high
#define PIN_SSR 5   //High when SSR is activated
#define PIN_TEMP 6 //Return the analog value of the temperature of the door
float aerror[20]; // 2 point should be enough
float ainterval[20];
float temp = 0;
float ratio = 0;
int k = 0;
float error = 0;
float derror = 0;
float ierror = 0;
float time_tot = 1;
float time_0[20];
float interval = millis();
bool heating_enable = true;
float Kp = 1.0;
float Kd = 150.0; //PID parameters to be tuned
float Ki = 0.01;
float setpoint = 38;


void setup() {
  Serial.begin(115200);
  Serial.println("Adafruit MAX31865 PT100 Sensor Test!");
  pinMode(PIN_SSR,OUTPUT); //Pin to activate the heating
  pinMode(PIN_ENABLE,INPUT); //Pin controlled by the ME4 to enable heating
  pinMode(PIN_TEMP,OUTPUT); //Pin to output temp to the ME4
  thermo.begin(MAX31865_4WIRE);  // set to 2WIRE or 4WIRE as necessary

  //analogWrite(PIN_TEMP,128);
}


void loop() {

  uint16_t rtd = thermo.readRTD();
  
  //Serial.print("RTD value: "); Serial.println(rtd);
  ratio = rtd;
  temp = thermo.temperature(RNOMINAL,RREF);
  error = setpoint-temp;
  interval = millis()-interval;
  ratio /= 32768;
  //Serial.print("Ratio = "); Serial.println(ratio,8);
  //Serial.print("Resistance = "); Serial.println(RREF*ratio,8);
  //Serial.print("Temperature = "); Serial.println(thermo.temperature(RNOMINAL, RREF));

  // Check and print any faults
  uint8_t fault = thermo.readFault();
  if (fault) {
    Serial.print("Fault 0x"); Serial.println(fault, HEX);
    if (fault & MAX31865_FAULT_HIGHTHRESH) {
      Serial.println("RTD High Threshold"); 
    }
    if (fault & MAX31865_FAULT_LOWTHRESH) {
      Serial.println("RTD Low Threshold"); 
    }
    if (fault & MAX31865_FAULT_REFINLOW) {
      Serial.println("REFIN- > 0.85 x Bias"); 
    }
    if (fault & MAX31865_FAULT_REFINHIGH) {
      Serial.println("REFIN- < 0.85 x Bias - FORCE- open"); 
    }
    if (fault & MAX31865_FAULT_RTDINLOW) {
      Serial.println("RTDIN- < 0.85 x Bias - FORCE- open"); 
    }
    if (fault & MAX31865_FAULT_OVUV) {
      Serial.println("Under/Over voltage"); 
    }
    thermo.clearFault();
  }
  //Serial.println();

  analogWrite(PIN_TEMP,int(ceil((temp-12.5)*255/30))); //Between 12.5 and 42.5 C -> 3.4V when not plugged by USB at 37.5
  if(k<20){
    aerror[k] = error; //array_error °C
    ainterval[k] = interval; //array_interval ms
    //time_0[k] = time_0[k-1] + interval;
    k++;
  }
  else{
    for(int i = 0; i<19;i++){
      aerror[i] = aerror[i+1];
      ainterval[i] = ainterval[i+1];
      time_tot += ainterval[i];
      time_0[i] = time_tot;
    }
    aerror[19] = error;
    ainterval[19] = interval;
    time_tot += interval;
    time_0[19] = time_tot;
    derror = (aerror[19]-aerror[10])/(time_0[19]-time_0[10])*1000;
    ierror = (aerror[19]*ainterval[19]+aerror[18]*ainterval[18])/(ainterval[19]+ainterval[18]);
  }
  // PID controller = Kp*e + Ki*integral(e) + Kd*de/dt
  // error > 0 when temp < setpoint
  //&& !digitalRead(PIN_ENABLE)
  Serial.print("Setpoint : ");Serial.print(Kp*error + Kd*derror + Ki*ierror + temp);
  Serial.print(" / Temperature : ");Serial.print(temp);
  Serial.print(" / derror : ");Serial.print(Kd*derror);
  Serial.print(" / error : ");Serial.print(error);
  Serial.println();
  //if(Kp*error + Kd*derror + Ki*ierror > 0 && digitalRead(PIN_ENABLE)){
  if(Kp*error + Kd*derror + Ki*ierror > 0){
    digitalWrite(PIN_SSR,HIGH);
    //Serial.println("SSR is active, door being heated");
  }
  else{digitalWrite(PIN_SSR,LOW);
  Serial.println("Temp control  not activated");
  }
  delay(20); //Wait 50 ms to be sure that the SSR was activated or deactivated
}
