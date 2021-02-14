#include <Arduino.h>

#include <OneWire.h>
#include <DallasTemperature.h>

#include "DFRobot_EC.h"
#include <EEPROM.h>

// Defining Pins of Arduino

#define ONE_WIRE_BUS 2           // Data wire of temperature sensor is plugged into pin 2 on the Arduino
#define Turbidity_Pin A0            //Turbidity Analog output to Arduino Analog Input 0
#define SensorPin A1            //pH meter Analog output to Arduino Analog Input 1
#define DO_PIN A2                //DO sensor output pin to Analog inpit 2
#define EC_PIN A3              //EC pin conected to A3


// Declaring variables


// For Temperature sensor

// Setup a oneWire instance to communicate with any OneWire devices
// (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);


// for turbidity

float ntu;

//for PH Sensor

#define Offset 0.00            //deviation compensate
//#define LED 13
#define samplingInterval 20
#define printInterval 800
#define ArrayLenth  40    //times of collection
int pHArray[ArrayLenth];   //Store the average value of the sensor feedback
int pHArrayIndex=0;

//for DO sensors


#define VREF 5000    //VREF (mv)
#define ADC_RES 1024 //ADC Resolution

//Single-point calibration Mode=0
//Two-point calibration Mode=1
#define TWO_POINT_CALIBRATION 0

//#define READ_TEMP (25) //Current water temperature ℃, Or temperature sensor function

//Single point calibration needs to be filled CAL1_V and CAL1_T
#define CAL1_V (1200) //mv
#define CAL1_T (25)   //℃
//Two-point calibration needs to be filled CAL2_V and CAL2_T
//CAL1 High temperature point, CAL2 Low temperature point
//#define CAL2_V (1300) //mv
//#define CAL2_T (15)   //℃

const uint16_t DO_Table[41] = {
    14460, 14220, 13820, 13440, 13090, 12740, 12420, 12110, 11810, 11530,
    11260, 11010, 10770, 10530, 10300, 10080, 9860, 9660, 9460, 9270,
    9080, 8900, 8730, 8570, 8410, 8250, 8110, 7960, 7820, 7690,
    7560, 7430, 7300, 7180, 7070, 6950, 6840, 6730, 6630, 6530, 6410};

uint8_t Temperaturet;
uint16_t ADC_Raw;
uint16_t ADC_Voltage;
uint16_t DO;



// For EC sensor

float voltage,ecValue,temperature;
DFRobot_EC ec;


// Function Definations




double avergearray(int* arr, int number){
  int i;
  int max,min;
  double avg;
  long amount=0;
  if(number<=0){
    Serial.println("Error number for the array to avraging!/n");
    return 0;
  }
  if(number<5){   //less than 5, calculated directly statistics
    for(i=0;i<number;i++){
      amount+=arr[i];
    }
    avg = amount/number;
    return avg;
  }else{
    if(arr[0]<arr[1]){
      min = arr[0];max=arr[1];
    }
    else{
      min=arr[1];max=arr[0];
    }
    for(i=2;i<number;i++){
      if(arr[i]<min){
        amount+=min;        //arr<min
        min=arr[i];
      }else {
        if(arr[i]>max){
          amount+=max;    //arr>max
          max=arr[i];
        }else{
          amount+=arr[i]; //min<=arr<=max
        }
      }//if
    }//for
    avg = (double)amount/(number-2);
  }//if
  return avg;
}


//Read DO Function

float readDO(uint32_t voltage_mv, uint8_t temperature_c)
{
#if TWO_POINT_CALIBRATION == 0
  uint16_t V_saturation = (uint32_t)CAL1_V + (uint32_t)35 * temperature_c - (uint32_t)CAL1_T * 35;
  return (voltage_mv * DO_Table[temperature_c] / V_saturation);
#else
  uint16_t V_saturation = (int16_t)((int8_t)temperature_c - CAL2_T) * ((uint16_t)CAL1_V - CAL2_V) / ((uint8_t)CAL1_T - CAL2_T) + CAL2_V;
  return (voltage_mv * DO_Table[temperature_c] / V_saturation);
#endif
}



// Sending Data to Nodemcu

#include <SoftwareSerial.h>
SoftwareSerial sport(3,4);//rx=13, tx= 4


//Structure defining
struct WQ_Parameters {
  float WQ_Temperature;
  float WQ_Turbidity;
  float WQ_pH;
  float WQ_DO;
  float WQ_EC;

};
WQ_Parameters wqms;
byte buf[sizeof(wqms)];




void setup(void)
{
  // start serial port
  Serial.begin(9600);
  Serial.println("Water Quality Monitoring System");

  // Start up the library
  sensors.begin();
    ec.begin();
sport.begin(9600);




}


void loop(void)
{
  // call sensors.requestTemperatures() to issue a global temperature
  // request to all devices on the bus
//  Serial.print(" Requesting temperatures...");



  sensors.requestTemperatures(); // Send the command to get temperatures
//  Serial.println("DONE");

  Serial.print("Temp = ");
  temperature= sensors.getTempCByIndex(0);
  Serial.print(temperature); // Why "byIndex"?
  Serial.print(" C*     ");
    // You can have more than one IC on the same bus.
    // 0 refers to the first IC on the wire


//Turbisity SensorPin


    int sensorValue = analogRead(Turbidity_Pin);

    float voltage_turb = sensorValue * (5.0 / 1024.0);
  //  Serial.print("Turbidity in Voltage is: ");
//  Serial.println(voltage_turb);
  Serial.print("Turbidity = ");
  ntu = -1120.4*square(voltage_turb)+5742.3*voltage_turb-4353.8;
Serial.print(ntu);
Serial.print(" NTU     ");

//PH sensor

  static unsigned long samplingTime = millis();
  static unsigned long printTime = millis();
  static float pHValue,voltage;
  if(millis()-samplingTime > samplingInterval)
  {
      pHArray[pHArrayIndex++]=analogRead(SensorPin);
      if(pHArrayIndex==ArrayLenth)pHArrayIndex=0;
      voltage = avergearray(pHArray, ArrayLenth)*5.0/1024;
      pHValue = 3.5*voltage+Offset;
      samplingTime=millis();
  }
  if(millis() - printTime > printInterval)   //Every 800 milliseconds, print a numerical, convert the state of the LED indicator
  {
  //  Serial.print("Voltage:");
        //Serial.print(voltage,2);
        Serial.print("pH = ");
    Serial.print(pHValue,2);
      //  digitalWrite(LED,digitalRead(LED)^1);
      Serial.print("     ");
        printTime=millis();
  }


  // DO SensorPin


//  Temperaturet = (uint8_t)READ_TEMP;
  ADC_Raw = analogRead(DO_PIN);
  ADC_Voltage = uint32_t(VREF) * ADC_Raw / ADC_RES;

  //Serial.print("Temperaturet:\t" + String(temperature) + "\t");
  //Serial.print("ADC RAW:\t" + String(ADC_Raw) + "\t");
  //Serial.print("ADC Voltage:\t" + String(ADC_Voltage) + "\t");
  float DO = (readDO(ADC_Voltage, temperature))/1000;
  Serial.print("DO = " + String(DO) + " mg/L     ");


  //EC SensorPin


  static unsigned long timepoint = millis();
  if(millis()-timepoint>1000U)  //time interval: 1s
  {
    timepoint = millis();
    voltage = analogRead(EC_PIN)/1024.0*5000;  // read the voltage
    //temperature = readTemperature();  // read your temperature sensor to execute temperature compensation
    ecValue =  ec.readEC(voltage,temperature);  // convert voltage to EC with temperature compensation
  //  Serial.print("temperature:");
  //  Serial.print(temperature,1);
    Serial.print("EC = ");
    Serial.print(ecValue,2);
    Serial.println(" mS/cm");
  }
//  ec.calibration(voltage,temperature);  // calibration process by Serail CMD


wqms.WQ_Temperature= temperature;
wqms.WQ_Turbidity = ntu;
wqms.WQ_pH = pHValue;
wqms.WQ_DO = DO;
wqms.WQ_EC = ecValue;



 memcpy(buf, &wqms, sizeof(wqms));
 sport.write(buf, sizeof(wqms));

 Serial.print("Data has been sent");

    delay(1000);



}
