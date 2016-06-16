#include <Wire.h>  // must be incuded here so that Arduino library object file references work
#include <RtcDS3231.h>
#include <SD.h>      // includes the Arduino SD Library 
#include <SoftwareSerial.h>
// pin configurations for SparkFun Vernier Shield
// A1 = Analog 1
// A2 = Analog 2
#define A1_5V 0

#define A2_5V 2

SoftwareSerial bluetooth(7, 6); //puerto serial especial para bluetooth
RtcDS3231 reloj;

char * filename = "tesis2.csv";  /* sets the filename for data - change this
  if you want to use a different file name
  data will be concatenated onto the existing
  file if it exists */


float Temp;
unsigned long thermistor;
int rawAnalogReading;

long tiempoPasado;
unsigned long timeRef;      // reference for starting time
unsigned long timeInterval;
unsigned long elapsedTime;
unsigned long ndx = 0;
int Frecuencia = 1;
const int ledPin = 13;      // LED pin on Vernier Shield

/* Global Variable declarations for SD Card Shield */
const int chipSelect = 8;
File dataFile;

int muxLSB = 10; //low byte of multiplexer
int muxMSB = 11; //high byte of multiplexer

int SensorRaw[2];
float SensorVoltage[2];
float VCC = 5.0;

float PhRecibido;

void setup()
{
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  bluetooth.begin(9600);
  reloj.Begin();

  pinMode(muxLSB, OUTPUT);
  pinMode(muxMSB, OUTPUT);

  /***********************
     / Setup SD Card
    /***********************/
  pinMode(chipSelect, OUTPUT);

  Serial.print("Initializing SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect))
  {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");

  Serial.println();

  /***********************
    / Print data header
    /***********************/
  Serial.println(" ");
  Serial.println("Vernier Format 2");
  Serial.println("Raw Readings taken using Ardunio");
  Serial.println("Data Set");
  Serial.print("Time");

  Serial.print("\t"); //tab character
  Serial.print ("Chan1");
  //Serial.print("\t"); //tab character
  //Serial.print("Chan2");


  //Serial.println("");
  //Serial.print("s");

  Serial.print("\t"); //tab character
  Serial.print ("Cยบ");
  //Serial.print("\t"); //tab character
  //Serial.print ("V");
  Serial.println();

  /*************************
     / Print header to SD Card
    /*************************/

  dataFile = SD.open(filename, FILE_WRITE);
  if (dataFile) // if it opens sucessfully
  {
    dataFile.close();
  }
  else  // if(datafile) -- error opening SD card
  {
    Serial.println("error opening file.");
    Serial.println();
  }



}

void loop()
{

  if (bluetooth.available()) {
    char a = bluetooth.read();
    if (a == 's') {
      int sensorAcalibrar = bluetooth.read() - '0';
      if (sensorAcalibrar <= 6 && sensorAcalibrar >= 0) {
        bluetooth.println("coloque el sensor en la sustancia 1");
        bluetooth.println("ingrese el valor de la sustancia 1");
        do {
          bluetooth.read();
        } while (bluetooth.available());
        delay(2000);
        do {

        } while (!bluetooth.available());
        float y1 = bluetooth.parseFloat();
        float x1 = analogRead(sensorAcalibrar);
        bluetooth.println("coloque el sensor en la sustancia 2");
        bluetooth.println("ingrese el valor de la sustancia 2");
        do {
          bluetooth.read();
        } while (bluetooth.available());
        delay(2000);
        do {

        } while (!bluetooth.available());
        float y2 = bluetooth.parseFloat();
        float x2 = analogRead(sensorAcalibrar);
        Serial.print("x1=");
        Serial.print(x1);
        Serial.print("y1=");
        Serial.println(y1);
        Serial.print("x2=");
        Serial.print(x2);
        Serial.print("y2=");
        Serial.println(y2);
        delay(500);

        float m = (y2-y1)/(x2-x1);
        Serial.print("la pendiente es:");
        Serial.println(m);
        

        


      }
    }
    else if (a == 'd') {
      dataFile = SD.open(filename);
      if (dataFile) {
        while (dataFile.available()) // if it opens sucessfully
        {
          char letra = dataFile.read();
          if (letra == '/n')
            delay(500);
          bluetooth.write(letra);
        }
        dataFile.close();
      }
    }
    else if ( a == 'c') {
      SD.remove(filename);
      bluetooth.println("Borrada SD");
    }
  }

  RtcDateTime TiempoMinutos = reloj.GetDateTime();

  if ( TiempoMinutos.Minute() != tiempoPasado && TiempoMinutos.Minute() % Frecuencia == 0) {
    tiempoPasado = TiempoMinutos.Minute();
    ndx++;
    digitalWrite(ledPin, LOW); // blink the LED off to show data being taken.

    // Read in sensor values
    SensorRaw[0] = analogRead(A1_5V);
    SensorRaw[1] = analogRead(A2_5V);

    //********************************************************************************
    thermistor = resistance(SensorRaw[0]);     // converts raw analog value to a resistance
    Temp = steinharthart(thermistor);              // Applies the Steinhart-hart equation
    //********************************************************************************

    // Convert to voltage values
    SensorVoltage[0] = SensorRaw[0] * VCC / 1023.0;
    SensorVoltage[1] = SensorRaw[1] * VCC / 1023.0;
    /* // uncomment these lines of code to use the +/- 10V sensors
        SensorRaw[0] = analogRead(A1_10V);
          SensorRaw[1] = analogRead(A2_10V);

          // Convert to voltage values (20V range, -10V offset)
          SensorVoltage[0] = SensorRaw[0]*20.0/1023.0 - 10.0;
          SensorVoltage[1] = SensorRaw[1]*20.0/1023.0 - 10.0;

    */

    //PhRecibido = ((-4) * (SensorVoltage[1]) + 13.96);

    dataFile = SD.open(filename, FILE_WRITE);
    // if the file is available, write to it:
    if (dataFile)
    {
      char fechaimprimible[20];
      RtcDateTime fechaactual = reloj.GetDateTime();
      snprintf_P(fechaimprimible,
                 sizeof(fechaimprimible),
                 PSTR("%02u/%02u/%04u %02u:%02u:%02u"),
                 fechaactual.Month(),
                 fechaactual.Day(),
                 fechaactual.Year(),
                 fechaactual.Hour(),
                 fechaactual.Minute(),
                 fechaactual.Second() );
      dataFile.print(fechaimprimible);
      dataFile.print(",");
      dataFile.print(Temp);
      dataFile.print(",");
      dataFile.print(SensorVoltage[1]);
      dataFile.println(";");
      //dataFile.print("\t");
      //dataFile.println(SensorVoltage[0]);
      dataFile.close();
    }

    // if the file isn't open, pop up an error:
    else
    {
      Serial.println("Error opening file.");
    }
    // Serial print to the serial monitor
    char fechaimprimible[20];


    RtcDateTime fechaactual = reloj.GetDateTime();
    snprintf_P(fechaimprimible,
               sizeof(fechaimprimible),
               PSTR("%02u/%02u/%04u %02u:%02u:%02u"),
               fechaactual.Month(),
               fechaactual.Day(),
               fechaactual.Year(),
               fechaactual.Hour(),
               fechaactual.Minute(),
               fechaactual.Second() );
    Serial.print(fechaimprimible);
    Serial.print("\t"); // tab character
    Serial.print(Temp);
    Serial.print("\t");
    Serial.print(SensorVoltage[1]);
    Serial.print("Ph");
    Serial.println();

    digitalWrite(ledPin, HIGH); // turn the LED back on to show data collection
    // duration is still running.
  }

} // end of loop



unsigned long resistance(unsigned long rawAnalogInput)
/* function to convert the raw Analog Input reading to a resistance value
   Schematic:
     [Ground] -- [thermistor] -------- | -- [15,000 ohm bridge resistor] --[Vcc (5v)]
                                       |
                                  Analog Pin 0

   For the circuit above:
   Resistance = ((rawAnalogInput*15000) /(1023 - rawAnalogInput))
*/
{
  unsigned long temp;  // temporary variable to store calculations in
  temp = (rawAnalogInput * 15000) / (1023 - rawAnalogInput);
  return temp; // returns the value calculated to the calling function.
}

float steinharthart(unsigned long resistance)
// function users steinhart-hart equation to return a temperature in degrees celsius.
/* Inputs ADC count from Thermistor and outputs Temperature in Celsius
   There is a huge amount of information on the web about using thermistors with the Arduino.
   Here we are concerned about using the Vernier Stainless Steel Temperature Probe TMP-BTA and the
   Vernier Surface Temperature Probe STS-BTA, but the general principles are easy to extend to other
   thermistors.
   This version utilizes the Steinhart-Hart Thermistor Equation:
      Temperature in Kelvin = 1 / {A + B[ln(R)] + C[ln(R)]^3}
     for the themistor in the Vernier TMP-BTA probe:
      A =0.00102119 , B = 0.000222468 and C = 1.33342E-7
      Using these values should get agreement within 1 degree C to the same probe used with one
      of the Vernier interfaces

*/
{
  float temp; // temporary variable to store calculations in
  float logRes = log(resistance);
  // calculating logirithms is time consuming for a microcontroller - so we just
  // do this once and store it to a variable.
  float k0 = 0.00102119;
  float k1 = 0.000222468;
  float k2 = 0.000000133342;

  temp = 1 / (k0 + k1 * logRes + k2 * logRes * logRes * logRes);
  temp = temp - 273.15;  // convert from Kelvin to Celsius
  return temp;

}



