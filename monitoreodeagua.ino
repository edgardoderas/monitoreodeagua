
#if defined(ESP8266)
#include <pgmspace.h>
#else
#include <avr/pgmspace.h>
#endif
#include <Wire.h>  // must be incuded here so that Arduino library object file references work
#include <RtcDS3231.h>
#include <SD.h>      // includes the Arduino SD Library 
#include <SoftwareSerial.h>
// pin configurations for SparkFun Vernier Shield
// A1 = Analog 1
// A2 = Analog 2
#define A1_5V 0
#define A1_10V 1
#define A2_5V 2
#define A2_10V 3

SoftwareSerial bluetooth(7, 6); //puerto serial especial para bluetooth
RtcDS3231 reloj;


char * filename = "tesis2.cvs";  /* sets the filename for data - change this
  if you want to use a different file name
  data will be concatenated onto the existing
  file if it exists */
float dataRate = 100;     // # of samples per second.
int duration = 5000;       // set the data collection duration in milliseconds
// default value is set to 5 seconds or 5000 milliseconds

float Temp;
unsigned long thermistor;
int rawAnalogReading;

long tiempoPasado;
unsigned long timeRef;      // reference for starting time
unsigned long timeInterval;
unsigned long elapsedTime;
unsigned long ndx = 0;

const int buttonPin = 12;   // digital button on Vernier Shield - used to start data collect
const int ledPin = 13;      // LED pin on Vernier Shield

/* Global Variable declarations for SD Card Shield */
const int chipSelect = 8;
File dataFile;

// variables used with VernierAnalogAutoID
//
int muxLSB = 10; //low byte of multiplexer
int muxMSB = 11; //high byte of multiplexer

int SensorRaw[2];
float SensorVoltage[2];
float VCC = 5.0;

void setup()
{
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  bluetooth.begin(9600);
  reloj.Begin();

  // set the timeInterval based on dataRate
  timeInterval = 1E3 / dataRate;

  // initialize the buttonPin as an INPUT with a pull-up resistor
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(muxLSB, OUTPUT);
  pinMode(muxMSB, OUTPUT);

  Serial.println("*************************************************");
  Serial.println("Push button (D12) to start data collection.");
  Serial.println("Use reset button to reset / stop data collection.");
  Serial.println("*************************************************");


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


  Serial.println("");
  Serial.print("s");

  Serial.print("\t"); //tab character
  Serial.print ("Cº");
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

  digitalWrite(ledPin, HIGH);
  timeRef = millis();
  ndx = 0;   // datapoint index


  tiempoPasado = millis();

} // end setup

void loop()
{

  if (bluetooth.available()) {
    char a = bluetooth.read();
    if (a == "d")(
        dataFile = SD.open(filename);
      if (dataFile) {
      while (dataFile) // if it opens sucessfully
        {
          char letra = dataFile.read();
          if (letra == '/n')
            delay(500);
          bluetooth.write(letra);

        }
        dataFile.close();
      }
  }
}

unsigned long currTime = millis();  // record current Time

if ( millis() > tiempoPasado + timeRef) {
  tiempoPasado = millis();
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
    //dataFile.print((currTime - timeRef) / 1E3, 3); // 4 decimal places
    dataFile.print(",");
    dataFile.print(Temp);
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
  //Serial.print("\t");
  //Serial.print(SensorVoltage[0]);
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



