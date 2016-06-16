#include <Wire.h>  // must be incuded here so that Arduino library object file references work
#include <RtcDS3231.h>
#include <SD.h>      // includes the Arduino SD Library 
#include <SoftwareSerial.h>
#define A1_5V 0
#define A2_5V 1
#define A3_5V 2
#define A4_5V 3
#define A5_5V 4
#define A6_5V 5

SoftwareSerial bluetooth(7, 6); //puerto serial especial para bluetooth
RtcDS3231 reloj;

char * filename2 = "calibracion.csv";
char * filename = "tesis2.csv";
long tiempoPasado;
int Frecuencia = 1;
const int chipSelect = 8;
File dataFile;
File dataFile2;
//int muxLSB = 10; //low byte of multiplexer
//int muxMSB = 11; //high byte of multiplexer

int SensorRaw[6];
float SensorVoltage[6];
float VCC = 5.0;
float c1;
float m;
float constante;
void setup()
{
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  bluetooth.begin(9600);
  reloj.Begin();

  //pinMode(muxLSB, OUTPUT);
  //  pinMode(muxMSB, OUTPUT);


  pinMode(chipSelect, OUTPUT);

  Serial.print("inicializando memoria sd...");
  if (!SD.begin(chipSelect))
  {
    Serial.println("memoria no inicializada o no esta presente");

    return;
  }
  Serial.println("memoria inicializada.");

  Serial.println();
  dataFile2 = SD.open(filename2, FILE_WRITE);
  if (dataFile2)
  {
    dataFile.close();
  }
  else
  {
    Serial.println("error opening file.");
    Serial.println();
  }
  dataFile = SD.open(filename, FILE_WRITE);
  if (dataFile)
  {
    dataFile.close();
  }
  else
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

        float m = (y2 - y1) / (x2 - x1);
        float constante= -m*x1+y1;
        c1 = m * SensorVoltage[0] + constante ;
        Serial.print("la pendiente es:");
        Serial.println(m);
        Serial.print("constante es:");
        Serial.println(constante);

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



    // leemos los sensores
    SensorRaw[0] = analogRead(A1_5V);
    SensorRaw[1] = analogRead(A2_5V);
    SensorRaw[2] = analogRead(A3_5V);
    SensorRaw[3] = analogRead(A4_5V);
    SensorRaw[4] = analogRead(A5_5V);
    SensorRaw[5] = analogRead(A6_5V);

    //********************************************************************************

    SensorVoltage[0] = SensorRaw[0] * VCC / 1023.0;
    SensorVoltage[1] = SensorRaw[1] * VCC / 1023.0;
    SensorVoltage[2] = SensorRaw[2] * VCC / 1023.0;
    SensorVoltage[3] = SensorRaw[3] * VCC / 1023.0;
    SensorVoltage[4] = SensorRaw[4] * VCC / 1023.0;
    SensorVoltage[5] = SensorRaw[5] * VCC / 1023.0;
    //abriendo carpeta filename2
    dataFile2 = SD.open(filename2, FILE_WRITE);
    // if the file is available, write to it:
    if (dataFile2)
    {
      dataFile2.print("la pendiente es:");
      dataFile2.println(m);
      dataFile2.print("la constante es:");
      dataFile2.println(constante);

      dataFile2.close();



      //abriendo carpeta filename
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
        dataFile.print(SensorVoltage[0]);
        dataFile.print(",");
        dataFile.print(SensorVoltage[1]);
        dataFile.print(",");
        dataFile.print(SensorVoltage[2]);
        dataFile.print(",");
        dataFile.print(SensorVoltage[3]);
        dataFile.print(",");
        dataFile.print(SensorVoltage[4]);
        dataFile.print(",");
        dataFile.print(SensorVoltage[5]);
        dataFile.close();
      }
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
      Serial.print("\t");
      Serial.print(SensorVoltage[0]);
      Serial.print("\t");
      Serial.print(SensorVoltage[1]);
      Serial.print("\t");
      Serial.print(SensorVoltage[2]);
      Serial.print("\t");
      Serial.print(SensorVoltage[3]);
      Serial.print("\t");
      Serial.print(SensorVoltage[4]);
      Serial.print("\t");
      Serial.print(SensorVoltage[5]);
      Serial.println();


    }
  }
}



