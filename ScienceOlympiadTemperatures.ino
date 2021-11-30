/*
 * 2020 Science Olympiad Detector Building
 * This code takes a voltage input from a TMP36 and returns the equivalent temperature reading
 * The thermistor was waterproofed using a plastic straw and silicone in order to measure the temperature of several different cups of water
 * Equations were determined by using several different regression methods from collected samples from water of known temperatures
 * Written for use with an Arduino MKR1000
 * 
 */

int sensorPin =   A0;      // select the input pin for the potentiometer
int blueLedPin =   3;      // select the pin for the Blue LED
int greenLedPin =  4;      // select the pin for the Blue LED
int redLedPin =    5;      // select the pin for the Blue LED

//LED Temperature Ranges
float blueMin = 0.0;
float blueMax = 33.3;
float greenMax = 66.6;
float redMax = 100.0;

int analogCounts = 0;      // variable to store the value coming from the sensor
double analogVolts;        // variable to store the analog voltage
double degC;               // variable to store the temperature in degC
double voltsSlope = 1.65/4096; // volts/count 1.65v AREF with a 12bit ADC

// TM36 Thermistor linear conversion constants (i.e. y = a * x + b)
double slope = 100;        //100 degC/volt (a)
double offset = -50;       //offset of -50 degC (b)

// Statistical processing to improve stability
#define numSamples 499
double rawValues[numSamples];
double avgTemperature;
double avgTemperature2;

//Temperature Calibration Constants
double thermistorLow = 1;
double thermistorHigh = 99;
double thermometerLow = 1;
double thermometerHigh = 99;
double adjustedValue;

double map_double(double x, double in_min, double in_max, double out_min, double out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void sort(double a[], int size) {
    for(int i=0; i<(size-1); i++) {
        for(int o=0; o<(size-(i+1)); o++) {
                if(a[o] > a[o+1]) {
                    double t = a[o];
                    a[o] = a[o+1];
                    a[o+1] = t;
                }
        }
    }
}

void setup() {
  //Serial Monitor Window Data
  Serial.begin(115200);
  
  // declare the LED pins as OUTPUTs:
  pinMode(blueLedPin, OUTPUT);
  pinMode(greenLedPin, OUTPUT);
  pinMode(redLedPin, OUTPUT);

  // configure the analog input for 12bit resolution
  analogReadResolution(12);
  // configure the analog input reference voltage to 1.65v
  analogReference(AR_INTERNAL1V65);
}

void loop() {
  // read the value from the sensor:
  for (int i = 0; i <= numSamples; i++) 
  {
    analogCounts = analogRead(A0);
    analogVolts = analogCounts * voltsSlope;
    degC = analogVolts * slope + offset;
    avgTemperature = avgTemperature + degC;
    rawValues[i] = degC;
    delayMicroseconds(100);
  }
  avgTemperature = avgTemperature / numSamples;

//Drop botton and top 25% of values to improve stability of reading
  sort(rawValues, numSamples);
  int count=0;
  for (int i = int(0.25*(numSamples+1)); i <= int(0.75*(numSamples+1)); i++) 
  {
    avgTemperature2 = avgTemperature2 + rawValues[i];
    count++;
  }
//Calculate the average of the 25th through the 75th percentile of values
  avgTemperature2 = avgTemperature2 / count;
  
  adjustedValue = map_double(avgTemperature, thermistorLow, thermistorHigh, thermometerLow,thermometerHigh);
  

  //Turn on the correct LED based on the temperature
  if ((avgTemperature >= blueMin) && (avgTemperature <= blueMax))
  {
    Serial.print("Blue Zone, Thermistor deg C = ");
    Serial.print(avgTemperature, 1);
    Serial.print(", AvgTemp2 = ");
    Serial.print(avgTemperature2, 1);
    Serial.print(", Adjusted deg C = ");
    Serial.println(adjustedValue, 1);
    digitalWrite(blueLedPin, HIGH);
    digitalWrite(greenLedPin, LOW);
    digitalWrite(redLedPin, LOW);
  }
  else if ((avgTemperature > blueMax) && (avgTemperature <= greenMax))
  {
    Serial.print("Green Zone, deg C = ");
    Serial.print(avgTemperature, 1);
    Serial.print(", AvgTemp2 = ");
    Serial.print(avgTemperature2, 1);
    Serial.print(", Adjusted deg C = ");
    Serial.println(adjustedValue, 1);
    digitalWrite(blueLedPin, LOW);
    digitalWrite(greenLedPin, HIGH);
    digitalWrite(redLedPin, LOW);
  }
  else if ((avgTemperature > greenMax) && (avgTemperature <= redMax))
  {
    Serial.print("Red Zone, deg C = ");
    Serial.print(avgTemperature, 1);
    Serial.print(", AvgTemp2 = ");
    Serial.print(avgTemperature2, 1);
    Serial.print(", Adjusted deg C = ");
    Serial.println(adjustedValue, 1);
    digitalWrite(blueLedPin, LOW);
    digitalWrite(greenLedPin, LOW);
    digitalWrite(redLedPin, HIGH);
  }
  else
  {
    Serial.print("Unknown Zone, deg C = ");
    Serial.print(avgTemperature, 1);
    Serial.print(", AvgTemp2 = ");
    Serial.print(avgTemperature2, 1);
    Serial.print(", Adjusted deg C = ");
    Serial.println(adjustedValue, 1);
    digitalWrite(blueLedPin, LOW);
    digitalWrite(greenLedPin, LOW);
    digitalWrite(redLedPin, LOW);
  }

  delay(120);

}
