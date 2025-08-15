#include <FilteredPID.h>  
#include <Adafruit_MAX31865.h>

#define PIN_INPUT A0
#define PIN_OUTPUT 3

#define RREF 430
#define RNOMINAL 100

Adafruit_MAX31865 thermo = Adafruit_MAX31865(10, 11, 12, 13);

// PID
double Setpoint = 0.0, Input, Output;
double aggKp = 62.6675, aggKi = 0, aggKd = 0;                 // Agresivo (lejos del setpoint)
double consKp = 0.2325, consKi = 0.0005, consKd = 7.6690;       // Conservador (cerca del setpoint)
double consN = 1.1370;                                         // Filtro derivativo conservador

// PID adaptativo con derivada filtrada
FilteredPID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);

// Buffers para control por pasos
float tempSteps[3] = {0};
int timeSteps[3] = {0};
int currentStep = 0;
const int totalSteps = 3;

bool holdStarted = false;
unsigned long holdStartTime = 0;
float tolerance = 2.0;

bool sendData = false;
unsigned long startTime;
bool done = false;

// Variables de pH
int buffer_arr[10], temp;
float ph_act, volt;

void setup() {
  Serial.begin(9600);
  thermo.begin(MAX31865_3WIRE);
  pinMode(PIN_OUTPUT, OUTPUT);

  Input = thermo.temperature(RNOMINAL, RREF);

  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(1000);
  myPID.SetOutputLimits(0, 100);
  myPID.SetFilterN(consN);

  // PWM lento en pin 3 (~61 Hz)
  TCCR2A = (1 << WGM21) | (1 << WGM20) | (1 << COM2B1);
  TCCR2B = (1 << CS22) | (1 << CS21) | (1 << CS20);
  OCR2A = 255;

  startTime = millis();
}

void loop() {
  // Leer comandos Serial
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command.equals("SEND_DATA")) {
      sendData = true;
    }

    if (command.startsWith("SET_STEPS:")) {
      parseSteps(command.substring(10));
      currentStep = 0;
      holdStarted = false;
      done = false;
      Serial.println("Steps received.");
    }
  }

  // Leer pH (mediana)
  for (int i = 0; i < 10; i++) {
    buffer_arr[i] = analogRead(PIN_INPUT);
    delay(10);
  }

  for (int i = 0; i < 9; i++) {
    for (int j = i + 1; j < 10; j++) {
      if (buffer_arr[i] > buffer_arr[j]) {
        temp = buffer_arr[i];
        buffer_arr[i] = buffer_arr[j];
        buffer_arr[j] = temp;
      }
    }
  }

  unsigned long avgval = 0;
  for (int i = 2; i < 8; i++) avgval += buffer_arr[i];
  volt = (float)avgval * 5.0 / 1024.0 / 6.0;
  ph_act = -6.6667 * volt + 25.733;

  // Leer temperatura
  float TemperaturePV = thermo.temperature(RNOMINAL, RREF);
  Input = TemperaturePV;

  // Control por pasos de temperatura
  if (!done && currentStep < totalSteps) {
    Setpoint = tempSteps[currentStep];

    if (abs(Input - Setpoint) <= tolerance) {
      if (!holdStarted) {
        holdStarted = true;
        holdStartTime = millis();
      }

      unsigned long elapsedHold = (millis() - holdStartTime) / 1000;
      if (elapsedHold >= timeSteps[currentStep]) {
        currentStep++;
        holdStarted = false;

        if (currentStep >= totalSteps) {
          Serial.println("%%END%%");
          done = true;
        }
      }
    } else {
      holdStarted = false;
    }
  }

  // Cambio de sintonía PID
  double gap = abs(Setpoint - Input);
  if (gap < 0.8) {
    myPID.SetTunings(consKp, consKi, consKd);
    myPID.SetFilterN(consN);
  } else {
    myPID.SetTunings(aggKp, aggKi, aggKd);
    myPID.SetFilterN(0.0); // No hay derivada en el modo agresivo
  }

  // Ejecutar PID
  myPID.Compute();

  // PWM (0–255)
  int pwmOutput = (Output * 255.0 / 100.0);
  analogWrite(PIN_OUTPUT, pwmOutput);

  // Cálculo de potencia (opcional)
  float OutputVolt = pwmOutput * 5.0 / 255.0;
  float PowerPV = (2000.0 / 5.0) * OutputVolt;

  // Enviar por Serial para PySerial
  if (sendData) {
    unsigned long elapsedTime = (millis() - startTime) / 1000;
    Serial.print(elapsedTime);
    Serial.print(",");
    Serial.print(TemperaturePV);
    Serial.print(",");
    Serial.print(Setpoint);
    Serial.print(",");
    Serial.print(pwmOutput);
    Serial.print(",");
    Serial.print(PowerPV);
    Serial.print(",");
    Serial.println(ph_act);
  }

  delay(1000);
}

// Parsear los pasos desde el comando Serial
void parseSteps(String input) {
  int pos = 0;
  int step = 0;

  while (pos < input.length() && step < totalSteps) {
    int commaIndex = input.indexOf(',', pos);
    String pair = (commaIndex == -1) ? input.substring(pos) : input.substring(pos, commaIndex);
    
    int dashIndex = pair.indexOf('-');
    if (dashIndex != -1) {
      tempSteps[step] = pair.substring(0, dashIndex).toFloat();
      timeSteps[step] = pair.substring(dashIndex + 1).toInt() * 60;
      step++;
    }

    if (commaIndex == -1) break;
    pos = commaIndex + 1;
  }
}
