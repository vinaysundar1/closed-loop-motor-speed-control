#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// --- OLED SETTINGS ---
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// --- PIN DEFINITIONS ---
const int ENB = 9;
const int IN3 = 8;
const int IN4 = 7;
const int encoderPin = 2;
const int potPin = A0;

// --- MOTOR SPECIFICATIONS ---
const float PPR = 170.0;

// --- PID VARIABLES ---
double kp = 0.5;
double ki = 0.5;
double kd = 0.08;

double setpoint = 0.0;
double currentRPM = 0.0;       
double rawRPM = 0.0;           
double motorOutput = 0.0;

double error = 0.0;
double previousError = 0.0;
double integral = 0.0;
double filteredDerivative = 0.0;

// --- FILTER SETTINGS ---
const double RPM_FILTER_ALPHA = 0.35;   
const double D_FILTER_ALPHA   = 0.2;    

// --- TIME & COUNTING VARIABLES ---
volatile long encoderTicks = 0;
long previousMillis = 0;
int interval = 100;

void setup() {
  Serial.begin(9600);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("SSD1306 allocation failed");
    for (;;);
  }
  display.clearDisplay();
  display.setTextColor(WHITE);

  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(encoderPin, INPUT_PULLUP);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  attachInterrupt(digitalPinToInterrupt(encoderPin), countTicks, RISING);
}

void loop() {

  long currentMillis = millis();

  // Read potentiometer and map to an RPM range (0 to 400 RPM)
  int potValue = analogRead(potPin);
  double newSetpoint =  map(potValue, 0, 1023, 0, 400);
  
  // Deadband filter to prevent tiny electrical potentiometer flickers from causing derivative kicks
  if (abs(newSetpoint - setpoint) > 3.0) {
    setpoint = newSetpoint;
  }

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    // 1. Calculate RAW RPM from tick count
    noInterrupts();
    long ticks = encoderTicks;
    encoderTicks = 0;
    interrupts();
    rawRPM = ((float)ticks / PPR) * (60000.0 / interval);

    // 1b. Low-pass filter the RPM reading
    currentRPM = RPM_FILTER_ALPHA * rawRPM + (1.0 - RPM_FILTER_ALPHA) * currentRPM;

    // 2. PID Calculations
    error = setpoint - currentRPM;
    double dt = interval / 1000.0;

    double p_term = kp * error;

    double raw_derivative = (error - previousError) / dt;
    previousError = error;

    filteredDerivative = D_FILTER_ALPHA * raw_derivative + (1.0 - D_FILTER_ALPHA) * filteredDerivative;
    double d_term = kd * filteredDerivative;

    double tentativeOutput = p_term + integral + d_term;

    bool willSaturateHigh = (tentativeOutput > 255) && (error > 0);
    bool willSaturateLow  = (tentativeOutput < 0)   && (error < 0);
    if (!willSaturateHigh && !willSaturateLow) {
      integral += ki * error * dt;
    }
    
    if (integral > 255) integral = 255;
    if (integral < -255) integral = -255;

    motorOutput = p_term + integral + d_term;
    if (motorOutput > 255) motorOutput = 255;
    if (motorOutput < 0) motorOutput = 0;

    // 5. Drive the Motor
    analogWrite(ENB, motorOutput);

    // 6. Serial Plotter output
    Serial.print("Min:0,");       
    Serial.print("Max:300,");     
    Serial.print("Setpoint:");
    Serial.print(setpoint);
    Serial.print(", RPM:");
    Serial.println(currentRPM);

    // 7. OLED
    updateOLED();
  }
}

void updateOLED() {
  display.clearDisplay();

  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("Target RPM: ");
  display.setTextSize(2);
  display.setCursor(0, 12);
  display.print(setpoint, 1);

  display.setTextSize(1);
  display.setCursor(0, 36);
  display.print("Actual RPM: ");
  display.setTextSize(2);
  display.setCursor(0, 48);
  display.print(currentRPM, 1);

  display.display();
}

void countTicks() {
  encoderTicks++;
}
