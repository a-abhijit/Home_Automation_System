#include <DHT.h>
#include <LiquidCrystal_I2C.h>

// Initializing the DHT sensor
#define DHTPIN 2
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

// Initializing the LCD
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Defining pin assignments
#define PIR_SENSOR_PIN 9
#define LDR_SENSOR_PIN 3
#define LED_PIN 13
#define TEMP_UP_PIN 5
#define TEMP_DOWN_PIN 4
#define TRIG_PIN 6
#define ECHO_PIN 7
#define BUZZER_PIN 8
#define SYSTEM_SWITCH_PIN 1

// Motor control pins
int enablePin = 10;
int motorPin1 = 11;
int motorPin2 = 12;

float currentTemp;
int tempThreshold = 35;
bool motionDetected = false;
bool lightDetected = false;
bool motorOn = false;
bool systemOn = true;

unsigned long motionStartTime = 0;
unsigned long motionEndTime = 0;
int count;

void setup() {
  // Beginning serial communication
  Serial.begin(9600);

  // Setting pin modes
  pinMode(PIR_SENSOR_PIN, INPUT);
  pinMode(LDR_SENSOR_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(TEMP_UP_PIN, INPUT_PULLUP);
  pinMode(TEMP_DOWN_PIN, INPUT_PULLUP);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(SYSTEM_SWITCH_PIN, INPUT_PULLUP);

  // Setting motor control pins as outputs
  pinMode(enablePin, OUTPUT);
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);

  // Starting the DHT sensor
  dht.begin();

  // Initializing the LCD
  lcd.init();
  // lcd.backlight();
}

void loop() {
  if (digitalRead(SYSTEM_SWITCH_PIN) == LOW) {
    Serial.println("Here");
    if (!systemOn) {
      systemOn = true;
      Serial.println("System turned ON");
    }
    else {
      systemOn = false;
      Serial.println("System turned OFF");
    }
  }
  if (!systemOn) {
    digitalWrite(LED_PIN, LOW);
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, LOW);
    digitalWrite(BUZZER_PIN, LOW);
    lcd.noBacklight();
    // return;
  }
  else {
  lcd.backlight();

  // Reading from DHT sensor
  currentTemp = dht.readTemperature();

  // Checking for motion
  motionDetected = digitalRead(PIR_SENSOR_PIN) == HIGH;
  
  // Checking brightness from the LDR sensor (active LOW)
  lightDetected = digitalRead(LDR_SENSOR_PIN) == LOW;

  // Controlling the LED
  if (motionDetected) {
    count = 1;
    motionStartTime = millis();
    activateSonar();
    Serial.println("Motion detected!");
    if (!lightDetected) {
      digitalWrite(LED_PIN, HIGH);
    }
    else {
      digitalWrite(LED_PIN, LOW);
    }
  }
  else {
    if (count==1) {
    motionEndTime = millis();
    unsigned long duration = motionEndTime - motionStartTime;
    Serial.print("Motion ended. Duration: ");
    Serial.print(duration);
    Serial.println(" ms");
    count--;
    }
    digitalWrite(LED_PIN, LOW);
    digitalWrite(BUZZER_PIN, LOW);
  }

  // Controlling the motor
  motorOn = motionDetected && currentTemp > tempThreshold;
  triggerMotor(motorOn);

  // Checking buttons for adjusting the temperature threshold
  if (digitalRead(TEMP_UP_PIN) == LOW) {
    tempThreshold += 1.0;
    delay(200);
  }
  if (digitalRead(TEMP_DOWN_PIN) == LOW) {
    tempThreshold -= 1.0;
    delay(200);
  }

  // Updating the LCD display
  updateLCD(currentTemp);

  // Adding a small delay to prevent excessive updates
  delay(1000);
  }
}

void updateLCD(float currentTemp) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Temp: ");
  lcd.print(currentTemp, 1);
  lcd.print(" C");
  
  lcd.setCursor(0, 1);
  lcd.print("Thr: ");
  lcd.print(tempThreshold, 1);
  lcd.print("C ");
  lcd.print(motorOn ? "Fan ON" : "Fan OFF");
}

void activateSonar() {
  long duration, distance;
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  duration = pulseIn(ECHO_PIN, HIGH);
  distance = duration * 0.034 / 2;

  Serial.print("Distance: ");
  Serial.println(distance);

  // int kidPetMaxHeight = 2;
  // int personMinHeight = 5;

  // if (distance > 0 && distance <= kidPetMaxHeight) {
  if (distance > 0 && distance <=6 && distance >3) {
    Serial.println("BUZZER ON");
    digitalWrite(BUZZER_PIN, HIGH); // Turning on buzzer for kids or pets
  }
  // else if (distance > personMinHeight) {
  else {
    Serial.println("BUZZER OFF");
    digitalWrite(BUZZER_PIN, LOW); // Turning off buzzer for regular person
  }
}

void triggerMotor(bool motorOn) {
  if (motorOn) {
    // Spinning the motor forward
    digitalWrite(motorPin1, HIGH);
    digitalWrite(motorPin2, LOW);
    analogWrite(enablePin, 128);
    // digitalWrite(enablePin, HIGH);
  } else {
    // Turning the motor off
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, LOW);
  }
}
