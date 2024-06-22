///low to midlow fill time =4.45
////MidLow to Mid fill time =4.30
////Mid to Full fill time = 
////
#include <Wire.h>

#include <LiquidCrystal_I2C.h>
//I2C Pins - SDA = GPIO21 || SCL = GPIO22
#define LOW_Float 33
#define MID_LOW_Float 32
#define MID_Float 35
#define FULL_Float 34
#define RELAY 25
#define BUZZER 26
#define LOW_LED  4//12//27       //RED
#define MID_LOW_LED 13          // YELLOW
#define MID_LED 14//12          // GREEN
#define FULL_LED 27             // FULL GREEN
#define Status_LED_PIN 2


/////Time define////////////
#define TIMEOUT_30_S 30000
#define TIMEOUT_45_S 45000
#define TIMEOUT_1_MIN 60000
/////////////////////////////////
int lowSensorValue, midLowSensorValue, midSensorValue, fullSensorValue;
bool LOW_LED_State = false, MID_LOW_LED_State = false, MID_LED_State = false, FULL_LED_State = false, Status_LED_PIN_State = false;

LiquidCrystal_I2C lcd(0x27, 16, 2);  // my I2C Address 0x25

unsigned long startMillis, mainLoopTime, tempTime, mainLoopTime1;
const unsigned long interval = 5 * 60 * 1000;  // 15 minutes in milliseconds

void setup() {
  Serial.begin(115200);
  Wire.begin();
  lcd.init();
  lcd.backlight();

  pinMode(LOW_Float, INPUT_PULLUP);
  pinMode(MID_LOW_Float, INPUT_PULLUP);
  pinMode(MID_Float, INPUT_PULLUP);
  pinMode(FULL_Float, INPUT_PULLUP);
  pinMode(RELAY, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  pinMode(LOW_LED, OUTPUT);
  pinMode(MID_LOW_LED, OUTPUT);
  pinMode(MID_LED, OUTPUT);
  pinMode(FULL_LED, OUTPUT);
  pinMode(Status_LED_PIN, OUTPUT);  // Set the load pin as an output
  // digitalWrite(Status_LED, HIGH);  // Turn the load on (assuming active high)
  startMillis = millis();      // Record the start time
  mainLoopTime = startMillis;  // mainLoopTime = millis();
  mainLoopTime1 = startMillis;
  All_Pins_Config();
  delay(500);
  // Test_HW();
  // I2C_ADDRESS();
}

void loop() {
  tempTime = millis();
  if (tempTime - mainLoopTime > 2000) {
    Debugg();
    mainLoopTime = tempTime;
  }
  if (tempTime - mainLoopTime1 > TIMEOUT_1_MIN) {
    Status_LED_PIN_State = false;
    mainLoopTime1 = tempTime;
  }

  // display();
  FloatSensor();
  // Status_LED();
  // MotorControl();
  
}

void load_OnTime() {
  unsigned long currentMillis = millis();  // Get the current time
  Serial.println(millis());
  // Check if 15 minutes have passed
  if (currentMillis - startMillis >= interval) {
    Serial.println("Pump off");
    // digitalWrite(Status_LED, LOW);  // Turn the load off (assuming active high)
    Serial.print("Off time Millis : ");
    Serial.println(millis());
  }
}
void I2C_ADDRESS() {
  byte error, address;
  int nDevices;
  Serial.println("Scanning...");
  nDevices = 0;
  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("I2C device found at   0x");
      if (address < 16) {
        Serial.print("0");
      }
      Serial.println(address, HEX);
      nDevices++;
    } else if (error == 4) {
      Serial.print("Unknow error at address 0x");
      if (address < 16) {
        Serial.print("0");
      }
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0) {
    Serial.println("No I2C devices found\n");
  } else {
    Serial.println("done\n");
  }
  delay(5000);
}
void FloatSensor() {
  lowSensorValue = digitalRead(LOW_Float);
  midLowSensorValue = digitalRead(MID_LOW_Float);
  midSensorValue = digitalRead(MID_Float);
  fullSensorValue = digitalRead(FULL_Float);

  int state = 0;
  if (lowSensorValue == LOW && midLowSensorValue == LOW && midSensorValue == LOW && fullSensorValue == LOW) state = 4;
  else if (lowSensorValue == LOW && midLowSensorValue == LOW && midSensorValue == LOW && fullSensorValue == HIGH) state = 3;
  else if (lowSensorValue == LOW && midLowSensorValue == LOW && midSensorValue == HIGH && fullSensorValue == HIGH) state = 2;
  else if (lowSensorValue == LOW && midLowSensorValue == HIGH && midSensorValue == HIGH && fullSensorValue == HIGH) state = 1;
  // MotorControl();
  switch (state) {
    case 1:  // LOW sensor activated
      LOW_LED_State = true;
      Status_LED_PIN_State = true;
      digitalWrite(RELAY, HIGH);  // Turn on motor
      Serial.print("Emergency Turn On Motor!  ");   
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Emergency Turn On Motor!");
      lcd.setCursor(0, 1);
      lcd.print("Water Tank Empty!");
      delay(500);
      break;
    case 2:  // MID_LOW sensor activated
      MID_LOW_LED_State = true;
      digitalWrite(RELAY, HIGH);  // Turn off motor
      Serial.print("Water Level Low!  ");
      break;
    case 3:  // MID sensor activated
      MID_LED_State = true;
      digitalWrite(RELAY, HIGH);  // Turn off motor
      Serial.print("Water Level MID!  ");
      break;
    case 4:  // FULL sensor activated
      FULL_LED_State = true;
      digitalWrite(RELAY, LOW);  // Turn off motor
      Serial.print("Water Level Full  ");
      break;
    default:
      digitalWrite(RELAY, LOW);  // Default state
      lcd.clear();
      lcd.setCursor(0, 1);
      lcd.print("Water Tank Full");
      delay(300);
      break;
  }
}
void Status_LED() {
  digitalWrite(LOW_LED, LOW_LED_State ? LOW : HIGH);
  digitalWrite(MID_LOW_LED, MID_LOW_LED_State ? LOW : HIGH);
  digitalWrite(MID_LED, MID_LED_State ? LOW : HIGH);
  digitalWrite(FULL_LED, FULL_LED_State ? LOW : HIGH);
  digitalWrite(Status_LED_PIN, Status_LED_PIN_State ? HIGH : LOW);
}
// void MotorControl() {

// }
void display() {

  // set cursor to first column, first row
  lcd.setCursor(0, 0);
  // print message
  lcd.print("Hello, World!");
  delay(1000);
  // clears the display to print new message
  lcd.clear();
  // set cursor to first column, second row
  lcd.setCursor(0, 1);
  lcd.print("Hello, World!");
  delay(1000);
  lcd.clear();
}

void Debugg() {
  // Debugging output
  Serial.print("LOW Sensor: ");           Serial.print(lowSensorValue);
  Serial.print(", MID_LOW Sensor: ");     Serial.print(midLowSensorValue);
  Serial.print(", MID Sensor: ");         Serial.print(midSensorValue);
  Serial.print(", FULL Sensor: ");        Serial.print(fullSensorValue);
  Serial.print(", Motor State: ");        Serial.println(digitalRead(RELAY));
}
void Test_HW(){
  digitalWrite(RELAY, 1);           delay(2000);  digitalWrite(RELAY, 0);             delay(2000);    // Relay
  digitalWrite(BUZZER, 1);          delay(2000);  digitalWrite(BUZZER, 0);            delay(2000);    // Buzzer
  digitalWrite(LOW_LED, 0);         delay(2000);  digitalWrite(LOW_LED, 1);           delay(2000);    // RED LED
  digitalWrite(MID_LOW_LED, 0);     delay(2000);  digitalWrite(MID_LOW_LED, 1);       delay(2000);    // LOWER MID YELLOW LED
  digitalWrite(MID_LED, 0);         delay(2000);  digitalWrite(MID_LED, 1);           delay(2000);    // UPPER MID GREEN LED
  digitalWrite(FULL_LED, 0);        delay(2000);  digitalWrite(FULL_LED, 1);          delay(2000);    // FULL LED
  digitalWrite(Status_LED_PIN, 1);  delay(2000);  digitalWrite(Status_LED_PIN, 0);    delay(2000);    // STATUS
}
void All_Pins_Config(){
  digitalWrite(RELAY, 0);             
  digitalWrite(BUZZER, 0);            
  digitalWrite(LOW_LED, 1);           
  digitalWrite(MID_LOW_LED, 1);      
  digitalWrite(MID_LED, 1);           
  digitalWrite(FULL_LED, 1);          
  digitalWrite(Status_LED_PIN, 0); 
}