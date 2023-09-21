#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <TinyGPS++.h>
#include <Servo.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);  // I2C address 0x27, 16 column and 2 rows


//*********************************** FOR ACCIDENT *****************************************************

const int xInput = A0;
const int yInput = A1;
const int zInput = A2;

int RawMin = 0;
int RawMax = 1023;
int count = 0;
const int sampleSize = 10;
byte updateflag;
byte updateflag1;
byte updateflag2;

//************************************ for buzzer ****************************************
const int BUTTON_PIN = 11;  // Arduino pin connected to button's pin
const int BUZZER_PIN = 5;   // Arduino pin connected to Buzzer's pin
//****************************************************************************************

int xaxis = 0, yaxis = 0, zaxis = 0;
int deltx = 0, delty = 0, deltz = 0;
int vibration = 2, devibrate = 75;
int magnitude = 0;
int sensitivity = 150;
bool buttonPressed = false;  // flag to track if the button has been pressed
bool buzzerOn = false;       // flag to track if the buzzer is currently on
//bool resetLoop = false;    // flag to track if the loop should be reset
bool codeExecuted = false;
boolean impactDetected = false;
//Used to run impact routine every 2mS.
unsigned long time1;
unsigned long impactTime;
unsigned long alertDelay = 30000;  //30 seconds

//******************************GPS****************************************

TinyGPSPlus gps;  // Create a TinyGPS++ object
double longi;
double lati;
boolean GPS = true;

unsigned long previousMillis1 = 0;

//======================================GSM============================================

String EmergencyContact = "+94xxxxxxxxx";
String Contact1 = "+94xxxxxxxxx";
String Contact2 = "+94xxxxxxxxx";
String url = "";
String phoneNum = "";

//======================================SERVO==========================================

Servo pcc;
Servo pcc1;
char val;
unsigned long previousMillis4 = 0;
unsigned long previousMillis5 = 0;
boolean pccstate1 = false;
boolean pccstate2 = false;
boolean lockon=false;

//======================================VOLTAGE========================================

int in = A4;
unsigned long previousMillis2 = 0;

//======================================FOR SPEED=======================================

// Constants
const int hallPin = 2;                            // Hall sensor pin
const float wheelCircumference = 2 * 3.14 * 0.2;  // Wheel circumference in meters (adjust as needed)
const unsigned long interval = 2000;              // Measurement interval in milliseconds

// Variables
volatile unsigned int hallCount = 0;  // Hall sensor count
unsigned long previousMillis = 0;
float speedKmph = 0.0;

void setup() {

  analogReference(EXTERNAL);
  Serial.begin(9600);

  // Begin serial communication with Arduino and SIM900
  Serial2.begin(9600);

  // Start the software serial port at the GPS's default baud
  Serial3.begin(9600);

  lcd.init();  // initialize the lcd
  lcd.backlight();
  // Attach interrupt to the Hall sensor pin
  attachInterrupt(digitalPinToInterrupt(hallPin), countPulse, RISING);

  pcc.attach(9);   //servo
  pcc1.attach(7);  //servo2
  pcc1.write(30);
  pcc.write(180);
  
  pinMode(in, INPUT);

  Serial2.println("AT+CMGF=1");  // Change sim900A  Serial to SIM900A
  delay(1000);
  Serial2.println("AT+CNMI=2,2,0,0,0");  // Configure module


  //*************************** ADXL335 ******************************************
  xaxis = analogRead(xInput);
  yaxis = analogRead(yInput);
  zaxis = analogRead(zInput);
  //**************************** For buzzer **************************************
  pinMode(BUTTON_PIN, INPUT_PULLUP);  // set Arduino pin to input pull-up mode
  pinMode(BUZZER_PIN, OUTPUT);        // set Arduino pin to output mode
  digitalWrite(BUZZER_PIN, LOW);
  //*****************************************************************************
  lcd.begin(0x27, 16, 2);
}

void loop() {

  lcd.init();

  unsigned long currentMillis1 = millis();
  if (currentMillis1 - previousMillis1 > 58000) {
    getGPS();
    previousMillis1 = currentMillis1;
  }

  accidentdetected();

  servo();
  //servo1();


  unsigned long currentMillis2 = millis();
  if (currentMillis2 - previousMillis2 > 2000) {
    voltage();
    previousMillis2 = currentMillis2;
  }

  speed();
  readMessege();
}


void accidentdetected() {


  //call impact routine every 2mS
  if (micros() - time1 > 1999) Impact();  // 2000micro seconds = 2 ms
  fall();

  if (updateflag1 == 1 && updateflag2 == 1 ) {
    updateflag = 1;
  }
  if (updateflag1 == 1 && updateflag2 == 0) {
    updateflag = 0;
  }

  if (updateflag == 1 ) {
    updateflag = 0;
    updateflag2 = 0;
    updateflag1 = 0;
    if(lockon==false){

    turnOnBuzzer();
    //Serial.println("Accident Detected");
    // Serial.println(magnitude);
      lcd.setCursor(0,1);
      lcd.print("Accident");
      lcd.setCursor(9,1);
      lcd.print(magnitude);

    impactDetected = true;
    impactTime = millis();
    }    
  }  //end of updateflag

  if (impactDetected == true) {
    if ((millis() - impactTime >= alertDelay) && digitalRead(BUZZER_PIN) == HIGH) {
      turnOffBuzzer();
      GPS = false;
      SendAlert(EmergencyContact, "Accident Detected! " + url);
      Serial.println("Sent Alert");
      impactDetected = false;
      impactTime = 0;
    }
  }  //end of impactDetected delay

  if (digitalRead(BUTTON_PIN) == LOW) {
    buttonPressed = true;
  }
  if (buttonPressed == true) {
    turnOffBuzzer();
    Serial.println("");

    buttonPressed = false;
    // impactDetected = false;
  }
}

void Impact() {
  //--------------------------------------------------------------
  time1 = micros();  // resets time value
  //--------------------------------------------------------------
  int oldx = xaxis;  //store previous axis readings for comparison
  int oldy = yaxis;
  int oldz = zaxis;

  xaxis = analogRead(xInput);
  yaxis = analogRead(yInput);
  zaxis = analogRead(zInput);

  //--------------------------------------------------------------
  //loop counter prevents false triggering. Vibration resets if there is an impact. Don't detect new changes until that "time" has passed.
  vibration--;
  //Serial.print("Vibration = "); Serial.println(vibration);
  if (vibration < 0) vibration = 0;
  //Serial.println("Vibration Reset!");

  if (vibration > 0) return;
  //--------------------------------------------------------------
  deltx = xaxis - oldx;
  delty = yaxis - oldy;
  deltz = zaxis - oldz;

  //Magnitude to calculate force of impact.
  magnitude = sqrt(sq(deltx) + sq(delty) + sq(deltz));

  if (magnitude >= sensitivity)  //impact detected
  {
    updateflag1 = 1;
    // reset anti-vibration counter
    vibration = devibrate;
  }

  else {
    //if (magnitude > 15)
    //Serial.println(magnitude);
    //reset magnitude of impact to 0
    magnitude = 0;
  }
}

void fall() {
  // read new state
  // int buttonState = digitalRead(BUTTON_PIN);

  // Read raw values
  int xRaw = ReadAxis(xInput);
  int yRaw = ReadAxis(yInput);
  int zRaw = ReadAxis(zInput);

  // Convert raw values to 'milli-Gs"
  long xScaled = map(xRaw, RawMin, RawMax, -3000, 3000);
  long yScaled = map(yRaw, RawMin, RawMax, -3000, 3000);
  long zScaled = map(zRaw, RawMin, RawMax, -3000, 3000);

  // Re-scale to fractional Gs
  float xAccel = xScaled / 1000.0;
  float yAccel = yScaled / 1000.0;
  float zAccel = zScaled / 1000.0;

  /*Serial.print("X, Y, Z  :: ");
    Serial.print(xRaw);
    lcd.setCursor(0,0);
    lcd.print(xRaw);
    Serial.print(", ");
    Serial.print(yRaw);
    Serial.print(", ");
    lcd.setCursor(5,0);
    lcd.print(yRaw);
    Serial.print(zRaw);
    lcd.setCursor(10,0);
    lcd.print(zRaw);
    Serial.print(" :: ");
    Serial.print(xAccel,0);
    Serial.print("G, ");
    Serial.print(yAccel,0);
    Serial.print("G, ");
    Serial.print(zAccel,0);
    Serial.println("G");*/

  
  if ((yRaw > 500 && yRaw < 565) && ((zRaw > 400 && zRaw < 435) || (zRaw > 600 && zRaw < 650))) {
    if (!codeExecuted) {
      updateflag2 = 1;
      Serial.println("Fall detected");
      codeExecuted = true;
    }
  } else if ((yRaw > 595 && yRaw < 6630) && (zRaw > 470 && zRaw < 550)) {
    codeExecuted = false;
    buttonPressed = false;
  }
}

// Take samples and return the average
int ReadAxis(int axisPin) {
  long reading = 0;
  analogRead(axisPin);
  delay(1);
  for (int i = 0; i < sampleSize; i++) {
    reading += analogRead(axisPin);
  }
  return reading / sampleSize;
}


/*=============================  Turn on and off buzzer =============================*/
void turnOnBuzzer() {
  digitalWrite(BUZZER_PIN, HIGH);  // turn on the buzzer
  buzzerOn = true;
}

void turnOffBuzzer() {
  digitalWrite(BUZZER_PIN, LOW);  // turn off the buzzer
  buzzerOn = false;
}

/*=============================  FOR GPS  ===========================================*/

void getGPS() {
  boolean newData = false;
  for (unsigned long start = millis(); millis() - start < 2000;) {
    while (Serial3.available() > 0 && GPS) {
      //Serial.println("Insode the while loop");
      if (gps.encode(Serial3.read())) {
        newData = true;
        break;
      }
    }  //end of while loop
  }

  if (newData)  //If newData is true
  {
    lati = gps.location.lat();
    longi = gps.location.lng();
    //Serial.print("\n\nLatitude: ");
    //Serial.println(lati, 6);
    //Serial.print("Longitude: ");
    //Serial.println(longi, 6);
    newData = false;
  } else {
   // Serial.println("No GPS data is available");
  }

  if (gps.location.isValid()) {
   // Serial.println("========Valid Location=======");
   // Serial.print("Latitude: ");
   // Serial.println(lati, 6);
   // Serial.print("Longitude: ");
   // Serial.println(longi, 6);
   // Serial.print("Altitude: ");
    //Serial.println(gps.altitude.meters());
    //delay(3000);
    url = "https://www.google.com/maps/search/?api=1&query=" + String(lati, 6) + "," + String(longi, 6);

  } else {
    //Serial.println("Location: Not Available");
    url = " Location: Not Available";
    //delay(3000);
  }
  //readMessege();

  // If 5000 milliseconds pass and there are no characters coming in
  // over the software serial port, show a "No GPS detected" error
  if (millis() > 5000 && gps.charsProcessed() < 10) {
    //Serial.println("No GPS detected");
    url = " Location: Not Available";
    return;
  }
  Serial2.println("AT+CMGF=1");  // Change sim900A  Serial to SIM900A
  delay(1000);
  Serial2.println("AT+CNMI=2,2,0,0,0");  // Configure module

}

/*=============================  FOR SEND LOCATION  ===========================================*/


void readMessege() {
  if (Serial2.available()) {
    String response = Serial2.readString();
    Serial.println(response);
    //remove "/n"
    String message = response;
    message.replace("\n", "");
    String ATcommand = message.substring(0, response.indexOf(':') - 1);
    ATcommand.trim();  // Trim white spaces

    if (ATcommand == "+CMT") {
      // Find the index of the first double quote
      int firstDoubleQuoteIndex = message.indexOf('\"');
      // Extract the phone number
      phoneNum = message.substring(firstDoubleQuoteIndex + 1, message.indexOf('\"', firstDoubleQuoteIndex + 1));
      int lastDoubleQuoteIndex = message.lastIndexOf('\"');
      // Extract the message
      String messageContent = message.substring(lastDoubleQuoteIndex + 1);
      messageContent.trim();
      Serial.println("Message: " + messageContent);
      Serial.println("phone number: " + phoneNum);
      //send the location
      if (phoneNum == Contact1 || phoneNum == Contact2) {  //can add more phone numbers here
        if (messageContent.equalsIgnoreCase("Location")) {
          //FoundLocation = false;
          //getGPS(phoneNum);
          GPS = false;
          SendAlert(phoneNum, url);
        }
      }
    }  //end of if ATcommand
  }
}

/*=============================  FOR SEND ALERT  ===========================================*/

void SendAlert(String phone, String link) {

  Serial.println("Initializing...");
  delay(1000);

  String message = link;

  Serial2.println("AT");  // Handshaking with SIM900
  updateSerial();

  Serial2.println("AT+CMGF=1");  // Configuring TEXT mode
  updateSerial();

  Serial2.print("AT+CMGS=\"" + phone + "\"\r");
  updateSerial();

  Serial2.print(message);  // Text content
  updateSerial();
  Serial2.write(26);
  GPS = true;
}


void updateSerial() {
  delay(500);
  while (Serial.available()) {
    Serial2.write(Serial.read());  // Forward what Serial received to Software Serial Port
  }
  while (Serial2.available()) {
    Serial.write(Serial2.read());  // Forward what Software Serial received to Serial Port
  }
}


//=================================for servo=================================================

void servo() {
  if (Serial.available()) {
    val = Serial.read();
    //Serial.println(val);

    if (val == '1') {
      pcc.write(0);
      pccstate1 = true;
      previousMillis4 = millis();
       // Store the current time

      // pinMode(15, LOW);

    } else if (val == '0') {
      pcc1.write(30);
      pccstate2 = true;
      previousMillis5 = millis();  // Store the current time

      // pinMode(15, LOW);
    }
  }

  // Check the delay for servo1
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis4 > 2000 && pccstate1) {
    servo1();
    pccstate1 = false;
    lockon=true; ///lock eka wahila
    
  }

  // Check the delay for servo2
  if (currentMillis - previousMillis5 > 2000 && pccstate2) {
    servo2();
    pccstate2 = false;
    lockon=false;
  }
}

void servo1() {
  pcc1.write(90);
}

void servo2() {
  pcc.write(180);
}



//=================================== for voltage ==================================================

void voltage() {

  float valv = analogRead(in);
  float val2 = (valv / 1024) * 5 * 0.33;
  float val3 = (val2 * 37.5) / 7.5;
  float avg = ((val3 - 5.5) / 2.9) * 100;
  
  //Serial.println(valv);
  //Serial.println(val2);
  //Serial.println(val3);
  Serial.print(avg);
  Serial.println("%");
  //delay(1000);
}

//=================================== FOR SPEED ==================================================

void speed() {
  lcd.clear();  // clear display
  lcd.setCursor(0, 0);         // move cursor to   (0, 0)
  //lcd.print("Arduino");        // print message at (0, 0)
  //delay(500);                  // display the above for two seconds
  unsigned long currentMillis = millis();

  // Calculate the elapsed time
  unsigned long elapsedTime = currentMillis - previousMillis;

  // Check if the measurement interval has passed
  if (elapsedTime >= interval) {
    // Calculate speed in km/h
    float speedMps = (float)hallCount * wheelCircumference / elapsedTime;  // Speed in meters per second
    speedKmph = speedMps * 3600;                                           // Speed in km/h

    // Display the speed
    //Serial.print("Speed: ");
    //Serial.print(speedKmph);
   // Serial.println(" km/h");

    // Reset the hall sensor count and previous time
    hallCount = 0;
    previousMillis = currentMillis;
  }
  lcd.clear();
  lcd.print("speed:");
  lcd.print(speedKmph);
  lcd.print("kmph");
}
// Interrupt service routine to count pulses from the Hall sensor
void countPulse() {
  hallCount++;
}