#include <Adafruit_NeoPixel.h>
#include <Adafruit_MLX90614.h>
#include <ArduinoBLE.h>
#include <SD.h> 

#define Lower_Limit 7000  // Lower temperature limit
#define Upper_Limit 9800  // Upper temperature limit

#define pass (void)0
#define PIN A1  // LED Pin
#define motor_pin A3  // Motor Pin
#define NUMPIXELS 7  // Number of led lights
#define MAX_LINE_LENGTH 80

Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);  // Initialize LEDs
Adafruit_MLX90614 mlx = Adafruit_MLX90614();  // Initialize infrared temperature sensor


//Bluetooth settings
BLEService radarService("87f23fe2-4b42-11ed-bdc3-0242ac120000"); 
// add BLEWrite·  

BLEStringCharacteristic temperatureCharacteristic(
  "87f23fe2-4b42-11ed-bdc3-0242ac12000A",   
  BLERead | BLENotify,                     
  20
);

// [B] brightness（phone -> Arduino)
BLEStringCharacteristic brightnessCharacteristic(
  "87f23fe2-4b42-11ed-bdc3-0242ac12000B",  //different UUID
  BLERead | BLENotify | BLEWrite,                      
  20
);

BLEStringCharacteristic inBreathTimeCharacteristic(
  "87f23fe2-4b42-11ed-bdc3-0242ac12000C",  //different UUID
  BLERead | BLENotify | BLEWrite,                      
  20
);

BLEStringCharacteristic outBreathTimeCharacteristic(
  "87f23fe2-4b42-11ed-bdc3-0242ac12000D",  //different UUID
  BLERead | BLENotify | BLEWrite,                      
  20
);

BLEStringCharacteristic motorStrengthCharacteristic(
  "87f23fe2-4b42-11ed-bdc3-0242ac12000E",  // different UUID
  BLERead | BLENotify | BLEWrite,
  20
);

// (1) Add a “request” characteristic that the iOS app will write “GETLIST” to:
BLEStringCharacteristic fileListRequestChar(
  "87f23fe2-4b42-11ed-bdc3-0242ac12000F",
  BLEWrite,   // app ⇒ Arduino only
  20          // max 20 bytes (we only need “GETLIST”)
);

  // (2) Add a “filename” characteristic that Arduino will use to send back 
  //     each filename via notifications:
BLEStringCharacteristic fileNameChar(
  "87f23fe2-4b42-11ed-bdc3-0242ac120010",
  BLERead    | BLENotify,
  20        // max 20 bytes per notification
);

// ── NEW: “File‐content request” → iOS writes “GETFILE:dataX.txt” here:
BLEStringCharacteristic fileContentRequestChar(
  "87f23fe2-4b42-11ed-bdc3-0242ac120011",
  BLEWrite,
  32    // allow up to 32‐byte filename
);

// ── NEW: “File‐content notify” → Arduino sends each line (and “EOF”):
BLEStringCharacteristic fileContentChar(
  "87f23fe2-4b42-11ed-bdc3-0242ac120012",
  BLERead | BLENotify,
  MAX_LINE_LENGTH
);


// Color progression of LEDs
float ColorArray[14][3] = {
  { 0.25, 0.25, 0.25 },  // gray
  { 0.3, 0, 0.3 },       // dark purple
  { 0.95, 0, 0 },        // red
  { 0.95, 0.15, 0 },     // orange-red
  { 0.95, 0.45, 0 },     // orange
  { 1, 1, 0 },           // yellow
  { 0.35, 1, 0 },        // yellow-green
  { 0, 1, 0.02 },        // green
  { 0, 1, 0.2 },         // green-cyan
  { 0, 1, 1 },           // cyan
  { 0, 0.125, 1 },       // blue
  { 0.15, 0, 1 },        // indigo
  { 0.6, 0, 1 },         // violet
  { 1, 1, 1 }            // white
};

// Colors for beginning vibration
float ColorArraySetup[6][3] = {
  { 0.8, 0, 0 },    // bright red
  { 0.4, 0, 0.8 },  // purple
  { 0, 0, 1 },      // blue
  { 0, 0.5, 0.7 },  // teal
  { 0, 1, 0 },      // green
  { 0, 0, 0 }       // black
}; 

// Constants
const int NumberOfColors = sizeof(ColorArray) / sizeof(ColorArray[0]);
float OneColorRange = (Upper_Limit - Lower_Limit) / (NumberOfColors - 1);   // Temperature range of one color
int color1;
int color2;
int color1_index;
int color2_index;
const int NumOfPoints = 50;
int TemperatureValues[NumOfPoints] = {};
float MotorStrength = 150; //added
float Brightness = 0;
float MaxBrightness = 125;  //Out of 255
float BreathPacer;   // Time relative to the start time of this breath cycle
float BreathTimer = 0;  // Start time of this breath cycle
float InbreathTime = 4500; 
float OutbreathTime = 9000;  
float BreathCycleTime = InbreathTime + OutbreathTime;
bool vibration = 0;   // Whether there is a current vibration
int vibration_start = 0;  // Start time of the current vibration
int no_vibration_count = 0;  // Time since the last vibration
int NumberOfLedsOn = 0;  // Number of leds in color1
int previous_num_led_on = 0;  // Number of leds in color1 in the previous loop
bool vibration_progress[NumberOfColors * 6];

bool cancelRequested = false;

// Constants for saving data
const int chipSelect = D7;  
File myFile;
int fileNumber = 0; 
unsigned long previousMillis = 0;   // time of previous data saving time
const long interval = 1000;   // save 1 temperature data point every "interval" ms

void setup() {
  Serial.begin(115200);

  pinMode(motor_pin, OUTPUT);  // Vibration motor pin
  mlx.begin();  // Infrared temperature sensor
  pixels.begin();  // LED
  pixels.clear();  

  // Create new file in SD card
  SD.begin(chipSelect); 
  String fileName = "data" + String(fileNumber) + ".txt";
  while (SD.exists(fileName)) {
    fileNumber++;  // Increment file number if it already exists
    fileName = "data" + String(fileNumber) + ".txt";
  }
  myFile = SD.open(fileName, FILE_WRITE);
  if (myFile) {
    Serial.print("Recording to ");
    Serial.println(fileName);
  } else {
    Serial.println("Error opening file for writing");
  }

  if (SD.exists("config.txt")) {
  File cfg = SD.open("config.txt", FILE_READ);
  if (cfg) {
    // read one full line
    String line = cfg.readStringUntil('\n');
    line.trim();                           // drop any \r or whitespace
    int stored = line.toInt();             // returns 0 if the line isn’t a number
    if (stored >= 0 && stored <= 255) {
      MaxBrightness = stored;
    }
    Serial.print("Loaded brightness from SD: ");
    Serial.println(MaxBrightness);
    cfg.close();
  } else {
    Serial.println("Error opening config.txt for read");
  }
  } else {
    Serial.println("config.txt not found; using default brightness");
  }

  BLE.begin(); //initialize BLE
  BLE.setDeviceName("CalmWand 14B");  // Sets the actual device name
  BLE.setLocalName("CalmWand 14B"); // Make sure this matches above
  BLE.setAdvertisedService(radarService);

  radarService.addCharacteristic(temperatureCharacteristic); 
  radarService.addCharacteristic(brightnessCharacteristic);  
  radarService.addCharacteristic(inBreathTimeCharacteristic);  
  radarService.addCharacteristic(outBreathTimeCharacteristic);  
  radarService.addCharacteristic(motorStrengthCharacteristic); 
  radarService.addCharacteristic(fileListRequestChar);
  radarService.addCharacteristic(fileNameChar);
  radarService.addCharacteristic(fileContentRequestChar);   // NEW
  radarService.addCharacteristic(fileContentChar);          // NEW

  BLE.addService(radarService);  // add service

  BLE.advertise();// start advertising

  brightnessCharacteristic.setValue(String(MaxBrightness));

  // 3) inbreathTime data
  String inhaleData = String(InbreathTime);
  inBreathTimeCharacteristic.setValue(inhaleData);

  // 4) outbreathTime data
  String exhaleData = String(OutbreathTime);
  outBreathTimeCharacteristic.setValue(exhaleData);

  // 5) motorStrength data
  String motorData = String(MotorStrength);
  motorStrengthCharacteristic.setValue(motorData);

  // Initialize vibration progress array
  for (int i = 0; i < NumberOfColors * 6; i++) {
    vibration_progress[i] = 0;
  }

  AnimateLEDs();  // Initial animation
}

// void AnimateLEDs() {
//   unsigned long startTime = millis();
//   unsigned long lastShiftTime = millis();

//   while (millis() - startTime < 6000) {  // 6 seconds
//     if (millis() - lastShiftTime >= 400) {  // Shift colors every 0.4 second
//       shiftColors();
//       lastShiftTime = millis();
//     }

//     // Set LED colors according to current state
//     for (int i = 0; i < 6; i++) {
//       int colorIndex = (i - 1) % 6;

//       uint8_t r = ColorArraySetup[i][0] * 60;
//       uint8_t g = ColorArraySetup[i][1] * 60;
//       uint8_t b = ColorArraySetup[i][2] * 60;

//       pixels.setPixelColor(i + 1, pixels.Color(r, g, b));
//     }

//     pixels.show();
//   }
// }

void AnimateLEDs() {
  for (int blink = 0; blink < 2; blink++) {
    for (int i = 0; i < NUMPIXELS; i++) {
      pixels.setPixelColor(i, pixels.Color(60, 60, 60));  // white light
    }
    pixels.show();
    delay(500);  // last 300ms

    // light off
    for (int i = 0; i < NUMPIXELS; i++) {
      pixels.setPixelColor(i, pixels.Color(0, 0, 0));
    }
    pixels.show();
    delay(300);  // blink interval
  }
}


void shiftColors() {
  float tempColor[3];
  memcpy(tempColor, ColorArraySetup[0], sizeof(tempColor));

  for (int i = 0; i < 5; i++) {
    memcpy(ColorArraySetup[i], ColorArraySetup[i + 1], sizeof(tempColor));
  }

  memcpy(ColorArraySetup[5], tempColor, sizeof(tempColor));
}


// Converts RGB values of a color into intger values
int ConvertRGBtoInt(float RGBValues[3], float Brightness) {

  int R = round(Brightness * (RGBValues[0]));
  int G = round(Brightness * (RGBValues[1]));
  int B = round(Brightness * (RGBValues[2]));

  int color = pixels.Color(R, G, B);

  return color;
};


// Determines two colors to be dislpayed according to temperature range
void DetermineColors(float Temp, int Colors[2], float ColorArray[NumberOfColors][3], float OneColorRange, int NumberOfColors, float Brightness) {

  for (int i = 0; i < NumberOfColors; i++) {
    if (Temp < Lower_Limit) {
      color1_index = 0;
      color2_index = 0;
      break;
    } else if (Temp < Lower_Limit + (i + 1) * OneColorRange) {
      color1_index = i;
      color2_index = i + 1;
      break;
    } else if (Temp > Upper_Limit) {
      color1_index = NumberOfColors - 1;
      color2_index = NumberOfColors - 1;
      break;
    } else {
      pass;
    }
  }

  Colors[0] = color1_index;
  Colors[1] = color2_index;
}

// Determine number of LEDs of the next color (Color2) to be on
int DetermineNumberofLedsOn(float Temp, float OneColorRange, int currentLedsOn) {
  float baseline = Temp - Lower_Limit;
  float remainder = fmod(baseline, OneColorRange);
  float OneLedRange = OneColorRange / 6;
  float threshold = 0.5 * OneLedRange;  // Adjust this value as needed

  int NumberOfLedsOn = static_cast<int>(remainder / OneLedRange);

  // Make NumberOfLedsOn harder to drop to prevent flickering between two colors
  if (NumberOfLedsOn < currentLedsOn) {
    if (remainder > (NumberOfLedsOn + 1) * OneLedRange - threshold) {
      NumberOfLedsOn = currentLedsOn;  // Maintain the current state
    }
  }
  NumberOfLedsOn = min(7, max(NumberOfLedsOn, 0));
  return NumberOfLedsOn;
}

// Assign colors to LEDs
void SettingPixelColor(int color1, int color2, int NumberOfLedsOn, float Temp, float OneColorRange) {

  float baseline = Temp - Lower_Limit;
  float remainder = fmod(baseline, OneColorRange);
  float OneLedRange = OneColorRange / 6;

  for (int i = 1; i < NumberOfLedsOn + 1; i++) {
    pixels.setPixelColor(i, color2);
  }
  delay(1);
  for (int i = NumberOfLedsOn + 1; i < 7; i++) {
    pixels.setPixelColor(i, color1);
  }
  delay(1);
}

// Running average of temperature in the last second
int RunningAverageTemp(int Temp, int NumOfPoints) {
  float AverageTemp = 0;
  int SumOfTemps = 0;
  int NumOfSteps = NumOfPoints - 1;

  if (millis() < 1000) {
    AverageTemp = Temp;
  } else {

    if (Temp > 0 and Temp < 20000) {

      for (int i = 0; i < NumOfSteps; i++) {
        TemperatureValues[i] = TemperatureValues[i + 1];
      }

      TemperatureValues[NumOfSteps] = Temp;

      for (int i = 0; i < NumOfPoints; i++) {
        SumOfTemps += TemperatureValues[i];
      }

      AverageTemp = SumOfTemps / NumOfPoints;

    }

    else {

      for (int i = 0; i < NumOfPoints; i++) {
        SumOfTemps += TemperatureValues[i];
      }

      AverageTemp = SumOfTemps / NumOfPoints;
    }
  }

  return AverageTemp;
}

//Sending data over BLE
void SendDataOverBLE(float AverageTemp, float currentBrightness, float inhaleTime, float exhaleTime, float motorStrength) {
  BLEDevice central = BLE.central(); // Listen for BLE central

  // if a central is connected:
  if (central && central.connected()) {
    // 1) temp data
    String tempData = String(AverageTemp);
    temperatureCharacteristic.setValue(tempData);
  }
}
void sendFileContent(const String &filename) {
  File f = SD.open(filename, FILE_READ);
  if (!f) return;

  cancelRequested = false;
  while (f.available()) {
    BLE.poll();                        // service any pending writes (e.g. CANCEL)
    if (cancelRequested) break;

    String line = f.readStringUntil('\n');
    if (line.length() > 0) {
      fileContentChar.setValue(line);
      // wait just long enough to let the BLE radio clear its buffer:
      unsigned long start = millis();
      while (millis() - start < 10) {
        BLE.poll();
        if (cancelRequested) break;
      }
    }
  }
  f.close();
  if (!cancelRequested) {
    fileContentChar.setValue("EOF");
    delay(10);
  }
}

void loop() {

  BLE.poll();

  if (brightnessCharacteristic.written()) {
    float v = constrain(brightnessCharacteristic.value().toFloat(), 0, 255);
    MaxBrightness = v;

    // Persist to SD
    SD.remove("config.txt");
    File cfg = SD.open("config.txt", FILE_WRITE);
    if (cfg) {
      cfg.println((int)MaxBrightness);
      cfg.close();
    }

    // Echo back
    brightnessCharacteristic.setValue(String(MaxBrightness));
}
  
  // update inhale time
  if (inBreathTimeCharacteristic.written()) {

    String inStr = inBreathTimeCharacteristic.value(); // In swift, the string send should be like "4500"
    float newInhale = inStr.toFloat();

    InbreathTime = newInhale; 
    Serial.print("New Inhale Time from BLE: ");
    Serial.println(InbreathTime);

    String inhaleData = String(InbreathTime);
    inBreathTimeCharacteristic.setValue(inhaleData);
  }

  // update exhale time
  if (outBreathTimeCharacteristic.written()) {

    String outStr = outBreathTimeCharacteristic.value();
    float newExhale = outStr.toFloat();

    OutbreathTime = newExhale;
    Serial.print("New Exhale Time from BLE: ");
    Serial.println(OutbreathTime);

      
    String exhaleData = String(OutbreathTime);
    outBreathTimeCharacteristic.setValue(exhaleData);
  }

  // update BreathCycleTime
  BreathCycleTime = InbreathTime + OutbreathTime;

  if (motorStrengthCharacteristic.written()) {
    String motorStr = motorStrengthCharacteristic.value(); 
    float newMotorStrength = motorStr.toFloat(); // 

    if (newMotorStrength < 0)   newMotorStrength = 0;
    if (newMotorStrength > 255) newMotorStrength = 255;

    MotorStrength = newMotorStrength;
    Serial.print("The motorStr is ");
    Serial.println(motorStr);
    Serial.print("New Motor Strength from BLE: ");
    Serial.println(MotorStrength);

    String motorData = String(MotorStrength);
    motorStrengthCharacteristic.setValue(motorData);
  }
  
  if (fileListRequestChar.written()) {
    String cmd = fileListRequestChar.value();  // e.g. “GETLIST”
    if (cmd == "GETLIST") {
      Serial.println("sendFileList() called.");

      File root = SD.open("/");
      if (!root) {
        Serial.println("❌ Could not open root directory!");
        return;
      }

      File entry = root.openNextFile();
      bool anyFile = false;
      while (entry) {
        if (!entry.isDirectory()) {
          String fname = String(entry.name());  // e.g. "data0.txt"

         // compute minutes length (optional)
          File f = SD.open(fname, FILE_READ);
          int lineCount = 0;
          while (f.available()) {
            String l = f.readStringUntil('\n');
            if (l.length() > 0) lineCount++;
          }
          f.close();
          int mins = max(1, lineCount / 60);
          String out = fname + ":" + String(mins);

          // 1) Send the notification
          fileNameChar.setValue(out);
          Serial.print("⏩ Notifying: ");
          Serial.println(out);

          // 2) Service BLE for ~10 ms instead of delay(100)
          unsigned long start = millis();
          while (millis() - start < 10) {
            BLE.poll();   // handle any incoming writes (e.g. CANCEL)
          }

          anyFile = true;
        }
        entry.close();
        entry = root.openNextFile();
      }
      root.close();

      if (!anyFile) {
        Serial.println("ℹ️  No files found.");
      }

      // finally, send END
      fileNameChar.setValue("END");
      Serial.println("🏁 All filenames sent; notifying END.");
      unsigned long start = millis();
      while (millis() - start < 10) {
        BLE.poll();
      }
    }
  }
  if (fileContentRequestChar.written()) {
    String cmd = fileContentRequestChar.value();
    Serial.print("📥 fileContentRequestChar: \"");
    Serial.print(cmd);
    Serial.println("\"");

    if (cmd.startsWith("GETFILE:")) {
      Serial.println("here");
      cancelRequested = false;  // clear any previous cancel
      String filename = cmd.substring(strlen("GETFILE:"));
      filename.trim();
      sendFileContent(filename);
    }
    else if (cmd == "CANCEL") {
      cancelRequested = true;
      Serial.println("⚠️ Cancel requested");
    }
  }

  //main loop
  int Colors[2];
  int StartTime = millis();
  int Temp = 100 * mlx.readObjectTempF();                     // get temparature
  float AverageTemp = RunningAverageTemp(Temp, NumOfPoints);  // find average of temperature
  BreathPacer = millis() - BreathTimer;

  // Calculate brightness based on inhale and exhale
  if (BreathPacer < InbreathTime) {
    Brightness = MaxBrightness * pow((BreathPacer / InbreathTime), 2);
  }
  else if (BreathPacer >= InbreathTime && BreathPacer <= BreathCycleTime) {
    Brightness = MaxBrightness * pow(-(BreathPacer - (InbreathTime + OutbreathTime)) / OutbreathTime, 2);
  } else {
    Brightness = 0;
  }

  // Reset Timer if in next breath cycle
  if (BreathPacer >= BreathCycleTime) {
    BreathTimer = millis();
  }

  DetermineColors(AverageTemp, Colors, ColorArray, OneColorRange, NumberOfColors, Brightness);  //determine colors
  NumberOfLedsOn = DetermineNumberofLedsOn(AverageTemp, OneColorRange, NumberOfLedsOn);         //determine the number of LEDs of each color

  // Reward vibration; use "vibration_progress" array to keep track of vibration progress and prevent flickering; if no new vibration progress for a while, reset vibration progress
  if ((NumberOfLedsOn > previous_num_led_on || (NumberOfLedsOn == 0 && previous_num_led_on == 5)) && vibration_progress[color1_index * 6 + NumberOfLedsOn] == 0) {
    vibration = 1;
    vibration_start = millis();
    for (int i = 0; i <= color1_index * 6 + NumberOfLedsOn; i++) {vibration_progress[i] = 1; }
  }
  if (NumberOfLedsOn == previous_num_led_on && vibration != 1) {
    no_vibration_count++;
    if (no_vibration_count >= 200) {
      for (int i = 0; i < NumberOfColors * 6; i++) { vibration_progress[i] = 0; }
    }
  } else {
    no_vibration_count = 0;
  }

  if (vibration == 1) {
    analogWrite(motor_pin, MotorStrength);
    if (millis() - vibration_start > 300) {vibration = 0; }
  } else {
    analogWrite(motor_pin, 0);
  }
  previous_num_led_on = NumberOfLedsOn;

  color1 = ConvertRGBtoInt(ColorArray[Colors[0]], Brightness); 
  color2 = ConvertRGBtoInt(ColorArray[Colors[1]], Brightness); 
  SettingPixelColor(color1, color2, NumberOfLedsOn, AverageTemp, OneColorRange);

  //send data through BLE
  SendDataOverBLE(AverageTemp, MaxBrightness, InbreathTime, OutbreathTime, MotorStrength);

  // Set center LED to green and blue for inhale and exhale
  if (BreathPacer < InbreathTime) {pixels.setPixelColor(0, pixels.Color(0, 20, 0)); }
  else if (BreathPacer < InbreathTime + OutbreathTime) { pixels.setPixelColor(0, pixels.Color(0, 0, 0)); }

  pixels.show();
  pixels.clear();

  // Save temperature data
  if (StartTime - previousMillis >= interval) {
      previousMillis = StartTime;
      
      if (myFile) {
        myFile.print(StartTime); myFile.print(" ");
        myFile.println(AverageTemp);
        myFile.flush();  // Ensure data is written to the file
        // Serial.print(StartTime); Serial.print(" ");
        // Serial.println(AverageTemp);
      } else {
        Serial.println("Error writing to file");
      }
    }

  // Serial.print(NumberOfLedsOn); Serial.print('\t'); Serial.print(previous_num_led_on); Serial.print('\t'); Serial.print(vibration); Serial.print('\t'); Serial.println(no_vibration_count);
  // Serial.print(AverageTemp); Serial.print('\t'); Serial.print(color1_index); Serial.print('\t'); Serial.print(NumberOfLedsOn); Serial.print('\t'); Serial.println(no_vibration_count);

  //Serial.print(StartTime); Serial.print(' '); Serial.print(BreathTimer); Serial.print(' '); Serial.println(Temp);  
  //Serial.println(Temp);

  while (millis() - StartTime < 20) {
    pass;
  }
}