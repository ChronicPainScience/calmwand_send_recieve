#include <Adafruit_NeoPixel.h>
#include <Adafruit_MLX90614.h>
#include <ArduinoBLE.h>
#include <SD.h>

#define Lower_Limit     7000    // Lower temperature limit
#define Upper_Limit     9800    // Upper temperature limit
#define pass            (void)0
#define PIN             A1      // LED Pin
#define motor_pin       A3      // Motor Pin
#define NUMPIXELS       7       // Number of LED lights
#define MAX_LINE_LENGTH 80

Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
Adafruit_MLX90614 mlx;

// Bluetooth service & characteristics
BLEService radarService("87f23fe2-4b42-11ed-bdc3-0242ac120000");

BLEStringCharacteristic temperatureCharacteristic(
  "87f23fe2-4b42-11ed-bdc3-0242ac12000A",
  BLERead | BLENotify, 20
);

BLEStringCharacteristic brightnessCharacteristic(
  "87f23fe2-4b42-11ed-bdc3-0242ac12000B",
  BLERead | BLENotify | BLEWrite, 20
);

BLEStringCharacteristic inBreathTimeCharacteristic(
  "87f23fe2-4b42-11ed-bdc3-0242ac12000C",
  BLERead | BLENotify | BLEWrite, 20
);

BLEStringCharacteristic outBreathTimeCharacteristic(
  "87f23fe2-4b42-11ed-bdc3-0242ac12000D",
  BLERead | BLENotify | BLEWrite, 20
);

BLEStringCharacteristic motorStrengthCharacteristic(
  "87f23fe2-4b42-11ed-bdc3-0242ac12000E",
  BLERead | BLENotify | BLEWrite, 20
);

// Request the file list
BLEStringCharacteristic fileListRequestChar(
  "87f23fe2-4b42-11ed-bdc3-0242ac12000F",
  BLEWrite, 20
);

// Notify each filename
BLEStringCharacteristic fileNameChar(
  "87f23fe2-4b42-11ed-bdc3-0242ac120010",
  BLERead | BLENotify, 20
);

// Request file content: "GETFILE:filename"
BLEStringCharacteristic fileContentRequestChar(
  "87f23fe2-4b42-11ed-bdc3-0242ac120011",
  BLEWrite, 32
);

// Notify file lines and "EOF"
BLEStringCharacteristic fileContentChar(
  "87f23fe2-4b42-11ed-bdc3-0242ac120012",
  BLERead | BLENotify, MAX_LINE_LENGTH
);

// LED color arrays
float ColorArray[14][3] = {
  {0.25, 0.25, 0.25}, {0.3, 0, 0.3}, {0.95, 0, 0},
  {0.95, 0.15, 0}, {0.95, 0.45, 0}, {1, 1, 0},
  {0.35, 1, 0}, {0, 1, 0.02}, {0, 1, 0.2},
  {0, 1, 1}, {0, 0.125, 1}, {0.15, 0, 1},
  {0.6, 0, 1}, {1, 1, 1}
};

float ColorArraySetup[6][3] = {
  {0.8, 0, 0}, {0.4, 0, 0.8}, {0, 0, 1},
  {0, 0.5, 0.7}, {0, 1, 0}, {0, 0, 0}
};

const int NumberOfColors = sizeof(ColorArray) / sizeof(ColorArray[0]);
float OneColorRange = (Upper_Limit - Lower_Limit) / (NumberOfColors - 1);
int color1, color2, color1_index, color2_index;

const int NumOfPoints = 50;
int TemperatureValues[NumOfPoints] = {};

float MotorStrength       = 150;
float Brightness          = 0;
float MaxBrightness       = 125;
float BreathPacer, BreathTimer = 0;
float InbreathTime        = 4500;
float OutbreathTime       = 9000;
float BreathCycleTime     = InbreathTime + OutbreathTime;

bool vibration            = false;
int vibration_start       = 0;
int no_vibration_count    = 0;
int NumberOfLedsOn        = 0;
int previous_num_led_on   = 0;
bool vibration_progress[NumberOfColors * 6];

bool cancelRequested = false;

// SD logging
const int chipSelect         = D7;
File myFile;
int fileNumber               = 0;
unsigned long previousMillis = 0;
const long interval          = 1000;  // 1 s

void setup() {
  Serial.begin(115200);

  pinMode(motor_pin, OUTPUT);
  mlx.begin();
  pixels.begin();
  pixels.clear();

  // Open new SD file
  SD.begin(chipSelect);
  String fileName = "data" + String(fileNumber) + ".txt";
  while (SD.exists(fileName)) {
    fileNumber++;
    fileName = "data" + String(fileNumber) + ".txt";
  }
  myFile = SD.open(fileName, FILE_WRITE);
  if (myFile) {
    Serial.print("Recording to "); Serial.println(fileName);
  } else {
    Serial.println("Error opening file for writing");
  }

  // Load brightness config
  if (SD.exists("config.txt")) {
    File cfg = SD.open("config.txt", FILE_READ);
    if (cfg) {
      String line = cfg.readStringUntil('\n');
      line.trim();
      int stored = line.toInt();
      if (stored >= 0 && stored <= 255) MaxBrightness = stored;
      cfg.close();
    }
  }

  // Setup BLE
  BLE.begin();
  BLE.setLocalName("My CalmWand");
  BLE.setAdvertisedService(radarService);

  radarService.addCharacteristic(temperatureCharacteristic);
  radarService.addCharacteristic(brightnessCharacteristic);
  radarService.addCharacteristic(inBreathTimeCharacteristic);
  radarService.addCharacteristic(outBreathTimeCharacteristic);
  radarService.addCharacteristic(motorStrengthCharacteristic);
  radarService.addCharacteristic(fileListRequestChar);
  radarService.addCharacteristic(fileNameChar);
  radarService.addCharacteristic(fileContentRequestChar);
  radarService.addCharacteristic(fileContentChar);

  BLE.addService(radarService);
  BLE.advertise();

  // Initial parameter values
  brightnessCharacteristic.setValue(String(MaxBrightness));
  inBreathTimeCharacteristic.setValue(String(InbreathTime));
  outBreathTimeCharacteristic.setValue(String(OutbreathTime));
  motorStrengthCharacteristic.setValue(String(MotorStrength));

  AnimateLEDs();
}

void loop() {
  BLE.poll();

  // Brightness write
  if (brightnessCharacteristic.written()) {
    float v = constrain(brightnessCharacteristic.value().toFloat(), 0, 255);
    MaxBrightness = v;
    SD.remove("config.txt");
    File cfg = SD.open("config.txt", FILE_WRITE);
    if (cfg) { cfg.println((int)MaxBrightness); cfg.close(); }
    brightnessCharacteristic.setValue(String(MaxBrightness));
  }

  // In-breath write
  if (inBreathTimeCharacteristic.written()) {
    InbreathTime = inBreathTimeCharacteristic.value().toFloat();
    inBreathTimeCharacteristic.setValue(String(InbreathTime));
  }

  // Out-breath write
  if (outBreathTimeCharacteristic.written()) {
    OutbreathTime = outBreathTimeCharacteristic.value().toFloat();
    outBreathTimeCharacteristic.setValue(String(OutbreathTime));
  }

  BreathCycleTime = InbreathTime + OutbreathTime;

  // Motor strength write
  if (motorStrengthCharacteristic.written()) {
    MotorStrength = constrain(motorStrengthCharacteristic.value().toFloat(), 0, 255);
    motorStrengthCharacteristic.setValue(String(MotorStrength));
  }

  // Handle GETLIST
  if (fileListRequestChar.written()) {
    String cmd = fileListRequestChar.value();
    if (cmd == "GETLIST") {
      Serial.println("sendFileList() called.");

      File root = SD.open("/");
      if (root) {
        File entry = root.openNextFile();
        bool anyFile = false;

        while (entry) {
          if (!entry.isDirectory()) {
            String fname = entry.name();
            File f = SD.open(fname, FILE_READ);
            int lineCount = 0;
            while (f.available()) {
              if (f.readStringUntil('\n').length()) lineCount++;
            }
            f.close();

            int mins = max(1, lineCount / 60);
            String out = fname + ":" + String(mins);
            fileNameChar.setValue(out);
            Serial.print("Notifying: "); Serial.println(out);

            unsigned long t0 = millis();
            while (millis() - t0 < 10) BLE.poll();

            anyFile = true;
          }
          entry.close();
          entry = root.openNextFile();
        }
        root.close();

        if (!anyFile) Serial.println("ℹNo files found.");
      } else {
        Serial.println("Could not open root directory!");
      }

      fileNameChar.setValue("END");
      Serial.println("🏁 All filenames sent; notifying END.");
      unsigned long t1 = millis();
      while (millis() - t1 < 10) BLE.poll();
    }
  }

  // Handle GETFILE / CANCEL
  if (fileContentRequestChar.written()) {
    String cmd = fileContentRequestChar.value();
    if (cmd.startsWith("GETFILE:")) {
      cancelRequested = false;
      String filename = cmd.substring(8);
      filename.trim();
      sendFileContent(filename);
    }
    else if (cmd == "CANCEL") {
      cancelRequested = true;
    }
  }

  // --- Biofeedback loop (temperature, LEDs, motor, SD logging) ---

  int Colors[2];
  int StartTime = millis();
  int Temp = 100 * mlx.readObjectTempF();
  float AverageTemp = RunningAverageTemp(Temp);
  BreathPacer = millis() - BreathTimer;

  // Brightness curve
  if (BreathPacer < InbreathTime) {
    Brightness = MaxBrightness * pow(BreathPacer / InbreathTime, 2);
  }
  else if (BreathPacer <= BreathCycleTime) {
    Brightness = MaxBrightness * pow((BreathCycleTime - BreathPacer) / OutbreathTime, 2);
  } else {
    Brightness = 0;
  }

  if (BreathPacer >= BreathCycleTime) {
    BreathTimer = millis();
  }

  DetermineColors(AverageTemp, Colors);
  NumberOfLedsOn = DetermineNumberofLedsOn(AverageTemp);

  SettingPixelColor(
    ConvertRGBtoInt(ColorArray[Colors[0]], Brightness),
    ConvertRGBtoInt(ColorArray[Colors[1]], Brightness),
    NumberOfLedsOn
  );

  SendDataOverBLE(AverageTemp);

  pixels.show();
  pixels.clear();

  // SD log every second
  if (millis() - previousMillis >= interval) {
    previousMillis = millis();
    if (myFile) {
      myFile.print(previousMillis);
      myFile.print(" ");
      myFile.println(AverageTemp);
      myFile.flush();
    }
  }

  while (millis() - StartTime < 20) pass;
}

// ── Helper Functions ──────────────────────────────────────────────────────────

void AnimateLEDs() {
  for (int b = 0; b < 2; b++) {
    for (int i = 0; i < NUMPIXELS; i++) {
      pixels.setPixelColor(i, pixels.Color(60, 60, 60));
    }
    pixels.show();
    delay(500);
    for (int i = 0; i < NUMPIXELS; i++) {
      pixels.setPixelColor(i, pixels.Color(0, 0, 0));
    }
    pixels.show();
    delay(300);
  }
}

void shiftColors() {
  float temp[3];
  memcpy(temp, ColorArraySetup[0], sizeof(temp));
  for (int i = 0; i < 5; i++) {
    memcpy(ColorArraySetup[i], ColorArraySetup[i + 1], sizeof(temp));
  }
  memcpy(ColorArraySetup[5], temp, sizeof(temp));
}

int ConvertRGBtoInt(float rgb[3], float bright) {
  int r = round(bright * rgb[0]);
  int g = round(bright * rgb[1]);
  int b = round(bright * rgb[2]);
  return pixels.Color(r, g, b);
}

void DetermineColors(float Temp, int Colors[2]) {
  for (int i = 0; i < NumberOfColors; i++) {
    if (Temp < Lower_Limit) {
      color1_index = color2_index = 0;
      break;
    }
    else if (Temp < Lower_Limit + (i + 1) * OneColorRange) {
      color1_index = i;
      color2_index = i + 1;
      break;
    }
    else if (Temp > Upper_Limit) {
      color1_index = color2_index = NumberOfColors - 1;
      break;
    }
  }
  Colors[0] = color1_index;
  Colors[1] = color2_index;
}

int DetermineNumberofLedsOn(float Temp) {
  float baseline = Temp - Lower_Limit;
  float remainder = fmod(baseline, OneColorRange);
  float oneLed = OneColorRange / 6;
  int leds = int(remainder / oneLed);
  leds = constrain(leds, 0, 6);
  return leds;
}

void SettingPixelColor(int c1, int c2, int numOn) {
  for (int i = 1; i <= numOn; i++) pixels.setPixelColor(i, c2);
  for (int i = numOn + 1; i < NUMPIXELS; i++) pixels.setPixelColor(i, c1);
}

float RunningAverageTemp(int Temp) {
  if (millis() < 1000) return Temp;
  int sum = 0;
  for (int i = 0; i < NumOfPoints - 1; i++) {
    TemperatureValues[i] = TemperatureValues[i + 1];
  }
  TemperatureValues[NumOfPoints - 1] = Temp;
  for (int i = 0; i < NumOfPoints; i++) sum += TemperatureValues[i];
  return sum / NumOfPoints;
}

void SendDataOverBLE(float temp) {
  BLEDevice central = BLE.central();
  if (central && central.connected()) {
    temperatureCharacteristic.setValue(String(temp));
  }
}

void sendFileContent(const String &filename) {
  File f = SD.open(filename, FILE_READ);
  if (!f) return;
  cancelRequested = false;
  while (f.available()) {
    BLE.poll();
    if (cancelRequested) break;
    String line = f.readStringUntil('\n');
    line.trim();
    if (line.length()) {
      fileContentChar.setValue(line);
      unsigned long t0 = millis();
      while (millis() - t0 < 10) {
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
