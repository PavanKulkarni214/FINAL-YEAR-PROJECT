#include <LiquidCrystal.h>

// ====================== Hardware Configuration ======================
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);
#define gsmSerial Serial1

// Pin Definitions
#define VOLTAGE_R A0
#define VOLTAGE_Y A1
#define VOLTAGE_B A2
#define CURRENT_R A3
#define CURRENT_Y A4
#define CURRENT_B A5
#define TEMP_POT  A6
#define RELAY_PIN 13
#define BUTTON_PIN 7
#define MODE_SELECT_PIN 8

// ====================== System Parameters ======================
#define MIN_VOLTAGE 180
#define MAX_VOLTAGE 260
#define MIN_CURRENT 0.060 // 60mA
#define MAX_CURRENT 0.100 // 100mA
#define MIN_TEMP 5
#define MAX_TEMP 80

// Scaling factors
#define POT_VOLTAGE_SCALE (300.0 / 1023.0)
#define POT_CURRENT_SCALE (0.2 / 1023.0)
#define POT_TEMP_SCALE (100.0 / 1023.0)
#define VREF 5.0 // Reference voltage for LM35

// ====================== Voltage Sensing ======================
const int adc_max = 695;              
const int adc_min = 332;                
const float volt_multi = 244.5;       // Calibrated scaling factor
float volt_multi_p = volt_multi * 1.4142;   
float volt_multi_n = -volt_multi_p;         
const int avg_window = 5;             // For smoothing
float avg_buffer_R[avg_window] = {0};
float avg_buffer_Y[avg_window] = {0};
float avg_buffer_B[avg_window] = {0};
int avg_index_R = 0;
int avg_index_Y = 0;
int avg_index_B = 0;

// ====================== Current Sensing ======================
const float current_calibrationFactor = 0.00430;  // From your working code
const int current_sampleCount = 1000;             // Samples for RMS calculation
float current_voltageOffset_R = 0;                // Offsets for each phase
float current_voltageOffset_Y = 0;
float current_voltageOffset_B = 0;

// ====================== System State ======================
String registeredNumbers[] = { "+917975004675" };
const int totalRegistered = sizeof(registeredNumbers) / sizeof(registeredNumbers[0]);

bool remoteControlEnabled = false;
bool manualControlState = false;
bool lastMotorState = false;
String lastFaultReason = "";
bool faultActive = false;
unsigned long faultDisplayStart = 0;
bool faultFlashShown = false;
String smsBuffer = "";
bool newSms = false;
bool lastButtonState = HIGH;
bool motorShouldRun = false;

// ====================== Setup ======================
void setup() {
  Serial.begin(9600);
  gsmSerial.begin(9600);
  lcd.begin(20, 4);

  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, HIGH); // Motor starts OFF
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(MODE_SELECT_PIN, INPUT_PULLUP);

  randomSeed(analogRead(0));
  initializeGSM();

  // Calibrate current sensors (important for accurate readings)
  current_voltageOffset_R = calibrateCurrentOffset(CURRENT_R);
  current_voltageOffset_Y = calibrateCurrentOffset(CURRENT_Y);
  current_voltageOffset_B = calibrateCurrentOffset(CURRENT_B);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("System Powered ON");
  lcd.setCursor(0, 1);
  lcd.print("Motor Status: OFF");

  // Notify registered numbers
  for (int i = 0; i < totalRegistered; i++) {
    sendSMS(registeredNumbers[i], "System ON. Motor is OFF.\nSend 'motoron' to start.");
  }
}

// ====================== Voltage Measurement ======================
float get_voltage(int pin) {
  float adc_sample;
  float volt_inst = 0;
  float sum = 0;
  float volt;
  long init_time = millis();
  int N = 0;

  while ((millis() - init_time) < 200) { // 200ms sampling window
    adc_sample = analogRead(pin);           
    volt_inst = map(adc_sample, adc_min, adc_max, volt_multi_n, volt_multi_p);
    sum += sq(volt_inst);                   
    N++;
    delay(1);
  }

  volt = sqrt(sum / N); // Calculate RMS voltage
  return volt;
}

float readVoltageSensor(int pin) {
  float raw_vrms;
  
  if (pin == VOLTAGE_R) {
    raw_vrms = get_voltage(VOLTAGE_R);
    // Apply moving average filter
    avg_buffer_R[avg_index_R] = raw_vrms;
    avg_index_R = (avg_index_R + 1) % avg_window;
    float filtered_vrms = 0;
    for (int i = 0; i < avg_window; i++) {
      filtered_vrms += avg_buffer_R[i];
    }
    return filtered_vrms / avg_window;
  }
  else if (pin == VOLTAGE_Y) {
    raw_vrms = get_voltage(VOLTAGE_Y);
    avg_buffer_Y[avg_index_Y] = raw_vrms;
    avg_index_Y = (avg_index_Y + 1) % avg_window;
    float filtered_vrms = 0;
    for (int i = 0; i < avg_window; i++) {
      filtered_vrms += avg_buffer_Y[i];
    }
    return filtered_vrms / avg_window;
  }
  else if (pin == VOLTAGE_B) {
    raw_vrms = get_voltage(VOLTAGE_B);
    avg_buffer_B[avg_index_B] = raw_vrms;
    avg_index_B = (avg_index_B + 1) % avg_window;
    float filtered_vrms = 0;
    for (int i = 0; i < avg_window; i++) {
      filtered_vrms += avg_buffer_B[i];
    }
    return filtered_vrms / avg_window;
  }
  
  return 0.0;
}

// ====================== Current Measurement ======================
float calibrateCurrentOffset(int pin) {
  long total = 0;
  for (int i = 0; i < 500; i++) { // Take 500 samples for calibration
    total += analogRead(pin);
    delay(1);
  }
  return total / 500.0; // Return average offset
}

float measureCurrentRMS(int pin, float voltageOffset) {
  long total = 0;
  for (int i = 0; i < current_sampleCount; i++) {
    float voltage = analogRead(pin) - voltageOffset;
    total += voltage * voltage; // Sum of squares
    delay(1);
  }
  return sqrt(total / (float)current_sampleCount) * current_calibrationFactor;
}

// Modified to force zero current when motor is off
float readCurrentSensor(int pin) {
  // Motor is OFF (RELAY_PIN HIGH = motor off)
  if (digitalRead(RELAY_PIN)) { 
    return 0.0; // Force zero current reading
  }

  // Motor is ON - return actual measurement
  if (pin == CURRENT_R) {
    return measureCurrentRMS(CURRENT_R, current_voltageOffset_R);
  } else if (pin == CURRENT_Y) {
    return measureCurrentRMS(CURRENT_Y, current_voltageOffset_Y);
  } else if (pin == CURRENT_B) {
    return measureCurrentRMS(CURRENT_B, current_voltageOffset_B);
  }

  return 0.0;
}


// ====================== Temperature Measurement ======================
float readStableLM35Temperature(int pin) {
  const int numSamples = 50;
  const float maxChange = 2.0; // Max allowed temp change (°C)
  static float prevTemp = 25.0; // Starting at room temp
  const float tempCalibrationOffset = 0.0;
  
  int sum = 0;
  int validSamples = 0;
  
  for (int i = 0; i < numSamples; i++) {
    int raw = analogRead(pin);
    float voltage = raw * (VREF / 1023.0);
    float currentTemp = voltage * 100.0; // LM35: 10mV/°C
    
    // Reject outliers (>5°C from previous)
    if (i == 0 || abs(currentTemp - prevTemp) < 5.0) {
      sum += raw;
      validSamples++;
    }
    delay(10);
  }
  
  if (validSamples == 0) return prevTemp;
  
  // Calculate and smooth temperature
  float avgVoltage = (sum / (float)validSamples) * (VREF / 1023.0);
  float currentTemp = avgVoltage * 100.0;
  float smoothedTemp = (prevTemp * 0.8) + (currentTemp * 0.2) + tempCalibrationOffset;
  
  // Limit rate of change
  if (abs(smoothedTemp - prevTemp) > maxChange) {
    smoothedTemp = (smoothedTemp > prevTemp) ? 
                  prevTemp + maxChange : 
                  prevTemp - maxChange;
  }
  
  prevTemp = smoothedTemp;
  return smoothedTemp;
}

// ====================== Safety Checking ======================
bool checkSafety(float vR, float vY, float vB, float t, String &reason) {
  bool ok = true;
  reason = "";

  if (vR < MIN_VOLTAGE || vR > MAX_VOLTAGE) reason += "R Volt Err, ", ok = false;
  if (vY < MIN_VOLTAGE || vY > MAX_VOLTAGE) reason += "Y Volt Err, ", ok = false;
  if (vB < MIN_VOLTAGE || vB > MAX_VOLTAGE) reason += "B Volt Err, ", ok = false;
  if (t < MIN_TEMP || t > MAX_TEMP) reason += "Temp Err, ", ok = false;

  if (reason.endsWith(", ")) reason.remove(reason.length() - 2);
  return ok;
}

// ====================== Display Functions ======================
void updateDisplay(float vR, float vY, float vB, float iR, float iY, float iB, float t, bool motorOn, String fault, bool usePot) {
  if (fault.length() > 0 && !faultFlashShown) {
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print("FAULT DETECTED!");
    lcd.setCursor(0, 1); lcd.print(fault.substring(0, 20));
    lcd.setCursor(0, 2); lcd.print("Motor: OFF");
    lcd.setCursor(0, 3); lcd.print("FAULT: "); lcd.print(fault.substring(0, 13));
    if (millis() - faultDisplayStart > 2000) faultFlashShown = true;
    return;
  }

  lcd.clear();
  // Line 1: Voltages
  lcd.setCursor(0, 0);
  lcd.print("VR:"); lcd.print(vR, 0);
  lcd.print(" VY:"); lcd.print(vY, 0);
  lcd.print(" VB:"); lcd.print(vB, 0);

  // Line 2: Currents (mA with 1 decimal)
  lcd.setCursor(0, 1);
  lcd.print("IR:"); lcd.print(iR * 1000, 1); 
  lcd.print(" IY:"); lcd.print(iY * 1000, 1); 
  
  // Line 3: Current and Temp
  lcd.setCursor(0, 2);
  lcd.print("IB:"); lcd.print(iB * 1000, 1); 
  lcd.print(" T: "); lcd.print(t, 1); lcd.print((char)223); lcd.print("C");

  // Line 4: Status
  lcd.setCursor(0, 3);
  if (faultActive) {
    lcd.print("FAULT: "); 
    lcd.print(lastFaultReason.substring(0, 13));
  } else {
    lcd.print("Motor:"); lcd.print(motorOn ? "ON " : "OFF");
    lcd.print(" Mode:"); lcd.print(usePot ? "POT" : "SENS");
  }
}

// ====================== Manual Button Control ======================
void checkManualButton(float vR, float vY, float vB, float temp) {
  bool reading = digitalRead(BUTTON_PIN);
  if (lastButtonState == HIGH && reading == LOW) {
    String reason = "";
    bool safe = checkSafety(vR, vY, vB, temp, reason);
    bool currentMotorState = !digitalRead(RELAY_PIN);
    
    if (!currentMotorState) { // Motor is OFF
      if (safe) {
        digitalWrite(RELAY_PIN, LOW); // Turn motor ON
        manualControlState = true;
        remoteControlEnabled = false;
        for (int i = 0; i < totalRegistered; i++) {
          sendSMS(registeredNumbers[i], "Motor turned ON manually.");
        }
      } else {
        for (int i = 0; i < totalRegistered; i++) {
          sendSMS(registeredNumbers[i], "Manual ON failed.\n" + reason);
        }
      }
    } else { // Motor is ON
      digitalWrite(RELAY_PIN, HIGH); // Turn motor OFF
      manualControlState = false;
      remoteControlEnabled = false;
      for (int i = 0; i < totalRegistered; i++) {
        sendSMS(registeredNumbers[i], "Motor turned OFF manually.");
      }
    }
    delay(300); // Debounce delay
  }
  lastButtonState = reading;
}

// ====================== GSM/SMS Functions ======================
void checkIncomingSMS() {
  while (gsmSerial.available()) {
    char c = gsmSerial.read();
    smsBuffer += c;
    if (smsBuffer.indexOf("+CMT:") != -1) newSms = true;
  }
}

void processSMSCommand() {
  int numStart = smsBuffer.indexOf("+CMT: \"") + 7;
  int numEnd = smsBuffer.indexOf("\"", numStart);
  String sender = smsBuffer.substring(numStart, numEnd);
  if (!isRegisteredNumber(sender)) { 
    sendSMS(sender, "Access Denied."); 
    return; 
  }

  bool usePot = digitalRead(MODE_SELECT_PIN);
  float vR, vY, vB, iR, iY, iB, temp;
  
  if (usePot) {
    // Potentiometer mode readings
    vR = analogRead(VOLTAGE_R) * POT_VOLTAGE_SCALE;
    vY = analogRead(VOLTAGE_Y) * POT_VOLTAGE_SCALE;
    vB = analogRead(VOLTAGE_B) * POT_VOLTAGE_SCALE;
    iR = analogRead(CURRENT_R) * POT_CURRENT_SCALE;
    iY = analogRead(CURRENT_Y) * POT_CURRENT_SCALE;
    iB = analogRead(CURRENT_B) * POT_CURRENT_SCALE;
    temp = analogRead(TEMP_POT) * POT_TEMP_SCALE;
  } else {
    // Sensor mode readings
    vR = readVoltageSensor(VOLTAGE_R);
    vY = readVoltageSensor(VOLTAGE_Y);
    vB = readVoltageSensor(VOLTAGE_B);
    iR = readCurrentSensor(CURRENT_R);
    iY = readCurrentSensor(CURRENT_Y);
    iB = readCurrentSensor(CURRENT_B);
    temp = readStableLM35Temperature(TEMP_POT);
  }

  String dummy;
  bool safeNow = checkSafety(vR, vY, vB, temp, dummy);

  String command = smsBuffer;
  command.toLowerCase();

  if (command.indexOf("motoron") != -1) {
    remoteControlEnabled = true;
    manualControlState = false;
    if (safeNow) {
      digitalWrite(RELAY_PIN, LOW); // Turn motor ON
      sendSMS(sender, "Motor ON command received.\nMotor is turned ON.");
    } else {
      digitalWrite(RELAY_PIN, HIGH); // Keep motor OFF
      sendSMS(sender, "Motor ON command received.\nCannot turn ON: Fault detected!");
    }
  } else if (command.indexOf("motoroff") != -1) {
    remoteControlEnabled = false;
    manualControlState = false;
    digitalWrite(RELAY_PIN, HIGH); // Turn motor OFF
    sendSMS(sender, "Motor OFF command received.\nMotor turned OFF.");
  } else if (command.indexOf("status") != -1) {
    String status = "";
    status += "Mode: " + String(usePot ? "POT" : "SENS") + "\n";
    status += "R:" + String(vR, 1) + "V Y:" + String(vY, 1) + "V B:" + String(vB, 1) + "V\n";
    status += "IR:" + String(iR * 1000, 1) + "mA IY:" + String(iY * 1000, 1) + "mA IB:" + String(iB * 1000, 1) + "mA\n";
    status += "T:" + String(temp, 1) + "C\n";
    status += "Motor: " + String(digitalRead(RELAY_PIN) == LOW ? "ON" : "OFF");
    sendSMS(sender, status);
  }
}

bool isRegisteredNumber(String num) {
  for (int i = 0; i < totalRegistered; i++) {
    if (num == registeredNumbers[i]) return true;
  }
  return false;
}

void sendSMS(String number, String message) {
  gsmSerial.print("AT+CMGS=\""); gsmSerial.print(number); gsmSerial.println("\"");
  delay(500);
  gsmSerial.print(message);
  gsmSerial.write(26); // Ctrl+Z
  delay(1000);
}

void sendFaultAlert(float vR, float vY, float vB, float iR, float iY, float iB, float temp, String reason, bool usePot) {
  String alert = "Motor STOPPED due to:\n" + reason;
  alert += "\nR:" + String(vR, 0) + "V Y:" + String(vY, 0) + "V B:" + String(vB, 0) + "V";
  alert += "\nIR:" + String(iR * 1000, 1) + "mA IY:" + String(iY * 1000, 1) + "mA IB:" + String(iB * 1000, 1) + "mA";
  alert += "\nT:" + String(temp, 1) + "C";
  alert += "\nMode: " + String(usePot ? "POT" : "SENS");

  for (int i = 0; i < totalRegistered; i++) {
    sendSMS(registeredNumbers[i], alert);
  }
}

void sendAllClearAlert() {
  for (int i = 0; i < totalRegistered; i++) {
    sendSMS(registeredNumbers[i], "All parameters normal.\nMotor ready to turn ON.");
  }
}

void initializeGSM() {
  delay(1000);
  gsmSerial.println("AT"); waitForResponse("OK", 1000);
  gsmSerial.println("AT+CMGF=1"); waitForResponse("OK", 1000);
  gsmSerial.println("AT+CNMI=2,2,0,0,0"); waitForResponse("OK", 1000);
  gsmSerial.println("AT+CMGDA=\"DEL ALL\""); waitForResponse("OK", 2000);
}

bool waitForResponse(String expected, unsigned long timeout) {
  unsigned long start = millis();
  String response = "";
  while (millis() - start < timeout) {
    while (gsmSerial.available()) {
      char c = gsmSerial.read();
      response += c;
      if (response.indexOf(expected) != -1) return true;
    }
  }
  return false;
}

// ====================== Main Loop ======================
void loop() {
  bool usePot = digitalRead(MODE_SELECT_PIN);

  // Read values based on mode
  float vR, vY, vB, iR, iY, iB, temp;
  
  if (usePot) {
    // Potentiometer mode - direct readings
    vR = analogRead(VOLTAGE_R) * POT_VOLTAGE_SCALE;
    vY = analogRead(VOLTAGE_Y) * POT_VOLTAGE_SCALE;
    vB = analogRead(VOLTAGE_B) * POT_VOLTAGE_SCALE;
    iR = analogRead(CURRENT_R) * POT_CURRENT_SCALE;
    iY = analogRead(CURRENT_Y) * POT_CURRENT_SCALE;
    iB = analogRead(CURRENT_B) * POT_CURRENT_SCALE;
    temp = analogRead(TEMP_POT) * POT_TEMP_SCALE;
  } else {
    // Sensor mode - accurate measurements
    vR = readVoltageSensor(VOLTAGE_R);
    vY = readVoltageSensor(VOLTAGE_Y);
    vB = readVoltageSensor(VOLTAGE_B);
    iR = readCurrentSensor(CURRENT_R);
    iY = readCurrentSensor(CURRENT_Y);
    iB = readCurrentSensor(CURRENT_B);
    temp = readStableLM35Temperature(TEMP_POT);
  }

  // Safety checks
  String faultReason = "";
  bool safe = checkSafety(vR, vY, vB, temp, faultReason);

  // Handle incoming SMS
  checkIncomingSMS();
  if (newSms) {
    processSMSCommand();
    newSms = false;
    smsBuffer = "";
  }

  // Motor control logic
  motorShouldRun = (manualControlState || remoteControlEnabled) && safe;
  digitalWrite(RELAY_PIN, motorShouldRun ? LOW : HIGH);

  // Fault handling
  if (!safe && !faultActive) {
    sendFaultAlert(vR, vY, vB, iR, iY, iB, temp, faultReason, usePot);
    faultActive = true;
    lastFaultReason = faultReason;
    faultDisplayStart = millis();
    faultFlashShown = false;
  }

  if (safe && faultActive) {
    sendAllClearAlert();
    faultActive = false;
    lastFaultReason = "";
    faultDisplayStart = 0;
    faultFlashShown = false;
  }

  // Update system state
  lastMotorState = motorShouldRun;
  updateDisplay(vR, vY, vB, iR, iY, iB, temp, motorShouldRun, faultActive ? lastFaultReason : "", usePot);
  checkManualButton(vR, vY, vB, temp);

  delay(500); // Main loop delay
}