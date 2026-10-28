
//////////////////////////////////////
//    ESP32-C3 LD2410C Security Light
//    Version:  1.0
//    October 2026
//////////////////////////////////////


#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <time.h>
#include <math.h>
#include <OneWire.h>
#include <DallasTemperature.h>


// --- Configuration ---
const char* ssid     = "XXXXXXXX";
const char* password = "XXXXXXXX";
const char* hostName = "esp32-C3_ld2410C_SecurityLight_V1";
bool cpu = false;
// --- Syslog Configuration ---
const char* SYSLOG_HOST = "192.168.XXX.XXX";
const uint16_t SYSLOG_PORT = 514;
WiFiUDP syslogUdp;

int dawn = 0;
int dusk = 0;
int timeInMinutes = 0;
struct tm timeinfo;
#define MIDNIGHT 1440
bool systemOnline = false;



// --- Warwick, UK Location Data ---
const float WARWICK_LAT = 52.2815;  
const float WARWICK_LON = -1.5816;  
#define TZ_INFO "GMT0BST,M3.5.0/1,M10.5.0/2"




// --- Pin Definitions ---
const uint8_t RADAR_PIN     =    1;    //  ESP32-C3                           // 4;      // GPIO4 (D2 on NodeMCU) - Interrupt pin
const uint8_t LED_PIN       =    3;    //  ESP32-C3                           // 5;      // GPIO5 (D1 on NodeMCU) - Indicator LED
const uint8_t MOSFET_PIN    =    0;    //  ESP32-C3                           // 13;     // GPIO13 (D7 on NodeMCU) - MOSFET Driver
const uint8_t ONE_WIRE_PIN  =   10; 
const uint8_t RX_PIN    =       20; 
const uint8_t TX_PIN    =       21; 
const uint8_t LED =      8;
const uint8_t MOSFET_LED  =      5;

/// --- DALLAS 18B20 -------
OneWire oneWire(ONE_WIRE_PIN);
DallasTemperature temperatureSensor(&oneWire);
float temperature = 0.0;


// --- Timing Variables ---
const uint32_t DELAY_INTERVAL     = 3 * 60 * 1000;    // 3 minutes output hold
const uint32_t STATUS_INTERVAL    = 10  * 60 * 1000;  // 10 minutes status print
const uint32_t CHECK_WIFI_INTERVAL = 30 * 1000;       // Check WiFi every 30 seconds
const uint32_t DEBOUNCE_THRESHOLD = 200;              // Target must be present for 200ms
const uint32_t LED_HOLD_INTERVAL  = 1500;             // Stabilise LED by keeping it on for 1.5s after clear

volatile uint32_t firstDetectionTime = 0;
volatile uint32_t lastDetectionTime = 0;
volatile bool pinHighState = false;
bool motionDetected = false;
uint32_t lastStatusTime = 0;
uint32_t lastWiFiCheckTime = 0;
unsigned long doCPU = 500;

// --- ISR (Interrupt Service Routine) ---
void IRAM_ATTR handleRadarInterrupt() {
  if (digitalRead(RADAR_PIN) == HIGH) {
    if (!pinHighState) {
      pinHighState = true;
      firstDetectionTime = millis();
    }
  } else {
    pinHighState = false;
  }
}

// --- Helper: Core Syslog Transmission Engine ---
void logToSyslog(uint8_t severity, const String& message) 
{
  String severityLevel[9] = {"EMERGENCY","ALERT","CRITICAL","ERROR","WARNING","NOTIFICATION","INFO","DEBUG","TELEMETRY" };
  if (WiFi.status() != WL_CONNECTED) {
    return;
  }
  uint8_t pri = severity; 
  syslogUdp.beginPacket(SYSLOG_HOST, SYSLOG_PORT);
  syslogUdp.printf("[ %s ] %s: %s", severityLevel[pri], hostName, message.c_str());
  syslogUdp.endPacket();
}

// --- Dynamic Offline Sunrise/Sunset Calculator Engine ---
int calculateSolarMinutes(int year, int month, int day, bool isSunset) {


/**
 * Calculates Sunrise or Sunset time in minutes from midnight for Warwick, UK.
 * Includes automatic Daylight Saving Time (BST) adjustment.
 * 
 * @param year        The calendar year (e.g. 2026)
 * @param month       The month (1 to 12)
 * @param day         The day of the month (1 to 31)
 * @param isSunset    Set to true for SUNSET, false for SUNRISE
 * @return            Time in minutes from midnight (0 = midnight)
 */

    // 1. Hardcoded coordinates for Warwick, Warwickshire, England
    const double latitude = 52.2835; 
    const double longitude = -1.5892; // West is negative

    // 2. Standard Zeniths for Sunrise/Sunset calculations
    const double zenith = 90.83333333333333; 

    // 3. Calculate Day of the Year
    int N1 = floor(275 * month / 9);
    int N2 = floor((month + 9) / 12);
    int N3 = (1 + floor((year - 4 * floor(year / 4) + 2) / 3));
    int N = N1 - (N2 * N3) + day - 30;

    // 4. Convert longitude to hour value and estimate an approximate time
    double lngHour = longitude / 15.0;
    double t = N + ((isSunset ? 18.0 : 6.0) - lngHour) / 24.0;

    // 5. Calculate Sun's Mean Anomaly (M)
    double M = (0.9856 * t) - 3.289;

    // 6. Calculate Sun's True Longitude (L)
    double L = M + (1.916 * sin(M * DEG_TO_RAD)) + (0.020 * sin(2 * M * DEG_TO_RAD)) + 282.634;
    L = fmod(L + 360.0, 360.0);

    // 7. Calculate Sun's Right Ascension (RA)
    double RA = atan(0.91764 * tan(L * DEG_TO_RAD)) * RAD_TO_DEG;
    RA = fmod(RA + 360.0, 360.0);

    // Adjust Right Ascension quadrant
    double Lquadrant  = (floor(L / 90.0)) * 90.0;
    double RAquadrant = (floor(RA / 90.0)) * 90.0;
    RA = RA + (Lquadrant - RAquadrant);
    RA = RA / 15.0; // Convert to hours

    // 8. Calculate Sun's Declination (sinDec and cosDec)
    double sinDec = 0.39782 * sin(L * DEG_TO_RAD);
    double cosDec = cos(asin(sinDec));

    // 9. Calculate Local Hour Angle (H)
    double cosH = (cos(zenith * DEG_TO_RAD) - (sinDec * sin(latitude * DEG_TO_RAD))) / (cosDec * cos(latitude * DEG_TO_RAD));
    
    // Return early if there is no sunrise/sunset (polar regions - rare but good for safety)
    if (cosH > 1) return -1;  // Sun stays down
    if (cosH < -1) return -1; // Sun stays up

    // 10. Calculate local hour angle (H) in hours
    double H = isSunset ? acos(cosH) * RAD_TO_DEG : 360.0 - acos(cosH) * RAD_TO_DEG;
    H = H / 15.0;

    // 11. Calculate local Mean Time of event (T) in UTC hours
    double T = H + RA - (0.06571 * t) - 6.622;
    double UT = T - lngHour;
    UT = fmod(UT + 24.0, 24.0); // Final UTC Time in decimal hours

    // Convert to absolute UTC minutes from midnight
    int minutesFromMidnightUTC = round(UT * 60.0);

    // 12. Calculate British Summer Time (DST) rules dynamically 
    // UK DST begins last Sunday of March, ends last Sunday of October
    bool isDST = false;
    
    if (month > 3 && month < 10) {
        isDST = true;
    } else if (month == 3) {
        // Last Sunday in March helper calculation
        int lastSundayMarch = 31 - ((5 + (int)(year * 5 / 4)) % 7);
        if (day >= lastSundayMarch) isDST = true;
    } else if (month == 10) {
        // Last Sunday in October helper calculation
        int lastSundayOctober = 31 - ((2 + (int)(year * 5 / 4)) % 7);
        if (day < lastSundayOctober) isDST = true;
    }

    // Apply standard UK offset: GMT = UTC+0, BST = UTC+1 hour (+60 minutes)
    int localMinutesFromMidnight = minutesFromMidnightUTC + (isDST ? 60 : 0);
    
    // Ensure the edge calculation handles rollover wraps beautifully
    return fmod(localMinutesFromMidnight + 1440, 1440);
}
// --- Helper: Verify Nighttime Window ---

void flashLed(){
cpu = !cpu;
digitalWrite(LED,cpu);
  
}

bool isNightTime() {
  
  
  dawn = calculateSolarMinutes(timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday, false);
  dusk  = calculateSolarMinutes(timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday, true);
  localTime();
   return (timeInMinutes >= dusk && timeInMinutes <= MIDNIGHT);
}

// --- Helper Function: Print System Status ---
void printSystemStatus() {
  
  uint32_t currentMillis = millis();
  uint32_t totalSeconds = currentMillis / 1000;
  uint32_t seconds = totalSeconds % 60;
  uint32_t totalMinutes = totalSeconds / 60;
  uint32_t minutes = totalMinutes % 60;
  uint32_t totalHours = totalMinutes / 60;
  uint32_t hours = totalHours % 24;
  uint32_t days = totalHours / 24;

  char timeString[64];
  strftime(timeString, sizeof(timeString), "%Y-%m-%d %H:%M:%S %Z", &timeinfo);

  Serial.println("\n=== Periodic Status Report ===");
  Serial.printf("Current Local Time: %s\n", timeString);
  Serial.printf("Night Mode Gate Active: %s\n", isNightTime() ? "YES" : "NO");
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("IP: %s\n", WiFi.localIP().toString().c_str());
    Serial.printf("BSSID (Router MAC): %s\n", WiFi.BSSIDstr().c_str());
    Serial.printf("RSSI (Signal Strength): %d dBm\n", WiFi.RSSI());
    Serial.printf("SSID: %s \n", ssid);
  } else {
    Serial.println("WiFi Status: DISCONNECTED");
  }
  Serial.printf("System Uptime: %ud %uh %um %us\n", days, hours, minutes, seconds);
  Serial.println("==============================");

  String logPayload = "UPTIME: " + String(days) + "d " + String(hours) + "h " + String(minutes) + "m " + String(seconds) + "s";
  logPayload += " | IP: " + WiFi.localIP().toString();
  logPayload += " | BSSID: " + WiFi.BSSIDstr();
  logPayload += " | RSSI: " + String(WiFi.RSSI()) + "dBm";
  logPayload += " | TEMPERATURE: " + String(temperature) + "°C";
  logPayload += " | NIGHTMODE: " + String(isNightTime() ? "Active" : "Inactive");
  logPayload += " | TIME IN MINUTES:" + String(timeInMinutes);
  logPayload += " | SUNSET:" + String(dusk);
  logPayload += " | SUNRISE:" + String(dawn);
  logPayload += " | DAYLIGHT SAVINGS: " + String(timeinfo.tm_isdst ? "Active" : "Inactive");
  
  logToSyslog(8, logPayload); 
}




void setup() {
  Serial.begin(115200);
  delay(500);
  
  Serial.println("\nStarting ESP32-C3 Radar Controller...");

  pinMode(RADAR_PIN, INPUT); 
  pinMode(LED_PIN, OUTPUT);
  pinMode(MOSFET_PIN, OUTPUT);
  pinMode(LED, OUTPUT);
  
  digitalWrite(LED_PIN, LOW);
  digitalWrite(MOSFET_PIN, LOW);
  digitalWrite(LED, HIGH);

  
  
  initWiFi();

  ArduinoOTA.setHostname(hostName);
  ArduinoOTA.begin();
  
  
  temperatureSensor.begin();
  dallas();
  
  attachInterrupt(digitalPinToInterrupt(RADAR_PIN), handleRadarInterrupt, CHANGE);
  ledTest(20);

  Serial.println("\nESP32-C3 Radar Controller Ready");
  isNightTime();
  printSystemStatus();
  lastStatusTime = millis();
  lastWiFiCheckTime = millis();
  doCPU = millis();
  
}

void loop() {
  ArduinoOTA.handle();
  localTime();  
  bool currentPinState = (digitalRead(RADAR_PIN) == HIGH);

if( (millis() - doCPU ) > 1000 )
{
  flashLed();
  doCPU = millis();
}

  // --- Debounce & Filtering Logic ---
  if (currentPinState) {
    if (!pinHighState) {
      pinHighState = true;
      firstDetectionTime = millis();
    }
    
    if (fmod(millis() - firstDetectionTime, 4294967295.0) >= DEBOUNCE_THRESHOLD) {
      if (!motionDetected) {
        logToSyslog(1, "Presence detected inside zone. Activating LEDS");
      }
      motionDetected = true;
      lastDetectionTime = millis(); 
    }
  } else {
    pinHighState = false;
  }


  // --- Indicator LED Control with Off-Delay Filter ---
  if (motionDetected && (millis() - lastDetectionTime < LED_HOLD_INTERVAL || currentPinState)) {
    // Keep LED solidly active during tracking or during the brief 1.5-second smoothing buffer
    digitalWrite(LED_PIN, HIGH);
    //digitalWrite(LED, LOW);
  } else {
    // Turn off safely after target clears and the smoothing window expires
    digitalWrite(LED_PIN, LOW);
    //digitalWrite(LED, HIGH);
  }

  // --- MOSFET Load Control ---
  if (motionDetected) {
          
   
    if (isNightTime() && systemOnline ) {
      digitalWrite(MOSFET_PIN, HIGH);
    } 


    if(!systemOnline) {   // no wifi, no NTP or incorrect local Time
      digitalWrite(MOSFET_PIN, HIGH);
    }                


    // Turn off MOSFET strictly when the 3-minute timeout has completely elapsed
    if ((millis() - lastDetectionTime >= DELAY_INTERVAL) && !currentPinState) {
      digitalWrite(MOSFET_PIN, LOW);
      motionDetected = false; 
      Serial.println("Timeout reached. Sensor clear. MOSFET turned OFF.");
      logToSyslog(1, "Timeout interval reached, detection zone CLEAR. MOSFET turned OFF.");
    }
  }

  // --- Non-blocking WiFi Health Check ---
  if (millis() - lastWiFiCheckTime >= CHECK_WIFI_INTERVAL) {
        maintainWiFi();
    lastWiFiCheckTime = millis();
    
  }

  // --- Non-blocking 60 -Minute Status ---
  if (millis() - lastStatusTime >= STATUS_INTERVAL) {
    printSystemStatus();
    lastStatusTime = millis(); 
  }
}
float dallas()  {
   temperatureSensor.requestTemperatures();
    temperature = temperatureSensor.getTempCByIndex(0);
  return temperature;
}


void ledTest(int num )  {

for(int x=1; x < num; ++x)  {       
  
  digitalWrite(LED, HIGH);
  digitalWrite(LED_PIN, HIGH);
  

      delay(125);
      
  digitalWrite(LED, LOW);
  digitalWrite(LED_PIN, LOW);
 

  delay(125);
}
   digitalWrite(MOSFET_PIN, LOW); 
}

int localTime()
{

  if (!getLocalTime(&timeinfo)) {
    logToSyslog(3, "Failed to obtain time");
    return 0;
  }


  timeInMinutes = (timeinfo.tm_hour * 60 ) + timeinfo.tm_min;
  return timeInMinutes;

}
// --- Helper Function: Non-blocking WiFi Health Check ---
void maintainWiFi() {  
   if(WiFi.status() != WL_CONNECTED ){
        WiFi.reconnect();
   }
 
}

void initWiFi() {

WiFi.mode(WIFI_STA);
  
  WiFi.begin(ssid,password);
  
  Serial.print("Connecting to WiFi");
  
  uint32_t startAttemptTime = millis();
 
  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 45000) {
    delay(500);
    Serial.print(".");
  }
  
  if (WiFi.status() == WL_CONNECTED) {
      Serial.println("\nWiFi connected.");
     
     // NTP
    configTzTime( TZ_INFO, "uk.pool.ntp.org","pool.ntp.org","time.nist.gov" );
    Serial.println("Synchronising time with NTP server...");
    logToSyslog(6, "System online.");
    systemOnline = true;
  } else {
    Serial.println("\nWiFi connection timed out. Booting in offline fallback mode.");
    systemOnline = true;
  }


  
}
