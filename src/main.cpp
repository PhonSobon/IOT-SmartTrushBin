#include <WiFi.h>
#include <HTTPClient.h>
#include <NewPing.h>
#include <ESP32Servo.h>
#include <TinyGPS++.h>
#include <PubSubClient.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <UniversalTelegramBot.h>
#include <FirebaseESP32.h>

WiFiClient espClient;
PubSubClient mqttClient(espClient);
TinyGPSPlus gps;
HardwareSerial SerialGPS(2);
Servo myServo;
WebServer server(80);
HTTPClient http;
FirebaseData firebaseData;
FirebaseConfig firebaseConfig;
FirebaseAuth firebaseAuth;

#define FIREBASE_HOST "https://training-smart-trash-bin-default-rtdb.asia-southeast1.firebasedatabase.app"
#define FIREBASE_AUTH "joAFigWH6tHGNgJ3MHS7fxTgKVqrxOAHReNusYJc"

const char* telegramBotToken = "7403627307:AAFB8MSZ_Q5lAVCMaERoEbXxH6AgZZViIXs";
const char* chatId = "-4246864928";
WiFiClientSecure client;
UniversalTelegramBot bot(telegramBotToken, client);

#define TRIG_PIN_FILL 14
#define ECHO_PIN_FILL 27
#define TRIG_PIN_DIST 25
#define ECHO_PIN_DIST 26
#define GPS_RX_PIN 16
#define GPS_TX_PIN 17
#define SERVO_PIN 13

#define DISTANCE_THRESHOLD 10  // cm for opening the lid
#define BIN_DEPTH 30  // cm, adjust according to your bin depth

unsigned long lidOpenTime = 0;  // To track when the lid was opened
const unsigned long lidDuration = 5000;  // Duration to keep the lid open (in milliseconds)
bool lidOpen = false;
int distance = 0;
int level = 0;
float lastFillPercentage = -1;  // To track the last sent fill percentage

// Ultrasonic Sensor Objects
NewPing levelSensor(TRIG_PIN_FILL, ECHO_PIN_FILL, 200);
NewPing distanceSensor(TRIG_PIN_DIST, ECHO_PIN_DIST, 200);

class UltrasonicController {
public:
    void setup() {
        myServo.attach(SERVO_PIN);
    }

    void update() {
        readLevelSensor();
        readDistanceSensor();
    }

private:
    void readLevelSensor() {
        int newLevel = levelSensor.ping_cm();
        if (newLevel > 0) level = newLevel;
        if (level < 0) level = 0;
        if (level > BIN_DEPTH) level = BIN_DEPTH;
    }

    void readDistanceSensor() {
        int newDistance = distanceSensor.ping_cm();
        if (newDistance > 0) distance = newDistance;
        else if (newDistance <= 0) distance = 200;

        if (distance > 0 && distance < DISTANCE_THRESHOLD) {
            openLid();
        }
    }

public:


  int getLevel() {
        return level;
    }

    int getDistance() {
        return distance;
    }

    void openLid() {
        myServo.write(90);
        if(!lidOpen){
            lidOpenTime = millis();
        }
        lidOpen = true;
        
    }

    void closeLid() {
        myServo.write(0);
        lidOpen = false;
        
    }
};

class GpsController {
public:
    void setup() {
        SerialGPS.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    }

    void readGps() {
        while (SerialGPS.available() > 0) {
            gps.encode(SerialGPS.read());
        }
    }

    float getLatitude() {
        return gps.location.lat();
    }

    float getLongitude() {
        return gps.location.lng();
    }
};

class FirebaseController{
public:
    void setupFirebase(){
        firebaseConfig.host = FIREBASE_HOST;
        firebaseConfig.signer.tokens.legacy_token = FIREBASE_AUTH;
        Firebase.begin(&firebaseConfig, &firebaseAuth);
        Firebase.reconnectWiFi(true);
    }
    void sendToFirebase(float fillPercentage, float lat, float lng) {
        String path = "/trash-bin-database/bin-1";  // Adjust the path as necessary
        if(lat == 0 || lng == 0){
            lat = 11.5703844;
            lng = 104.8939048;
        }
        FirebaseJson json;
        json.set("fill_percentage", fillPercentage);
        json.set("latitude", lat);
        json.set("longitude", lng);

        if (Firebase.updateNode(firebaseData, path, json)) {
        Serial.println("Data sent to Firebase successfully");
        } else {
            Serial.println("Failed to send data to Firebase");
        }
    }
};



class TrashBinServer {
public:
    void setup() {
        server.on("/data", HTTP_GET, [this]() { handleData(); });
        server.on("/openLid", HTTP_POST, [this]() { handleOpenLid(); });
        server.on("/closeLid", HTTP_POST, [this]() { handleCloseLid(); });
        server.on("/fetchGPS", HTTP_POST, [this]() { handleFetchGPS(); });
        server.onNotFound([this]() { handleNotFound(); });
        server.begin();
    }

    void handleClient() {
        server.handleClient();
    }

    void sendPeriodicData() {
        unsigned long currentTime = millis();
        float currentFillPercentage = calculateFillPercentage();

        if (currentTime - lastSentTime >= sendInterval || abs(currentFillPercentage - lastFillPercentage) >= 10) {
            DynamicJsonDocument doc(200);
            doc["fill_percentage"] = currentFillPercentage;
            doc["lid_open"] = lidOpen ? "true" : "false";
            doc["latitude"] = gps.location.lat();
            doc["longitude"] = gps.location.lng();
            String response;
            serializeJson(doc, response);
            server.sendHeader("Access-Control-Allow-Origin", "*");
            server.send(200, "application/json", response);
            lastSentTime = currentTime;
            lastFillPercentage = currentFillPercentage;
            Serial.println("Sent periodic data update");

            firebaseController.sendToFirebase(currentFillPercentage, gps.location.lat(), gps.location.lng());
        }
    }

private:
    UltrasonicController ultrasonicController;
    FirebaseController firebaseController;
    unsigned long lastSentTime = 0;
    const unsigned long sendInterval = 5000;  // Send data every 5 seconds

    void handleData() {
        DynamicJsonDocument doc(200);
        doc["fill_percentage"] = calculateFillPercentage();
        doc["lid_open"] = lidOpen ? "true" : "false";
        doc["latitude"] = gps.location.lat();
        doc["longitude"] = gps.location.lng();
        String response;
        serializeJson(doc, response);
        server.sendHeader("Access-Control-Allow-Origin", "*");
        server.send(200, "application/json", response);
        Serial.println("Received request for /data");
    }

    void handleOpenLid() {
        ultrasonicController.openLid();
        server.sendHeader("Access-Control-Allow-Origin", "*");
        server.send(200, "text/plain", "Lid opened");
        Serial.println("Received request to open lid");
    }

    void handleCloseLid() {
        ultrasonicController.closeLid();
        server.sendHeader("Access-Control-Allow-Origin", "*");
        server.send(200, "text/plain", "Lid closed");
        Serial.println("Received request to close lid");
    }

    void handleFetchGPS() {
        DynamicJsonDocument doc(200);
        doc["latitude"] = gps.location.lat();
        doc["longitude"] = gps.location.lng();
        String response;
        serializeJson(doc, response);
        server.sendHeader("Access-Control-Allow-Origin", "*");
        server.send(200, "application/json", response);
        Serial.println("Received request to fetch GPS");
    }

    void handleNotFound() {
        server.sendHeader("Access-Control-Allow-Origin", "*");
        server.send(404, "text/plain", "Not found");
    }

    int calculateFillPercentage() {
        return 100 - (level * 100 / BIN_DEPTH);
    }
};


class TelegramController {
public:
    void checkMessage(){
        int numNewMessages = bot.getUpdates(bot.last_message_received + 1);
        if(numNewMessages > 0){
            receiveMessage(numNewMessages);
        }
    }
    void sendMessages(String localIP){
        
    }
private:
    void receiveMessage(int numNewMessages){
        for(int i = 0; i < numNewMessages; i++){
            String text = bot.messages[i].text;
            Serial.println("Message: " + text);
            if(text == "open"){
                ultrasonicController.openLid();
            }else if(text == "close"){
                ultrasonicController.closeLid();
            }
        }
    }
private:
    UltrasonicController ultrasonicController;
};

class WiFiController {
public:
    void setup() {
        for (int i = 0; i < sizeof(wifiSSID) / sizeof(wifiSSID[0]); ++i) {
            if (connectToWiFi(wifiSSID[i], wifiPassword[i])) {
                if (WiFi.status() == WL_CONNECTED) {
                    Serial.print("IP Address: ");
                    Serial.println(WiFi.localIP());
                    String localIP = WiFi.localIP().toString();
                    telegramController.sendMessages(localIP);
                }
                break;
            }
        }
    }

private:
    TelegramController telegramController;
    static constexpr int maxAttempt = 5;
    const char* wifiSSID[5] = {"Leyy Leyy", "SMART-WIFI-F215", "Phanna", "Nova Coffee F1", "Norak3"};
    const char* wifiPassword[5] = {"Leyy0824", "66C90FC7", "qwerty123", "Nova1234", "Norak@2022"};

    bool connectToWiFi(const char* ssid, const char* password) {
        WiFi.begin(ssid, password);
        unsigned long startAttemptTime = millis();
        while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 5000) { // 5 second timeout
            delay(100);
        }
        return WiFi.status() == WL_CONNECTED;
    }
};

class Arduino {
public:
    void setup() {
        Serial.begin(9600);
        wifiController.setup();
        gpsController.setup();
        ultrasonicController.setup();
        trashBinServer.setup();
        client.setInsecure();
        
        firebaseController.setupFirebase();
    }

    void loop() {
        if (WiFi.status() != WL_CONNECTED) {
            wifiController.setup();
        }
        mqttClient.loop();
        ultrasonicController.update();
        gpsController.readGps();
        trashBinServer.handleClient();
        trashBinServer.sendPeriodicData();
        telegramController.checkMessage();
        if (lidOpen && millis() - lidOpenTime >= lidDuration){
            ultrasonicController.closeLid();
        }
        delay(500); // Adjust the delay if needed
    }

private:
    WiFiController wifiController;
    UltrasonicController ultrasonicController;
    GpsController gpsController;
    TrashBinServer trashBinServer;
    TelegramController telegramController;
    FirebaseController firebaseController;
};

Arduino arduino;

void setup() {
    arduino.setup();
}

void loop() {
    arduino.loop();
}
