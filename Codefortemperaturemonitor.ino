#include <WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>
#include <MySQL_Connection.h>
#include <MySQL_Cursor.h>

#define DHTPIN 4            // Pin where the DHT22 is connected
#define DHTTYPE DHT22       // Type of DHT sensor
#define THRESHOLD_TEMP 30   // Threshold temperature
#define THRESHOLD_HUMIDITY 60 // Threshold humidity
#define INTERVAL_READINGS 600000 // Interval between readings (10 minutes)
#define HEARTBEAT_INTERVAL 1800000 // Interval for checking heartbeat (30 minutes)

const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";

const char* mqtt_server = "MQTT_BROKER_IP";
const int mqtt_port = 1883;
const char* mqtt_user = "MQTT_USERNAME";
const char* mqtt_password = "MQTT_PASSWORD";
const char* mqtt_client_id = "esp32_client";
const char* mqtt_topic = "temperature_humidity";

DHT dht(DHTPIN, DHTTYPE);

WiFiClient espClient;
PubSubClient client(espClient);

IPAddress server_addr(MYSQL_SERVER_IP_ADDRESS); // IP of the MySQL server here
char user[] = "USERNAME";                        // MySQL username
char password[] = "PASSWORD";                    // MySQL password
char db[] = "DATABASE_NAME";                    // MySQL database name

SoftwareSerial SIM800(2, 3); // RX, TX

const int manualOverridePin = 5; // Pin for manual override switch

bool manualOverrideEnabled = false;

unsigned long previousReadingsMillis = 0;
unsigned long previousHeartbeatMillis = 0;

LiquidCrystal_I2C lcd(0x27, 16, 2);  // I2C address 0x27, 16 columns and 2 rows

void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) {
  // handle incoming messages
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect(mqtt_client_id, mqtt_user, mqtt_password)) {
      Serial.println("connected");
      client.publish(mqtt_topic, "connected");
      client.subscribe("inTopic");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(9600);
  lcd.init();                      // Initialize LCD
  lcd.backlight();                 // Turn on backlight
  lcd.setCursor(0, 0);
  lcd.print("Initializing...");
  delay(2000);
  lcd.clear();
  pinMode(manualOverridePin, INPUT_PULLUP); // Set pin for manual override switch as input with internal pull-up resistor
  dht.begin();
  SIM800.begin(9600);

  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
}

void loop() {
  unsigned long currentMillis = millis();

  // Check manual override switch state
  if (digitalRead(manualOverridePin) == LOW) {
    manualOverrideEnabled = true;
  } else {
    manualOverrideEnabled = false;
  }

  if (currentMillis - previousReadingsMillis >= INTERVAL_READINGS) {
    previousReadingsMillis = currentMillis;
    if (!client.connected()) {
      reconnect();
    }
    client.loop();

    float humidity = dht.readHumidity();
    float temperature = dht.readTemperature();

    if (isnan(humidity) || isnan(temperature)) {
      lcd.setCursor(0, 0);
      lcd.print("Sensor error!");
      lcd.setCursor(0, 1);
      lcd.print("Check connections.");
      Serial.println("Failed to read from DHT sensor!");
      delay(1000);
      return;
    }

    lcd.setCursor(0, 0);
    lcd.print("Temp: ");
    lcd.print(temperature);
    lcd.print(" C");
    lcd.setCursor(0, 1);
    lcd.print("Humidity: ");
    lcd.print(humidity);
    lcd.print("%");

    String msg = String(temperature) + "," + String(humidity);
    char char_msg[msg.length() + 1];
    msg.toCharArray(char_msg, msg.length() + 1);
    client.publish(mqtt_topic, char_msg);

    MySQL_Connection conn((Client *)&espClient);
    if (conn.connect(server_addr, 3306, user, password)) {
      Serial.println("Connected to MySQL server!");
      MySQL_Cursor *cur_mem = new MySQL_Cursor(&conn);
      cur_mem->execute("USE " + String(db));
      cur_mem->execute("INSERT INTO sensor_data (temperature, humidity) VALUES (" + String(temperature) + ", " + String(humidity) + ")");
      delete cur_mem;
      conn.close();
      Serial.println("Data logged into MySQL database!");
    } else {
      Serial.println("Connection to MySQL server failed!");
    }

    if (!manualOverrideEnabled && (temperature > THRESHOLD_TEMP || humidity > THRESHOLD_HUMIDITY)) {
      Serial.println("Temperature or humidity threshold exceeded. Sending message...");
      SIM800.println("AT+CMGF=1");
      delay(100);
      SIM800.println("AT+CMGS=\"+XXXXXXXXXXXX\"");
      delay(100);
      SIM800.print("Temperature: ");
      SIM800.print(temperature);
      SIM800.print("°C, Humidity: ");
      SIM800.print(humidity);
      SIM800.println("%");
      delay(100);
      SIM800.println((char)26);
      delay(1000);
      Serial.println("Message sent successfully!");

      lcd.setCursor(0, 0);
      lcd.print("Threshold exceeded!");
      lcd.setCursor(0, 1);
      lcd.print("Sending message...");
    }

    delay(100);
  }

  if (currentMillis - previousHeartbeatMillis >= HEARTBEAT_INTERVAL) {
    previousHeartbeatMillis = currentMillis;

    if (dht.readHumidity() == NAN || dht.readTemperature() == NAN) {
            lcd.setCursor(0, 0);
      lcd.print("Sensor error!");
      lcd.setCursor(0, 1);
      lcd.print("Check connections.");

      Serial.println("Sensor is not working!");

      Serial.println("Sending alert message...");
      SIM800.println("AT+CMGF=1");
      delay(100);
      SIM800.println("AT+CMGS=\"+XXXXXXXXXXXX\"");
      delay(100);
      SIM800.println("Sensor is not working!");
      delay(100);
      SIM800.println((char)26);
      delay(1000);
      Serial.println("Alert message sent successfully!");
    } else {
      lcd.setCursor(0, 0);
      lcd.print("Sensor working fine.");

      Serial.println("Sensor is working fine!");
    }
  }
}
 