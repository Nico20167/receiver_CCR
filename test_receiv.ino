#include <esp_now.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <SPIFFS.h>
#include <map>

#define MAX_CAN_MESSAGES 4

// Structure pour stocker un message CAN
typedef struct {
  uint32_t can_id;
  uint8_t data[8];
  uint8_t data_len;
} CAN_Message;

// Structure pour stocker un tableau de messages CAN
typedef struct {
  CAN_Message messages[MAX_CAN_MESSAGES];
} CAN_MessageBundle;

CAN_MessageBundle receivedBundle;

// MQTT Setup
const char* mqtt_broker = "192.168.86.52";  // Remplace par l'IP de ton broker
const char* mqtt_username = "esp1";
const char* mqtt_password = "esp1";
const int mqtt_port = 1883;

WiFiClient espClient;
PubSubClient mqttClient(espClient);

// Structure pour stocker les associations MAC-Topic
std::map<String, String> macTopicMap;

// Fonction pour lire le fichier JSON et remplir la map
void readJSONFile() {
  File file = SPIFFS.open("/mac_topic_mapping.json", FILE_READ);
  if (!file) {
    Serial.println("Failed to open file for reading");
    return;
  }

  StaticJsonDocument<1024> doc;
  DeserializationError error = deserializeJson(doc, file);
  if (error) {
    Serial.println("Failed to parse JSON file");
    file.close();
    return;
  }

  JsonArray mappings = doc["mappings"];
  for (JsonObject mapping : mappings) {
    const char* macAddress = mapping["MAC_Address"];
    const char* topic = mapping["Topic"];
    macTopicMap[macAddress] = topic;
  }

  file.close();
}

void OnDataRecv(const esp_now_recv_info_t* recv_info, const uint8_t* incomingData, int len) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           recv_info->src_addr[0], recv_info->src_addr[1], recv_info->src_addr[2],
           recv_info->src_addr[3], recv_info->src_addr[4], recv_info->src_addr[5]);

  Serial.print("Adresse MAC reçue: ");
  Serial.println(macStr);

  if (len != sizeof(CAN_Message)) {
    Serial.printf("Invalid data size: %d bytes (expected: %d bytes)\n", len, sizeof(CAN_Message));
    return;
  }

  CAN_Message message;
  memcpy(&message, incomingData, len);

  // Afficher les détails du message reçu
  Serial.printf("CAN ID: 0x%03X\n", message.can_id);
  Serial.print("Data: ");
  for (int i = 0; i < message.data_len; i++) {
    Serial.printf("%02X ", message.data[i]);
  }
  Serial.println();

  // Envoi des données via MQTT pour chaque message
  sendCANDataToMQTT(macStr, message);
}


// Fonction pour envoyer les données CAN au serveur MQTT
void sendCANDataToMQTT(const char* macAddress, CAN_Message message) {
  char payload[512];  // Taille du buffer JSON

  // Trouver le topic MQTT correspondant à l'adresse MAC
  String topicData = macTopicMap[macAddress];

  if (topicData.isEmpty()) {
    Serial.println("⚠️ MAC Address non reconnue, message ignoré.");
    return;
  }

  // Création des documents JSON pour les messages MQTT
  StaticJsonDocument<1024> docData;  // Contiendra toutes les données normales
  StaticJsonDocument<512> docError;  // Contiendra uniquement les erreurs

  JsonObject dataRoot = docData.to<JsonObject>();
  JsonObject errorRoot = docError.to<JsonObject>();

  bool has_errors = false;  // Indicateur de la présence d'erreurs

    // Vérification de la taille du message CAN
    if (message.data_len != 8) {
    Serial.println("❌ Invalid message size");
    errorRoot["error"] = "Invalid message size";
    has_errors = true;
    } else {
    switch (message.can_id) {
      case 0x171: {
        uint16_t raw_voltage = (message.data[1] << 8) | message.data[0];
        int16_t raw_current = (message.data[3] << 8) | message.data[2];
        float BMS_Pack_Voltage = raw_voltage * 0.0078125;
        float BMS_Pack_Current = raw_current * 0.03125;

        uint8_t byte4 = message.data[4];
        uint8_t byte5 = message.data[5];
        uint8_t byte6 = message.data[6];
        uint8_t byte7 = message.data[7];

        dataRoot["BMS_Pack_Voltage"] = BMS_Pack_Voltage;
        dataRoot["BMS_Pack_Current"] = BMS_Pack_Current;

        // Liste des erreurs
        const char *error_messages[] = {
          "BMS_ERR_Temp_Powerstage_1", "BMS_ERR_Temp_Powerstage_2", "BMS_ERR_Charge_Current",
          "BMS_ERR_Discharge_Current", "BMS_ERR_Pack_Voltage_Max", "BMS_ERR_Analog_Overvoltage",
          "BMS_ERR_Curr_Sensor_Offset", "BMS_ERR_EEPROM", "BMS_ERR_CM_CRC",
          "BMS_ERR_External_Enable", "BMS_ERR_CM_Alert", "BMS_ERR_CM_Fault",
          "BMS_ERR_Powerstage", "BMS_ERR_PreCharge", "BMS_ERR_Output_Voltage_High",
          "BMS_ERR_Pack_Voltage_Min", "BMS_ERR_Discharge_Voltage", "BMS_ERR_CM_Cell_Undervoltage",
          "BMS_ERR_CM_Cell_Overvoltage", "BMS_ERR_Analog_Overcurrent", "BMS_ERR_Overtemp_Charge",
          "BMS_ERR_Overtemp_Discharge", "BMS_ERR_Undertemp_Charge", "BMS_ERR_Undertemp_Discharge",
          "BMS_ERR_Curr_Flow_Passive_State", "BMS_ERR_CAN_Timeout"
        };

        uint32_t error_bits = (byte7 << 24) | (byte6 << 16) | (byte5 << 8) | byte4;

        for (int i = 0; i < 26; i++) {
          if (error_bits & (1 << i)) {
            Serial.printf("⚠️ Error detected: %s\n", error_messages[i]);
            errorRoot[error_messages[i]] = 1;
            has_errors = true;
          }
        }

        break;
      }

      case 0x172: {
        uint16_t bms_state = (message.data[1] << 8) | message.data[0];
        int8_t bms_soc = message.data[2];
        uint8_t bms_soh = message.data[3] * 0.5;
        uint16_t bms_remaining_capacity = (message.data[5] << 8) | message.data[4];
        uint16_t bms_full_charge_capacity = (message.data[7] << 8) | message.data[6];

        dataRoot["BMS_State"] = bms_state;
        dataRoot["BMS_SOC"] = bms_soc;
        dataRoot["BMS_SOH"] = bms_soh;
        dataRoot["Remaining_Capacity"] = bms_remaining_capacity;
        dataRoot["Full_Charge_Capacity"] = bms_full_charge_capacity;

        break;
      }

      case 0x176: {
        int16_t temp_powerstage_1 = (message.data[1] << 8) | message.data[0];
        int16_t temp_powerstage_2 = (message.data[3] << 8) | message.data[2];
        int16_t temp_mcu = (message.data[5] << 8) | message.data[4];
        int8_t temp_cell_1 = message.data[6];
        int8_t temp_cell_2 = message.data[7];

        dataRoot["Temp_Powerstage_1"] = temp_powerstage_1;
        dataRoot["Temp_Powerstage_2"] = temp_powerstage_2;
        dataRoot["Temp_MCU"] = temp_mcu;
        dataRoot["Temp_Cell_1"] = temp_cell_1;
        dataRoot["Temp_Cell_2"] = temp_cell_2;

        break;
      }

      case 0x17B: {
        uint32_t BMS_Serial_Number = (message.data[3] << 24) | (message.data[2] << 16) | (message.data[1] << 8) | message.data[0];
        uint16_t BMS_Num_Cycles = (message.data[5] << 8) | message.data[4];

        dataRoot["BMS_Serial_Number"] = BMS_Serial_Number;
        dataRoot["BMS_Num_Cycles"] = BMS_Num_Cycles;

        break;
      }

      default:
        Serial.printf("Unknown CAN ID: 0x%03X\n", message.can_id);
        errorRoot["error"] = "Unknown CAN ID";
        has_errors = true;
        break;
    }
  }

  // Envoi des données normales au topic du chargeur
  char payloadData[1024];
  serializeJson(docData, payloadData);
  Serial.printf("📤 Sending data to MQTT topic: %s\n", topicData.c_str());

  mqttClient.publish(topicData.c_str(), payloadData);

  // Envoi des erreurs uniquement si elles existent
  if (has_errors) {
    char payloadError[512];
    serializeJson(docError, payloadError);
    Serial.print("📢 MQTT Publish Error Topic: ");
    Serial.println("bcn/box1/error");

    mqttClient.publish("bcn/box1/error", payloadError);
  }
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.begin("Yego Motor Garage", "yego@zamora");

  // Connexion Wi-Fi
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  // Initialisation de SPIFFS
  if (!SPIFFS.begin(true)) {
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }

  // Lire le fichier JSON et remplir la map
  readJSONFile();

  // Initialisation de ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW initialization failed");
    return;
  }
  esp_now_register_recv_cb(OnDataRecv);
  Serial.println("ESP-NOW Receiver initialized and waiting for data...");

  // Connexion au broker MQTT
  mqttClient.setServer(mqtt_broker, mqtt_port);
  while (!mqttClient.connected()) {
    if (mqttClient.connect("ESP32-Receiver", mqtt_username, mqtt_password)) {
      Serial.println("Connected to MQTT Broker");
    } else {
      Serial.print("Failed to connect to MQTT, retrying in 2 seconds...");
      delay(2000);
    }
  }
}

void loop() {
  mqttClient.loop();
  delay(1000);
}
