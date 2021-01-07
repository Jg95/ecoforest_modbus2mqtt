/*
Project: Modbus Temperature Humidity Sensor
Programmer: Shahrulnizam Mat Rejab
Board: TTGO ESP32 Display
Last Modified: 1 July 2020
Website: http://shahrulnizam.com
*/

#include <WiFi.h>
#include <PubSubClient.h>

// Replace the next variables with your SSID/Password combination
const char* ssid = "YoMeQuedoEnCasa";
const char* password = "Ng8LguaakSfQ5Nc";

// Add your MQTT Broker IP address, example:
const char* mqtt_server = "192.168.1.20";
const uint16_t mqtt_port = 1883;
const char* mqtt_client_name = "ESP8266Client";
const char* mqtt_user_name = "mqtt";
const char* mqtt_user_pass = "mqttpass";
const char mqtt_error_topic[100] = "ecoforest/error";

const char* online_topic = "ecoforest/esp_running";
uint32_t online_period = 60ul * 1000ul;
uint32_t online_timer = online_period + 1ul;

// Declare wifi and mqtt target
WiFiClient espClient;
PubSubClient client(espClient);

// Serial variables
#define modbus        Serial1
const uint32_t serial_baud = 115200;
const uint32_t modbus_baud = 19200;
const uint32_t modbus_conf = SERIAL_8N2;
const int8_t modbus_rx = 4;
const int8_t modbus_tx = 16;

// Modbus function codes
static const uint8_t ID = 17u;
static const uint8_t READ_COIL = 1u;
static const uint8_t WRITE_COIL = 5u;
static const uint8_t READ_HOLDING = 3u;
static const uint8_t WRITE_HOLDING = 6u;

// Modbus response timeout in milliseconds
static const uint32_t modbus_timeout = 1000ul;

bool refresh = true;

// Create the data structures for the ModBus interactions

struct WriteMBRegister
{
  uint8_t fnc; // Modbus function code to read
  char reg_name[100]; // MQTT topic name
  uint16_t addr; // Modbus address
};

struct ReadMBRegister
{
  uint8_t fnc; // Modbus function code to read
  char reg_name[100]; // MQTT topic name
  uint16_t addr; // Modbus address
  uint32_t period_ms; // Reading period of the variable in milliseconds
  uint32_t timer_ms; // Timer for the reading period
  String old; // Old value of the variable
};

// Create the ModBus write array
static const uint8_t WRITE_NUM = 7u;
WriteMBRegister write_list[WRITE_NUM] =
{
  {WRITE_HOLDING, "ecoforest/working_program_by_bus_in", 5221u},
  {WRITE_HOLDING, "ecoforest/bus_dhw_demand_in", 5222u},
  {WRITE_HOLDING, "ecoforest/bus_dg1_demand_in", 5224u},
  {WRITE_HOLDING, "ecoforest/bus_sg2_demand_in", 5225u},
  {WRITE_HOLDING, "ecoforest/bus_sg3_demand_in", 5226u},
  {WRITE_HOLDING, "ecoforest/bus_sg4_demand_in", 5227u},
  {WRITE_COIL, "ecoforest/on_off_control_by_bus_in", 53u}
};

// Create the ModBus read array
static const uint8_t READ_NUM = 59u;
ReadMBRegister read_list[READ_NUM] =
{
  {READ_HOLDING, "ecoforest/temperatura_exterior", 11u, 10ul * 60ul * 1000ul, 0ul, ""},
  {READ_HOLDING, "ecoforest/temperatura_impulsion_pozos", 1u, 30ul * 1000ul, 0ul, ""},
  {READ_HOLDING, "ecoforest/temperatura_retorno_de_pozos", 2u, 30ul * 1000ul, 0ul, ""},
  {READ_HOLDING, "ecoforest/temperatura_impulsion_calefaccion", 3u, 30ul * 1000ul, 0ul, ""},
  {READ_HOLDING, "ecoforest/temperatura_retorno_de_calefaccion", 4u, 30ul * 1000ul, 0ul, ""},
  {READ_HOLDING, "ecoforest/temperatura_aspiracion_del_compresor", 5u, 30ul * 1000ul, 0ul, ""},
  {READ_HOLDING, "ecoforest/presion_de_aspiracion_del_compresor", 6u, 30ul * 1000ul, 0ul, ""},
  {READ_HOLDING, "ecoforest/presion_de_descarga_del_compresor", 7u, 30ul * 1000ul, 0ul, ""},
  {READ_HOLDING, "ecoforest/temperatura_de_acs", 8u, 30ul * 1000ul, 0ul, ""},
  {READ_HOLDING, "ecoforest/presion_circuito_de_pozos", 13u, 30ul * 1000ul, 0ul, ""},
  {READ_HOLDING, "ecoforest/presion_circuito_de_calefaccion", 14u, 30ul * 1000ul, 0ul, ""},
  {READ_HOLDING, "ecoforest/cop", 30u, 30ul * 1000ul, 0ul, ""},
  {READ_HOLDING, "ecoforest/pf", 31u, 30ul * 1000ul, 0ul, ""},
  {READ_HOLDING, "ecoforest/temperatura_de_condensacion", 94u, 30ul * 1000ul, 0ul, ""},
  {READ_HOLDING, "ecoforest/grado_de_recalentamiento", 132u, 30ul * 1000ul, 0ul, ""},
  {READ_HOLDING, "ecoforest/posicion_de_valvula_de_expansion", 133u, 30ul * 1000ul, 0ul, ""},
  {READ_HOLDING, "ecoforest/temperatura_impulsion_sg2", 194u, 30ul * 1000ul, 0ul, ""},
  {READ_HOLDING, "ecoforest/temperatura_impulsion_sg3", 195u, 30ul * 1000ul, 0ul, ""},
  {READ_HOLDING, "ecoforest/temperatura_impulsion_sg4", 196u, 30ul * 1000ul, 0ul, ""},
  {READ_HOLDING, "ecoforest/temperatura_evaporacion", 199u, 30ul * 1000ul, 0ul, ""},
  {READ_HOLDING, "ecoforest/eer", 202u, 30ul * 1000ul, 0ul, ""},
  {READ_HOLDING, "ecoforest/temperatura_descarga_compresor", 203u, 30ul * 1000ul, 0ul, ""},
  {READ_COIL, "ecoforest/alarma", 50u, 10ul * 60ul * 1000ul, 0ul, ""},
  {READ_HOLDING, "ecoforest/horas_funcionamiento_l", 79u + 5001u, 1ul * 24ul * 60ul * 60ul * 1000ul, 0ul, ""},
  {READ_HOLDING, "ecoforest/energia_de_condensacion_enero", 143u + 5001u, 1ul * 24ul * 60ul * 60ul * 1000ul, 0ul, ""},
  {READ_HOLDING, "ecoforest/energia_de_condensacion_febrero", 144u + 5001u, 1ul * 24ul * 60ul * 60ul * 1000ul, 0ul, ""},
  {READ_HOLDING, "ecoforest/energia_de_condensacion_marzo", 145u + 5001u, 1ul * 24ul * 60ul * 60ul * 1000ul, 0ul, ""},
  {READ_HOLDING, "ecoforest/energia_de_condensacion_abril", 146u + 5001u, 1ul * 24ul * 60ul * 60ul * 1000ul, 0ul, ""},
  {READ_HOLDING, "ecoforest/energia_de_condensacion_mayo", 147u + 5001u, 1ul * 24ul * 60ul * 60ul * 1000ul, 0ul, ""},
  {READ_HOLDING, "ecoforest/energia_de_condensacion_junio", 148u + 5001u, 1ul * 24ul * 60ul * 60ul * 1000ul, 0ul, ""},
  {READ_HOLDING, "ecoforest/energia_de_condensacion_julio", 149u + 5001u, 1ul * 24ul * 60ul * 60ul * 1000ul, 0ul, ""},
  {READ_HOLDING, "ecoforest/energia_de_condensacion_agosto", 150u + 5001u, 1ul * 24ul * 60ul * 60ul * 1000ul, 0ul, ""},
  {READ_HOLDING, "ecoforest/energia_de_condensacion_septiembre", 151u + 5001u, 1ul * 24ul * 60ul * 60ul * 1000ul, 0ul, ""},
  {READ_HOLDING, "ecoforest/energia_de_condensacion_octubre", 152u + 5001u, 1ul * 24ul * 60ul * 60ul * 1000ul, 0ul, ""},
  {READ_HOLDING, "ecoforest/energia_de_condensacion_noviembre", 153u + 5001u, 1ul * 24ul * 60ul * 60ul * 1000ul, 0ul, ""},
  {READ_HOLDING, "ecoforest/energia_de_condensacion_diciembre", 154u + 5001u, 1ul * 24ul * 60ul * 60ul * 1000ul, 0ul, ""},
  {READ_HOLDING, "ecoforest/energia_electrica_consumida_enero", 167u + 5001u, 1ul * 24ul * 60ul * 60ul * 1000ul, 0ul, ""},
  {READ_HOLDING, "ecoforest/energia_electrica_consumida_febrero", 168u + 5001u, 1ul * 24ul * 60ul * 60ul * 1000ul, 0ul, ""},
  {READ_HOLDING, "ecoforest/energia_electrica_consumida_marzo", 169u + 5001u, 1ul * 24ul * 60ul * 60ul * 1000ul, 0ul, ""},
  {READ_HOLDING, "ecoforest/energia_electrica_consumida_abril", 170u + 5001u, 1ul * 24ul * 60ul * 60ul * 1000ul, 0ul, ""},
  {READ_HOLDING, "ecoforest/energia_electrica_consumida_mayo", 171u + 5001u, 1ul * 24ul * 60ul * 60ul * 1000ul, 0ul, ""},
  {READ_HOLDING, "ecoforest/energia_electrica_consumida_junio", 172u + 5001u, 1ul * 24ul * 60ul * 60ul * 1000ul, 0ul, ""},
  {READ_HOLDING, "ecoforest/energia_electrica_consumida_julio", 173u + 5001u, 1ul * 24ul * 60ul * 60ul * 1000ul, 0ul, ""},
  {READ_HOLDING, "ecoforest/energia_electrica_consumida_agosto", 174u + 5001u, 1ul * 24ul * 60ul * 60ul * 1000ul, 0ul, ""},
  {READ_HOLDING, "ecoforest/energia_electrica_consumida_septiembre", 175u + 5001u, 1ul * 24ul * 60ul * 60ul * 1000ul, 0ul, ""},
  {READ_HOLDING, "ecoforest/energia_electrica_consumida_octubre", 176u + 5001u, 1ul * 24ul * 60ul * 60ul * 1000ul, 0ul, ""},
  {READ_HOLDING, "ecoforest/energia_electrica_consumida_noviembre", 177u + 5001u, 1ul * 24ul * 60ul * 60ul * 1000ul, 0ul, ""},
  {READ_HOLDING, "ecoforest/energia_electrica_consumida_diciembre", 178u + 5001u, 1ul * 24ul * 60ul * 60ul * 1000ul, 0ul, ""},
  {READ_HOLDING, "ecoforest/contador_arranques_l", 280u + 5001u, 1ul * 24ul * 60ul * 60ul * 1000ul, 0ul, ""},
  {READ_HOLDING, "ecoforest/rpm_compresor", 1u + 5001u, 30ul * 1000ul, 0ul, ""},
  {READ_HOLDING, "ecoforest/temperatura_inverter", 4u + 5001u, 30ul * 1000ul, 0ul, ""},
  {READ_HOLDING, "ecoforest/consumo_electrico", 81u + 5001u, 30ul * 1000ul, 0ul, ""},
  {READ_HOLDING, "ecoforest/potencia_condensacion", 82u + 5001u, 30ul * 1000ul, 0ul, ""},
  {READ_HOLDING, "ecoforest/potencia_evaporacion", 184u + 5001u, 30ul * 1000ul, 0ul, ""},
  {READ_HOLDING, "ecoforest/demanda_acs_bus", 221u + 5001u, 30ul * 1000ul, 0ul, ""},
  {READ_HOLDING, "ecoforest/demanda_z1_bus", 223u + 5001u, 30ul * 1000ul, 0ul, ""},
  {READ_HOLDING, "ecoforest/demanda_z2_bus", 224u + 5001u, 30ul * 1000ul, 0ul, ""},
  {READ_HOLDING, "ecoforest/demanda_z3_bus", 225u + 5001u, 30ul * 1000ul, 0ul, ""},
  {READ_HOLDING, "ecoforest/demanda_z4_bus", 226u + 5001u, 30ul * 1000ul, 0ul, ""}
};

void setup()
{
  // Start serial ports
  Serial.begin(serial_baud);
  modbus.begin(modbus_baud, modbus_conf, modbus_rx, modbus_tx);

  // Connect to the wifi network
  setup_wifi();

  // Configure the mqtt client
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  client.setBufferSize(10000u);
}

void loop()
{
  // Check the wifi connection and reconnect
  if (WiFi.status() != WL_CONNECTED) {setup_wifi();}
  
  // Check the connection with the mqtt server and reconnect if needed
  if (!client.connected()) {reconnect();}

  // Manage mqtt callbacks
  client.loop();

  // Publish the online topic
  if (millis() - online_timer >= online_period)
  {
    Serial.println("Publish online topic");
    bool success = client.publish(online_topic, "online");
    if (success) {Serial.println("Message Published");}
    else {Serial.println("Publish failed");}
    online_timer = millis();
  }

  // Publish all mqtt sensor topics
  for (uint8_t i = 0; i < READ_NUM; i++)
  {
    if (millis() - read_list[i].timer_ms >= read_list[i].period_ms || refresh)
    { 
      String mqtt_payload;
      bool valid = true;
      
      // Read the modbus registers and form the mqtt message
      valid = readRegister(read_list[i], mqtt_payload);

      // Publish the mqtt message
      char mqtt_topic[100];
      if (valid)
      {
        for (uint8_t j = 0; j < 100; j++) {mqtt_topic[j] = read_list[i].reg_name[j];}
      }
      else
      {
        for (uint8_t j = 0; j < 100; j++) {mqtt_topic[j] = mqtt_error_topic[j];}
      }

      if (!refresh && mqtt_payload == read_list[i].old)
      {
        Serial.println("Same content, don't publish");
      }
      else
      {
        Serial.print("Publish on ");
        Serial.print(mqtt_topic);
        Serial.print(": ");
        Serial.println(mqtt_payload.c_str());
        bool success = client.publish(mqtt_topic, mqtt_payload.c_str());
        if (success) {Serial.println("Message Published");}
        else {Serial.println("Publish failed");}
      }

      read_list[i].old = mqtt_payload;

      // Update the timer for the next message
      if (valid) {read_list[i].timer_ms = millis();}
    }
  }

  if (refresh) {refresh = false;}
}

uint8_t waitResponse(uint8_t* response_data)
{
  // Wait for the modbus response
  uint32_t start_waiting = millis();
  bool valid = true;
  while (valid && !modbus.available())
  {
    if (millis() - start_waiting > modbus_timeout)
    {
      valid = false;
    }
  }

  // Read all the response data
  uint8_t i = 0;

  if (valid)
  {
    while (modbus.available()) {response_data[i++] = modbus.read();}
  
    // Remove initial zeros from the response
    while (response_data[0]==0)
    {
      for(uint8_t j = 0; j < i; j++) {response_data[j]=response_data[j+1];}
      i--;
    }
  
    // Print the raw response for debugging
    if(i > 1)
    {
      Serial.println("Response:");
      for(uint8_t j = 0; j < i; j++)
      {
        if(response_data[j] < 0x10) {Serial.print("0");}
        Serial.print(response_data[j], HEX);
        Serial.print(" ");
      }
      Serial.println();  
    }
  }

  return i;
}

bool readRegister(ReadMBRegister& reg, String& mqtt_payload)
{
  Serial.print("Read modbus for ");
  Serial.println(reg.reg_name);

  bool valid = true;

  uint8_t request_data[8];
  Modbus_request(ID, reg.fnc, reg.addr, 1, request_data);

  uint8_t response_data[20];
  uint8_t response_length = waitResponse(response_data);

  if (response_length == 0)
  {
    valid = false;
    mqtt_payload = String("Error: Timeout");
    Serial.println(mqtt_payload.c_str());
  }

  // Compute the response checksum
  uint8_t crc_l;
  uint8_t crc_h;
  if (valid)
  {
    uint16_t crc = CRC16(response_data, response_length - 2);
    crc_l = crc&0xFF;
    crc_h = (crc>>8)&0xFF;
  }

  // Check the response validity
  if (valid && response_data[0] != ID)
  {
    valid = false;
    mqtt_payload = "Error: Wrong modbus machine id";
    Serial.println(mqtt_payload.c_str());
  }
  
  if (valid && response_data[1] != reg.fnc)
  {
    valid = false;
    mqtt_payload = "Error: Wrong modbus function code acknowkedge";
    Serial.println(mqtt_payload.c_str());
  }
  
  if (valid &&
    ((response_data[response_length - 2] != crc_l) ||
    (response_data[response_length - 1] != crc_h)))
  {
    valid = false;
    mqtt_payload = "Error: Wrong checksum";
    Serial.println(mqtt_payload.c_str());
  }

  // Fill the json string with the register data
  if (valid)
  {
    uint16_t response_value;
    if (reg.fnc == READ_COIL) {response_value = response_data[3];}
    else {response_value = ((uint16_t(response_data[3])<<8) + response_data[4]);}
    mqtt_payload = String(response_value);
  }

  else
  {
    mqtt_payload += String(" Reading register: ") + String(reg.reg_name);
  }

  return valid;
}

void callback(char* topic, uint8_t* message, uint16_t message_length)
{
  // Print the message arrived on the topic
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;
  
  for (uint16_t i = 0; i < message_length; i++)
  {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();

  // Check the received message and write the modbus register
  for (uint8_t i = 0; i < WRITE_NUM; i++)
  {
    if (String(topic) == write_list[i].reg_name)
    {
      // Set the value to write on the modbus register
      uint16_t value = messageTemp.toInt();
      if (write_list[i].fnc == WRITE_COIL && value == 1) {value = 0xFF00;}

      // Send the modbus request to write
      uint8_t request_data[8];
      Modbus_request(ID, write_list[i].fnc, write_list[i].addr, value, request_data);

      // Check the response
      bool valid = true;
      uint8_t response_data[20];
      uint8_t response_length = waitResponse(response_data);

      String err_string;
      if (response_length == 0)
      {
        valid = false;
        err_string = String("Error: Timeout");
        Serial.println(err_string.c_str());
      }

      if (valid)
      {
        refresh = true;
      }
      else
      {
        err_string += String(" Writing register: ") + String(write_list[i].reg_name);
        client.publish(mqtt_error_topic, err_string.c_str());
      }
    }
  }
}

void Modbus_request(uint8_t id, uint8_t code, uint16_t addr,
  uint16_t quantity, uint8_t* request_data)
{
  // Write the request data
  uint8_t address[8];
  address[0]=id;
  address[1]=code;
  address[2]=addr>>8;
  address[3]=addr;
  address[4]=quantity>>8;
  address[5]=quantity;
  uint16_t crc=CRC16(address, 6);
  address[6]=crc&0xFF;
  address[7]=(crc>>8)&0xFF;

  // Print the request content
  Serial.print("Request ");
  Serial.println();
  for(uint8_t i = 0; i < 8; i++)
  {
    if(address[i]<0x10) {Serial.print("0");}
    Serial.print(address[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  // Send the modbus request
  for(uint8_t i = 0; i < 8; i++) {modbus.write(address[i]);}

  // Return the request data
  if (request_data != NULL) {request_data = address;}
}

uint16_t CRC16(uint8_t* data_word, uint8_t data_length)
{
  // Compute the checksum of a data word
  uint16_t CheckSum = 0xFFFFu;
  for (uint8_t j = 0; j < data_length; j++)
  {
    CheckSum = CheckSum ^ data_word[j];
    for(uint8_t i = 0; i < 8; i++)
    {
      if( (CheckSum) & 0x0001u == 1 ) {CheckSum = (CheckSum>>1) ^ 0xA001u;}
      else {CheckSum = CheckSum>>1;}
    }
  }
  return CheckSum;   
}

void setup_wifi()
{
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  while (WiFi.status() != WL_CONNECTED)
  {
    WiFi.disconnect(true);
    WiFi.begin(ssid, password);
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void reconnect()
{
  // Loop until we're reconnected
  while (!client.connected())
  {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(mqtt_client_name, mqtt_user_name, mqtt_user_pass))
    {
      Serial.println("connected");
      
      // Subscribe
      for (uint8_t i = 0; i < WRITE_NUM; i++)
      {
        client.subscribe(write_list[i].reg_name);
      }
    }
    
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
