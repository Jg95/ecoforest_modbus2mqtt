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

struct ModBusRegister
{
  uint8_t fnc; // Modbus function code to read
  char reg_name[100]; // MQTT topic name
  uint16_t addr; // Modbus address
};

struct SyncRegisters
{
  uint32_t period_ms; // Period of the synchronized data read
  uint32_t timer_ms; // Timer to wait for the period
  uint8_t num_registers; // Number of registers to read
  char topic[100]; // Topic to publish the mqtt message
  ModBusRegister list[100]; // List of registers to read
};

// Create the ModBus write array
static const uint8_t WRITE_NUM = 18u;
ModBusRegister write_list[WRITE_NUM] =
{
  {WRITE_HOLDING, "ecoforest/bus_dhw_setpoint", 134u},
  {WRITE_HOLDING, "ecoforest/bus_heating_dg1_setpoint", 135u},
  {WRITE_HOLDING, "ecoforest/bus_heating_sg2_setpoint", 136u},
  {WRITE_HOLDING, "ecoforest/bus_heating_sg3_setpoint", 137u},
  {WRITE_HOLDING, "ecoforest/bus_heating_sg4_setpoint", 138u},
  {WRITE_HOLDING, "ecoforest/bus_cooling_dg1_setpoint", 139u},
  {WRITE_HOLDING, "ecoforest/bus_cooling_sg2_setpoint", 140u},
  {WRITE_HOLDING, "ecoforest/bus_cooling_sg3_setpoint", 141u},
  {WRITE_HOLDING, "ecoforest/bus_cooling_sg4_setpoint", 142u},
  {WRITE_HOLDING, "ecoforest/bus_pool_setpoint", 143u},
  {WRITE_HOLDING, "ecoforest/working_program_by_bus", 5221u},
  {WRITE_HOLDING, "ecoforest/bus_dhw_demand", 5222u},
  {WRITE_HOLDING, "ecoforest/bus_pool_demand", 5223u},
  {WRITE_HOLDING, "ecoforest/bus_dg1_demand", 5224u},
  {WRITE_HOLDING, "ecoforest/bus_sg2_demand", 5225u},
  {WRITE_HOLDING, "ecoforest/bus_sg3_demand", 5226u},
  {WRITE_HOLDING, "ecoforest/bus_sg4_demand", 5227u},
  {WRITE_COIL, "ecoforest/on_off_control_by_bus", 53u}
};

// Create the ModBus read array
static const uint8_t SYNC_NUM = 3u;
SyncRegisters sync_list[SYNC_NUM] =
{
  // One second synchronized registers (1)
  {10000ul, 0ul, 49u, "ecoforest/10_second",
    {
      {READ_HOLDING, "brine_temp_out", 1u},
      {READ_HOLDING, "brine_temp_in", 2u},
      {READ_HOLDING, "heating_temp_out", 3u},
      {READ_HOLDING, "heating_temp_in", 4u},
      {READ_HOLDING, "compressor_suction_temperature", 5u},
      {READ_HOLDING, "compresor_suction_pressure", 6u},
      {READ_HOLDING, "compressor_discharge_pressure", 7u},
      {READ_HOLDING, "dhw_temperature", 8u},
      {READ_HOLDING, "outdoor_temperature", 11u},
      {READ_HOLDING, "brine_circuit_pressure", 13u},
      {READ_HOLDING, "heating_circuit_pressure", 14u},
      {READ_HOLDING, "brine_temp_air_unit_outlet", 29u},
      {READ_HOLDING, "cop", 30u},
      {READ_HOLDING, "pf", 31u},
      {READ_HOLDING, "condensation_temperature", 94u},
      {READ_HOLDING, "superheat", 132u},
      {READ_HOLDING, "expansion_valve_position", 133u},
      {READ_HOLDING, "bus_dhw_setpoint", 134u},
      {READ_HOLDING, "bus_pool_setpoint", 143u},
      {READ_HOLDING, "supply_temperature_sg2", 194u},
      {READ_HOLDING, "supply_temperature_sg3", 195u},
      {READ_HOLDING, "supply_temperature_sg4", 196u},
      {READ_HOLDING, "start_temperature_for_dhw", 198u},
      {READ_HOLDING, "evaporation_temperature", 199u},
      {READ_HOLDING, "heating_buffer_tank_temperature", 200u},
      {READ_HOLDING, "cooling_buffer_tank_temperature", 201u},
      {READ_HOLDING, "eer", 202u},
      {READ_HOLDING, "compresor_discharge_temperature", 203u},
      {READ_HOLDING, "compressor_rpm", 5002u},
      {READ_HOLDING, "scroll_temp", 5004u},
      {READ_HOLDING, "inverter_temperature", 5005u},
      {READ_HOLDING, "working_hours_l", 5080u},
      {READ_HOLDING, "working_hours_h", 5081u},
      {READ_HOLDING, "power_consumption", 5082u},
      {READ_HOLDING, "condensation_capacity", 5083u},
      {READ_HOLDING, "evaporation_capacity", 5185u},
      {READ_HOLDING, "working_program_by_bus", 5221u},
      {READ_HOLDING, "bus_dhw_demand", 5222u},
      {READ_HOLDING, "bus_pool_demand", 5223u},
      {READ_HOLDING, "bus_dg1_demand", 5224u},
      {READ_HOLDING, "bus_sg2_demand", 5225u},
      {READ_HOLDING, "bus_sg3_demand", 5226u},
      {READ_HOLDING, "bus_sg4_demand", 5227u},
      {READ_HOLDING, "number_of_starts_l", 5281u},
      {READ_HOLDING, "number_of_starts_h", 5282u},
      {READ_COIL, "active_alarm", 50u},
      {READ_COIL, "on_off_control_by_bus", 53u},
      {READ_COIL, "summer", 127u},
      {READ_COIL, "winter", 128u}
    }
  },

  // One month synchronized registers (2)
  {24ul * 60ul * 60ul * 1000ul, 0ul, 39u, "ecoforest/1_day",
    {
      {READ_HOLDING, "condensation_energy_january", 5144u},
      {READ_HOLDING, "condensation_energy_february", 5145u},
      {READ_HOLDING, "condensation_energy_march", 5146u},
      {READ_HOLDING, "condensation_energy_april", 5147u},
      {READ_HOLDING, "condensation_energy_may", 5148u},
      {READ_HOLDING, "condensation_energy_june", 5149u},
      {READ_HOLDING, "condensation_energy_july", 5150u},
      {READ_HOLDING, "condensation_energy_august", 5151u},
      {READ_HOLDING, "condensation_energy_september", 5152u},
      {READ_HOLDING, "condensation_energy_october", 5153u},
      {READ_HOLDING, "condensation_energy_november", 5154u},
      {READ_HOLDING, "condensation_energy_december", 5155u},
      {READ_HOLDING, "evaporation_energy_january", 5156u},
      {READ_HOLDING, "evaporation_energy_february", 5157u},
      {READ_HOLDING, "evaporation_energy_march", 5158u},
      {READ_HOLDING, "evaporation_energy_april", 5159u},
      {READ_HOLDING, "evaporation_energy_may", 5160u},
      {READ_HOLDING, "evaporation_energy_june", 5161u},
      {READ_HOLDING, "evaporation_energy_july", 5162u},
      {READ_HOLDING, "evaporation_energy_august", 5163u},
      {READ_HOLDING, "evaporation_energy_september", 5164u},
      {READ_HOLDING, "evaporation_energy_october", 5165u},
      {READ_HOLDING, "evaporation_energy_november", 5166u},
      {READ_HOLDING, "evaporation_energy_december", 5167u},
      {READ_HOLDING, "electrical_consumption_january", 5168u},
      {READ_HOLDING, "electrical_consumption_february", 5169u},
      {READ_HOLDING, "electrical_consumption_march", 5170u},
      {READ_HOLDING, "electrical_consumption_april", 5171u},
      {READ_HOLDING, "electrical_consumption_may", 5172u},
      {READ_HOLDING, "electrical_consumption_june", 5173u},
      {READ_HOLDING, "electrical_consumption_july", 5174u},
      {READ_HOLDING, "electrical_consumption_august", 5175u},
      {READ_HOLDING, "electrical_consumption_september", 5176u},
      {READ_HOLDING, "electrical_consumption_october", 5177u},
      {READ_HOLDING, "electrical_consumption_november", 5178u},
      {READ_HOLDING, "electrical_consumption_december", 5179u},
      {READ_HOLDING, "software_version_1", 5285u},
      {READ_HOLDING, "software_version_2", 5286u},
      {READ_HOLDING, "software_version_3", 5287u}
    }
  },

  {60ul * 60ul * 1000ul, 0ul, 8u, "ecoforest/1_hour",
    {
      {READ_HOLDING, "bus_heating_dg1_setpoint", 135u},
      {READ_HOLDING, "bus_heating_sg2_setpoint", 136u},
      {READ_HOLDING, "bus_heating_sg3_setpoint", 137u},
      {READ_HOLDING, "bus_heating_sg4_setpoint", 138u},
      {READ_HOLDING, "bus_cooling_dg1_setpoint", 139u},
      {READ_HOLDING, "bus_cooling_sg2_setpoint", 140u},
      {READ_HOLDING, "bus_cooling_sg3_setpoint", 141u},
      {READ_HOLDING, "bus_cooling_sg4_setpoint", 142u}
    }
  }
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

  // Publish all mqtt sensor topics
  for (uint8_t i = 0; i < SYNC_NUM; i++)
  {
    if (millis() - sync_list[i].timer_ms >= sync_list[i].period_ms || refresh)
    { 
      String mqtt_payload = "{";
      bool valid = true;
      
      for (uint8_t j = 0; valid && (j < sync_list[i].num_registers); j++)
      {
        if (j != 0) {mqtt_payload += ", ";}
        
        // Read the modbus registers and form the mqtt message
        valid = readRegister(sync_list[i].list[j], mqtt_payload);
      }
      
      mqtt_payload += "}";

      // Publish the mqtt message
      char mqtt_topic[100];
      if (valid)
      {
        for (uint8_t j = 0; j < 100; j++) {mqtt_topic[j] = sync_list[i].topic[j];}
      }
      else
      {
        for (uint8_t j = 0; j < 100; j++) {mqtt_topic[j] = mqtt_error_topic[j];}
      }
      
      Serial.print("Publish on ");
      Serial.print(mqtt_topic);
      Serial.print(": ");
      Serial.println(mqtt_payload.c_str());
      bool success = client.publish(mqtt_topic, mqtt_payload.c_str());
      if (success) {Serial.println("Message Published");}
      else {Serial.println("Publish failed");}

      // Update the timer for the next message
      if (valid) {sync_list[i].timer_ms = millis();}
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

bool readRegister(const ModBusRegister& reg, String& mqtt_payload)
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
    mqtt_payload += String("\"") + String(reg.reg_name) + "\": " + String(response_value);
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
