#include <memory>
#include <Arduino.h>
#include "sensesp.h"
#include "sensesp/sensors/analog_input.h"
#include "sensesp/sensors/digital_input.h"
#include "sensesp/sensors/sensor.h"
#include "sensesp/signalk/signalk_output.h"
#include "sensesp/system/lambda_consumer.h"
#include "sensesp_app_builder.h"

#define RS485_TX 25
#define RS485_RX 33
#define RS485_EN 5

HardwareSerial mod(1);

std::shared_ptr<sensesp::SKOutput<float>> wind_direction_sk_output;
std::shared_ptr<sensesp::SKOutput<float>> wind_speed_sk_output;

using namespace sensesp;

uint16_t calculateCRC(uint8_t *data, uint8_t length) {
  uint16_t crc = 0xFFFF;
  for (uint8_t i = 0; i < length; i++) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x0001) {
        crc >>= 1;
        crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

void decodeAndPrint(uint8_t* data) {
  uint16_t speedRaw = (data[0] << 8) | data[1];
  uint16_t directionRaw = (data[4] << 8) | data[5];

  float speedMps = speedRaw / 10.0;
  float speedKnots = speedMps * 1.94384;
  float directionDeg = directionRaw / 10.0;
  float reversedDirectionDeg = fmod((360.0 - directionDeg), 360.0);
  if (reversedDirectionDeg < 0) reversedDirectionDeg += 360.0;

  // Filter out invalid values
  if (speedMps <= 0.0 || reversedDirectionDeg < 0.0 || reversedDirectionDeg > 360.0) {
    Serial.println("Filtered out invalid wind data");
    return;
  }

  Serial.print("Wind Speed: ");
  Serial.print(speedKnots);
  Serial.println(" knots)");

  Serial.print("Wind Direction (reversed): ");
  Serial.print(reversedDirectionDeg);
  Serial.println(" degrees");
  Serial.println();

  if (wind_speed_sk_output) wind_speed_sk_output->set_input(speedMps);
  if (wind_direction_sk_output) wind_direction_sk_output->set_input(reversedDirectionDeg);
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("Modbus Test Ready");
  mod.begin(9600, SERIAL_8N1, RS485_RX, RS485_TX);
  pinMode(RS485_EN, OUTPUT);
  digitalWrite(RS485_EN, LOW);

  SetupLogging(ESP_LOG_DEBUG);

  SensESPAppBuilder builder;
  sensesp_app = (&builder)
                    ->set_hostname("my-sensesp-project")
                    ->set_wifi_client("VM2FA118", "Vwkc5bdgGvxs")
                    ->set_sk_server("192.168.0.21", 3000)
                    ->get_app();

  const uint8_t kAnalogInputPin = 36;
  const unsigned int kAnalogInputReadInterval = 500;
  const float kAnalogInputScale = 3.3;

  auto analog_input = std::make_shared<AnalogInput>(
      kAnalogInputPin, kAnalogInputReadInterval, "", kAnalogInputScale);

  analog_input->attach([analog_input]() {
    debugD("Analog input value: %f", analog_input->get());
  });

  const uint8_t kDigitalOutputPin = 15;
  const unsigned int kDigitalOutputInterval = 650;
  pinMode(kDigitalOutputPin, OUTPUT);
  event_loop()->onRepeat(kDigitalOutputInterval, [kDigitalOutputPin]() {
    digitalWrite(kDigitalOutputPin, !digitalRead(kDigitalOutputPin));
  });

  const uint8_t kDigitalInput1Pin = 14;
  auto digital_input1 = std::make_shared<DigitalInputChange>(
      kDigitalInput1Pin, INPUT_PULLUP, CHANGE);

  auto digital_input1_consumer = std::make_shared<LambdaConsumer<bool>>(
      [](bool input) { debugD("Digital input value changed: %d", input); });

  digital_input1->connect_to(digital_input1_consumer);

  const uint8_t kDigitalInput2Pin = 13;
  const unsigned int kDigitalInput2Interval = 1000;
  pinMode(kDigitalInput2Pin, INPUT_PULLUP);

  auto digital_input2 = std::make_shared<RepeatSensor<bool>>(
      kDigitalInput2Interval,
      [kDigitalInput2Pin]() { return digitalRead(kDigitalInput2Pin); });

  auto aiv_metadata = std::make_shared<SKMetadata>("V", "Analog input voltage");
  auto aiv_sk_output = std::make_shared<SKOutput<float>>(
      "sensors.analog_input.voltage",
      "/Sensors/Analog Input/Voltage",
      aiv_metadata
  );

  ConfigItem(aiv_sk_output)
      ->set_title("Analog Input Voltage SK Output Path")
      ->set_description("The SK path to publish the analog input voltage")
      ->set_sort_order(100);

  analog_input->connect_to(aiv_sk_output);

  auto di2_metadata = std::make_shared<SKMetadata>("", "Digital input 2 value");
  auto di2_sk_output = std::make_shared<SKOutput<bool>>(
      "sensors.digital_input2.value",
      "/Sensors/Digital Input 2/Value",
      di2_metadata
  );

  ConfigItem(di2_sk_output)
      ->set_title("Digital Input 2 SK Output Path")
      ->set_sort_order(200);

  digital_input2->connect_to(di2_sk_output);

  wind_direction_sk_output = std::make_shared<SKOutput<float>>(
      "environment.wind.directionApparent",
      "/Environment/Wind/DirectionApparent"
  );

  wind_speed_sk_output = std::make_shared<SKOutput<float>>(
      "environment.wind.speedApparent",
      "/Environment/Wind/SpeedApparent"
  );

  while (true) {
    loop();
  }
}

void loop() {
  uint8_t request[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x04, 0x44, 0x09};
  digitalWrite(RS485_EN, HIGH);
  delay(10);
  mod.write(request, sizeof(request));
  mod.flush();
  delay(10);
  digitalWrite(RS485_EN, LOW);
  delay(100);

  uint8_t response[20];
  int index = 0;
  unsigned long startTime = millis();
  while ((millis() - startTime) < 500) {
    if (mod.available()) {
      response[index++] = mod.read();
      if (index >= sizeof(response)) break;
    }
  }

  Serial.print("Raw Modbus response: ");
  for (int i = 0; i < index; i++) {
    Serial.printf("%02X ", response[i]);
  }
  Serial.println();

  if (index >= 11) {
    uint16_t received_crc = (response[index - 1] << 8) | response[index - 2];
    uint16_t calculated_crc = calculateCRC(response, index - 2);
    if (received_crc != calculated_crc) {
      Serial.println("CRC check failed");
      return;
    }
    decodeAndPrint(&response[3]);
  } else {
    Serial.println("No valid response");
  }

  event_loop()->tick();
}
