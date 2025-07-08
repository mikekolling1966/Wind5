// Signal K application template file.
//
// This application demonstrates core SensESP concepts in a very
// concise manner. You can build and upload the application as is
// and observe the value changes on the serial port monitor.
//
// You can use this source file as a basis for your own projects.
// Remove the parts that are not relevant to you, and add your own code
// for external hardware libraries.

#include <memory>

#include "sensesp.h"
#include "sensesp/sensors/analog_input.h"
#include "sensesp/sensors/digital_input.h"
#include "sensesp/sensors/sensor.h"
#include "sensesp/signalk/signalk_output.h"
#include "sensesp/system/lambda_consumer.h"
#include "sensesp_app_builder.h"
#include <Arduino.h>
#define RS485_TX 25
#define RS485_RX 33
#define RS485_EN 5
HardwareSerial mod(1);


std::shared_ptr<sensesp::SKOutput<float>> wind_direction_sk_output;
std::shared_ptr<sensesp::SKOutput<float>> wind_speed_sk_output;








using namespace sensesp;

// The setup function performs one-time application initialization.
void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("Modbus Test Ready");
  mod.begin(9600, SERIAL_8N1, RS485_RX, RS485_TX);
  pinMode(RS485_EN, OUTPUT);
  digitalWrite(RS485_EN, LOW);


  SetupLogging(ESP_LOG_DEBUG);

  // Construct the global SensESPApp() object
  SensESPAppBuilder builder;
  sensesp_app = (&builder)
                    // Set a custom hostname for the app.
                    ->set_hostname("my-sensesp-project")
                    // Optionally, hard-code the WiFi and Signal K server
                    // settings. This is normally not needed.
                    ->set_wifi_client("VM2FA118", "Vwkc5bdgGvxs")
                    //->set_wifi_access_point("My AP SSID", "my_ap_password")
                    ->set_sk_server("192.168.0.21", 3000)
                    ->get_app();

  // GPIO number to use for the analog input
  const uint8_t kAnalogInputPin = 36;
  // Define how often (in milliseconds) new samples are acquired
  const unsigned int kAnalogInputReadInterval = 500;
  // Define the produced value at the maximum input voltage (3.3V).
  // A value of 3.3 gives output equal to the input voltage.
  const float kAnalogInputScale = 3.3;

  // Create a new Analog Input Sensor that reads an analog input pin
  // periodically.
  auto analog_input = std::make_shared<AnalogInput>(
      kAnalogInputPin, kAnalogInputReadInterval, "", kAnalogInputScale);

  // Add an observer that prints out the current value of the analog input
  // every time it changes.
  analog_input->attach([analog_input]() {
    debugD("Analog input value: %f", analog_input->get());
  });

  // Set GPIO pin 15 to output and toggle it every 650 ms

  const uint8_t kDigitalOutputPin = 15;
  const unsigned int kDigitalOutputInterval = 650;
  pinMode(kDigitalOutputPin, OUTPUT);
  event_loop()->onRepeat(kDigitalOutputInterval, [kDigitalOutputPin]() {
    digitalWrite(kDigitalOutputPin, !digitalRead(kDigitalOutputPin));
  });

  // Read GPIO 14 every time it changes

  const uint8_t kDigitalInput1Pin = 14;
  auto digital_input1 = std::make_shared<DigitalInputChange>(
      kDigitalInput1Pin, INPUT_PULLUP, CHANGE);

  // Connect the digital input to a lambda consumer that prints out the
  // value every time it changes.

  // Test this yourself by connecting pin 15 to pin 14 with a jumper wire and
  // see if the value changes!

  auto digital_input1_consumer = std::make_shared<LambdaConsumer<bool>>(
      [](bool input) { debugD("Digital input value changed: %d", input); });

  digital_input1->connect_to(digital_input1_consumer);

  // Create another digital input, this time with RepeatSensor. This approach
  // can be used to connect external sensor library to SensESP!

  const uint8_t kDigitalInput2Pin = 13;
  const unsigned int kDigitalInput2Interval = 1000;

  // Configure the pin. Replace this with your custom library initialization
  // code!
  pinMode(kDigitalInput2Pin, INPUT_PULLUP);

  // Define a new RepeatSensor that reads the pin every 100 ms.
  // Replace the lambda function internals with the input routine of your custom
  // library.

  // Again, test this yourself by connecting pin 15 to pin 13 with a jumper
  // wire and see if the value changes!

  auto digital_input2 = std::make_shared<RepeatSensor<bool>>(
      kDigitalInput2Interval,
      [kDigitalInput2Pin]() { return digitalRead(kDigitalInput2Pin); });

  // Connect the analog input to Signal K output. This will publish the
  // analog input value to the Signal K server every time it changes.
  auto aiv_metadata = std::make_shared<SKMetadata>("V", "Analog input voltage");
  auto aiv_sk_output = std::make_shared<SKOutput<float>>(
      "sensors.analog_input.voltage",   // Signal K path
      "/Sensors/Analog Input/Voltage",  // configuration path, used in the
                                        // web UI and for storing the
                                        // configuration
      aiv_metadata
  );

  ConfigItem(aiv_sk_output)
      ->set_title("Analog Input Voltage SK Output Path")
      ->set_description("The SK path to publish the analog input voltage")
      ->set_sort_order(100);

  analog_input->connect_to(aiv_sk_output);

  // Connect digital input 2 to Signal K output.
  auto di2_metadata = std::make_shared<SKMetadata>("", "Digital input 2 value");
  auto di2_sk_output = std::make_shared<SKOutput<bool>>(
      "sensors.digital_input2.value",    // Signal K path
      "/Sensors/Digital Input 2/Value",  // configuration path
      di2_metadata
  );

  ConfigItem(di2_sk_output)
      ->set_title("Digital Input 2 SK Output Path")
      ->set_sort_order(200);

  digital_input2->connect_to(di2_sk_output);

wind_direction_sk_output = std::make_shared<sensesp::SKOutput<float>>(
    "environment.wind.directionApparent", 
    "/Environment/Wind/DirectionApparent"
);

wind_speed_sk_output = std::make_shared<sensesp::SKOutput<float>>(
    "environment.wind.speedApparent",
    "/Environment/Wind/SpeedApparent"
);






  // To avoid garbage collecting all shared pointers created in setup(),
  // loop from here.
  while (true) {
    loop();
  }
}

void decodeAndPrint(uint8_t* data) {
  uint16_t speedRaw = (data[0] << 8) | data[1];
  uint16_t directionRaw = (data[4] << 8) | data[5];

  float speedMps = speedRaw / 10.0;
  float speedKnots = speedMps * 1.94384;

  float directionDeg = directionRaw / 10.0;
  float reversedDirectionDeg = fmod((360.0 - directionDeg), 360.0);
  if (reversedDirectionDeg < 0) reversedDirectionDeg += 360.0;

  // ✅ This is the log you want to match
  Serial.print("Wind Speed: ");
 // Serial.print(speedMps);
 // Serial.print(" m/s (");
  Serial.print(speedKnots);
  Serial.println(" knots)");

  Serial.print("Wind Direction (reversed): ");
  Serial.print(reversedDirectionDeg);
  Serial.println(" degrees");

  Serial.println();

  // ✅ SEND EXACTLY THESE VALUES TO SK — no conversion, no filtering
  if (wind_speed_sk_output) wind_speed_sk_output->set_input(speedMps);
  if (wind_direction_sk_output) wind_direction_sk_output->set_input(reversedDirectionDeg);
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
    decodeAndPrint(&response[3]);
  } else {
    Serial.println("No valid response");
  }
  


  event_loop()->tick(); 


}
