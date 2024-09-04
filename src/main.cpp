
#include "sensesp/sensors/analog_input.h"
#include "sensesp/sensors/digital_input.h"
#include "sensesp/sensors/sensor.h"
#include "sensesp/signalk/signalk_output.h"
#include "sensesp/system/lambda_consumer.h"
#include "sensesp_app_builder.h"
#include "sensesp/transforms/linear.h"
#include <Adafruit_BMP280.h>
#include <Wire.h>
#include <Arduino.h>

using namespace sensesp;

reactesp::ReactESP app;

/// BMP280 ///
Adafruit_BMP280 bmp280; // I2C

// Define the function that will be called every time we want
// an updated temperature value from the sensor. The sensor reads degrees
// Celsius, but all temps in Signal K are in Kelvin, so add 273.15.
float read_temp_callback() { return (bmp280.readTemperature() + 273.15); }
// Define the function for reading the barometric pressure from the sensor.
float read_barometric_pressure_callback() { return bmp280.readPressure(); }

// The setup function performs one-time application initialization.
void setup()
{

#ifndef SERIAL_DEBUG_DISABLED
  SetupSerialDebug(115200);
#endif

  // Start of steps to find the I2C address
  /*
    Wire.begin();         // Initialize the I2C bus
    Serial.begin(115200); // Start the serial communication at 115200 baud rate
    while (!Serial)
      ; // Wait for the serial port to open (for Leonardo boards)
    Serial.println("\nI2C Scanner");
  */

  // Construct the global SensESPApp() object
  SensESPAppBuilder builder;
  sensesp_app = (&builder)
                    // Set a custom hostname for the app.
                    ->set_hostname("drama-sensesp_outside")
                    // Optionally, hard-code the WiFi and Signal K server
                    // settings. This is normally not needed.
                    ->set_wifi("drama_network", "sv_drama")
                    ->set_sk_server("10.10.10.1", 3000)
                    ->enable_uptime_sensor()
                    ->enable_ota("drama")
                    ->get_app();

  // Initialize the BMP280 using the default address
  if (!bmp280.begin(0x76))
  {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1)
      ;
  }

  auto outside_temp_metadata =
      new SKMetadata("K",                   // units
                     "Outside Temperature", // display name
                     "Outside Temperature", // description
                     "Outside Temp",        // short name
                     10.                    // timeout, in seconds
      );

  auto outside_pressure_metadata =
      new SKMetadata("K",                           // units
                     "Outside Barometric Pressure", // display name
                     "Outside Barometric Pressure", // description
                     "Outside Pressure",            // short name
                     10.                            // timeout, in seconds
      );

  // Read the sensor every 2 seconds
  unsigned int read_interval = 2000;

  // Engine room temperature
  auto *outside_temp =
      new RepeatSensor<float>(read_interval, read_temp_callback);
  // Set the Signal K Path for the output
  const char *sk_path_temp = "environment.outside.temperature";
  // Send the temperature to the Signal K server as a Float
  outside_temp->connect_to(new SKOutputFloat(sk_path_temp, outside_temp_metadata));

  // Outside barometric pressure
  auto *outside_barometric_pressure =
      new RepeatSensor<float>(read_interval, read_barometric_pressure_callback);
  // Set the Signal K Path for the output
  const char *sk_path_pressure = "environment.outside.barometricPressure";
  // Send the pressure to the Signal K server as a Float
  outside_barometric_pressure->connect_to(new SKOutputFloat(sk_path_pressure, outside_pressure_metadata));

  // Start networking, SK server connections and other SensESP internals
  sensesp_app->start();
}

void loop()
{
  // Second step of finding I2C Address
  /*
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for (address = 1; address < 127; address++)
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("  !");
      nDevices++;
    }
    else if (error == 4)
    {
      Serial.print("Unknown error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");

  // delay(1000); // wait 1 seconds for the next scan
  */

  app.tick();
}
