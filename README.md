# Smart-Motion-Sensor
ESP32-based smart alarm system using PIR motion, ultrasonic distance sensing, LDR light detection, DHT11 temperature/humidity, RGB LED status, a physical toggle button, and Bluetooth Low Energy (BLE) notifications. Includes adjustable sensitivity using a potentiometer and edge-triggered alert messaging.


# ESP32 Smart Alarm with BLE

This project is an ESP32 based alarm system that uses:

- PIR motion sensor
- Ultrasonic distance sensor
- LDR (photoresistor) for light level
- Potentiometer for adjustable distance threshold
- DHT11 temperature and humidity sensor
- RGB LED for status
- Pushbutton to toggle the system on or off
- BLE (Bluetooth Low Energy) to send alarm events to a phone or computer

The LED is green when the system is idle and red when the alarm is triggered.  
Alarm events are only sent once per detection (on the rising edge), so the serial monitor and BLE output do not spam.

## Hardware

Tested with an ESP32 DevKit style board.

**Digital pins**

- PIR: GPIO 5
- Ultrasonic trigger: GPIO 18
- Ultrasonic echo: GPIO 19
- Button (toggle): GPIO 23 (wired to ground, using INPUT_PULLUP)
- DHT11 data: GPIO 14
- RGB LED:
  - Red: GPIO 25
  - Green: GPIO 26
  - Blue: GPIO 27

**Analog pins**

- LDR: GPIO 34
- Potentiometer: GPIO 15

The LDR reading is compared to `LDR_DARK_THRESHOLD` (default 3000 out of 4095) to decide if the room is considered dark.

## Alarm Logic

- If the system is disabled (button toggled off), no alarm is triggered.
- If it is dark, the alarm is automatically triggered.
- If it is not dark, the alarm triggers only when:
  - PIR motion sensor detects motion, **and**
  - The measured distance is less than the threshold set by the potentiometer.

On alarm:

- RGB LED turns red.
- A message is printed to the serial monitor.
- A BLE notification is sent with sensor details.

When there is no alarm:

- RGB LED is green.

## BLE

The ESP32 advertises as:

- Name: `ESP32_Alarm`

Custom service and characteristics (replace with your own UUIDs if desired):

- Service UUID: `d0afc074-0001-4b0e-bb34-15d2b1d00001`
- TX Characteristic (notify): `d0afc074-0003-4b0e-bb34-15d2b1d00003`
- RX Characteristic (write): `d0afc074-0002-4b0e-bb34-15d2b1d00002`

Use an app like **LightBlue** or **nRF Connect** to:

- Connect to `ESP32_Alarm`
- Subscribe to the TX characteristic to see alarm messages
- Optionally write to the RX characteristic to send commands in the future

## Libraries

Install these libraries in the Arduino IDE:

- **DHT sensor library** (Adafruit)
- **Adafruit Unified Sensor** (dependency of DHT)
- **ESP32 BLE Arduino** by Neil Kolban / nkolban

## Building and Uploading

1. Open the project in Arduino IDE or VS Code with PlatformIO.
2. Select the correct board (e.g., "ESP32 Dev Module").
3. Select the correct COM port.
4. Compile and upload.

## Future Ideas

- Parse BLE commands from the phone to:
  - Toggle `systemEnabled`
  - Change the LDR dark threshold
  - Change min and max distance
- Log events to an SD card
- Use WiFi or MQTT instead of or in addition to BLE
