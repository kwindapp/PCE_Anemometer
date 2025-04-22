#include <RTCZero.h>
#include <LoRaWan.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>

RTCZero rtc;

unsigned char data[10];     // LoRaWAN data packet buffer
char buffer[256];           // Buffer for text responses from LoRa module

const int powerpin = 38;
const int sensorpin = 21;

const int RecordTime = 10;  // Measurement duration in seconds
int InterruptCounter;
float windspeed;
float windsum = 0;
float windmax = 0;
float windmin = 9999;
int sampleCount = 0;

int flagMinutes = 2;  // Sleep duration flag (1–5 mins)

void countup() {
  InterruptCounter++;
}

void measure() {
  InterruptCounter = 0;
  attachInterrupt(digitalPinToInterrupt(sensorpin), countup, RISING);
  delay(1000 * RecordTime);
  detachInterrupt(digitalPinToInterrupt(sensorpin));

  float currentWind = (float)InterruptCounter / (float)RecordTime * 2.4;

  windspeed = currentWind;
  windsum += currentWind;
  sampleCount++;

  if (currentWind > windmax) windmax = currentWind;
  if (currentWind < windmin) windmin = currentWind;
}

void setup(void) {
  pinMode(powerpin, OUTPUT);
  pinMode(sensorpin, INPUT_PULLUP);
  digitalWrite(powerpin, HIGH);

  delay(5000);
  SerialUSB.begin(115200);
  delay(500);
  SerialUSB.println("Connect Helium LoRaWAN board started!");

  lora.init();
  memset(buffer, 0, 256);
  lora.getVersion(buffer, 256, 1);
  SerialUSB.print(buffer);

  memset(buffer, 0, 256);
  lora.getId(buffer, 256, 1);
  SerialUSB.print(buffer);

  lora.setId(NULL, "6081F9EAE996E", "6081F927CE20");
  lora.setKey(NULL, NULL, "F17522A10F4A5ADFBA7C3");

  lora.setDeciveMode(LWOTAA);
  lora.setDataRate(DR3, EU868);
  lora.setAdaptiveDataRate(true);

  lora.setChannel(0, 868.1);
  lora.setChannel(1, 868.3);
  lora.setChannel(2, 868.5);
  lora.setChannel(3, 867.1);
  lora.setChannel(4, 867.3);
  lora.setChannel(5, 867.5);
  lora.setChannel(6, 867.7);
  lora.setChannel(7, 867.9);

  lora.setDutyCycle(true);
  lora.setJoinDutyCycle(true);
  lora.setPower(14);

  while (!lora.setOTAAJoin(JOIN, 20));

  lora.setPort(33);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
}

void loop(void) {
  int16_t int16_vbattery;
  float windavg;

  digitalWrite(powerpin, HIGH);
  delay(1000);

  // Reset tracking variables
  windsum = 0;
  windmax = 0;
  windmin = 9999;
  sampleCount = 0;

  // Take multiple measurements
  int sampleWindowSeconds = 60;
  int intervalSeconds = 10;
  int numSamples = sampleWindowSeconds / intervalSeconds;

  for (int i = 0; i < numSamples; i++) {
    measure();
    delay(1000);  // wait a second between samples
  }

  digitalWrite(powerpin, LOW);

  windavg = windsum / sampleCount;
  int16_vbattery = lora.getBatteryVoltage();

  SerialUSB.print("Windspeed Last: "); SerialUSB.println(windspeed);
  SerialUSB.print("Wind Avg: "); SerialUSB.println(windavg);
  SerialUSB.print("Wind Max: "); SerialUSB.println(windmax);
  SerialUSB.print("Wind Min: "); SerialUSB.println(windmin);
  SerialUSB.print("VBat: "); SerialUSB.println(int16_vbattery);

  // Convert all to knots x100
  int16_t avg_knots = windavg * 100.0 * 0.539957;
  int16_t max_knots = windmax * 100.0 * 0.539957;
  int16_t min_knots = windmin * 100.0 * 0.539957;

  data[0] = (byte)(avg_knots >> 8);
  data[1] = (byte)(avg_knots & 0xFF);
  data[2] = (byte)(max_knots >> 8);
  data[3] = (byte)(max_knots & 0xFF);
  data[4] = (byte)(min_knots >> 8);
  data[5] = (byte)(min_knots & 0xFF);
  data[6] = (byte)(int16_vbattery >> 8);
  data[7] = (byte)(int16_vbattery & 0xFF);

  int payloadSize = 8;
  SerialUSB.println("Payload:");
  for (int i = 0; i < payloadSize; i++) {
    SerialUSB.print("0x");
    if (data[i] < 0x10) SerialUSB.print("0");
    SerialUSB.print(data[i], HEX);
    SerialUSB.print(" ");
  }
  SerialUSB.println();

  bool result = lora.transferPacket(data, payloadSize, 5);
  if (result) {
    short length;
    short rssi;
    memset(buffer, 0, 256);
    length = lora.receivePacket(buffer, 256, &rssi);
    if (length) {
      SerialUSB.print("Length: "); SerialUSB.println(length);
      SerialUSB.print("RSSI: "); SerialUSB.println(rssi);
      SerialUSB.print("Data: ");
      for (unsigned char i = 0; i < length; i++) {
        SerialUSB.print("0x");
        SerialUSB.print(buffer[i], HEX);
        SerialUSB.print(" ");
      }
      SerialUSB.println();
      SerialUSB.println("Data sent successfully.");
    }
  }

  SerialUSB.print("Interval: ");
  SerialUSB.print(flagMinutes);
  SerialUSB.println(" minutes");

  flagMinutes = constrain(flagMinutes, 1, 5);
  doSleep((flagMinutes * 60 - 8) * 1000);

  lora.setPort(33);
}

void doSleep(uint32_t millis) {
  if (!rtc.isConfigured()) {
    rtc.begin(false);
  }

  uint32_t now = rtc.getEpoch();
  rtc.setAlarmEpoch(now + millis / 1000);
  rtc.enableAlarm(rtc.MATCH_HHMMSS);
  rtc.standbyMode();
}
