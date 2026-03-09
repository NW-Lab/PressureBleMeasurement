#include <M5GFX.h>
#include <M5Unified.h>
#include <BleSerial.h>
#include "BleBufferedSerial.h"
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP5xx.h"

#define SEALEVELPRESSURE_HPA (1013.25)

BleBufferedSerial SerialBT1;

Adafruit_BMP5xx bmp;                                       // Create BMP5xx object
bmp5xx_powermode_t desiredMode = BMP5XX_POWERMODE_NORMAL;  // Cache desired power mode

M5Canvas canvas(&M5.Lcd);

void setup() {
  M5.begin();

  M5.Power.setExtOutput(true);  // EXT_5V OUTPUT
  M5.Speaker.end();
  M5.Display.setRotation(1);
  M5.Display.setColorDepth(1);
  //int textsize = M5.Display.height() / 60;
  //if (textsize == 0) {
  //  textsize = 1;
  // }
  //M5.Display.setTextSize(textsize);
  M5.Display.clear(TFT_BLACK);
  M5.Display.print("PressureBT");
  canvas.setColorDepth(1);
  canvas.createSprite(240, 135);
  SerialBT1.begin("PressureBT");

  // Try to initialize the sensor
  // For I2C mode (default):
  if (!bmp.begin(BMP5XX_ALTERNATIVE_ADDRESS, &Wire)) {
    while (1) delay(10);
  }

  // Demonstrate all setter functions with range documentation

  /* Temperature Oversampling Settings:
   * BMP5XX_OVERSAMPLING_1X   - 1x oversampling (fastest, least accurate)
   * BMP5XX_OVERSAMPLING_2X   - 2x oversampling  
   * BMP5XX_OVERSAMPLING_4X   - 4x oversampling
   * BMP5XX_OVERSAMPLING_8X   - 8x oversampling
   * BMP5XX_OVERSAMPLING_16X  - 16x oversampling
   * BMP5XX_OVERSAMPLING_32X  - 32x oversampling
   * BMP5XX_OVERSAMPLING_64X  - 64x oversampling
   * BMP5XX_OVERSAMPLING_128X - 128x oversampling (slowest, most accurate)
   */
  bmp.setTemperatureOversampling(BMP5XX_OVERSAMPLING_2X);

  /* Pressure Oversampling Settings (same options as temperature):
   * Higher oversampling = better accuracy but slower readings
   * Recommended: 16X for good balance of speed/accuracy
   */
  bmp.setPressureOversampling(BMP5XX_OVERSAMPLING_16X);

  /* IIR Filter Coefficient Settings:
   * BMP5XX_IIR_FILTER_BYPASS   - No filtering (fastest response)
   * BMP5XX_IIR_FILTER_COEFF_1  - Light filtering
   * BMP5XX_IIR_FILTER_COEFF_3  - Medium filtering
   * BMP5XX_IIR_FILTER_COEFF_7  - More filtering
   * BMP5XX_IIR_FILTER_COEFF_15 - Heavy filtering
   * BMP5XX_IIR_FILTER_COEFF_31 - Very heavy filtering
   * BMP5XX_IIR_FILTER_COEFF_63 - Maximum filtering
   * BMP5XX_IIR_FILTER_COEFF_127- Maximum filtering (slowest response)
   */
  bmp.setIIRFilterCoeff(BMP5XX_IIR_FILTER_COEFF_3);

  /* Output Data Rate Settings (Hz):
   * BMP5XX_ODR_240_HZ, BMP5XX_ODR_218_5_HZ, BMP5XX_ODR_199_1_HZ
   * BMP5XX_ODR_179_2_HZ, BMP5XX_ODR_160_HZ, BMP5XX_ODR_149_3_HZ
   * BMP5XX_ODR_140_HZ, BMP5XX_ODR_129_8_HZ, BMP5XX_ODR_120_HZ
   * BMP5XX_ODR_110_1_HZ, BMP5XX_ODR_100_2_HZ, BMP5XX_ODR_89_6_HZ
   * BMP5XX_ODR_80_HZ, BMP5XX_ODR_70_HZ, BMP5XX_ODR_60_HZ, BMP5XX_ODR_50_HZ
   * BMP5XX_ODR_45_HZ, BMP5XX_ODR_40_HZ, BMP5XX_ODR_35_HZ, BMP5XX_ODR_30_HZ
   * BMP5XX_ODR_25_HZ, BMP5XX_ODR_20_HZ, BMP5XX_ODR_15_HZ, BMP5XX_ODR_10_HZ
   * BMP5XX_ODR_05_HZ, BMP5XX_ODR_04_HZ, BMP5XX_ODR_03_HZ, BMP5XX_ODR_02_HZ
   * BMP5XX_ODR_01_HZ, BMP5XX_ODR_0_5_HZ, BMP5XX_ODR_0_250_HZ, BMP5XX_ODR_0_125_HZ
   */
  bmp.setOutputDataRate(BMP5XX_ODR_50_HZ);

  /* Power Mode Settings:
   * BMP5XX_POWERMODE_STANDBY     - Standby mode (no measurements)
   * BMP5XX_POWERMODE_NORMAL      - Normal mode (periodic measurements)
   * BMP5XX_POWERMODE_FORCED      - Forced mode (single measurement then standby)
   * BMP5XX_POWERMODE_CONTINUOUS  - Continuous mode (fastest measurements)
   * BMP5XX_POWERMODE_DEEP_STANDBY - Deep standby (lowest power)
   */
  desiredMode = BMP5XX_POWERMODE_NORMAL;
  bmp.setPowerMode(desiredMode);

  /* Enable/Disable Pressure Measurement:
   * true  - Enable pressure measurement (default)
   * false - Disable pressure measurement (temperature only)
   */
  bmp.enablePressure(true);

  /* Interrupt Configuration:
   * BMP5XX_INTERRUPT_PULSED / BMP5XX_INTERRUPT_LATCHED - Interrupt mode
   * BMP5XX_INTERRUPT_ACTIVE_LOW / BMP5XX_INTERRUPT_ACTIVE_HIGH - Interrupt polarity  
   * BMP5XX_INTERRUPT_PUSH_PULL / BMP5XX_INTERRUPT_OPEN_DRAIN - Interrupt drive
   * BMP5XX_INTERRUPT_DATA_READY, BMP5XX_INTERRUPT_FIFO_FULL, etc. - Interrupt sources (can combine with |)
   */
  bmp.configureInterrupt(BMP5XX_INTERRUPT_LATCHED, BMP5XX_INTERRUPT_ACTIVE_HIGH, BMP5XX_INTERRUPT_PUSH_PULL, BMP5XX_INTERRUPT_DATA_READY, true);
}
float buffer_pressure[241];
int n = 0;
void loop() {
  
  // Check if new data is ready before reading
  if (!bmp.dataReady()) {
    return;
  }

  // Data is ready, perform reading
  if (!bmp.performReading()) {
    return;
  }

  //Serial.print(F("Temperature = "));
  //Serial.print(bmp.temperature);
  //Serial.println(F(" °C"));

  float pressure = bmp.pressure;
  //Serial.print(F("Pressure = "));
  //Serial.println(pressure);
  //Serial.println(F(" hPa"));

  //Serial.print(F("Approx. Altitude = "));
  //Serial.print(bmp.readAltitude(SEALEVELPRESSURE_HPA));
  //Serial.println(F(" m"));

  //Serial.println(F("---"));
  float max_pressure = 0;
  float min_pressure = 9999;
  buffer_pressure[n] = pressure;
  if (n == 240) {
    for (int i = 0; i < 240; i++) {
      buffer_pressure[i] = buffer_pressure[i + 1];
    }
  }

  for (int i = 0; i < n; i++) {
    if (max_pressure < buffer_pressure[i]) max_pressure = buffer_pressure[i];
    if (min_pressure > buffer_pressure[i]) min_pressure = buffer_pressure[i];
  }

  float H = (max_pressure - min_pressure) / 133.0;

  canvas.clear(TFT_BLACK);
  canvas.setColor(TFT_WHITE);

  for (int i = 0; i < n; i++) {
    canvas.drawPixel(i, 134-(buffer_pressure[i] - min_pressure) / H);
  }
  SerialBT1.printf("%f\r\n", pressure);
  SerialBT1.flush();
  canvas.pushSprite(0, 0);
  if (n < 240) n++;
  delay(1);
}