#include <Adafruit_ISM330DHCX.h>

Adafruit_ISM330DHCX ism330dhcx;
void setup(void) {
  Serial.begin(115200);
 
  if (!ism330dhcx.begin_I2C()) {
    // if (!ism330dhcx.begin_SPI(LSM_CS)) {
    // if (!ism330dhcx.begin_SPI(LSM_CS, LSM_SCK, LSM_MISO, LSM_MOSI)) {
    Serial.println("Failed to find ISM330DHCX chip");
    while (1) {
      delay(10);
    }
  }
}

void loop() {
  //  /* Get a new normalized sensor event */
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  ism330dhcx.getEvent(&accel, &gyro, &temp);

  /* Display the results (acceleration is measured in m/s^2) */

  Serial.print(accel.acceleration.x);
  Serial.print(" ");
  Serial.print(accel.acceleration.y);
  Serial.print(" ");
  Serial.print(accel.acceleration.z);

  Serial.print(" ");
  Serial.print(gyro.gyro.x);
  Serial.print(" ");
  Serial.print(gyro.gyro.y);
  Serial.print(" ");
  Serial.print(gyro.gyro.z);
  
  Serial.println();
  
  delay(10);

}
