// Standalone VL53L0X test: is the sensor itself OK?
// Wiring: VIN 5V, GND, SDA -> A4, SCL -> A5. Library: "VL53L0X" by Pololu.
// Upload to the master (or any Uno), open the Serial Monitor at 9600 baud.
//
// Each line:  mm=<range>  status=<s>  signal=<MCPS>  ambient=<MCPS>  spads=<n>
//   status 11 = good measurement, 9 = mixed returns, 4 = no target, others = fail
//   signal    = strength of the returned light. Big (> ~5) at a fixed range
//               no matter where you point it = light bouncing back from
//               something stuck on/near the lens (protective film, cover,
//               a part of the robot touching the cone)
//   ambient   = background infrared (sunlight / lamps); very high = can't see
//
// Type a key in the Serial Monitor to switch settings:
//   d = library defaults (33 ms per reading, signal limit 0.25)
//   a = high accuracy (200 ms per reading)
//   l = long range (signal limit 0.1, longer laser pulses)

#include <Wire.h>
#include <VL53L0X.h>

VL53L0X tof;

void setDefaults(){
  tof.setSignalRateLimit(0.25);
  tof.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 14);
  tof.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 10);
  tof.setMeasurementTimingBudget(33000);
  Serial.println("# settings: defaults");
}

void setup(){
  Serial.begin(9600);
  Wire.begin();
  tof.setTimeout(500);
  if (!tof.init()){
    Serial.println("# VL53L0X NOT FOUND at 0x29 - check 5V, GND, SDA=A4, SCL=A5");
    while (1){}
  }
  Serial.println("# VL53L0X found");
  setDefaults();
}

void loop(){

  if (Serial.available()){
    char c = Serial.read();
    tof.stopContinuous();
    if (c == 'd'){ setDefaults(); }
    if (c == 'a'){
      setDefaults();
      tof.setMeasurementTimingBudget(200000);
      Serial.println("# settings: high accuracy (200 ms)");
    }
    if (c == 'l'){
      tof.setSignalRateLimit(0.1);
      tof.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
      tof.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
      tof.setMeasurementTimingBudget(33000);
      Serial.println("# settings: long range");
    }
  }

  uint16_t mm = tof.readRangeSingleMillimeters();   // waits for one reading
  if (tof.timeoutOccurred()){ Serial.println("# timeout"); return; }

  // the result block stays readable after the measurement
  uint8_t status = (tof.readReg(VL53L0X::RESULT_RANGE_STATUS) >> 3) & 0x0F;
  float signal  = tof.readReg16Bit(VL53L0X::RESULT_RANGE_STATUS + 6) / 128.0;  // 9.7 fixed point
  float ambient = tof.readReg16Bit(VL53L0X::RESULT_RANGE_STATUS + 8) / 128.0;
  float spads   = tof.readReg16Bit(VL53L0X::RESULT_RANGE_STATUS + 2) / 256.0;  // 8.8 fixed point

  Serial.print("mm=");       Serial.print(mm);
  Serial.print("  in=");     Serial.print(mm / 25.4, 1);
  Serial.print("  status="); Serial.print(status);
  Serial.print("  signal="); Serial.print(signal, 2);
  Serial.print("  ambient=");Serial.print(ambient, 2);
  Serial.print("  spads=");  Serial.println(spads, 1);

  delay(100);
}
