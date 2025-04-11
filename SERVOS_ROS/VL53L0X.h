
#include "Adafruit_VL53L0X.h" // https://github.com/adafruit/Adafruit_VL53L0X
#include "io_expender_manager.h"

#define LOX_SDA 26
#define LOX_SCL 25

// address we will assign if dual sensor is present
#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31

// set the pins to shutdown
#define SHT_LOX1 106
#define SHT_LOX2 107

// objects for the vl53l0x
Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();

// this holds the measurement
VL53L0X_RangingMeasurementData_t measure1;
VL53L0X_RangingMeasurementData_t measure2;


/*
    Reset all sensors by setting all of their XSHUT pins low for delay(10), then set all XSHUT high to bring out of reset
    Keep sensor #1 awake by keeping XSHUT pin high
    Put all other sensors into shutdown by pulling XSHUT pins low
    Initialize sensor #1 with lox.begin(new_i2c_address) Pick any number but 0x29 and it must be under 0x7F. Going with 0x30 to 0x3F is probably OK.
    Keep sensor #1 awake, and now bring sensor #2 out of reset by setting its XSHUT pin high.
    Initialize sensor #2 with lox.begin(new_i2c_address) Pick any number but 0x29 and whatever you set the first sensor to
 */
void setID(IOPotentiallyExpended* io) {
  // all reset
  io->myDigitalWrite(SHT_LOX1, LOW);    
  io->myDigitalWrite(SHT_LOX2, LOW);
  delay(10);
  // all unreset
  io->myDigitalWrite(SHT_LOX1, HIGH);
  io->myDigitalWrite(SHT_LOX2, HIGH);
  delay(10);

  // activating LOX1 and resetting LOX2
  io->myDigitalWrite(SHT_LOX1, HIGH);
  io->myDigitalWrite(SHT_LOX2, LOW);

  // initing LOX1
  if(!lox1.begin(LOX1_ADDRESS)) {
    Serial.println(F("Failed to boot first VL53L0X"));
    while(1);
  }
  delay(10);

  // activating LOX2
  io->myDigitalWrite(SHT_LOX2, HIGH);
  delay(10);

  //initing LOX2
  #ifdef USE_SECOND_SENSOR
  if(!lox2.begin(LOX2_ADDRESS)) {
    Serial.println(F("Failed to boot second VL53L0X"));
    while(1);
  }
  #endif
}

bool obstaclePresent(VL53L0X_RangingMeasurementData_t* mesure, uint16_t distanceMax_mm)
{
  if (mesure->RangeStatus != 4)
  {
    return false;
  }
  if (mesure->RangeMilliMeter > distanceMax_mm)
  {
    return false;
  }
  return true;
}

bool read_dual_sensors(IOPotentiallyExpended* io, 
VL53L0X_RangingMeasurementData_t* measureFront,
VL53L0X_RangingMeasurementData_t* measureRear
)
{
  return lox1.rangingTest(measureFront, false) && // pass in 'true' to get debug data printout!
  lox2.rangingTest(measureRear, false); // pass in 'true' to get debug data printout!
}

void read_dual_sensors(IOPotentiallyExpended* io) {
  
  lox1.rangingTest(&measure1, false); // pass in 'true' to get debug data printout!
  lox2.rangingTest(&measure2, false); // pass in 'true' to get debug data printout!

  // print sensor one reading
  Serial.print(F("1: "));
  if(measure1.RangeStatus != 4) {     // if not out of range
    Serial.print(measure1.RangeMilliMeter);
  } else {
    Serial.print(F("Out of range"));
  }
  
  Serial.print(F(" "));

  // print sensor two reading
  Serial.print(F("2: "));
  if(measure2.RangeStatus != 4) {
    Serial.print(measure2.RangeMilliMeter);
  } else {
    Serial.print(F("Out of range"));
  }
  
  Serial.println();
}

void setupVL53L0X(IOPotentiallyExpended* io) {
  io->myPinMode(SHT_LOX1, OUTPUT);
  io->myPinMode(SHT_LOX2, OUTPUT);

  Serial.println(F("Shutdown pins inited..."));

  //Wire.begin(LOX_SDA, LOX_SCL);

  io->myDigitalWrite(SHT_LOX1, LOW);
  io->myDigitalWrite(SHT_LOX2, LOW);

  Serial.println(F("Both in reset mode...(pins are low)"));
  
  
  Serial.println(F("Starting..."));
  setID(io);
}
