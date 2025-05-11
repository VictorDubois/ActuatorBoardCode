/*
pressure_range (kPa)   K value
131<P≤260               32
65<P≤131                64
32<P≤65                 128
16<P≤32                 256
8<P≤16                  512
4<P≤8                   1024
2≤P≤4                   2048
1≤P<2                   4096
P<1                     8192
*/

#define K 512                     // @TODO see table above to choose the correct value for your application
#define I2C_address_Pressure 0x6D // Unfortunately, it does not seem possible to change it. To use multiple sensors, use the analog version (XGZP6897A)

//----write One Byte of Data, Data from Arduino to the sensor----
// Write "thedata" to the sensor's address of "addr"
void write_one_byte(uint8_t device_address, uint8_t addr, uint8_t thedata)
{
  Wire.beginTransmission(device_address);
  Wire.write(addr);
  Wire.write(thedata);
  Wire.endTransmission();
}

//----Read One Byte of Data, Data from the sensor to the Arduino ----
uint8_t Read_One_Byte(uint8_t device_address, uint8_t addr)
{
  uint8_t nb_bytes = 1;
  Wire.beginTransmission(device_address);
  Wire.write(addr);
  Wire.endTransmission();
  Wire.requestFrom(device_address, nb_bytes);
  return Wire.read(); // Receive a byte as character
}

double readPressure()
{
  unsigned char pressure_H, pressure_M, pressure_L, temperature_H, temperature_L;
  // temporary variables of pressure and temperature
  long int pressure_adc, temperature_adc;
  // The value of pressure and temperature converted by the sensor’s ADC
  double pressure, temperature;
  // The calibrated value of pressure

  write_one_byte(I2C_address_Pressure, 0x30, 0x0A);

  // indicate a combined conversion (once temperature conversion immediately followed by once sensor signal conversion)
  // more measurement method, check Register 0x30

  long timeout = 0;
  while ((Read_One_Byte(I2C_address_Pressure, 0x30) & 0x08) > 0) // && timeout < 65000)
  {
    timeout++;
    if (timeout > 5000)
    {
      // Serial.println("Timeout! Please check the connections");
      break;
    }
  }
  // Judge whether Data collection is over
  delay(20);

  pressure_H = Read_One_Byte(I2C_address_Pressure, 0x06);
  pressure_M = Read_One_Byte(I2C_address_Pressure, 0x07);
  pressure_L = Read_One_Byte(I2C_address_Pressure, 0x08);

  // Read ADC output Data of Pressure
  pressure_adc = pressure_H * 65536 + pressure_M * 256 + pressure_L;
  // Compute the value of pressure converted by ADC
  if (pressure_adc > 8388608)
  {
    pressure = (pressure_adc - 16777216) / K; // unit is Pa, select appropriate K value according to pressure range.
  }
  else
  {
    pressure = pressure_adc / K; // unit is Pa, select appropriate K value according to pressure range. //The conversion formula of calibrated pressure, its unit is Pa
  }

  // Debug, feel free to comment this out
  /*Serial.print("Pressure: ");
  Serial.print(int(pressure));
  Serial.print(", Temperature: ");
  Serial.println(int(temperature));*/

  temperature_H = Read_One_Byte(I2C_address_Pressure, 0x09);
  temperature_L = Read_One_Byte(I2C_address_Pressure, 0x0A);
  // Read ADC output data of temperature
  temperature_adc = temperature_H * 256 + temperature_L;
  // Compute the value of temperature converted by ADC
  temperature = (temperature_adc - 65536) / 256; // is deg C
  // else // I do not understand this else without an "if" in the documentation
  temperature = temperature_adc / 256; // unit is ℃

  // The conversion formula of calibrated temperature, its unit is Centigrade

  return pressure;
}
