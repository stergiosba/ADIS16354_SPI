/*
  Arduino UNO and ADIS16354

  pin 10               CS -     SS
  pin 11               DIN -    MOSI
  pin 12               DOUT -   MISO
  pin 13               SCK -    SCLK
*/

#include <SPI.h>

#define SUPPLY_OUT 0X02
#define XGYRO_OUT 0X04
#define YGYRO_OUT 0X06
#define ZGYRO_OUT 0X08
#define XACCL_OUT 0X0A
#define YACCL_OUT 0X0C
#define ZACCL_OUT 0X0E
#define XTEMP_OUT 0X10
#define YTEMP_OUT 0X12
#define ZTEMP_OUT 0X14
#define AUX_ADC 0X16

#define XGYRO_OFF 0X1A
#define YGYRO_OFF 0X1C
#define ZGYRO_OFF 0X1E
#define XACCL_OFF 0X20
#define YACCL_OFF 0X22
#define ZACCL_OFF 0X24
#define ALM_MAG1 0X26
#define ALM_MAG2 0X28
#define ALM_SMPL1 0X2A
#define ALM_SMPL2 0X2C
#define ALM_CTRL 0X2E
#define AUX_DAC 0X30

#define GPIO_CTRL 0X32
#define MSC_CTRL 0X34
#define SMPL_PRD 0X36 // TS = TB × (NS + 1)

#define SENS_AVG 0X38
#define SLP_CNT 0X3A
#define STATUS 0X3C
#define COMMAND 0X3E

static constexpr uint16_t DIR_WRITE = 0x80;
int incomingByte = 0;
int SS = 10;

void setup()
{
  Serial.begin(9600);
  pinMode(SS, LOW);
  SPI.begin();
  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE3));
}

// convert 12 bit integer format to int16.
static int16_t convert12BitToINT16(uint16_t word)
{
  int16_t output = 0;

  if ((word >> 11) & 0x1)
  {
    // sign extend
    output = (word & 0xFFF) | 0xF000;
  }
  else
  {
    output = (word & 0x0FFF);
  }

  return output;
}

// convert 14 bit integer format to int16.
static int16_t convert14BitToINT16(uint16_t word)
{
  int16_t output = 0;

  if ((word >> 13) & 0x1)
  {
    // sign extend
    output = (word & 0x3FFF) | 0xC000;
  }
  else
  {
    output = (word & 0x3FFF);
  }

  return output;
}

static constexpr int16_t combine(uint8_t msb, uint8_t lsb)
{
  return (msb << 8u) | lsb;
}

void RegisterWrite(uint16_t reg, uint16_t value)
{
  uint8_t CMD[4]{};
  CMD[0] = (((static_cast<uint16_t>(reg)) & 0x00FF) | DIR_WRITE);
  CMD[1] = (0x00FF & value);
  CMD[2] = (((static_cast<uint16_t>(reg) + 1) & 0x00FF) | DIR_WRITE);
  CMD[3] = ((0xFF00 & value) >> 8);

  digitalWrite(SS, LOW);
  SPI.transfer(CMD[0]);
  delayMicroseconds(9);

  SPI.transfer(CMD[1]);
  delayMicroseconds(9);

  SPI.transfer(CMD[2]);
  delayMicroseconds(9);

  SPI.transfer(CMD[3]);
  delayMicroseconds(9);
  digitalWrite(SS, HIGH);
}

uint16_t RegisterRead(uint16_t reg)
{
  uint8_t CMD[2]{};
  CMD[0] = ((static_cast<uint16_t>(reg)) & 0x00FF);
  CMD[1] = 0x00;

  // ADIS 16354 has 16 bits that we need to read, remembering to discard 1st return

  digitalWrite(SS, LOW);
  SPI.transfer(CMD[0]);
  delayMicroseconds(9);
  SPI.transfer(CMD[1]);
  delayMicroseconds(9);
  CMD[0] = SPI.transfer(reg);
  delayMicroseconds(9);
  CMD[1] = SPI.transfer(reg);
  delayMicroseconds(9);
  digitalWrite(SS, HIGH);

  int16_t reg_data = combine(CMD[0], CMD[1]);
  return reg_data;
}

void loop()
{
  // Read gyro/accel output registers directly (14-bit data)
  int16_t xgyro = RegisterRead(XGYRO_OUT);
  int16_t ygyro = RegisterRead(YGYRO_OUT);
  int16_t zgyro = RegisterRead(ZGYRO_OUT);
  int16_t xaccel = RegisterRead(XACCL_OUT);
  int16_t yaccel = RegisterRead(YACCL_OUT);
  int16_t zaccel = RegisterRead(ZACCL_OUT);
  // gyro temperature measurement (12-bit data)
  uint16_t temp_x = RegisterRead(XTEMP_OUT);
  uint16_t temp_y = RegisterRead(YTEMP_OUT);
  uint16_t temp_z = RegisterRead(ZTEMP_OUT);

  // convert 14-bit data to 16-bit
  int16_t gyro_x = convert14BitToINT16(xgyro);
  int16_t gyro_y = convert14BitToINT16(ygyro);
  int16_t gyro_z = convert14BitToINT16(zgyro);
  int16_t accel_x = convert14BitToINT16(xaccel);
  int16_t accel_y = convert14BitToINT16(yaccel);
  int16_t accel_z = convert14BitToINT16(zaccel);
  // convert 12-bit data to 16-bit and set temperature scale (0.145 °C/LSB, 25 °C = 0x000)
  const float x_gyro_temperature = (convert12BitToINT16(temp_x)) * 0.1453f;
  const float y_gyro_temperature = (convert12BitToINT16(temp_y)) * 0.1453f;
  const float z_gyro_temperature = (convert12BitToINT16(temp_z)) * 0.1453f;
  const float temperature = (x_gyro_temperature + y_gyro_temperature + z_gyro_temperature) / 3.f;

  // sensor's frame is +x forward, +y left, +z up
  // flip y & z to publish right handed with z down (x forward, y right, z down)
  accel_y = (accel_y == INT16_MIN) ? INT16_MAX : -accel_y;
  accel_z = (accel_z == INT16_MIN) ? INT16_MAX : -accel_z;
  gyro_y = (gyro_y == INT16_MIN) ? INT16_MAX : -gyro_y;
  gyro_z = (gyro_z == INT16_MIN) ? INT16_MAX : -gyro_z;

  print_readings()
}

void print_readings()
{
  // Gyro data
  Serial.print("Gyro x:");
  Serial.print(gyro_x);
  Serial.print(" ");
  Serial.print("Gyro y:");
  Serial.print(":");
  Serial.print(gyro_y);
  Serial.print(" ");
  Serial.print("Gyro z:");
  Serial.print(":");
  Serial.print(gyro_z);
  Serial.println();

  // Accel data
  Serial.print("Acceleration x:");
  Serial.print(accel_x);
  Serial.print(" ");
  Serial.print("Acceleration y:");
  Serial.print(accel_y);
  Serial.print(" ");
  Serial.print("Acceleration z:");
  Serial.print(accel_z);
  Serial.println();

  // Temp data
  /*
  Serial.print("Temperature:");
  Serial.print(" ");
  Serial.print(temperature);
  Serial.println();
  */
}