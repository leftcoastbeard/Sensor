#include "application.h"

/*********************************************************************/
#define HTU21DF_I2CADDR       0x40
#define HTU21DF_READTEMP      0xE3
#define HTU21DF_READHUM       0xE5
#define HTU21DF_WRITEREG       0xE6
#define HTU21DF_READREG       0xE7
#define HTU21DF_RESET       0xFE

class Adafruit_HTU21DF {
 public:
  Adafruit_HTU21DF();
  bool begin(void);
  float readTemperature(void);
  float readHumidity(void);
  void reset(void);
 private:
  bool readData(void);
  float humidity, temp;
};
/*********************************************************************/

/*********************************************************************/
#define S7_BLANK    0x00
#define S7_DP		    0x20	//Add this to get the decimal point
#define S7_DEG		  0x04	//Only valid for DIGIT0
#define S7_COLON	  0x03  //Only valid for DIGIT0
#define S7_ZERO     0x5f
#define S7_ONE      0x06
#define S7_TWO      0x9b
#define S7_THREE    0x8f
#define S7_FOUR     0xc6
#define S7_FIVE     0xcd
#define S7_SIX      0xdd
#define S7_SEVEN    0x47
#define S7_EIGHT    0xdf
#define S7_NINE     0xcf
#define S7_A        0xd7
#define S7_B        0xdc
#define S7_C        0x59
#define S7_D        0x9e
#define S7_E        0xd9
#define S7_F        0xd1
#define SERIAL_HEAD 0x57 //'w'
#define SERIAL_TAIL 0x77 //'W'
#define SERIAL_LENGTH 7
/*********************************************************************/

Adafruit_HTU21DF htu = Adafruit_HTU21DF();

int led = D7;
bool led_state = HIGH;
float current_temp,current_humi;
unsigned char serial_data[SERIAL_LENGTH]  = {SERIAL_HEAD,S7_DEG,S7_SIX,S7_NINE+S7_DP,S7_NINE,S7_C,SERIAL_TAIL};

void setup(){
  Serial.begin(9600);
  Serial.println("HTU21D-F test");

  Serial1.begin(9600);
  for(int i = 0; i<SERIAL_LENGTH;i++){
    Serial1.write(serial_data[i]);
  }

  if (!htu.begin()) {
    Serial.println("Couldn't find sensor!");
    while (1);
  }

  pinMode(led,OUTPUT);
  current_temp = 0.0f;
  current_humi = 0.0f;

  Particle.variable("temperature",current_temp);
  Particle.variable("humidity",current_humi);
}

void serialEvent1(){
  char c = Serial1.read();
  Serial.print(c);
}

void loop(){

  current_temp = htu.readTemperature();
  current_humi = htu.readHumidity();
  Serial.print("Temp: "); Serial.print(current_temp);
  Serial.print("\t\tHum: "); Serial.println(current_humi);
  digitalWrite(led,led_state);
  led_state = !led_state;
  for(int i = 0; i<SERIAL_LENGTH;i++){
    Serial1.write(serial_data[i]);
  }
  delay(1000);
}

/*********************************************************************/

Adafruit_HTU21DF::Adafruit_HTU21DF() {
}

bool Adafruit_HTU21DF::begin(void) {
  Wire.begin();

  reset();

  Wire.beginTransmission(HTU21DF_I2CADDR);
  Wire.write(HTU21DF_READREG);
  Wire.endTransmission();
  Wire.requestFrom(HTU21DF_I2CADDR, 1);
  return (Wire.read() == 0x2); // after reset should be 0x2
}

void Adafruit_HTU21DF::reset(void) {
  Wire.beginTransmission(HTU21DF_I2CADDR);
  Wire.write(HTU21DF_RESET);
  Wire.endTransmission();
  delay(15);
}


float Adafruit_HTU21DF::readTemperature(void) {

  // OK lets ready!
  Wire.beginTransmission(HTU21DF_I2CADDR);
  Wire.write(HTU21DF_READTEMP);
  Wire.endTransmission();

  delay(50); // add delay between request and actual read!

  Wire.requestFrom(HTU21DF_I2CADDR, 3);
  while (!Wire.available()) {}

  uint16_t t = Wire.read();
  t <<= 8;
  t |= Wire.read();

  uint8_t crc = Wire.read();

  float temp = t;
  temp *= 175.72;
  temp /= 65536;
  temp -= 46.85;

  return temp;
}


float Adafruit_HTU21DF::readHumidity(void) {
  // OK lets ready!
  Wire.beginTransmission(HTU21DF_I2CADDR);
  Wire.write(HTU21DF_READHUM);
  Wire.endTransmission();

  delay(50); // add delay between request and actual read!

  Wire.requestFrom(HTU21DF_I2CADDR, 3);
  while (!Wire.available()) {}

  uint16_t h = Wire.read();
  h <<= 8;
  h |= Wire.read();

  uint8_t crc = Wire.read();

  float hum = h;
  hum *= 125;
  hum /= 65536;
  hum -= 6;

  return hum;
}

/*********************************************************************/
