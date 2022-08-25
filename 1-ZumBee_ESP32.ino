#include <PID_v1.h>
#include <OPT3101.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include "BluetoothSerial.h"
#include "DeteccionOPT.h"
#include "doEncode.h"
#include "leerBT.h"
#include "monitorBat.h"
#include "posicionBNO.h"
#include "xyAdiferencial.h"
BluetoothSerial SerialBT;
OPT3101 sensor;

#define IntPin_A   39  //  encoder L, 1000 pasos son 180mm
#define IntPin_B   36
#define IntPin_C   34  //  encoder R
#define IntPin_D   35
#define IntPin_OPT 12
#define IntPin_BNO 13

#define IZQ_AVZ      33
#define IZQ_RET      32
#define IZQ_PWM      27

#define DER_AVZ      25
#define DER_RET      26
#define DER_PWM      14

const int DER_PWM_Ch = 0;
const int IZQ_PWM_Ch = 2;
const int PWM_Res = 10;
const int PWM_Freq = 20000;

//Variables para conectar PID
double SetpointL, counter_L, OutputL;
double SetpointR, counter_R, OutputR;

//Constructor de PID y valores iniciales
double Kp = 2.6, Ki = 0.8, Kd = 0.4;
PID myPID_L(&counter_L, &OutputL, &SetpointL, KpL, KiL, KdL, DIRECT);
PID myPID_R(&counter_R, &OutputR, &SetpointR, KpR, KiR, KdR, DIRECT);

// Valores para encoder e ISR
void ICACHE_RAM_ATTR doEncodeA();
void ICACHE_RAM_ATTR doEncodeB();
void ICACHE_RAM_ATTR doEncodeC();
void ICACHE_RAM_ATTR doEncodeD();

const int timeThreshold = 5;
long timeCounter = 0;

volatile long ISRCounter_L = 0;
bool IsCW_L = true;

volatile long ISRCounter_R = 0;
bool IsCW_R = true;

//  Variables modo, velocidad y JOYSTICK
int mode;
char BluetoothData;
int pad_x, pad_y = 512;
int Ispeed, Dspeed = 0;

//Arrays amplitud y distancia OPT3101
uint16_t amplitudes[3];
int16_t distances[3];
volatile bool dataReady = false;
void ICACHE_RAM_ATTR setDataReadyFlag()
{
  dataReady = true;
}

// Constantes para trasladar los datos en bruto del joystick pad_x pad_y
// a velocidades en cada rueda RSpeed LSpeed
const int MIN_RAW_ADC = 0;
const int MAX_RAW_ADC = 1023;
const int MAX_SPEED_SETTING =  1023;
const int MIN_SPEED_SETTING = -MAX_SPEED_SETTING;
const int MAX_TURN_DELTA =  512; // velocidad maxima [in/de]crease a girar; Left
const int MIN_TURN_DELTA = -MAX_TURN_DELTA; // Right

//Variables BNO055
double xPos = 0, yPos = 0, headingVel = 0;
uint16_t BNO055_SAMPLERATE_DELAY_MS = 10; //how often to read data from the board
uint16_t PRINT_DELAY_MS = 500; // how often to print the data
uint16_t printCount = 0; //counter to avoid printing every 10MS sample
int posActual, rumbo;

//velocity = accel*dt (dt in seconds)
//position = 0.5*accel*dt^2
double ACCEL_VEL_TRANSITION =  (double)(BNO055_SAMPLERATE_DELAY_MS) / 1000.0;
double ACCEL_POS_TRANSITION = 0.5 * ACCEL_VEL_TRANSITION * ACCEL_VEL_TRANSITION;
double DEG_2_RAD = 0.01745329251; //trig functions require radians, BNO055 outputs degrees
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

void setup() {

  myPID_L.SetMode(AUTOMATIC);
  myPID_R.SetMode(AUTOMATIC);

  Serial.begin(115200);   //Serial para debug en el PC
  SerialBT.begin("ZumBeeESP");

  pinMode(IntPin_A, INPUT_PULLUP);
  pinMode(IntPin_B, INPUT_PULLUP);
  pinMode(IntPin_C, INPUT_PULLUP);
  pinMode(IntPin_D, INPUT_PULLUP);
  pinMode (IZQ_PWM, OUTPUT);
  pinMode (IZQ_AVZ, OUTPUT);
  pinMode (IZQ_RET, OUTPUT);
  pinMode (DER_PWM, OUTPUT);
  pinMode (DER_AVZ, OUTPUT);
  pinMode (DER_RET, OUTPUT);

  attachInterrupt(IntPin_A, doEncodeA, CHANGE);
  attachInterrupt(IntPin_B, doEncodeB, CHANGE);
  attachInterrupt(IntPin_C, doEncodeC, CHANGE);
  attachInterrupt(IntPin_D, doEncodeD, CHANGE);

  ledcSetup(IZQ_PWM_Ch, PWM_Freq, PWM_Res);
  ledcSetup(DER_PWM_Ch, PWM_Freq, PWM_Res);

  ledcAttachPin(IZQ_PWM, IZQ_PWM_Ch);
  ledcAttachPin(DER_PWM, DER_PWM_Ch);

  digitalWrite(IZQ_AVZ, LOW);
  digitalWrite(IZQ_RET, LOW);

  digitalWrite(DER_AVZ, LOW);
  digitalWrite(DER_RET, LOW);

  Wire.begin();

  if (!bno.begin())
  {
    Serial.print("IMU no detectada");
  }

  sensor.init();
  if (sensor.getLastError())
  {
    Serial.print(F("Fallo al iniciar OPT3101: error "));
    Serial.println(sensor.getLastError());
    while (1) ;
  }
  sensor.setContinuousMode();
  sensor.enableDataReadyOutput(1);
  sensor.setFrameTiming(32);
  sensor.setChannel(OPT3101ChannelAutoSwitch);
  sensor.setBrightness(OPT3101Brightness::Adaptive);

  attachInterrupt(digitalPinToInterrupt(IntPin_OPT), setDataReadyFlag, RISING);
  sensor.enableTimingGenerator();
}

void loop() {


lecturasEncoders ();


 // Serial.print (" L "); Serial.print (counter_L); Serial.print (", R  "); Serial.println (counter_R);
  debugSerial ();
  
  // esquivaObstaculos ();
  // alcanzarPosicion (180);

  leerBT();
  switch (mode) {
    case 1:
      xyAdiferencial();
      break;
    case 2:
      esquivaObstaculos ();
      break;
    case 3:
      alcanzarPosicion (90);
      break;
    default:
      paroMotores ();
      break;
  }

}

void lecturasEncoders () {
  if (counter_L != ISRCounter_L || counter_R != ISRCounter_R)
  {
    counter_L = ISRCounter_L; 
    counter_R = ISRCounter_R;
  }
}

void debugSerial () {
  unsigned long tStart = micros();
  sensors_event_t orientationData , linearAccelData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  //  bno.getEvent(&angVelData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);

  xPos = xPos + ACCEL_POS_TRANSITION * linearAccelData.acceleration.x;
  yPos = yPos + ACCEL_POS_TRANSITION * linearAccelData.acceleration.y;


  // velocity of sensor in the direction it's facing
  headingVel = ACCEL_VEL_TRANSITION * linearAccelData.acceleration.x / cos(DEG_2_RAD * orientationData.orientation.x);
  if (printCount * BNO055_SAMPLERATE_DELAY_MS >= PRINT_DELAY_MS) {
    SerialBT.print("Heading: ");
    SerialBT.println(orientationData.orientation.x);
    //  Serial.print("Position: ");
    //  Serial.print(xPos);
    //  Serial.print(" , ");
    //  Serial.println(yPos);
    //  Serial.print("Speed: ");
    //  Serial.println(headingVel);

    SerialBT.print (" L "); SerialBT.print (counter_L); SerialBT.print (", R  "); SerialBT.println (counter_R);
    if (dataReady)
    {
      sensor.readOutputRegs();
      distances[sensor.channelUsed] = sensor.distanceMillimeters;
      SerialBT.print("D IZQ "); SerialBT.print(distances[0]); SerialBT.print(", D Cnt "); SerialBT.print(distances[1]); SerialBT.print(", D DCH "); SerialBT.println(distances[2]);
      SerialBT.println("-------");
    }
    printCount = 0;
    dataReady = false;
  }
  else {
    printCount = printCount + 1;
  }
}
