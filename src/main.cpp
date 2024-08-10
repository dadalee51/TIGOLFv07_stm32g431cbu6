#include <SimpleFOC.h>
/**
 * This is specifically for TIGOLFv07 stm32g431cbu6
 * Date: Aug 9th 2024
 * 
*/
// MA
TwoWire i2c1 = TwoWire(PB7, PA15); // sda, scl
MagneticSensorI2C sensorA = MagneticSensorI2C(AS5600_I2C);
BLDCMotor motorA = BLDCMotor(7);
BLDCDriver3PWM driverA = BLDCDriver3PWM(PA8, PA9, PA10, PB13);
// MB
TwoWire i2c2 = TwoWire(PF0, PC4); // sda2, scl2
MagneticSensorI2C sensorB = MagneticSensorI2C(AS5600_I2C);
BLDCMotor motorB = BLDCMotor(7);
BLDCDriver3PWM driverB = BLDCDriver3PWM(PC6, PA14, PB9, PC13);


float target_angle = 0;
float target_velocity = 10;
HardwareSerial hs(PC11, PC10);
void setup()
{

//init RGB
  //analogWrite(PB10, 0xf4);
  //analogWrite(PB15, 0x54);
  //analogWrite(PA13, 0x03);

  // MA
  sensorA.init(&i2c2);
  motorA.linkSensor(&sensorA);
  driverA.voltage_power_supply = 8;
  driverA.init();
  motorA.linkDriver(&driverA);
  motorA.foc_modulation = FOCModulationType::SpaceVectorPWM;
  // motor.controller = MotionControlType::angle;
  motorA.controller = MotionControlType::velocity;
  motorA.PID_velocity.P = 0.2f;
  motorA.PID_velocity.I = 0.1f;
  motorA.PID_velocity.D = 0;
  motorA.voltage_limit = 8;
  motorA.LPF_velocity.Tf = 0.01f;
  motorA.P_angle.P = 20;
  motorA.velocity_limit = 100;
  motorA.PID_velocity.output_ramp = 1000;

  // MB
  sensorB.init(&i2c1);
  motorB.linkSensor(&sensorB);
  driverB.voltage_power_supply = 8;
  driverB.init();
  motorB.linkDriver(&driverB);
  motorB.foc_modulation = FOCModulationType::SpaceVectorPWM;
  // motorB.controller = MotionControlType::angle;
  motorB.controller = MotionControlType::velocity;
  motorB.PID_velocity.P = 0.2f;
  motorB.PID_velocity.I = 0.1f;
  motorB.PID_velocity.D = 0;
  motorB.voltage_limit = 8;
  motorB.LPF_velocity.Tf = 0.01f;
  motorB.P_angle.P = 20;
  motorB.velocity_limit = 100;
  motorB.PID_velocity.output_ramp = 1000;


  // init and FOC area
  motorA.init();
  delay(100);
  motorB.init();
  delay(100);
  
  motorA.initFOC();
  delay(100);
  motorB.initFOC();
  delay(100);
  // initialize sensors at PB14, PB12, PB11,PB1,  ir LED @PA_12
  pinMode(PA0, INPUT_ANALOG); // A1
  pinMode(PA1, INPUT_ANALOG);
  pinMode(PA2, INPUT_ANALOG);
  pinMode(PA3, INPUT_ANALOG);
  pinMode(PA4, INPUT_ANALOG);
  pinMode(PB0, INPUT_ANALOG);
  pinMode(PB1, INPUT_ANALOG); // Sen1 /A7
  pinMode(PB2, INPUT_ANALOG); // Sen2 /A8
  pinMode(PC13, OUTPUT);
  digitalWrite(PC13, 1);
   pinMode(PB10, OUTPUT_OPEN_DRAIN); //R
   pinMode(PB15, OUTPUT_OPEN_DRAIN); //G
  // pinMode(PA13, OUTPUT_OPEN_DRAIN); //B
  pinMode(PC15, OUTPUT); // IRled GP2
  pinMode(PC14, OUTPUT); // IRled GP1 SENsled 1, 2
  
  //init IR led 1
  digitalWrite(PA4, 1);
  hs.begin(115200);
  hs.println("LFv7---");
  motorB.useMonitoring(hs);
  _delay(100);


}
long a1 = 0;
long a2 = 0;
long a3 = 0;
long a4 = 0;
int counter_g = 0;
int g_switch = 0;
void loop()
{

  motorA.loopFOC();
  motorB.loopFOC();

  motorA.move(target_velocity);
  motorB.move(target_velocity);
  counter_g++;
  if(counter_g> 100){
    g_switch=~g_switch;
    digitalWrite(PB15, g_switch);
    counter_g=0;
  }
  
  motorB.monitor();
  
  //delay(500);
  // hs.println("hello");

}