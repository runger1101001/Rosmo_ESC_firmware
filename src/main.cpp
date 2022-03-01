
#include <Arduino.h>
#include "SimpleFOC.h"
#include "SimpleFOCDrivers.h"

#include "./hw_setup.h"
#include "./NeoPixelStatus.h"
#include "encoders/as5048a/MagneticSensorAS5048A.h"



#define SERIAL_SPEED 115200
#define FIRMWARE_VERSION "0.1"

#define DEFAULT_VOLTAGE_LIMIT 6.0f


#define MOTOR_PP 7
#define MOTOR_RES 12.6f



// define SPI instances
SPIClass SPI_Sensor0 = SPIClass(COPI0_PIN, CIPO0_PIN, SCLK0_PIN);
SPIClass SPI_Sensor1 = SPIClass(COPI1_PIN, CIPO1_PIN, SCLK1_PIN);
SPIClass SPI_MBus = SPIClass(COPI2_PIN, CIPO2_PIN, SCLK2_PIN);
SPISettings AS5048SlowSPISettings(2000000, AS5048_BITORDER, SPI_MODE1); 

// define I2C instances
// TODO

// Status LED
NeoPixelStatus statusLED = NeoPixelStatus(STATUS_LED_PIN, 1);

// motor 0
MagneticSensorAS5048A* sensor0 = new MagneticSensorAS5048A(SENSOR0_nCS_PIN, true, AS5048SlowSPISettings);
BLDCDriver6PWM driver0 = BLDCDriver6PWM(M0_INUH_PIN, M0_INUL_PIN, M0_INVH_PIN, M0_INVL_PIN, M0_INWH_PIN, M0_INWL_PIN, M0_nSLEEP_PIN);
BLDCMotor motor0 = BLDCMotor(MOTOR_PP, MOTOR_RES);

// motor 1
MagneticSensorAS5048A* sensor1 = new MagneticSensorAS5048A(SENSOR1_nCS_PIN, true, AS5048SlowSPISettings);
BLDCDriver6PWM driver1 = BLDCDriver6PWM(M1_INUH_PIN, M1_INUL_PIN, M1_INVH_PIN, M1_INVL_PIN, M1_INWH_PIN, M1_INWL_PIN, M1_nSLEEP_PIN);
BLDCMotor motor1 = BLDCMotor(MOTOR_PP, MOTOR_RES);

// voltage
float vbus = 6.0f;

// main loop speed tracking
int count = 0;
long ts = 0;




void setup() {
  // initialize serial port on USB, debug output goes to this
  Serial.begin(SERIAL_SPEED);
  SimpleFOCDebug::enable();
  while (!Serial) ; // wait for serial port to connect - remove this later
  SimpleFOCDebug::print("Welcome to RosmoESC, v");
  SimpleFOCDebug::println(FIRMWARE_VERSION);  
  
  // set status LED
  statusLED.init();
  delayMicroseconds(250);
  statusLED.setStatus(SimpleFOCStatusIndication::SIMPLEFOC_INITIALIZING);
  delayMicroseconds(250);
  statusLED.setStatus(SimpleFOCStatusIndication::SIMPLEFOC_INITIALIZING);

  // configure voltage sensing

  // configure motor drivers, 6-PWM
  SimpleFOCDebug::println("Initializing drivers...");
  driver0.voltage_power_supply = vbus;
  driver1.voltage_power_supply = vbus;
  driver0.voltage_limit = DEFAULT_VOLTAGE_LIMIT;
  driver1.voltage_limit = DEFAULT_VOLTAGE_LIMIT;
  if (!driver0.init())
    SimpleFOCDebug::println("Driver 0 init failed!");
  if (!driver1.init())
    SimpleFOCDebug::println("Driver 1 init failed!");

  // configure current sensing

  // configure SPI for sensors, defer initialization
  SimpleFOCDebug::println("Initializing sensors...");
  sensor0->init(&SPI_Sensor0); // TODO make configurable and defer initialization
  sensor1->init(&SPI_Sensor1);

  // configure BLDC motors, defer initialization
  SimpleFOCDebug::println("Initializing motors...");
  motor0.linkSensor(sensor0);
  motor0.linkDriver(&driver0);
  motor1.linkSensor(sensor1);
  motor1.linkDriver(&driver1);

  motor0.voltage_limit = motor1.voltage_limit = DEFAULT_VOLTAGE_LIMIT;
  motor0.velocity_limit = motor1.velocity_limit = 200.0f; // 200rad/s is pretty fast
  motor0.PID_velocity.P = motor1.PID_velocity.P = 0.2f;
  motor0.PID_velocity.I = motor1.PID_velocity.I = 2.0f;
  motor0.PID_velocity.D = motor1.PID_velocity.D = 0.0f;
  motor0.PID_velocity.output_ramp = motor1.PID_velocity.output_ramp = 1000.0f;
  motor0.PID_velocity.limit = motor1.PID_velocity.limit = DEFAULT_VOLTAGE_LIMIT; // TODO check this
  motor0.P_angle.P = motor1.P_angle.P = 20.0f;
  motor0.LPF_velocity.Tf = motor1.LPF_velocity.Tf = 0.001f;
  motor0.foc_modulation = motor1.foc_modulation = FOCModulationType::SinePWM;
  motor0.controller = motor1.controller = MotionControlType::velocity;
  motor0.motion_downsample = motor1.motion_downsample = 4;
  motor0.torque_controller = motor1.torque_controller = TorqueControlType::voltage;
  motor0.target = motor1.target = 1.0f;

  if (driver0.initialized) {
    motor0.init();
    if (motor0.motor_status==FOCMotorStatus::motor_uncalibrated) {
      if (sensor0 != NULL)
        motor0.initFOC();
      SimpleFOCDebug::println("Motor 0 intialized.");
    }
    else
      SimpleFOCDebug::println("Motor 0 init failed!");
  }
  else
    SimpleFOCDebug::println("Motor 0 not initialized.");

  if (driver1.initialized) {
    motor1.init();
    if (motor1.motor_status==FOCMotorStatus::motor_uncalibrated) {
      if (sensor1 != NULL)
        motor1.initFOC();
      SimpleFOCDebug::println("Motor 1 intialized.");
    }
    else
      SimpleFOCDebug::println("Motor 1 init failed!");
  }
  else
    SimpleFOCDebug::println("Motor 1 not initialized.");

  // initialize I2C

  // intialize I2CCommander

  // initialize SerialCommmander

  SimpleFOCDebug::println("Startup complete.");
  ts = millis();  

}







bool setupComplete() {
  // check if hardware is initialized
  if (motor0.motor_status==FOCMotorStatus::motor_uncalibrated || motor0.motor_status==FOCMotorStatus::motor_ready)
    return true;
  if (motor1.motor_status==FOCMotorStatus::motor_uncalibrated || motor1.motor_status==FOCMotorStatus::motor_ready)
    return true;
  return false;
}






void mainLoop() {
  while (true) {
    if (motor0.motor_status==FOCMotorStatus::motor_uncalibrated || motor0.motor_status==FOCMotorStatus::motor_ready)
      motor0.move();
    if (motor1.motor_status==FOCMotorStatus::motor_uncalibrated || motor1.motor_status==FOCMotorStatus::motor_ready)
      motor1.move();
    if (motor0.motor_status==FOCMotorStatus::motor_ready)
      motor0.loopFOC();
    if (motor1.motor_status==FOCMotorStatus::motor_ready)
      motor1.loopFOC();

    count++;  // increment iteration counter
    if (millis() - ts > 1000) {
      ts = millis();
      Serial.print("Iteration/s: ");
      Serial.println(count);
      count = 0;
    }

    // // update voltage sense
    // voltageSense.update();
    // // run commander
    // commander.run();
    // monitoring.run();
  }
}




void loop() {

  if (setupComplete()) {
    mainLoop();
  }
  // update sensors if initialized
  if (sensor0!=NULL)
    sensor0->update();
  if (sensor1!=NULL)
    sensor1->update();
  // // update voltage sense
  // voltageSense.update();
  // // update current sense
  // currentSense.update();
  // // run commander
  // commander.run();
  // monitoring.run();

  count++;
  if (millis() - ts > 1000) {
    ts = millis();
    Serial.print("Iteration/s: ");
    Serial.println(count);
    Serial.print("Angles: ");
    Serial.print(sensor0->getAngle());
    Serial.print(", ");
    Serial.println(sensor1->getAngle());
    count = 0;
  }

  delayMicroseconds(500); // 2kHz will be enough for setup-time

}