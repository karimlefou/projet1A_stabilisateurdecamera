#include <SimpleFOC.h>
#include <Wire.h>
#include <SparkFunLSM9DS1.h>
#include <math.h>

// Déclaration du capteur AS5048B (I2C)
#define AS5048B_I2C_ADDRESS 0x41

#define LSM9DS1_M   0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG  0x6B // Would be 0x6A if SDO_AG is LOW

// Déclaration de l'accéléromètre LSM9DS1
LSM9DS1 imu;

// Déclaration du moteur brushless
BLDCMotor motor = BLDCMotor(11, 5.5); // 7 paires de pôles
BLDCDriver3PWM driver = BLDCDriver3PWM(9, 5, 6, 8); // Broches PWM et enable

// Angle cible à suivre
float target_angle = 0;
float u;
float eps;
float w;
float K=0.00001;

/*float epsa;
float pk;
float ik;
float wa;
*/
//target variable
float target_velocity = 5;
// instantiate the commander
Commander command = Commander(Serial);

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Initialisation de l'accéléromètre LSM9DS1
  if (!imu.begin()) {
    Serial.println("Échec de l'initialisation du LSM9DS1 !");
    while (1);
  }

  // velocity PID controller parameters
  // default P=0.5 I = 10 D =0
  motor.PID_velocity.P = 0.00001;
  motor.PID_velocity.I = 20;
  motor.PID_velocity.D = 0.001;
  // jerk control using voltage voltage ramp
  // default value is 300 volts per sec  ~ 0.3V per millisecond
  motor.PID_velocity.output_ramp = 1000;

  // velocity low pass filtering
  // default 5ms - try different values to see what is the best. 
  // the lower the less filtered
  motor.LPF_velocity.Tf = 0.01;

  // angle PID controller 
  // default P=20
  motor.P_angle.P = 25; 
  motor.P_angle.I = 0;  // usually only P controller is enough 
  motor.P_angle.D = 0;  // usually only P controller is enough 
  // acceleration control using output ramp
  // this variable is in rad/s^2 and sets the limit of acceleration
  motor.P_angle.output_ramp = 10000; // default 1e6 rad/s^2

  // angle low pass filtering
  // default 0 - disabled  
  // use only for very noisy position sensors - try to avoid and keep the values very small
  motor.LPF_angle.Tf = 0; // default 0

  // setting the limits
  //  maximal velocity of the position control
  motor.velocity_limit = 4; // rad/s - default 20

  // Initialisation du driver
  driver.voltage_power_supply = 12; // Alimentation du driver
  driver.init();
  motor.linkDriver(&driver);
  driver.voltage_limit = 4;
  // Configuration du moteur
  motor.voltage_limit = 3;
  //motor.current_limit = 2; // Amps
  motor.velocity_limit = 40;
  motor.controller = MotionControlType::velocity_openloop;

  // Initialisation du moteur et du FOC
  motor.init();
  if (motor.initFOC() == 0) {
    Serial.println("Échec de l'initialisation du FOC !");
  } else {
    Serial.println("FOC initialisé avec succès !");
  }
}

void loop() {
  motor.loopFOC();

  // Lecture de l'angle brut du capteur AS5048B
  Wire.beginTransmission(AS5048B_I2C_ADDRESS);
  Wire.write(0xFE);
  Wire.endTransmission(false);
  Wire.requestFrom(AS5048B_I2C_ADDRESS, 2);

  // Lecture des données de l'accéléromètre LSM9DS1
  if (imu.accelAvailable()) {
    imu.readAccel();
    Serial.print("AccX: "); Serial.print(imu.ax);
    Serial.print(" | AccY: "); Serial.print(imu.ay);
    Serial.print(" | AccZ: "); Serial.println(imu.az);
    
    eps = target_angle - imu.ax;
    w = K * eps;
    if(w>1){
      u = 1;
    }
    else if(w<-1){
      u = -1;
    }
    else{
      u = w;
    }
    motor.move(u*target_velocity);
    command.run();
    //target_angle+=5;
    //target_angle = fmod(target_angle, 360.0);

    //epsa = gx;
    //pk = K * epsa;
    //ik = ik + K * 50e-6/50 * epsa;
    //wa = pk + ik;
    //if(wa>1){
    //  u = 1;
    //}
    //else if(w<-1){
    //  u = -1;
    //}
    //else{
    //  u = w;
    //}
  }
}