#include "main.h"

Madgwick filter;
MPU9255 mpu;
MPU9250 mpu9250;

// Timer
unsigned long microsPerReading, microsPrevious = 0;

// Magnetic declination for Singapore in degrees +(West)
const float declination = 0.04;

// IMU Sensor
float ax, ay, az = 0;
float gx, gy, gz = 0;
float mx, my, mz = 0;

float roll, pitch, heading, heading2 = 0; 
float currHeading, prevHeading = 0;

float offsetHeading, offsetHeading2 = 0;

uint8_t v1 = 0;
uint8_t v2 = 0;

const int enableIMU = 4;    //First IMU ADO pin to D4
const int enableIMU2 = 6;   //Second IMU ADO pin to D6

// Optical Flow Sensor
Optical_Flow_Sensor flow(10, PAA5100);  //Optical flow sensor Chip Select (CS) to pin D10

const double height_from_gnd = 20;    //Height in mm
const float scaler = 5;               //Adjust for sensitivity for different surfaces

// Sensor FOV is 42 degrees
// height_from_gnd*2*tan(42/2) is dist covered by sensor
// Sensor resolution is 35*35

const double scale_factor = height_from_gnd*2*tan(42/2)/(35*scaler); 

int16_t deltaX,deltaY;          //Pixel changes for successive frames
double distX = 0, distY = 0;    //Change in distance in mm

void setup() {
  Serial.begin(115200);     //Initialize Serial port (USB port)
  Serial1.begin(115200);    //Initialize UART (TX,RX) -> pins D0,D1
  Wire.begin();             //Start I2C for IMU communication
  delay(1500);

  //IMU Setup
  MPU9250Setting setting;
  setting.accel_fs_sel = ACCEL_FS_SEL::A2G;                 //set accel to 2g scale
  setting.gyro_fs_sel = GYRO_FS_SEL::G250DPS;               //set gyro scale to 250deg/sec sens
  setting.mag_output_bits = MAG_OUTPUT_BITS::M16BITS;
  setting.fifo_sample_rate = FIFO_SAMPLE_RATE::SMPL_1000HZ; //1kHz sample rate
  setting.gyro_fchoice = 0x03;
  setting.gyro_dlpf_cfg = GYRO_DLPF_CFG::DLPF_3600HZ;
  setting.accel_fchoice = 0x01;
  setting.accel_dlpf_cfg = ACCEL_DLPF_CFG::DLPF_99HZ;

  digitalWrite(enableIMU, LOW);       //Set sensor ADO pin low for I2C Addr (0x68)
  digitalWrite(enableIMU2, HIGH);     //Otherwise, ADO pin high for I2C Addr (0x69), when using 2 IMUs together

  //First IMU
  if (!mpu9250.setup(0x68))
  {
    while (1) {
        Serial.println("First IMU wrong!");
        delay(1000);
    }
  }

  /*//Second IMU
  digitalWrite(enableIMU, HIGH);
  digitalWrite(enableIMU2, LOW);
  if (!mpu9250.setup(0x68)) {  // change to your own address
      while (1) {
          Serial.println("Second IMU wrong!");
          delay(1000);
      }
  }
  */

  //Madgwick filter, update rate
  filter.begin(11);

  /*//If IMU calibration is required
  Serial.println("Accel/Gyro calibration starts in 5 seconds!");
  Serial.println("Please leave the device still on the flat plane.");
  mpu9250.verbose(true);
  delay(5000);
  mpu9250.calibrateAccelGyro();

  Serial.println("Magneto calibration will start in 5 seconds!");
  Serial.println("Please wave the IMU in a figure eight until done.");
  delay(5000);
  mpu9250.calibrateMag();

  mpu9250.verbose(false);
  */

  /*//If calibration is required
  Serial.println("Accel Gyro calibration will start in 5sec.");
  Serial.println("Please leave the device still on the flat plane.");
  mpu.verbose(true);
  delay(5000);
  mpu.calibrateAccelGyro();

  Serial.println("Mag calibration will start in 5sec.");
  Serial.println("Please Wave device in a figure eight until done.");
  delay(5000);
  mpu.calibrateMag();

  print_calibration();
  mpu.verbose(false);

  //Save to eeprom
  saveCalibration();
  */

  //EEPROM, if output is NaN or all zeros, do calibration
  loadCalibration();
  printCalibration();

  //Start optical flow
  if (!flow.begin()) {
    Serial.println("Initialization of the flow sensor failed");
    while(!flow.begin()){ delay(500); };
  }
  
  delay(50);

  //First IMU heading tare
  while(1){
    digitalWrite(enableIMU, LOW);
    digitalWrite(enableIMU2, HIGH);
    
    mpu.read_acc();
    mpu.read_gyro();
    mpu.read_mag();
  
    //Convert from raw data to gravity and degrees/second units
    ax = process_acceleration(mpu.ax,scale_2g);
    ay = process_acceleration(mpu.ay,scale_2g);
    az = process_acceleration(mpu.az,scale_2g);
    gx = process_angular_velocity(mpu.gx,scale_250dps);
    gy = process_angular_velocity(mpu.gy,scale_250dps);
    gz = process_angular_velocity(mpu.gz,scale_250dps);
    mx = process_magnetic_flux(mpu.mx,mpu.mx_sensitivity);
    my = process_magnetic_flux(mpu.my,mpu.my_sensitivity);
    mz = process_magnetic_flux(mpu.mz,mpu.mz_sensitivity);
  
    //Update the filter, computes orientation
    filter.update(gx, gy, gz, ax, ay, az, mx, my, mz);
    filter.updateIMU(gx, gy, gz, ax, ay, az);
  
    heading = filter.getYaw();
    heading += declination;
    if (heading < 0) {
      heading += 360;
    } else if (heading >= 360) {
      heading -= 360;
    }
    currHeading = heading;
    if(currHeading == prevHeading){
      offsetHeading = currHeading;
      currHeading = 0;
      heading = 0;
      Serial.println("First IMU resetted");
      break;
    }
    else{
      Serial.print("First IMU, not yet reset");
      Serial.println(currHeading);
    }
    prevHeading = currHeading;
  }

  //Second IMU headiing tare
  /*while(1){
    digitalWrite(enableIMU, HIGH);
    digitalWrite(enableIMU2, LOW);
    
    mpu.read_acc();
    mpu.read_gyro();
    mpu.read_mag();
  
    //Convert from raw data to gravity and degrees/second units
    ax = process_acceleration(mpu.ax,scale_2g);
    ay = process_acceleration(mpu.ay,scale_2g);
    az = process_acceleration(mpu.az,scale_2g);
    gx = process_angular_velocity(mpu.gx,scale_250dps);
    gy = process_angular_velocity(mpu.gy,scale_250dps);
    gz = process_angular_velocity(mpu.gz,scale_250dps);
    mx = process_magnetic_flux(mpu.mx,mpu2.mx_sensitivity);
    my = process_magnetic_flux(mpu.my,mpu2.my_sensitivity);
    mz = process_magnetic_flux(mpu.mz,mpu2.mz_sensitivity);
  
    //Update the filter, computes orientation
    filter.update(gx, gy, gz, ax, ay, az, mx, my, mz);
    filter.updateIMU(gx, gy, gz, ax, ay, az);
  
    heading2 = filter.getYaw();
    heading2 += declination;
    if (heading2 < 0) {
      heading2 += 360;
    } else if (heading >= 360) {
      heading2 -= 360;
    }
    currHeading = heading2;
    if(currHeading == prevHeading){
      offsetHeading2 = currHeading;
      currHeading = 0;
      heading2 = 0;
      Serial.println("Second IMU resetted");
      break;
    }
    else{
      Serial.print("Second IMU, not yet reset");
      Serial.println(currHeading);
    }
    prevHeading = currHeading;
  }*/

  //Change division equal to filter update rate
  microsPerReading = 1000000 / 11;
  microsPrevious = micros();
}

void first_imu(){
  mpu.read_acc();
  mpu.read_gyro();
  mpu.read_mag();
  
  //Convert from raw data to gravity and degrees/second units
  ax = process_acceleration(mpu.ax,scale_2g);
  ay = process_acceleration(mpu.ay,scale_2g);
  az = process_acceleration(mpu.az,scale_2g);
  gx = process_angular_velocity(mpu.gx,scale_250dps);
  gy = process_angular_velocity(mpu.gy,scale_250dps);
  gz = process_angular_velocity(mpu.gz,scale_250dps);
  mx = process_magnetic_flux(mpu.mx,mpu.mx_sensitivity);
  my = process_magnetic_flux(mpu.my,mpu.my_sensitivity);
  mz = process_magnetic_flux(mpu.mz,mpu.mz_sensitivity);

  //Update the filter, computes orientation
  filter.update(gx, gy, gz, ax, ay, az, mx, my, mz);
  filter.updateIMU(gx, gy, gz, ax, ay, az);

  heading = filter.getYaw();
  heading += declination;
  if ((heading - offsetHeading) < 0) {
    heading += 360;
  } else if ((heading - offsetHeading) >= 360) {
    heading -= 360;
  }
}

void second_imu(){
  mpu.read_acc();
  mpu.read_gyro();
  mpu.read_mag();
  
  //Convert from raw data to gravity and degrees/second units
  ax = process_acceleration(mpu.ax,scale_2g);
  ay = process_acceleration(mpu.ay,scale_2g);
  az = process_acceleration(mpu.az,scale_2g);
  gx = process_angular_velocity(mpu.gx,scale_250dps);
  gy = process_angular_velocity(mpu.gy,scale_250dps);
  gz = process_angular_velocity(mpu.gz,scale_250dps);
  mx = process_magnetic_flux(mpu.mx,mpu.mx_sensitivity);
  my = process_magnetic_flux(mpu.my,mpu.my_sensitivity);
  mz = process_magnetic_flux(mpu.mz,mpu.mz_sensitivity);

  //Update the filter, computes orientation
  filter.update(gx, gy, gz, ax, ay, az, mx, my, mz);
  filter.updateIMU(gx, gy, gz, ax, ay, az);

  heading2 = filter.getYaw();
  heading2 += declination;
  if ((heading2 - offsetHeading2) < 0) {
    heading2 += 360;
  } else if ((heading2 - offsetHeading2) >= 360) {
    heading2 -= 360;
  }
}

void loop() {
  unsigned long microsNow;
  microsNow = micros();
  
  if(microsNow - microsPrevious >= microsPerReading){
    //First IMU read
    digitalWrite(enableIMU, LOW);
    digitalWrite(enableIMU2, HIGH);

    first_imu();

    //Second IMU read
    /*digitalWrite(enableIMU, HIGH);
    digitalWrite(enableIMU2, LOW);

    second_imu();*/
    
    microsPrevious = microsPrevious + microsPerReading;
  }

  //Update optical flow
  flow.readMotionCount(&deltaX, &deltaY);
  distX += scale_factor*deltaX;
  distY += scale_factor*deltaY;

  Serial.println((heading - offsetHeading));
  Serial1.write('A');
  Serial1.print((heading - offsetHeading));
  Serial1.write('E');
  delay(10);
  delay(15);
  
  Serial1.write('X');
  Serial1.print(distX);
  Serial1.write('C');
  delay(10);
  delay(15);
  
  Serial1.write('Y');
  Serial1.print(distY);
  Serial1.write('D');
  delay(10);
  delay(15);
  
  v1 = Serial1.read();
  v2 = Serial1.read();
  
  if(v1 == 'W' && v2 == 'B'){
    reset_IMU();
    deltaX = 0;
    deltaY = 0;
    distY = 0;
    distX = 0;
    v1 = 'E';
    v2 = 'R';
    Serial1.flush();
    delay(10);
  }

  /*//Check readings
  Serial.print("aX:"); Serial.print(ax);
  Serial.print(",aY:"); Serial.print(ay);
  Serial.print(",aZ:"); Serial.print(az);
  Serial.print(",gX:"); Serial.print(roll);
  Serial.print(",gY:"); Serial.print(pitch);
  Serial.print(",gZ:"); Serial.println(heading);*/
}

void reset_IMU(){
  MPU9250Setting setting;
  setting.accel_fs_sel = ACCEL_FS_SEL::A2G;                   //set accel to 2g scale
  setting.gyro_fs_sel = GYRO_FS_SEL::G250DPS;                 //set gyro scale to 250deg/sec sens
  setting.mag_output_bits = MAG_OUTPUT_BITS::M16BITS;
  setting.fifo_sample_rate = FIFO_SAMPLE_RATE::SMPL_1000HZ;   //1kHz sample rate
  setting.gyro_fchoice = 0x03;
  setting.gyro_dlpf_cfg = GYRO_DLPF_CFG::DLPF_3600HZ;
  setting.accel_fchoice = 0x01;
  setting.accel_dlpf_cfg = ACCEL_DLPF_CFG::DLPF_99HZ;

  //First IMU check
  digitalWrite(enableIMU, LOW);
  digitalWrite(enableIMU2, HIGH);
  
  if (!mpu9250.setup(0x68)) {  // change to your own address
      while (1) {
          Serial.println("First IMU Wrong!");
          delay(1000);
      }
  }

  /*//Second IMU check
  digitalWrite(enableIMU, HIGH);
  digitalWrite(enableIMU2, LOW);
  
  if (!mpu9250.setup(0x68)) {  // change to your own address
      while (1) {
          Serial.println("Second IMU Wrong!");
          delay(1000);
      }
  }*/
  
  filter.begin(11);
  loadCalibration();

  //First IMU tare
  while(1){
    digitalWrite(enableIMU, LOW);
    digitalWrite(enableIMU2, HIGH);
    
    mpu.read_acc();
    mpu.read_gyro();
    mpu.read_mag();
  
    // convert from raw data to gravity and degrees/second units
    ax = process_acceleration(mpu.ax,scale_2g);
    ay = process_acceleration(mpu.ay,scale_2g);
    az = process_acceleration(mpu.az,scale_2g);
    gx = process_angular_velocity(mpu.gx,scale_250dps);
    gy = process_angular_velocity(mpu.gy,scale_250dps);
    gz = process_angular_velocity(mpu.gz,scale_250dps);
    mx = process_magnetic_flux(mpu.mx,mpu.mx_sensitivity);
    my = process_magnetic_flux(mpu.my,mpu.my_sensitivity);
    mz = process_magnetic_flux(mpu.mz,mpu.mz_sensitivity);
  
    // update the filter, which computes orientation
    filter.update(gx, gy, gz, ax, ay, az, mx, my, mz);
    filter.updateIMU(gx, gy, gz, ax, ay, az);
  
    heading = filter.getYaw();
    heading += declination;
    if (heading < 0) {
      heading += 360;
    } else if (heading >= 360) {
      heading -= 360;
    }
    currHeading = heading;
    if(currHeading == prevHeading){
      offsetHeading = currHeading;
      currHeading = 0;
      heading = 0;
      Serial.println("First IMU, not reset yet");
      break;
    }
    else{
      Serial.print("First IMU, not reset yet");
    }
    prevHeading = currHeading;
  }

  //Second IMU tare
  /*while(1){
    digitalWrite(enableIMU, HIGH);
    digitalWrite(enableIMU2, LOW);
    
    mpu.read_acc();
    mpu.read_gyro();
    mpu.read_mag();
  
    // convert from raw data to gravity and degrees/second units
    ax = process_acceleration(mpu.ax,scale_2g);
    ay = process_acceleration(mpu.ay,scale_2g);
    az = process_acceleration(mpu.az,scale_2g);
    gx = process_angular_velocity(mpu.gx,scale_250dps);
    gy = process_angular_velocity(mpu.gy,scale_250dps);
    gz = process_angular_velocity(mpu.gz,scale_250dps);
    mx = process_magnetic_flux(mpu.mx,mpu.mx_sensitivity);
    my = process_magnetic_flux(mpu.my,mpu.my_sensitivity);
    mz = process_magnetic_flux(mpu.mz,mpu.mz_sensitivity);
  
    // update the filter, which computes orientation
    filter.update(gx, gy, gz, ax, ay, az, mx, my, mz);
    filter.updateIMU(gx, gy, gz, ax, ay, az);
  
    heading2 = filter.getYaw();
    heading2 += declination;
    if (heading2 < 0) {
      heading2 += 360;
    } else if (heading2 >= 360) {
      heading2 -= 360;
    }
    currHeading = heading2;
    if(currHeading == prevHeading){
      offsetHeading2 = currHeading;
      currHeading = 0;
      heading2 = 0;
      Serial.println("Second IMU, not reset yet");
      break;
    }
    else{
      Serial.print("Second IMU, not reset yet");
    }
    prevHeading = currHeading;
  }*/
  
  delay(50);
  microsPrevious = micros();
}
