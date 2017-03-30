#include <Wire.h>
#include "Math3D.h"

//#define SHIFT 256
#define SHIFT 1

#define Serial SerialUSB
 
#define    MPU9250_ADDRESS            0x68
#define    MAG_ADDRESS                0x0C
 
#define    GYRO_FULL_SCALE_250_DPS    0x00  
#define    GYRO_FULL_SCALE_500_DPS    0x08
#define    GYRO_FULL_SCALE_1000_DPS   0x10
#define    GYRO_FULL_SCALE_2000_DPS   0x18
 
#define    ACC_FULL_SCALE_2_G        0x00  
#define    ACC_FULL_SCALE_4_G        0x08
#define    ACC_FULL_SCALE_8_G        0x10
#define    ACC_FULL_SCALE_16_G       0x18
 
 
 
// This function read Nbytes bytes from I2C device at address Address. 
// Put read bytes starting at register Register in the Data array. 
void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.endTransmission();
 
  // Read Nbytes
  Wire.requestFrom(Address, Nbytes); 
  uint8_t index=0;
  while (Wire.available())
    Data[index++]=Wire.read();
}
 
 
// Write a byte (Data) in device (Address) at register (Register)
void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.write(Data);
  Wire.endTransmission();
}

uint32_t last_t = micros();
 
// Initializations
void setup()
{
  // Arduino initializations

  while(!Serial){}
  
  Wire.begin();
  Serial.begin(115200);

  delay(100);
 
  // Configure gyroscope range
  I2CwriteByte(MPU9250_ADDRESS,27,GYRO_FULL_SCALE_2000_DPS);
  // Configure accelerometers range
  I2CwriteByte(MPU9250_ADDRESS,28,ACC_FULL_SCALE_4_G);
  // Set by pass mode for the magnetometers
  I2CwriteByte(MPU9250_ADDRESS,0x37,0x02);
 
  // Request first magnetometer single measurement
  I2CwriteByte(MAG_ADDRESS,0x0A,0x01);
 
 
}
 
 
long int cpt=0;
//const char* filename = "datalogger.txt";
//auto fp = fopen(filename, "w");
float xvel = 0, yvel = 0, xpos = 0, ypos = 0;
int xdeg = 0, ydeg = 0, zdeg = 0;
Quat testQuat;


Quat AttitudeEstimateQuat;

Vec3 correction_Body, correction_World;
Vec3 Accel_Body, Accel_World;
Vec3 GyroVec;

const Vec3 VERTICAL = Vector(0.0f, 0.0f, 1.0f);  // vertical vector in the World frame
float axg = 0, ayg = 0, azg = 0;

// Main loop, read and display data
void loop()
{
  uint32_t t = micros();
  uint32_t dt = t-last_t;
  last_t = t;
  // _______________
  // ::: Counter :::
 
  // Display data counter
  //if(cpt % 100 == 0){
  //Serial.print (cpt,DEC);
  //Serial.print ("\t");
  //}
  
 /*if(cpt < 1000){
    fprintf(fp,"%d",cpt);
    fputc('\t', fp);

 }*/
   cpt++;

 
  // ____________________________________
  // :::  accelerometer and gyroscope ::: 
 
  // Read accelerometer and gyroscope
  uint8_t Buf[14];
  I2Cread(MPU9250_ADDRESS,0x3B,14,Buf);
 
 
  // Create 16 bits values from 8 bits data
 
  // Accelerometer
  int16_t ax=(-(Buf[0]<<8 | Buf[1]));
  int16_t ay=(-(Buf[2]<<8 | Buf[3]));
  int16_t az=(Buf[4]<<8 | Buf[5]);

  axg = (float(ax) / (8192));
  ayg= (float(ay) / (8192));
  azg = (float(az) / (8192));
 
  // Gyroscope
  int16_t gx=-(Buf[8]<<8 | Buf[9]);
  int16_t gy=-(Buf[10]<<8 | Buf[11]);
  int16_t gz=Buf[12]<<8 | Buf[13];

  
 // This section of code correctly positions and update the unit quaternion for orientation  
  

    GyroVec  = Vector((gx / 180)* PI, (gy / 180)* PI, (gz / 180)* PI);  // move gyro data to vector structure
    Accel_Body = Vector(axg, ayg, azg);  // move accel data to vector structure

    Accel_World = Rotate(AttitudeEstimateQuat, Accel_Body); // rotate accel from body frame to world frame

    correction_World = CrossProd(Accel_World, VERTICAL); // cross product to determine error

    Vec3 correction_Body = Rotate(correction_World, AttitudeEstimateQuat); // rotate correction vector to body frame

    GyroVec = Sum(GyroVec, correction_Body);  // add correction vector to gyro data

    Quat incrementalRotation = Quaternion(GyroVec, dt);  // create incremental rotation quat

    AttitudeEstimateQuat = Mul(incrementalRotation, AttitudeEstimateQuat);  // quaternion integration (rotation composting through multiplication)

  
  // Accelerometer

  //Quat changeQuat = Quaternion(Vector((gx / 180)* PI, (gy / 180)* PI, (gz / 180)* PI), dt);
  //testQuat = Mul(changeQuat, testQuat);
  auto testRotate = Rotate(AttitudeEstimateQuat, Vector(axg,ayg,azg));
  
  
   
    // Display values
    /*
  float del_xvel = axg * (dt*1e-6);
  xvel += del_xvel;
  float del_yvel = ayg * (dt*1e-6);
  yvel += del_yvel;

  float del_xpos = xvel * (dt*1e-6);
  xpos += del_xpos;
  float del_ypos = yvel * (dt*1e-6);
  ypos += del_ypos;
  */
  //int32_t del_xpos = xvel * dt*1e-6;
  //xpos += del_xpos;
  //int32_t del_ypos = yvel *dt*1e-6;
  //ypos += del_ypos;
  
  
  if(cpt % 50 == 0){
    //Serial.print (ax); 
    //Serial.print ("\t");
    //Serial.print (ay);
    //Serial.print ("\t");
    //Serial.print (az);
    //Serial.print ("\t");
    Serial.print (axg); 
    Serial.print ("\t");
    Serial.print (ayg);
    Serial.print ("\t");
    //Serial.print (abs(testRotate.z) - 1);
    //Serial.print ("\t");
    
    //Serial.print (xpos / SHIFT,DEC);

    
    
  //Serial.print ("\t");
  //Serial.print (az,DEC);  
  //Serial.print ("\t");
    Serial.println("");
  }
  /*
if(cpt < 1000){
  fprintf(fp,"%d",ax);
  fputc('\t',fp);
  fprintf(fp,"%d",ay);
  fputc('\t',fp);
  fprintf(fp,"%d",az);;
  fputc('\t',fp);
  fputc('\n',fp);
  
}*/

 
  // _____________________
  // :::  Magnetometer ::: 
 
 
  // Read register Status 1 and wait for the DRDY: Data Ready
 
  /*uint8_t ST1;
  do
  {
    I2Cread(MAG_ADDRESS,0x02,1,&ST1);
  }
  while (!(ST1&0x01));
 
  // Read magnetometer data  
  uint8_t Mag[7];  
  I2Cread(MAG_ADDRESS,0x03,7,Mag);
 
 
  // Create 16 bits values from 8 bits data
 
  // Magnetometer
  int16_t mx=-(Mag[3]<<8 | Mag[2]);
  int16_t my=-(Mag[1]<<8 | Mag[0]);
  int16_t mz=-(Mag[5]<<8 | Mag[4]);
 
 
  // Magnetometer
  Serial.print (mx+200,DEC); 
  Serial.print ("\t");
  Serial.print (my-70,DEC);
  Serial.print ("\t");
  Serial.print (mz-700,DEC);  
  Serial.print ("\t");
 */

 /*
  double ysqr = AttitudeEstimateQuat.y * AttitudeEstimateQuat.y;
  double roll, pitch, yaw;
  // roll (x-axis rotation)
  double t0 = +2.0 * (AttitudeEstimateQuat.w * AttitudeEstimateQuat.x + AttitudeEstimateQuat.y * AttitudeEstimateQuat.z);
  double t1 = +1.0 - 2.0 * (AttitudeEstimateQuat.x * AttitudeEstimateQuat.x + ysqr);
  roll = atan2(t0, t1);

  // pitch (y-axis rotation)
  double t2 = +2.0 * (AttitudeEstimateQuat.w * AttitudeEstimateQuat.y - AttitudeEstimateQuat.z * AttitudeEstimateQuat.x);
  t2 = t2 > 1.0 ? 1.0 : t2;
  t2 = t2 < -1.0 ? -1.0 : t2;
  pitch = asin(t2);

  // yaw (z-axis rotation)
  double t3 = +2.0 * (AttitudeEstimateQuat.w * AttitudeEstimateQuat.z + AttitudeEstimateQuat.x * AttitudeEstimateQuat.y);
  double t4 = +1.0 - 2.0 * (ysqr + AttitudeEstimateQuat.z * AttitudeEstimateQuat.z);  
  yaw = atan2(t3, t4);
  */

 //if(cpt == 1000){
 // fclose(fp);
 //}
 
  // End of line
  //delay(10);    
}
