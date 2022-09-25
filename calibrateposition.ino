#include<Wire.h>
const int MPU = 0x68; //MPU6050 I2C address
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
float AcX_lp, AcY_lp, AcZ_lp, GyX_hp, GyY_hp, GyZ_hp;
float AcX_scaled, AcY_scaled, AcZ_scaled, GyX_scaled, GyY_scaled, GyZ_scaled;
float prev_GyX, prev_GyY, prev_GyZ;
float Ac_roll, Ac_pitch, Gy_roll, Gy_pitch;
float dt, currTime, prevTime;
float max_Acx=4235,max_Acy=4049,max_Acz=4455;
float min_Acx=-3910,min_Acy=-4086,min_Acz=-3850;
float fullscale,Bias;
int arry[6000];
const float acc_roll_error = 4.73, acc_pitch_error = -14.00, gyX_offset = -0.86, gyY_offset = 0.02, gyZ_offset = -0.25;
const float alpha_lp = 0.70, alpha_hp = 0.60, alpha = 0.60;
bool Run = true;
int countdata;
void setup() {
  Serial.begin(500000);

  // Start communication with MPU6050
  Wire.begin(4, 5);
  Wire.beginTransmission(MPU); 
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);

  // Config Acc Sensitivity
  Wire.beginTransmission(MPU);
  Wire.write(0x1C);               // Talk to the ACCEL_CONFIG register
  Wire.write(0x10);               // Set the register bits as 00010000 (+/- 8g full scale range)
  Wire.endTransmission(true);

  // Config Gyro Sensitivity
  Wire.beginTransmission(MPU);
  Wire.write(0x1B);               // Talk to the GYRO_CONFIG register
  Wire.write(0x10);               // Set the register bits as 00010000 (1000 deg/s full scale)
  Wire.endTransmission(true);

}

void loop() {
  if (Run){
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);               // Acc out
    Wire.endTransmission(true);
    Wire.requestFrom(MPU, 14); 

    AcX = Wire.read() << 8 | Wire.read();
    AcY = Wire.read() << 8 | Wire.read();
    AcZ = Wire.read() << 8 | Wire.read();
    Tmp = Wire.read() << 8 | Wire.read();
    GyX = Wire.read() << 8 | Wire.read();
    GyY = Wire.read() << 8 | Wire.read();
    GyZ = Wire.read() << 8 | Wire.read();
    
//    countdata+=1;
//      min_Acz = findminn(AcZ,min_Acz);
//      max_Acz = findmaxx(AcZ,max_Acz);
//      min_Acy = findminn(AcY,min_Acy);
//      max_Acy = findmaxx(AcY,max_Acy);
//      min_Acx = findminn(AcX,min_Acx);
//      max_Acx = findmaxx(AcX,max_Acx);
    
    fullscale,Bias = fullscale_Bias(min_Acz,max_Acz);
    AcZ_scaled = calibrated(AcZ,fullscale,Bias);

    
    fullscale,Bias = fullscale_Bias(min_Acy,max_Acy);
    AcY_scaled = calibrated(AcY,fullscale,Bias);

    
    fullscale,Bias = fullscale_Bias(min_Acx,max_Acx);
    AcX_scaled = calibrated(AcX,fullscale,Bias);
    
    // Accelerometer uses lowpass
    AcX_lp = AcX_lp * alpha_lp + AcX * (1 - alpha_lp);
    AcY_lp = AcY_lp * alpha_lp + AcY * (1 - alpha_lp);
    AcZ_lp = AcZ_lp * alpha_lp + AcZ * (1 - alpha_lp);


    

    Ac_roll = (atan(AcY_scaled / (sqrt(AcX_scaled * AcX_scaled) + (AcZ_scaled * AcZ_scaled)))) * 57.2957795131;    
    Ac_pitch = (atan((-AcX_scaled) / (sqrt(AcY_scaled * AcY_scaled) + (AcZ_scaled * AcZ_scaled)))) * 57.2957795131;
  
    // cal dt
    prevTime = currTime;
    currTime = millis();
    dt = (currTime - prevTime) / 1000.0;

    // Gyro uses highpass
    GyX_hp = GyX_hp * (1 - alpha_hp) + (GyX - prev_GyX) * (1 - alpha_hp);
    GyY_hp = GyY_hp * (1 - alpha_hp) + (GyY - prev_GyY) * (1 - alpha_hp);
    GyZ_hp = GyZ_hp * (1 - alpha_hp) + (GyZ - prev_GyZ) * (1 - alpha_hp);

    GyX_scaled = (GyX_hp / 131.0) - gyX_offset;
    GyY_scaled = (GyY_hp / 131.0) - gyY_offset;
    GyZ_scaled = (GyZ_hp / 131.0) - gyZ_offset;

    // roll, pitch from gyro + acc
    Gy_roll = (1 - alpha) * (Gy_roll + GyX_scaled * dt) + alpha * Ac_roll;
    Gy_pitch = (1 - alpha) * (Gy_pitch + GyY_scaled * dt) + alpha * Ac_pitch;

    Serial.print(Gy_roll); Serial.print(" , ");
    Serial.print(Gy_pitch); Serial.print("   |   ");

    Serial.print(Ac_roll); Serial.print(" , ");
    Serial.print(Ac_pitch); Serial.print("   |   ");

    Serial.print(AcX_scaled); Serial.print(" , ");
    Serial.print(AcY_scaled); Serial.print(" , ");
    Serial.print(AcZ_scaled); Serial.print("   |    ");

  
    Serial.print(GyX_scaled); Serial.print(" , ");
    Serial.print(GyY_scaled); Serial.print(" , ");
    Serial.print(GyZ_scaled); Serial.println();

//    Serial.print(min_Acx); Serial.print(" , ");
//    Serial.print(min_Acy); Serial.print(" , ");
//    Serial.print(min_Acz); Serial.print("   |    ");
//  
//    Serial.print(max_Acx); Serial.print(" , ");
//    Serial.print(max_Acy); Serial.print(" , ");
//    Serial.print(max_Acz); Serial.println();

//    Serial.print(countdata); Serial.print(" , ");
    
    delay(100);
  }
}

// function to determine error

//
//float min_max(float AcX_scaled,float AcY_scaled,float AcZ_scaled){
//  if (max_Acx<AcZ_scaled){
//    max_Acz = AcZ_scaled;
//  }
//  if (max_Acx<AcY_scaled){
//    max_Acy = AcY_scaled;
//  }
//  if (max_Acx<AcX_scaled){
//    max_Acx = AcX_scaled;
//  }
//  if (min_Acx>AcZ_scaled){
//    min_Acz = AcZ_scaled;
//  }
//  if (min_Acx>AcY_scaled){
//    min_Acy = AcY_scaled;
//  }
//  if (min_Acx>AcX_scaled){
//    min_Acx = AcX_scaled;
//  }
//  return max_Acz,min_Acz;
//}

float findminn(int raw,float minn){
  if (minn>raw){
    arry[raw*(-1)]+=1;
    if(arry[raw*(-1)]>10){
        minn = raw;
      }
  }
  return minn;
}
float findmaxx(int raw,float maxx){
  if (maxx<raw){
    arry[raw]+=1;
    if(arry[raw]>10){
        maxx = raw;
      }
  }
  return maxx;
}

float fullscale_Bias(float minn,float maxx){
    fullscale = (maxx - minn)/2;
    Bias = (maxx + minn)/2;
    return fullscale,Bias;
    
}
float calibrated(float accX_raw,float accX_fullscale,float accX_bias){
    float accX_calibrated = (accX_raw - accX_bias) /accX_fullscale * 4096.0f;
    return accX_calibrated;
}
