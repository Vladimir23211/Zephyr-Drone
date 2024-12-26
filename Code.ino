  long long vreme = 0;
  float desiredThrottle=0;
  // Radio
  #include <SPI.h>
  #include <nRF24L01.h>
  #include <RF24.h>
  #define CE_PIN 33
  #define CNS_PIN 10
  RF24 radio(CE_PIN, CNS_PIN); // CE, CSN pins
  const byte address[6] = "00001";  //  adresa sendera
  volatile uint16_t received[4] = {0};

  #include <Wire.h>
  float RateRoll, RatePitch, RateYaw;
  float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
  int RateCalibrationNumber;
float ReceiverValue[]={0, 0, 0, 0, 0, 0, 0, 0};
  uint32_t LoopTimer;
  float DesiredRateRoll, DesiredRatePitch, DesiredRateYaw;
  float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;
  float InputRoll, InputThrottle, InputPitch, InputYaw;
  float PrevErrorRateRoll, PrevErrorRatePitch, PrevErrorRateYaw;
  float PrevItermRateRoll, PrevItermRatePitch, PrevItermRateYaw;
  float PIDReturn[] = {0, 0, 0};
 float PRateRoll = 1.0; float PRatePitch = PRateRoll; float PRateYaw = 11;
  float IRateRoll = 3; float IRatePitch = IRateRoll; float IRateYaw = 2.3;
  float DRateRoll = 0.01; float DRatePitch = DRateRoll; float DRateYaw = 0;
  float MotorInput1, MotorInput2, MotorInput3, MotorInput4, previous1, previous2, previous3, previous4;
  float AccX, AccY, AccZ;
  float AngleRoll, AnglePitch;
  float KalmanAngleRoll = 0, KalmanUncertaintyAngleRoll = 2 * 2;
  float KalmanAnglePitch = 0, KalmanUncertaintyAnglePitch = 2 * 2;
  float Kalman1DOutput[] = {0, 0};
  float DesiredAngleRoll, DesiredAnglePitch;
  float ErrorAngleRoll, ErrorAnglePitch;
  float PrevErrorAngleRoll, PrevErrorAnglePitch;
  float PrevItermAngleRoll, PrevItermAnglePitch;
  float PAngleRoll = 2; float PAnglePitch = PAngleRoll;
  float IAngleRoll = 0; float IAnglePitch = IAngleRoll;
  float DAngleRoll = 0; float DAnglePitch = DAngleRoll;
  uint16_t dig_T1, dig_P1;
  int16_t dig_T2, dig_T3, dig_P2, dig_P3, dig_P4, dig_P5;
  int16_t dig_P6, dig_P7, dig_P8, dig_P9;
  float AltitudeBarometer, AltitudeBarometerStartUp;
  float AccZInertial;
  #include <BasicLinearAlgebra.h>
  using namespace BLA;
  float AltitudeKalman, VelocityVerticalKalman;
  BLA::Matrix<2, 2, float> F; BLA::Matrix<2, 1, float> G;
  BLA::Matrix<2, 2, float> P; BLA::Matrix<2, 2, float> Q;
  BLA::Matrix<2, 1, float> S; BLA::Matrix<1, 2, float> H;
  BLA::Matrix<2, 2, float> I; BLA::Matrix<1, 1, float> Acc;
  BLA::Matrix<2, 1, float> K; BLA::Matrix<1, 1, float> R;
  BLA::Matrix<1, 1, float> L; BLA::Matrix<1, 1, float> M;
  float DesiredVelocityVertical, ErrorVelocityVertical;
  float PVelocityVertical = 3.5, IVelocityVertical = 0.0015, DVelocityVertical = 0.01;
  float PrevErrorVelocityVertical, PrevItermVelocityVertical;
  void kalman_2d(void)
  {
    Acc = {AccZInertial};
    S = F * S + G * Acc;
    P = F * P * ~F + Q;
    L = H * P * ~H + R;
    K = P * ~H * Inverse(L);
    M = {AltitudeBarometer};
    S = S + K * (M - H * S);
    AltitudeKalman = S(0, 0);
    VelocityVerticalKalman = S(1, 0);
    P = (I - K * H) * P;
  }
  void barometer_signals(void)
  {
    Wire.beginTransmission(0x76);
    Wire.write(0xF7);
    Wire.endTransmission();
    Wire.requestFrom(0x76, 6);
    uint32_t press_msb = Wire.read();
    uint32_t press_lsb = Wire.read();
    uint32_t press_xlsb = Wire.read();
    uint32_t temp_msb = Wire.read();
    uint32_t temp_lsb = Wire.read();
    uint32_t temp_xlsb = Wire.read();
    unsigned long int adc_P = (press_msb << 12) | (press_lsb << 4) | (press_xlsb >> 4);
    unsigned long int adc_T = (temp_msb << 12) | (temp_lsb << 4) | (temp_xlsb >> 4);
    signed long int var1, var2;
    var1 = ((((adc_T >> 3) - ((signed long int)dig_T1 << 1))) * ((signed long int)dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((signed long int)dig_T1)) * ((adc_T >> 4) - ((signed long int)dig_T1))) >> 12) * ((signed long int)dig_T3)) >> 14;
    signed long int t_fine = var1 + var2;
    unsigned long int p;
    var1 = (((signed long int)t_fine) >> 1) - (signed long int)64000;
    var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((signed long int)dig_P6);
    var2 = var2 + ((var1 * ((signed long int)dig_P5)) << 1);
    var2 = (var2 >> 2) + (((signed long int)dig_P4) << 16);
    var1 = (((dig_P3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) + ((((signed long int)dig_P2) * var1) >> 1)) >> 18;
    var1 = ((((32768 + var1)) * ((signed long int)dig_P1)) >> 15);
    if (var1 == 0)
    {
      p = 0;
    }
    p = (((unsigned long int)(((signed long int)1048576) - adc_P) - (var2 >> 12))) * 3125;
    if (p < 0x80000000)
    {
      p = (p << 1) / ((unsigned long int)var1);
    }
    else
    {
      p = (p / (unsigned long int)var1) * 2;
    }
    var1 = (((signed long int)dig_P9) * ((signed long int)(((p >> 3) * (p >> 3)) >> 13))) >> 12;
    var2 = (((signed long int)(p >> 2)) * ((signed long int)dig_P8)) >> 13;
    p = (unsigned long int)((signed long int)p + ((var1 + var2 + dig_P7) >> 4));
    double pressure = (double)p / 100;
    AltitudeBarometer = 44330 * (1 - pow(pressure / 1013.25, 1 / 5.255)) * 100;
  }
  void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement)
  {
    KalmanState = KalmanState + 0.004 * KalmanInput;
    KalmanUncertainty = KalmanUncertainty + 0.004 * 0.004 * 4 * 4;
    float KalmanGain = KalmanUncertainty * 1 / (1 * KalmanUncertainty + 3 * 3);
    KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState);
    KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty;
    Kalman1DOutput[0] = KalmanState;
    Kalman1DOutput[1] = KalmanUncertainty;
  }
  void radio_init() {
    pinMode(CE_PIN, OUTPUT);
    pinMode(CNS_PIN, OUTPUT);

    SPI.begin();
    SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0)); // Set SPI clock to 10 MHz

    while(!radio.begin()){
      Serial.println("ne radi radio");
      delay(100);
    }
    radio.openReadingPipe(0, address);
    radio.setPALevel(RF24_PA_MIN);
    radio.setDataRate(RF24_250KBPS);
    radio.setChannel(108);  // Set the channel to 108
    radio.startListening();
  }
  void read_receiver(void)
  {  
    if (radio.available()){
      radio.read((void*)&received, sizeof(received));
    
      ReceiverValue[0] = map(received[2], 0, 1024, 1000, 2000);  // roll (napred nazad)
      ReceiverValue[1] = map(received[3], 0, 1024, 1000, 2000);  // pitch (levo desno)
      ReceiverValue[2] = map(received[1], 0, 1024, 1000, 2000); // throttle
      ReceiverValue[3] = map(received[0], 0, 1024, 1000, 2000); // jaw (u mestu se okrece)
      if (ReceiverValue[0] > 1470 && ReceiverValue[0] < 1530) ReceiverValue[0] = 1500;
      if (ReceiverValue[1] > 1470 && ReceiverValue[1] < 1530) ReceiverValue[1] = 1500;
      //if (ReceiverValue[3] > 1470 && ReceiverValue[3] < 1530) ReceiverValue[3] = 1500;
      ReceiverValue[3] = 1500;
      Serial.println("Received: ");
      Serial.println(received[0]);
      Serial.println(received[1]);
      Serial.println(received[2]);
      Serial.println(received[3]);

    }

    // if (desiredThrottle > ReceiverValue[2]) ReceiverValue[2] += 5;
    // if (desiredThrottle < ReceiverValue[2]) ReceiverValue[2] -= 5;
    // ReceiverValue[2] = constrain(ReceiverValue[2], 1000, 2000);

    
  }
  void gyro_signals(void)
  {
    Wire.beginTransmission(0x68);
    Wire.write(0x1A);
    Wire.write(0x05);
    Wire.endTransmission();
    Wire.beginTransmission(0x68);
    Wire.write(0x1C);
    Wire.write(0x10);
    Wire.endTransmission();
    Wire.beginTransmission(0x68);
    Wire.write(0x3B);
    Wire.endTransmission();
    Wire.requestFrom(0x68, 6);
    int16_t AccXLSB = Wire.read() << 8 | Wire.read();
    int16_t AccYLSB = Wire.read() << 8 | Wire.read();
    int16_t AccZLSB = Wire.read() << 8 | Wire.read();
    Wire.beginTransmission(0x68);
    Wire.write(0x1B);
    Wire.write(0x8);
    Wire.endTransmission();
    Wire.beginTransmission(0x68);
    Wire.write(0x43);
    Wire.endTransmission();
    Wire.requestFrom(0x68, 6);
    int16_t GyroX = Wire.read() << 8 | Wire.read();
    int16_t GyroY = Wire.read() << 8 | Wire.read();
    int16_t GyroZ = Wire.read() << 8 | Wire.read();
    RateRoll = (float)GyroX / 65.5;
    RatePitch = (float)GyroY / 65.5;
    RateYaw = (float)GyroZ / 65.5;
     AccX = (float)AccXLSB / 4096-0.02;
    AccY = (float)AccYLSB / 4096+0.06;
    AccZ = (float)AccZLSB / 4096-0.05;
    AngleRoll = atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * 1 / (3.142 / 180);
    AnglePitch = -atan(AccX / sqrt(AccY * AccY + AccZ * AccZ)) * 1 / (3.142 / 180);
  }
void pid_equation(float Error, float P , float I, float D, float PrevError, float PrevIterm) {
  float Pterm=P*Error;
  float Iterm=PrevIterm+I*(Error+PrevError)*0.004/2;
  if (Iterm > 400) Iterm=400;
  else if (Iterm <-400) Iterm=-400;
  float Dterm=D*(Error-PrevError)/0.004;
  float PIDOutput= Pterm+Iterm+Dterm;
  if (PIDOutput>400) PIDOutput=400;
  else if (PIDOutput <-400) PIDOutput=-400;
  PIDReturn[0]=PIDOutput;
  PIDReturn[1]=Error;
  PIDReturn[2]=Iterm;
}
  void reset_pid(void)
  {
    PrevErrorRateRoll = 0;
    PrevErrorRatePitch = 0;
    PrevErrorRateYaw = 0;
    PrevItermRateRoll = 0;
    PrevItermRatePitch = 0;
    PrevItermRateYaw = 0;
    PrevErrorAngleRoll = 0;
    PrevErrorAnglePitch = 0;
    PrevItermAngleRoll = 0;
    PrevItermAnglePitch = 0;
    PrevErrorVelocityVertical = 0;
    PrevItermVelocityVertical = 0;
  }
  float rampUp(float targetSpeed, float currentSpeed, float rampRate) {
    if (currentSpeed < targetSpeed) {
        currentSpeed += rampRate;
        if (currentSpeed > targetSpeed) {
            currentSpeed = targetSpeed;
        }
    }
    return currentSpeed;
  }
  void setup()
  {
    Serial.begin(57600);
    radio_init();
    Wire.setClock(400000);
    Wire.begin();
    delay(250);
    Wire.beginTransmission(0x68);
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission();
    Wire.beginTransmission(0x76);
    Wire.write(0xF4);
    Wire.write(0x57);
    Wire.endTransmission();
    Wire.beginTransmission(0x76);
    Wire.write(0xF5);
    Wire.write(0x14);
    Wire.endTransmission();
    uint8_t data[24], i = 0;
    Wire.beginTransmission(0x76);
    Wire.write(0x88);
    Wire.endTransmission();
    Wire.requestFrom(0x76, 24);
    while (Wire.available())
    {
      data[i] = Wire.read();
      i++;
    }
    dig_T1 = (data[1] << 8) | data[0];
    dig_T2 = (data[3] << 8) | data[2];
    dig_T3 = (data[5] << 8) | data[4];
    dig_P1 = (data[7] << 8) | data[6];
    dig_P2 = (data[9] << 8) | data[8];
    dig_P3 = (data[11] << 8) | data[10];
    dig_P4 = (data[13] << 8) | data[12];
    dig_P5 = (data[15] << 8) | data[14];
    dig_P6 = (data[17] << 8) | data[16];
    dig_P7 = (data[19] << 8) | data[18];
    dig_P8 = (data[21] << 8) | data[20];
    dig_P9 = (data[23] << 8) | data[22];
    delay(250);
    for (RateCalibrationNumber = 0; RateCalibrationNumber < 2000; RateCalibrationNumber++)
    {
      gyro_signals();
      RateCalibrationRoll += RateRoll;
      RateCalibrationPitch += RatePitch;
      RateCalibrationYaw += RateYaw;
      barometer_signals();
      AltitudeBarometerStartUp += AltitudeBarometer;
      delay(1);
    }
    RateCalibrationRoll /= 2000;
    RateCalibrationPitch /= 2000;
    RateCalibrationYaw /= 2000;
    AltitudeBarometerStartUp /= 2000;
    F = {1, 0.004,
        0, 1};
    G = {0.5 * 0.004 * 0.004,
        0.004};
    H = {1, 0};
    I = {1, 0,
        0, 1};
    Q = G * ~G * 10.0f * 10.0f;
    R = {30 * 30};
    P = {0, 0,
        0, 0};
    S = {0,
        0};
      analogWriteFrequency(1, 250);
      analogWriteFrequency(2, 250);
      analogWriteFrequency(3, 250);
      analogWriteFrequency(4, 250);
      analogWriteResolution(12);
    
    delay(250);



  analogWrite(1, 2000);
    analogWrite(3, 2000);
    analogWrite(2, 2000);
    analogWrite(4, 2000);

    delay(5000);
  analogWrite(1, 1000);
    analogWrite(3, 1000);
    analogWrite(2, 1000);
    analogWrite(4, 1000);

delay(3000);

  while (ReceiverValue[2] < 1000 || ReceiverValue[2] > 1050) {
    read_receiver();
    Serial.println("Spusti brzinu, brzina tretnutna:  " +String(ReceiverValue[2]));
    delay(4);
  }
    LoopTimer = micros();

    previous1 = 1000; previous2 = 1000;
    previous3 = 1000; previous4 = 1000;

  }

  void loop()
  {
    gyro_signals();
    RateRoll -= RateCalibrationRoll;
    RatePitch -= RateCalibrationPitch;
    RateYaw -= RateCalibrationYaw;
    kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
    KalmanAngleRoll = Kalman1DOutput[0]; KalmanUncertaintyAngleRoll = Kalman1DOutput[1];
    kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
    KalmanAnglePitch = Kalman1DOutput[0]; KalmanUncertaintyAnglePitch = Kalman1DOutput[1];
    AccZInertial = -sin(AnglePitch * (3.142 / 180)) * AccX + cos(AnglePitch * (3.142 / 180)) * sin(AngleRoll * (3.142 / 180)) * AccY + cos(AnglePitch * (3.142 / 180)) * cos(AngleRoll * (3.142 / 180)) * AccZ;
    AccZInertial = (AccZInertial - 1) * 9.81 * 100;
    barometer_signals();
    AltitudeBarometer -= AltitudeBarometerStartUp;
    kalman_2d();
    read_receiver();
    DesiredAngleRoll = 0.10 * (ReceiverValue[0] - 1500);
    DesiredAnglePitch = 0.10 * (ReceiverValue[1] - 1500);
    DesiredRateYaw = 0.15 * (ReceiverValue[3] - 1500);
    DesiredVelocityVertical = 0.3 * (ReceiverValue[2] - 1500);
    Serial.print("DesiredAngleRoll:" + String(DesiredAngleRoll));
    Serial.print("  DesiredAnglePitch:" + String(DesiredAnglePitch));
    Serial.print("  DesiredRateYaw:" + String(DesiredRateYaw));
    Serial.println("  DesiredVelocityVertical:" + String(DesiredVelocityVertical));

    ErrorVelocityVertical = DesiredVelocityVertical - VelocityVerticalKalman;
    pid_equation(ErrorVelocityVertical, PVelocityVertical, IVelocityVertical, DVelocityVertical, PrevErrorVelocityVertical, PrevItermVelocityVertical);
    InputThrottle = 1500 + PIDReturn[0];

    PrevErrorVelocityVertical = PIDReturn[1];
    PrevItermVelocityVertical = PIDReturn[2];
    ErrorAngleRoll = DesiredAngleRoll - KalmanAngleRoll;
    ErrorAnglePitch = DesiredAnglePitch - KalmanAnglePitch;
    pid_equation(ErrorAngleRoll, PAngleRoll, IAngleRoll, DAngleRoll, PrevErrorAngleRoll, PrevItermAngleRoll);
    DesiredRateRoll = PIDReturn[0];
    PrevErrorAngleRoll = PIDReturn[1];
    PrevItermAngleRoll = PIDReturn[2];
    pid_equation(ErrorAnglePitch, PAnglePitch, IAnglePitch, DAnglePitch, PrevErrorAnglePitch, PrevItermAnglePitch);
    DesiredRatePitch = PIDReturn[0];
    PrevErrorAnglePitch = PIDReturn[1];
    PrevItermAnglePitch = PIDReturn[2];
    ErrorRateRoll = DesiredRateRoll - RateRoll;
    ErrorRatePitch = DesiredRatePitch - RatePitch;
    ErrorRateYaw = DesiredRateYaw - RateYaw;
    pid_equation(ErrorRateRoll, PRateRoll, IRateRoll, DRateRoll, PrevErrorRateRoll, PrevItermRateRoll);
    InputRoll = PIDReturn[0];
    PrevErrorRateRoll = PIDReturn[1];
    PrevItermRateRoll = PIDReturn[2];
    pid_equation(ErrorRatePitch, PRatePitch, IRatePitch, DRatePitch, PrevErrorRatePitch, PrevItermRatePitch);
    InputPitch = PIDReturn[0];
    PrevErrorRatePitch = PIDReturn[1];
    PrevItermRatePitch = PIDReturn[2];
    pid_equation(ErrorRateYaw, PRateYaw, IRateYaw, DRateYaw, PrevErrorRateYaw, PrevItermRateYaw);
    InputYaw = PIDReturn[0];
    PrevErrorRateYaw = PIDReturn[1];
    PrevItermRateYaw = PIDReturn[2];
    if (InputThrottle > 1800) InputThrottle = 1800;
    MotorInput1 = 1.024 * (InputThrottle - InputPitch - InputRoll - InputYaw);
    MotorInput2 = 1.024 * (InputThrottle + InputPitch - InputRoll + InputYaw);
    MotorInput3 = 1.024 * (InputThrottle + InputPitch + InputRoll - InputYaw);
    MotorInput4 = 1.024 * (InputThrottle - InputPitch + InputRoll + InputYaw);

    // float ramp = 10;
    // MotorInput1 = rampUp(MotorInput1, previous1, ramp);  // rampRate of 5.0 can be adjusted
    // MotorInput2 = rampUp(MotorInput2, previous2, ramp);
    // MotorInput3 = rampUp(MotorInput3, previous3, ramp);
    // MotorInput4 = rampUp(MotorInput4, previous4, ramp);

    if (MotorInput1 > 2000) MotorInput1 = 1999;
    if (MotorInput2 > 2000) MotorInput2 = 1999;
    if (MotorInput3 > 2000) MotorInput3 = 1999;
    if (MotorInput4 > 2000) MotorInput4 = 1999;

    int ThrottleIdle=1150;
  if (MotorInput1 < ThrottleIdle) MotorInput1 =  ThrottleIdle;
  if (MotorInput2 < ThrottleIdle) MotorInput2 =  ThrottleIdle;
  if (MotorInput3 < ThrottleIdle) MotorInput3 =  ThrottleIdle;
  if (MotorInput4 < ThrottleIdle) MotorInput4 =  ThrottleIdle;

  int ThrottleCutOff=1000;
  if (ReceiverValue[2]<1050 ) {
    MotorInput1=ThrottleCutOff; 
    MotorInput2=ThrottleCutOff;
    MotorInput3=ThrottleCutOff; 
    MotorInput4=ThrottleCutOff;
    reset_pid();
  }


    // previous1 = MotorInput1; previous2 = MotorInput2;
    // previous3 = MotorInput3; previous4 = MotorInput4;

    analogWrite(1, MotorInput1);
    analogWrite(3, MotorInput2);
    analogWrite(2, MotorInput3);
    analogWrite(4, MotorInput4);

    Serial.print("motor1:" + String(MotorInput1));
    Serial.print("  motor2:" + String(MotorInput3));
    Serial.print("  motor3:" + String(MotorInput3));
    Serial.println("  motor4:" + String(MotorInput4));

    while (micros() - LoopTimer < 4000);
    LoopTimer = micros();
  }
