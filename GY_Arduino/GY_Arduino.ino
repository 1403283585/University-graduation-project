#include <FlexiTimer2.h>

#include "Wire.h"
#include <I2Cdev.h>
#include <MPU6050.h>
#include "math.h"

//GA6-B
const int LED = 44; //LED
const int BUTTON = 45; //Button
static char text_content[200] = {'\0'};
static char end_char[2]       = {0x1A, '\0'};

//HC-SR04
const int TrigPin = 2;
const int EchoPin = 3;
float cm;

//MPU6050
MPU6050 accelgyro;
float SVM;
static int s = 0;  //延时控制量
static int j = 0;  //短信开关
static int z = 0;  //z开关
static int flgt = 0;

unsigned long now, lastTime = 0;
float dt;                                   //微分时间
int16_t ax, ay, az, gx, gy, gz;             //加速度计陀螺仪原始数据
float aax = 0, aay = 0, aaz = 0, agx = 0, agy = 0, agz = 0; //角度变量
long axo = 0, ayo = 0, azo = 0;             //加速度计偏移量
long gxo = 0, gyo = 0, gzo = 0;             //陀螺仪偏移量

float pi = 3.1415926;
float AcceRatio = 16384.0;                  //加速度计比例系数 ±2g
float GyroRatio = 131.0;                    //陀螺仪比例系数

uint8_t n_sample = 8;                       //加速度计滤波算法采样个数
float aaxs[8] = {0}, aays[8] = {0}, aazs[8] = {0};         //x,y轴采样队列
long aax_sum, aay_sum, aaz_sum;                     //x,y轴采样和

float a_x[10] = {0}, a_y[10] = {0}, a_z[10] = {0} , g_x[10] = {0} , g_y[10] = {0}, g_z[10] = {0}; //加速度计协方差计算队列
float Px = 1, Rx, Kx, Sx, Vx, Qx;           //x轴卡尔曼变量
float Py = 1, Ry, Ky, Sy, Vy, Qy;           //y轴卡尔曼变量
float Pz = 1, Rz, Kz, Sz, Vz, Qz;           //z轴卡尔曼变量

//ATGM332D
#define GpsSerial  Serial3
#define DebugSerial Serial

struct
{
  char GPS_Buffer[80];
  bool isGetData;		//是否获取到GPS数据
  bool isParseData;	//是否解析完成
  char UTCTime[11];		//UTC时间
  char latitude[11];		//纬度
  char N_S[2];		//N/S
  char longitude[12];		//经度
  char E_W[2];		//E/W
  bool isUsefull;		//定位信息是否有效
} Save_Data;

const unsigned int gpsRxBufferLength = 600;
char gpsRxBuffer[gpsRxBufferLength];
unsigned int ii = 0;

//PDU短信数据
static char *send_pdu[] =
{
  "AT+CMGF=0",
  "AT+CNMI=2,1",
  "AT+CMGS=29",
  "0011640D91683100652720F10008AA0E80014EBA90475230537196694E86",
  end_char
};

static char *send_text[] =
{
  "AT+CMGS=\"+8613005672021\"",
  text_content,
  end_char
};

//发送text短信
void usart_text() {
  int i = 0;
  while (send_text[i] != NULL) {
    if (i == 1) {
      Serial2.print(send_text[i]);

    } else {
      Serial2.println(send_text[i]);
    }
    delay(100);
    i++;
  }
  z = 0;
}


//发送PDU短信
void usart_re() {
  int i = 0;
  while (send_pdu[i] != NULL) {
    if (i == 3) {
      Serial2.print(send_pdu[i]);

    } else {
      Serial2.println(send_pdu[i]);
    }
    delay(100);
    /*  调试用代码
    String incomedate = "";
    while (Serial.available() > 0)//串口接收到数据
    {
      incomedate += char(Serial.read());//获取串口接收到的数据
      delay(2);
    }
    if (incomedate.length() > 0){
      Serial2.println(incomedate);
      //return incomedate;
    }
    incomedate = "";
    while (Serial2.available() > 0)//串口接收到数据
    {
      incomedate += char(Serial2.read());//获取串口接收到的数据
      delay(2);
    }
    if (incomedate.length() > 0){
      Serial.println(incomedate);
      //return incomedate;
    }
    */
    i++;

  }

}

//警告函数
void JingGao() {
  unsigned char i = 0;
  unsigned char JingGao[10];

  JingGao[0] = 0xFD;
  JingGao[1] = 0x00;
  JingGao[2] = 0x07;
  JingGao[3] = 0x01;
  JingGao[4] = 0x08;
  JingGao[5] = 0xBE;
  JingGao[6] = 0xAF;
  JingGao[7] = 0xB8;
  JingGao[8] = 0xE6;
  JingGao[9] = 0xBC;

  for (i = 0; i < 10; i++) {
    Serial1.write(JingGao[i]);
  }
}

//求救函数
void QiuJiu() {
  unsigned char i = 0;
  unsigned char head[18];

  head[0] = 0xFD;
  head[1] = 0x00;
  head[2] = 0x0F;
  head[3] = 0x01;
  head[4] = 0x00;
  head[5] = 0xC0;
  head[6] = 0xCF;
  head[7] = 0xC8;
  head[8] = 0xCB;
  head[9] = 0xD0;
  head[10] = 0xE8;
  head[11] = 0xD2;
  head[12] = 0xAA;
  head[13] = 0xB0;
  head[14] = 0xEF;
  head[15] = 0xD6;
  head[16] = 0xFA;
  head[17] = 0xCC;

  for (i = 0; i < 18; i++) {
    Serial1.write(head[i]);
  }

}

//Kalman
void Kalman() {
  unsigned long now = millis();             //当前时间(ms)
  dt = (now - lastTime) / 1000.0;           //微分时间(s)
  lastTime = now;                           //上一次采样时间(ms)


  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); //读取六轴原始数值


  float accx = ax / AcceRatio;              //x轴加速度
  float accy = ay / AcceRatio;              //y轴加速度
  float accz = az / AcceRatio;              //z轴加速度


  aax = atan(accy / accz) * (-180) / pi;    //y轴对于z轴的夹角
  aay = atan(accx / accz) * 180 / pi;       //x轴对于z轴的夹角
  aaz = atan(accz / accy) * 180 / pi;       //z轴对于y轴的夹角


  aax_sum = 0;                              // 对于加速度计原始数据的滑动加权滤波算法
  aay_sum = 0;
  aaz_sum = 0;

  for (int i = 1; i < n_sample; i++)
  {
    aaxs[i - 1] = aaxs[i];
    aax_sum += aaxs[i] * i;
    aays[i - 1] = aays[i];
    aay_sum += aays[i] * i;
    aazs[i - 1] = aazs[i];
    aaz_sum += aazs[i] * i;

  }

  aaxs[n_sample - 1] = aax;
  aax_sum += aax * n_sample;
  aax = (aax_sum / (11 * n_sample / 2.0)) * 9 / 7.0; //角度调幅至0-90°
  aays[n_sample - 1] = aay;                      //此处应用实验法取得合适的系数
  aay_sum += aay * n_sample;                     //本例系数为9/7
  aay = (aay_sum / (11 * n_sample / 2.0)) * 9 / 7.0;
  aazs[n_sample - 1] = aaz;
  aaz_sum += aaz * n_sample;
  aaz = (aaz_sum / (11 * n_sample / 2.0)) * 9 / 7.0;


  float gyrox = - (gx - gxo) / GyroRatio * dt; //x轴角速度
  float gyroy = - (gy - gyo) / GyroRatio * dt; //y轴角速度
  float gyroz = - (gz - gzo) / GyroRatio * dt; //z轴角速度
  agx += gyrox;                             //x轴角速度积分
  agy += gyroy;                             //x轴角速度积分
  agz += gyroz;

  /* kalman start */
  Sx = 0; Rx = 0;
  Sy = 0; Ry = 0;
  Sz = 0; Rz = 0;

  for (int i = 1; i < 10; i++)
  { //测量值平均值运算
    a_x[i - 1] = a_x[i];                    //即加速度平均值
    Sx += a_x[i];
    a_y[i - 1] = a_y[i];
    Sy += a_y[i];
    a_z[i - 1] = a_z[i];
    Sz += a_z[i];

  }

  a_x[9] = aax;
  Sx += aax;
  Sx /= 10;                                 //x轴加速度平均值
  a_y[9] = aay;
  Sy += aay;
  Sy /= 10;                                 //y轴加速度平均值
  a_z[9] = aaz;
  Sz += aaz;
  Sz /= 10;


  for (int i = 0; i < 10; i++)
  {
    Rx += sq(a_x[i] - Sx);
    Ry += sq(a_y[i] - Sy);
    Rz += sq(a_z[i] - Sz);

  }

  Rx = Rx / 9;                              //得到方差
  Ry = Ry / 9;
  Rz = Rz / 9;

  Px = Px + 0.0025;                         // 0.0025在下面有说明...
  Kx = Px / (Px + Rx);                      //计算卡尔曼增益
  agx = agx + Kx * (aax - agx);             //陀螺仪角度与加速度计速度叠加
  Px = (1 - Kx) * Px;                       //更新p值


  Py = Py + 0.0025;
  Ky = Py / (Py + Ry);
  agy = agy + Ky * (aay - agy);
  Py = (1 - Ky) * Py;

  Pz = Pz + 0.0025;
  Kz = Pz / (Pz + Rz);
  agz = agz + Kz * (aaz - agz);
  Pz = (1 - Kz) * Pz;


  /* kalman end */
  SVM = sqrt(pow(accx, 2) + pow(accy, 2) + pow(accz, 2));

  Serial.print(SVM); Serial.print(",");
  Serial.print(agx); Serial.print(",");
  Serial.print(agy); Serial.print(",");
  Serial.print(agz); Serial.println();
  //Serial.println(digitalRead(BUTTON));
}

//GPS
void errorLog(int num)
{
  DebugSerial.print("ERROR");
  DebugSerial.println(num);
}

//输出解析后的数据
void printGpsBuffer()
{
  if (Save_Data.isParseData)
  {
    Save_Data.isParseData = false;

    DebugSerial.print("Save_Data.UTCTime = ");
    DebugSerial.println(Save_Data.UTCTime);


    if (Save_Data.isUsefull)
    {


      Save_Data.isUsefull = false;
      DebugSerial.print("Save_Data.latitude = ");
      //strcpy(text_content, "latitude = ");
      DebugSerial.println(Save_Data.latitude);
      //strcpy(text_content,Save_Data.latitude);
      DebugSerial.print("Save_Data.N_S = ");
      //strcpy(text_content,"N_S = ");
      DebugSerial.println(Save_Data.N_S);
      //strcpy(text_content,Save_Data.N_S);
      DebugSerial.print("Save_Data.longitude = ");
      //strcpy(text_content,"longitude = ");
      DebugSerial.println(Save_Data.longitude);
      //strcpy(text_content,Save_Data.longitude);
      DebugSerial.print("Save_Data.E_W = ");
      //strcpy(text_content,"E_W = ");
      DebugSerial.println(Save_Data.E_W);
      // strcpy(text_content,Save_Data.E_W);


    }
    else
    {
      DebugSerial.println("GPS DATA is not usefull!");
    }

  }
}

//解析GPS数据
void parseGpsBuffer()
{
  char *subString;
  char *subStringNext;
  if (Save_Data.isGetData)
  {
    Save_Data.isGetData = false;
    DebugSerial.println("**************");
    //DebugSerial.println(Save_Data.GPS_Buffer);//GNRMC

    memset(text_content, '\0', sizeof(text_content));
    strcpy(text_content, Save_Data.GPS_Buffer);
    DebugSerial.println(text_content);
    usart_text();



    for (int i = 0 ; i <= 6 ; i++)
    {
      if (i == 0)
      {
        if ((subString = strstr(Save_Data.GPS_Buffer, ",")) == NULL)
          errorLog(1);	//解析错误
      }
      else
      {
        subString++;
        if ((subStringNext = strstr(subString, ",")) != NULL)
        {
          char usefullBuffer[2];
          switch (i)
          {
            case 1: memcpy(Save_Data.UTCTime, subString, subStringNext - subString); break;	//获取UTC时间
            case 2: memcpy(usefullBuffer, subString, subStringNext - subString); break;	//获取UTC时间
            case 3: memcpy(Save_Data.latitude, subString, subStringNext - subString); break;	//获取纬度信息
            case 4: memcpy(Save_Data.N_S, subString, subStringNext - subString); break;	//获取N/S
            case 5: memcpy(Save_Data.longitude, subString, subStringNext - subString); break;	//获取纬度信息
            case 6: memcpy(Save_Data.E_W, subString, subStringNext - subString); break;	//获取E/W

            default: break;
          }

          subString = subStringNext;
          Save_Data.isParseData = true;
          if (usefullBuffer[0] == 'A')
            Save_Data.isUsefull = true;
          else if (usefullBuffer[0] == 'V')
            Save_Data.isUsefull = false;

        }
        else
        {
          errorLog(2);	//解析错误
        }
      }


    }
  }
}

//获取GPS数据
void gpsRead() {
  while (GpsSerial.available())
  {
    gpsRxBuffer[ii++] = GpsSerial.read();
    if (ii == gpsRxBufferLength)clrGpsRxBuffer();
  }

  char* GPS_BufferHead;
  char* GPS_BufferTail;
  if ((GPS_BufferHead = strstr(gpsRxBuffer, "$GPRMC,")) != NULL || (GPS_BufferHead = strstr(gpsRxBuffer, "$GNRMC,")) != NULL )
  {
    if (((GPS_BufferTail = strstr(GPS_BufferHead, "\r\n")) != NULL) && (GPS_BufferTail > GPS_BufferHead))
    {
      memcpy(Save_Data.GPS_Buffer, GPS_BufferHead, GPS_BufferTail - GPS_BufferHead);
      Save_Data.isGetData = true;

      clrGpsRxBuffer();
    }
  }
}

void clrGpsRxBuffer(void)
{
  memset(gpsRxBuffer, 0, gpsRxBufferLength);      //清空
  ii = 0;
}

//判断跌倒以及发出求救
void BoolM() {
  s = 0;
  if (SVM >= 2.00) {
    for (s ; s < 300 ; s++) {
      flgt = 0;
      digitalWrite(LED, HIGH);

      delay(1);
      Kalman();
    }

    if (agx >= 70 || agx <= -70 || agy >= 70 || agy <= -70) {

      s = 300;
      flgt = 1;
      j = 1;
      JingGao();
      z = 1;

    }
    /*
    while(agx >= 70||agx <= -70||agy >= 70||agy <= -70){
      flgt = 0;
      s++;
      digitalWrite(LED, HIGH);
      Kalman();
      delay(1);
      if(s == 300){
        flgt = 1;
        j = 1;
        JingGao();
        delay(200);
        break;
      }
    }
    */
  }

  while (flgt) {
    s++;
    delay(1);
    Serial.println(s);
    if (s == 1500) {

      //usart_re();
      while (z) {
        gpsRead();	//获取GPS数据
        parseGpsBuffer();//解析GPS数据
        printGpsBuffer();//输出解析后的数据
        //Serial.println(text_content);
        QiuJiu();
        Serial.println("z");
        delay(3000);
      }

      break;

    }

    if (digitalRead(BUTTON) == LOW) { // 若按键被按下
      delay(200); //等待跳过按键抖动的不稳定过程
      if (digitalRead(BUTTON) == LOW) { // 若按键被按下
        flgt = 0;

        s = 0;
        j = 0;
        z = 0;
        Serial.println("reset1");
        break;
      }
    }
  }

  while (j) {
    QiuJiu();
    Serial.println("kkkk");
    delay(2000);

    if (digitalRead(BUTTON) == LOW) { // 若按键被按下
      delay(200); //等待跳过按键抖动的不稳定过程
      if (digitalRead(BUTTON) == LOW) { // 若按键被按下
        flgt = 0;
        s = 0;
        j = 0;
        z = 0;
        Serial.println("reset2");
        break;
        //digitalWrite(BUTTON, LOW);
      }
    }

  }

}


//配置
void setup()
{
  Wire.begin();
  Serial.begin(9600);  //串口0，即Pin0及Pin1  RX\TX
  Serial1.begin(9600);  //串口1，即Pin19及Pin18  RX\TX SYN6288
  Serial2.begin(115200); //串口2，即Pin17及Pin16  RX\TX GA6-B
  Serial3.begin(9600); //串口3，即Pin15及Pin14  RX\TX ATGM332D
  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();  //初始化

  DebugSerial.println("Wating...");

  Save_Data.isGetData = false;
  Save_Data.isParseData = false;
  Save_Data.isUsefull = false;

  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  unsigned short times = 200;             //采样次数
  for (int i = 0; i < times; i++)
  {
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); //读取六轴原始数值
    axo += ax; ayo += ay; azo += az;      //采样和
    gxo += gx; gyo += gy; gzo += gz;

  }

  axo /= times; ayo /= times; azo /= times; //计算加速度计偏移
  gxo /= times; gyo /= times; gzo /= times; //计算陀螺仪偏移


  pinMode(TrigPin, OUTPUT);
  pinMode(EchoPin, INPUT);
  pinMode(BUTTON, INPUT); //定义BUTTON为输入模式；
  pinMode(LED, OUTPUT); //定义LED为输出模式；

  //memset(text_content, '\0', sizeof(text_content));

  Serial.println("OK");

}



//循环
void loop()
{
  //发一个10ms的高脉冲去触发TrigPin
  digitalWrite(TrigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(TrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(TrigPin, LOW);
  digitalWrite(LED, LOW);


  cm = pulseIn(EchoPin, HIGH) / 58.0; //算成厘米
  cm = (int(cm * 100.0)) / 100.0; //保留两位小数

  Serial.print(cm);
  Serial.print("cm");
  Serial.println();
  //gpsRead();	//获取GPS数据
  //parseGpsBuffer();//解析GPS数据
  //printGpsBuffer();//输出解析后的数据

  if (cm <= 50) {
    JingGao();
    delay(1000);
    /*
    usart_re();
       delay(3000);
       gpsRead();	//获取GPS数据
       parseGpsBuffer();//解析GPS数据
       printGpsBuffer();//输出解析后的数据
       Serial.println(text_content);
       usart_text();
       delay(3000);
     */
  }


  //Kalman();
  // Serial.print(SVM);Serial.print(",");
  // Serial.print(agx);Serial.print(",");
  // Serial.print(agy);Serial.print(",");
  // Serial.print(agz);Serial.println();
  //BoolM();


  gpsRead();	//获取GPS数据
  parseGpsBuffer();//解析GPS数据


  /*
  if (digitalRead(BUTTON) != LOW) { // 若按键被按下
    delay(200); //等待跳过按键抖动的不稳定过程
    if (digitalRead(BUTTON) != LOW){ // 若按键被按下
      //usart_re();
    }
  }
  */

  /*
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  Serial.print("a/g:\t");
  Serial.print(ax); Serial.print("\t");
  Serial.print(ay); Serial.print("\t");
  Serial.print(az); Serial.print("\t");
  Serial.print(gx); Serial.print("\t");
  Serial.print(gy); Serial.print("\t");
  Serial.println(gz);
  */

  delay(200);
}
