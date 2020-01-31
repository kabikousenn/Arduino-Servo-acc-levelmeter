#include <MsTimer2.h>
#include "RTClib.h"
RTC_DS1307 rtc;
#include <SD.h>
#include <SPI.h>
#include <Servo.h>

//setup priginal pins
int buzzer = 9;
const int over_G_LED = 3;
const int err_LED = 4;

//setup SD lib.
File dataFile;
const int SDCS = 10;

//variable for acc_meter
float acc_x = 0, acc_y = 0, acc_z = 0;
const int CS = 8;

//setup servo
Servo S1;
const int S1_port = 6;
Servo S2;
const int S2_port = 5;

const int Gain = 3;
float S1_pos = 90;      //x軸用
float S1_parray[6] = {90,90,90,90,90,90};
float S2_pos = 90;      //y軸用
float S2_parray[6] = {90,90,90,90,90,90};

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  Serial.println("program start.");
  delay(10);
  pinMode(err_LED,OUTPUT);
  pinMode(over_G_LED, OUTPUT);
  pinMode(SS, OUTPUT);
  pinMode(CS, OUTPUT);

  //setup pins mode
  pinMode(buzzer,OUTPUT);
  digitalWrite(SS,HIGH);
  digitalWrite(CS,HIGH);

  SD_init();

  //setup intterupt timer
  MsTimer2::set(10, measure);

  //MPU initing
  MPU_init();

  //Servo init
  S1.attach(S1_port);
  S2.attach(S2_port);
  S1.write(S1_pos);
  S2.write(S2_pos);


 //start intterupt
 MsTimer2::start();
}

void loop() {
  // put your main code here, to run repeatedly:
  
  //measure();
}

void SD_init()
{
  Serial.print("Initializing SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin(SDCS)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    while (1);
  }
  Serial.println("card initialized.");
}

void MPU_init()
{

  SPI.begin();

  SPI.setClockDivider(SPI_CLOCK_DIV16);
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE3);

  writeByte(28,24);

  /*
  //Who am I
  unsigned int He_says = readByte(0x75);
  if(He_says != 0x71){
    Serial.println("Communication Error!!!");
    Serial.print("He says, 0x");
    Serial.println(He_says,HEX);
    Serial.println("Address List output");
    Serial.println("\nADD : VAL");
    unsigned int i = 0;
    for (i = 0;i <= 0x7e;i++)
    {
      unsigned int ret = readByte(i);
      Serial.print(i,HEX);
      Serial.print(" : ");
      Serial.println(ret,HEX);
    }
    
    //while (1){
      digitalWrite(err_LED,HIGH);
      delay(100);
      digitalWrite(err_LED,LOW);
      delay(100);
    //}
  }
  */
  delay(1000);
}

void measure(void)
{
  //加速度計測
  uint8_t ax_h = readByte(59);
  uint8_t ax_l = readByte(60);
  uint8_t ay_h = readByte(61);
  uint8_t ay_l = readByte(62);
  uint8_t az_h = readByte(63);
  uint8_t az_l = readByte(64);
  long ax = ax_h;
  ax = ax*0x100 + ax_l;
  float acx = 0;
  if(bitRead(ax,15) == 1)
  {
    acx = 0b111111111111111 - bitClear(ax,15);
    acx = -1*acx / 2048;
  }
  else{
    acx = ax;
    acx = acx / 2048;
  }
  
  long ay = ay_h;
  ay = ay*0x100 + ay_l;
  float acy = 0;
  if(bitRead(ay,15) == 1)
  {
    acy = 0b111111111111111 - bitClear(ay,15);
    acy = -1*acy / 2048;
  }
  else{
    acy = ay;
    acy = acy / 2048;
  }

  long az = az_h;
  az = az*0x100 + az_l;
  float acz = 0;
    if(bitRead(az,15) == 1)
  {
    acz = 0b111111111111111 - bitClear(az,15);
    acz = -1*acz / 2048;
  }
  else{
    acz = az;
    acz = acz / 2048;
  }

  //加速度をグローバル変数に代入
  acc_x = acx;
  acc_y = acy;
  acc_z = acz;

  Serial.print(acc_x);
  Serial.print(",");
  Serial.print(acc_y);
  Serial.print(",");
  Serial.print(acc_z);
  Serial.print("\r\n");


  //ジャイロ計測
  

  

  //データ書き込み
  dataFile = SD.open("DataLog.txt", FILE_WRITE);

  if (dataFile) {
    dataFile.print(acc_x);
    dataFile.print(",");
    dataFile.print(acc_y);
    dataFile.print(",");
    dataFile.print(acc_z);
    dataFile.print("\r\n");
    dataFile.close();
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening datalog.txt");
  }


  //ここからは取得データからの動作

  int i = 0;

  if(S1_pos >= 180)S1_pos = 180;
  if(S1_pos <= 0)S1_pos = 0;
  for(i = 0;i<= 4;i++)S1_parray[i+1] = S1_parray[i];
  S1_parray[0] = (acc_x * Gain);
  S1_pos = S1_pos + (S1_parray[0] + S1_parray[1] + S1_parray[3] + S1_parray[4])/4;
  S1.write(S1_pos);
  Serial.print(S1_pos);
  

  if(S2_pos >= 180)S2_pos = 180;
  if(S2_pos <= 0)S2_pos = 0;
  for(i = 0;i<= 4;i++)S2_parray[i+1] = S2_parray[i];
  S2_parray[0] = (acc_y * Gain);
  S2_pos = S2_pos + (S2_parray[0] + S2_parray[1] + S2_parray[3] + S2_parray[4])/4;
  S2.write(S2_pos);
  Serial.println(S2_pos);

  
  if(acc_x >= 1.1){
    digitalWrite(buzzer,HIGH);
    digitalWrite(over_G_LED,HIGH);
  }
  else{
    digitalWrite(buzzer,LOW);
    digitalWrite(over_G_LED,LOW);
  }
  
}


uint8_t readByte(uint8_t regAddr) {
  uint8_t data;
  digitalWrite(CS, LOW);
  SPI.transfer(regAddr | 0x80);   //Enter Read Mode
  data = SPI.transfer(0x00);
  digitalWrite(CS, HIGH);
  return data;
}

void writeByte(uint8_t regAddr, uint8_t value) {
  digitalWrite(CS, LOW);
  SPI.transfer(regAddr);        //Enter Write Mode
  SPI.transfer(value);
  digitalWrite(CS, HIGH);
}