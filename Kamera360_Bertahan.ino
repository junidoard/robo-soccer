#include <MsTimer2.h>
#include <LiquidCrystal.h>
#include <EEPROM.h>
#include <SoftwareSerial.h>
#include <Ultrasonic.h>

SoftwareSerial Serial4(A15, 51); // RX, TX
SoftwareSerial Serial5(50, 52); // RX, TX
SoftwareSerial Serial6(46, 48); // RX, TX

LiquidCrystal lcd(34, 36, 38, 40, 42, 44);

#define sw6 41
#define sw5 33
#define sw4 35
#define sw3 37
#define sw2 39
#define sw1 31

#define pwmL1 11
#define pwmL2 10
#define pwmR1 5
#define pwmR2 4
#define pwmB1 7
#define pwmB2 6
#define pwmD1 13
#define pwmD2 12

#define IR 27

#define kick 45
#define kick1 49
#define kick2 47

#define intPinL 3
#define intPinR 20
#define intPinB 21
#define digPinL 30
#define digPinR 26
#define digPinB 28

//#define intPinL 18
//#define intPinR 19
//#define digPinL 24
//#define digPinR 22

#define adcJarBola A0
#define adcJarL A4
#define adcJarD A5
#define adcJarR A11
#define adcGaris A9

#define sel1 23 //49
#define sel2 25 //47
#define sel3 27 //45

#define addTimeSerang 1
#define addTimeTengah 2
#define addTimeBlkang 3
#define addTeam 4

float const maxRPM = 530;

//float const rRobot=0.195; //19.5 cm
//float const dRobot=0.39; //39 cm
float const rRobot = 0.185; //18.5 cm
float const dRobot = 0.37; //37 cm
float const perRotasi = 270; //270 pulsa
float const perRotasi1 = 800; //270 pulsa
float const dRoda = 10; //10 cm
float const dRoda1 = 6.5; //6.5 cm
float const kelRoda = (dRoda * 22 / 7);
float const kelRoda1 = (dRoda1 * 22 / 7);
float const jarPerPulsa = kelRoda / perRotasi;
float const jarPerPulsa1 = kelRoda1 / perRotasi1;
int const jarbo = 3;

float   rpmL, rpmR, rpmB, pwmL, pwmR, pwmB, jarL, jarR, jarB, jarL1, jarR1, jarB1, kecL, kecR, kecB, errorL, errorR, errorB, Theta, T, rotpsL, rotpsR, dataGyro,
        rpsL, rpsR, rpsB, error, error1, lastError, lastError1, PID, mmpsL, mmpsR, mmpsB, errorL1, errorR1, errorB1, errorIL, errorIR, errorIB, errorIB1, radpsL, radpsR,
        vL, vR, vB, vX, vY, vT, iX, iY, iT, derajat, errorX, errorY, errorX1, errorY1, errorT, xTarget, yTarget, tTarget;

float jarX = 0, jarY = 0, jarXRosi = 0, jarYRosi = 0, tRosi;

String inString = "";
int xBola = -1, yBola = -1, xBola360 = -1, yBola360 = -1, xBola1, yBola1, xBola2, yBola2, xBolaH, yBolaH, m, mm, tt, a, titikTuju, thetaNow, tengahX, tengahY, countPos, kir, kan, bel;
int xLawan = -1, yLawan = -1, xLawan1, yLawan1;
float dBola, rBola, dBola1, rBola1, rLawan, dLawan;

char data;
long count, L, R, B, intL, intR, intB, frekL, frekR, frekB;
int spX = 410, spY = 590, spX360 = 410, spY360 = 200, spX1 = 390, spY1 = 300; //spX=400, spY=430;
int spXLawan = 410, spYLawan = 200;//belum fix
int posisiBola, detekLawan = 0, detekBola = 0, detekBola360 = 0, countRun, countTendang, countManual, detikManual, detikRun, detikBertahan, detikRunBertahan, detikRunSerang, countDetek, countGaDetek;
byte  pilihTendang = 1, enablePosisioning = 'N', enableMain = 'N', enableGyro = 1, strategi = 1, strategiKick = 1, dapatBola = 'N', dapatLawan = 'N', intruksi = 'S', lihatBolaRosi = 'Y',
      intruksiFull = 'S', intruksiR = 'S', intruksiPlay = 'K', team, enableReadThetaNow, runIntruksi = 'Y', enableDetek = 'N', bolehMain = 'N', lihatBola = 'Y', lihatLawan = 'Y',
      koreksiBola = 'N', sudahMundur = 'N', enableScanBola = 'N', enableKickOff = 'N', enablePenalty = 'N', enableGoalKick = 'N', enableInisial = 'N';
byte  enableBertahan = 'Y', enableSerang = 'Y', enableBertahanSerang = 'Y';
byte  sudahTendang = 'N', belumMundur = 'Y', runBertahan = 'Y', bertahanKickOff = 'N'; 
unsigned char timeSerang, timeTengah, timeBlkang;

Ultrasonic ultbeL(A3), ultdeL(A2), ultki(A1), ultbeR(A12), ultdeR(A13), ultka(A14);

const char *tampilEnableGyro[2] = {
  "Off",
  "On "
};

const char *tampilStart[10] = {
  "Auto Play C ",
  "Auto Play M ",
  "Auto Cyan   ",
  "Auto Magenta",
  "Mnual Serang",
  "Mnual Tengah",
  "Mnual Blkang",
};

const char *tampilPlayer[7] = {
  "Serang",
  "Tengah",
  "Blkang",
};

const char *tampilTest[10] = {
  "PID BolaA ",
  "PID BolaB ",
  "Kinematik ",
  "Kick1     ",
  "Kick2     ",
  "Umpan     ",
  "keDepan   ",
  "keDepan1  ",
  "keBelakang",
  "Manual    ",
};

void setup() {
  pinMode(pwmL1, OUTPUT);
  pinMode(pwmL2, OUTPUT);
  pinMode(pwmR1, OUTPUT);
  pinMode(pwmR2, OUTPUT);
  pinMode(pwmB1, OUTPUT);
  pinMode(pwmB2, OUTPUT);
  pinMode(pwmD1, OUTPUT);
  pinMode(pwmD2, OUTPUT);

  pinMode(sel1, OUTPUT);
  pinMode(sel2, OUTPUT);
  pinMode(sel3, OUTPUT);

  pinMode(kick, OUTPUT);
  pinMode(kick1, OUTPUT);
  pinMode(kick2, OUTPUT);

  pinMode(9, OUTPUT);
  pinMode(8, OUTPUT);
  digitalWrite(9, 1);
  digitalWrite(10, 1);

  MsTimer2::set(100, interrupt0);
  MsTimer2::start();
  attachInterrupt(digitalPinToInterrupt(intPinL), interruptL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(intPinR), interruptR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(intPinB), interruptB, CHANGE);

  lcd.begin(20, 4);
  Serial.begin(9600);
  Serial1.begin(9600);
  Serial2.begin(9600);
  Serial3.begin(38400);
  Serial4.begin(9600);
  Serial5.begin(9600);
  Serial6.begin(9600);

  pinMode(IR, INPUT_PULLUP);
  pinMode(sw1, INPUT_PULLUP);
  pinMode(sw2, INPUT_PULLUP);
  pinMode(sw3, INPUT_PULLUP);
  pinMode(sw4, INPUT_PULLUP);
  pinMode(sw5, INPUT_PULLUP);
  pinMode(sw6, INPUT_PULLUP);

  pinMode(intPinL, INPUT_PULLUP);
  pinMode(intPinR, INPUT_PULLUP);
  pinMode(intPinB, INPUT_PULLUP);
  pinMode(digPinL, INPUT_PULLUP);
  pinMode(digPinR, INPUT_PULLUP);
  pinMode(digPinB, INPUT_PULLUP);

  digitalWrite(kick, HIGH);
  digitalWrite(kick1, HIGH);
  digitalWrite(kick2, HIGH);

  motor(0, 0, 0);

  timeSerang = EEPROM.read(addTimeSerang);
  timeTengah = EEPROM.read(addTimeTengah);
  timeBlkang = EEPROM.read(addTimeBlkang);
  team = EEPROM.read(addTeam);
}

byte baruReset = 'Y';

void loop() {
  if (baruReset == 'Y') {
    jarX = 0;
    jarY = 0;
    Serial2.print(String("#?,") + (int)jarX + String(",") + (int)jarY + String(",") + (int)Theta + String(",") + (int)xBola + String(",") + (int)yBola + String(",\n"));
    Serial2.print(String("*!,") + (int)xTarget + String(",") + (int)yTarget + String(",") + (int)tTarget + String(",\n"));
    runFullAuto();
    motor(0, 0, 0);
  }
  baruReset = 'N';

  parseBola();

  lcd.setCursor(0, m); lcd.print(">");

  lcd.setCursor(1, 0); lcd.print("Start: "); lcd.print(tampilStart[mm]);
  lcd.setCursor(1, 1); lcd.print("Gyro : "); lcd.print(Theta); lcd.print("  ");
//  lcd.setCursor(1, 2); lcd.print("Setting Bertahan");
  lcd.setCursor(1, 2); lcd.print("Test : "); lcd.print(tampilTest[tt]);
  lcd.setCursor(1, 3); 
  lcd.print("x"); lcd.print(xBola); lcd.print(" y"); lcd.print(yBola);
  lcd.print(" X"); lcd.print(xBola360); lcd.print(" Y"); lcd.print(yBola360); lcd.print("  ");
  
  if (U()) {
    lcd.setCursor(0, m);
    lcd.print(" ");
    m--;
    delay(150);
  }
  if (D()) {
    lcd.setCursor(0, m);
    lcd.print(" ");
    m++;
    delay(150);
  }
  if (P() && m == 0) {
    mm--;
    delay(150);
  }
  if (M() && m == 0) {
    mm++;
    delay(150);
  }
  if (P() && m == 2) {
    tt--;
    delay(150);
  }
  if (M() && m == 2) {
    tt++;
    delay(150);
  }

  if (m < 0) {
    m = 3;
  } if (m > 3) {
    m = 0;
  }
  if (mm < 2) {
    mm = 3;
  } if (mm > 3) {
    mm = 2;
  }
  if (tt < 0) {
    tt = 5;
  } if (tt > 5) {
    tt = 0;
  }

  if (C() && m == 1) {
    Serial3.println("0#");
    jarX = 0;
    jarY = 0;
    jarL = 0;
    jarR = 0;
    jarB = 0;
  }
  else if (C()) {
    dribble(-255);
    delay(200);
    dribble(0);
    motor(0, 0, 0);
  }

//  if (O() && m == 2) { setting(); }
  if (O() && m == 3) { cekSensor(); }

  if (O() && m == 0 && mm == 2) {
    team = 'c';
    EEPROM.write(addTeam, team);
    runFullAuto();
  }
  if (O() && m == 0 && mm == 3) {
    team = 'm';
    EEPROM.write(addTeam, team);
    runFullAuto();
  }

  if (O() && m == 2 && tt == 0) { testPidBola360(); }
  if (O() && m == 2 && tt == 1) { testPidBolaBertahan();
    //testPidBolaXY(); 
  }
  if (O() && m == 2 && tt == 2) { testRunKinematik(); }
  if (O() && m == 2 && tt == 3) { tendang1(); }
  if (O() && m == 2 && tt == 4) { tendang2(); }
  if (O() && m == 2 && tt == 5) { tendang3(); }
  if (O() && m == 2 && tt == 6) { manualKontrol(); }

  count++;
  if (count > 20) {
    //    Serial2.print(String("#?,") + (int)jarX + String(",") + (int)jarY + String(",") + (int)Theta + String(",\n"));
    Serial2.print(String("#?,") + (int)jarX + String(",") + (int)jarY + String(",") + (int)Theta + String(",") /*+ (int)lihatBola*/+ (int)xBola + String(",") + (int)yBola + String(",\n"));
    Serial2.print(String("*!,") + (int)xTarget + String(",") + (int)yTarget + String(",") + (int)tTarget + String(",\n"));

    count = 0;
  }

  listenCoach();
}

void interrupt0() {
  jarL = jarPerPulsa1 * frekL;
  jarR = jarPerPulsa1 * frekR;

  rpmL = frekL * 600 / perRotasi1;
  rpmR = frekR * 600 / perRotasi1;
  rpmB = frekB * 600 / perRotasi;

  mmpsL = kelRoda * rpmL / 60;
  mmpsR = kelRoda * rpmR / 60;
  mmpsB = kelRoda * rpmB / 60;
    
  //1 rpm = 0.104719755 rps rpm(rotasi per menit) rps(radian per seconds)
  //1 rotasi = 360 derajat = 2 pi radian
  //1 rotasi = 360*pi/180 = 2 pi radian
  //1 rps = 2 pi/60 radian/sec
  //1 rps = 0.104719755 rpm

  rpsL = rpmL * 0.104719755;
  rpsR = rpmR * 0.104719755;
  rpsB = rpmB * 0.104719755;

  updateJarak();
  //updateJarak1();

  frekL = frekR = frekB = 0;

  countRun++;
  if (countRun > 10) {
    detikRun++;
    detikBertahan++;
    detikRunBertahan++;
    detikRunSerang++;
    countRun = 0;
  }

  countManual++;
  countDetek++;
  countGaDetek++;
  countTendang++;
}
void interruptL() {
  if (digitalRead(intPinL) != digitalRead(digPinL)) {
    intL++;
    frekL++;
  }
  else {
    intL--;
    frekL--;
  }
}
void interruptR() {
  if (digitalRead(intPinR) != digitalRead(digPinR)) {
    intR++;
    frekR++;
  }
  else {
    intR--;
    frekR--;
  }
}
void interruptB() {
  if (digitalRead(intPinB) != digitalRead(digPinB)) {
    intB++;
    frekB++;
  }
  else {
    intB--;
    frekB--;
  }
}

bool O() {
  if (digitalRead(sw1) == 0) {
    return true;
  }
  else {
    return false;
  }
}
bool D() {
  if (digitalRead(sw2) == 0) {
    return true;
  }
  else {
    return false;
  }
}
bool U() {
  if (digitalRead(sw3) == 0) {
    return true;
  }
  else {
    return false;
  }
}
bool P() {
  if (digitalRead(sw4) == 0) {
    return true;
  }
  else {
    return false;
  }
}
bool M() {
  if (digitalRead(sw5) == 0) {
    return true;
  }
  else {
    return false;
  }
}
bool C() {
  if (digitalRead(sw6) == 0) {
    return true;
  }
  else {
    return false;
  }
}
bool irSw() {
  if (digitalRead(IR) == 0) {
    return true;
  }
  else {
    return false;
  }
}

void motor(int L, int R, int B) {
  if (L > 0) {
    analogWrite(pwmL1, L);
    digitalWrite(pwmL2, 0);
  } else if (L < 0) {
    analogWrite(pwmL1, 255 + L);
    digitalWrite(pwmL2, 1);
  } else {
    digitalWrite(pwmL1, 0);
    digitalWrite(pwmL2, 0);
  }

  if (R > 0) {
    analogWrite(pwmR1, R);
    digitalWrite(pwmR2, 0);
  } else if (R < 0) {
    analogWrite(pwmR1, 255 + R);
    digitalWrite(pwmR2, 1);
  } else {
    digitalWrite(pwmR1, 0);
    digitalWrite(pwmR2, 0);
  }

  if (B > 0) {
    analogWrite(pwmB1, B);
    digitalWrite(pwmB2, 0);
  } else if (B < 0) {
    analogWrite(pwmB1, 255 + B);
    digitalWrite(pwmB2, 1);
  } else {
    digitalWrite(pwmB1, 0);
    digitalWrite(pwmB2, 0);
  }
  pwmL = L;
  pwmR = R;
  pwmB = B;
}
void maju(int L, int R) {
  motor(-L, R, 0);
}
void mundur(int L, int R) {
  motor(L, -R, 0);
}
void geserKiri(int L) {
  motor(L / 2, L / 2, -L);
}
void geserKanan(int R) {
  motor(-R / 2, -R / 2, R);
}
void berhenti() {
  motor(0, 0, 0);
}

void majuGyro(int L, int R, int yaw) {
  float kp = 2, ki = 0, kd = 3 , PID;

  int error = yaw - Theta;

  PID = (kp * error) + (kd * (error - error1));

  error1 = error;

}

void tendang() {
  //  if(countTendang>100){
  //    digitalWrite(kick,HIGH);
  //    delay(300);
  //    digitalWrite(kick,LOW);
  //    countTendang=0;
  //  }
  //tendang1();
  if (pilihTendang == 1 && countTendang > 5) {
    tendang1();
    countTendang = 0;
    pilihTendang = 2;
  }
  else if (pilihTendang == 2 && countTendang > 5) {
    tendang2();
    countTendang = 0;
    pilihTendang = 1;
  }

}
void tendang1() {
  digitalWrite(kick, LOW);
  delay(300);
  digitalWrite(kick, HIGH);
  //  countTendang=0;
}
void tendang2() {
  digitalWrite(kick1, LOW);
  delay(300);
  digitalWrite(kick1, HIGH);
  //  countTendang=0;
}
void tendang3() {
  digitalWrite(kick2, LOW);
  delay(300);
  digitalWrite(kick2, HIGH);
  //  countTendang=0;
}

void dribble(int PWM) {
  if (PWM > 0) {
    analogWrite(pwmD1, PWM);
    digitalWrite(pwmD2, 0);
  } else if (PWM < 0) {
    digitalWrite(pwmD1, 0);
    analogWrite(pwmD2, -PWM);
  } else {
    digitalWrite(pwmD1, 0);
    digitalWrite(pwmD2, 0);
  }
}

void parseGyro() {
  int dataG;
  //  while (Serial2.available() > 0) {
  //    int inChar = Serial2.read();
  //  Serial3.listen();
  while (Serial3.available() > 0) {
    int inChar = Serial3.read();

    if (inChar != '\n') {
      inString += (char)inChar;
    }
    else {
      dataGyro = inString.toFloat();
      inString = "";
    }
  }
  //  int data;
  //
  //  char getData = Serial2.read();
  //
  //  if (getData == 'g') {
  //    data = Serial2.parseInt();
  //    if (Serial2.read() == '#') {
  //       dataGyro=data;
  //    }
  //  }
}

void parseBola() {
  //  while (Serial.available() > 0) {
  //    int inChar = Serial.read();
  //
  //    if (inChar != '\n') {
  //      inString += (char)inChar;
  //    }
  //    else {
  //      xBola = inString.toFloat();
  //      inString = "";
  //    }
  //  }

  int data_x, data_y, data_x1, data_y1, data_x2, data_y2;

  char getData = Serial1.read();

  //  Serial.println(getData);
  //  X kamera 360 dan x kamera depan

  if (getData == 'x') {
    data_x1 = Serial1.parseInt();
    //    Serial.print(data_x);
    if (Serial1.read() == 'y') {
      data_y1 = Serial1.parseInt();
      //       Serial.println(data_y);
      if (Serial1.read() == '*') {
        xBola360 = data_x1;
        yBola360 = data_y1;
      }
      if (xBola >= 0 || xBola360 >= 0) {
        lihatBola = 1;
      }
      else {
        lihatBola = 0;
      }
    }
  }
  if (getData == 'X') {
    data_x = Serial1.parseInt();
    //    Serial.print(data_x);
    if (Serial1.read() == 'Y') {
      data_y = Serial1.parseInt();
      //       Serial.println(data_y);
      if (Serial1.read() == '*') {
        xBola = data_x;
        yBola = data_y;
      }
      if (xBola >= 0 || xBola360 >= 0) {
        lihatBola = 1;
      }
      else {
        lihatBola = 0;
      }
    }
  }
  if (getData == 'H') {
    data_x2 = Serial1.parseInt();
    //    Serial.print(data_x);
    if (Serial1.read() == 'h') {
      data_y2 = Serial1.parseInt();
      //       Serial.println(data_y);
      if (Serial1.read() == '*') {
        xLawan = data_x2;
        yLawan = data_y2;
      }
      if (xLawan >= 0) {
        lihatLawan = 1;
      }
      else {
        lihatLawan = 0;
      }
    }
  }
}

void kameraHp() {
  int data_x, data_y;
  String f, h;

  Serial4.listen();
  char getData = Serial4.read();

  if (getData == 'x') {
    data_x = Serial4.parseInt();
    if (Serial4.read() == 'y') {
      data_y = Serial4.parseInt();
      if (Serial4.read() == '*') {
        xBola = data_x;
        yBola = data_y;
      }
    }
  }
}

void runXYPwm(int X, int Y, int W, int maxx) {

  vL = -(sin(radians(150)) * X) + (cos(radians(150)) * Y) + (W);
  vR = -(sin(radians(30)) * X) + (cos(radians(30)) * Y) + (W);
  vB = -(sin(radians(270)) * X) + (cos(radians(270)) * Y) + (W);

  if (vL > maxx) {
    vL = maxx;
  }
  if (vR > maxx) {
    vR = maxx;
  }
  if (vB > maxx) {
    vB = maxx;
  }
  if (vL < -maxx) {
    vL = -maxx;
  }
  if (vR < -maxx) {
    vR = -maxx;
  }
  if (vB < -maxx) {
    vB = -maxx;
  }

  motor(vL, vR, vB);
}

//peke 2 rotary (kiri dan kanan) 90 derajat
void updateJarak() {
  float vX, vY, w;
  float offsetX = 0.99, offsetY = 1.02; //x=400 y=600 eksternal fixx y belum coba x
  //float offsetX = 5.8, offsetY = 3.5; //fix x=250 y=400
  //float offsetX = 5.8, offsetY = 3.9; //fix x=250 y=450
  //float offsetX = 4.5, offsetY = 3.5; //fix x=200 y=400
  //float offsetX = 4.5, offsetY = 3.7; //fix x=200 y=425
  //float offsetX = 4.5, offsetY = 3.9; //fix x=200 y=450
  //float offsetX = 4.5, offsetY = 3.29; //fix x=200 y=375

  vX = (-cos(radians(45)) * jarR - cos(radians(45)) * jarL);
  vY = (sin(radians(45)) * jarR - sin(radians(45)) * jarL);

  jarX += (cos(radians(Theta)) * vX + sin(radians(Theta)) * vY) * offsetX;
  jarY += (-sin(radians(Theta)) * vX + cos(radians(Theta)) * vY) * offsetY;

  parseGyro(); //Theta=dataGyro;
  if (dataGyro < 0) {
    Theta = 360 + dataGyro;
  } else {
    Theta = dataGyro;
  }
  if (Theta > 180) {
    Theta = Theta - 360;
  }
}

//pake 3 rotary (kiri, kanan dan belakang)
void updateJarak1() {
  float vX, vY, w;
  float offsetX = 1.05, offsetY = 1.05;
  float acuan = 1.071;

  vX = ((2 * rpsB - rpsL - rpsR) / 6.85);      //dibagi 6.85 atau 2
  vY = ((sqrt(3) * rpsR - sqrt(3) * rpsL) / 6.85); //dibagi 6.85 atau 2
  w  = (-(rpsL + rpsR + rpsB) / acuan) / 2;

  jarX += (cos(radians(Theta)) * vX + sin(radians(Theta)) * vY) * offsetX;
  jarY += (-sin(radians(Theta)) * vX + cos(radians(Theta)) * vY) * offsetY;

  parseGyro(); //Theta=dataGyro;
  if (dataGyro < 0) {
    Theta = 360 + dataGyro;
  } else {
    Theta = dataGyro;
  } if (Theta > 180) {
    Theta = Theta - 360;
  }
}

void displayJarak() {
  lcd.setCursor(0, 1);
  lcd.print("X"); lcd.print((int)jarX);
  lcd.print(" Y"); lcd.print((int)jarY);
  lcd.print(" T"); lcd.print((int)Theta); lcd.print("   ");
}

void runKinematik1(int x, int y, int t, int maxPWM) {
  float kpXY = 2.2, kiXY = 0.06, kdXY = 0;
  float kpT = 2.2, kiT = 0.05, kdT = 0, maxRPM = maxPWM;
  float eTheta;
  
  errorX = x - jarX;
  errorY = y - jarY;
  if(t>165 && t<-165){
    if(Theta<0){
      eTheta = 360+Theta;
    }
    errorT = eTheta - t;
  }
  else {errorT = Theta - t;}

  long r = sqrt(((errorX / 10) * (errorX / 10)) + ((errorY / 10) * (errorY / 10)));
  r *= 10;

  if (r < 10) {
    iX += errorX;
    iY += errorY;
    if (errorT < 15 && errorT > -15 ) {
      iT += errorT;
    }
  }

  vX = errorX * kpXY + (iX * kiXY);
  vY = errorY * kpXY + (iY * kiXY);
  vT = errorT * kpT + (iT * kiT);

  vB = ((vY) * (-sin(radians(Theta)))) + ((vX) * (cos(radians(Theta)))) + ((vT / 3));
  vL = ((vY) * (-sin(radians(60 - Theta)))) + ((vX) * (-cos(radians(60 - Theta)))) + ((vT / 3));
  vR = ((vY) * ( sin(radians(60 + Theta)))) + ((vX) * (-cos(radians(60 + Theta)))) + ((vT / 3));

  if (vL > maxRPM) {
    vL = maxRPM;
  }
  if (vR > maxRPM) {
    vR = maxRPM;
  }
  if (vB > maxRPM) {
    vB = maxRPM;
  }
  if (vL < -maxRPM) {
    vL = -maxRPM;
  }
  if (vR < -maxRPM) {
    vR = -maxRPM;
  }
  if (vB < -maxRPM) {
    vB = -maxRPM;
  }

  if ((errorX < 5 && errorX > -5) && (errorY < 5 && errorY > -5) && (errorT < 5 && errorT > -5)) {
    motor(0, 0, 0);
    iX = iY = iT = 0;
    titikTuju = 1;
  }
  else {
    motor(vL, vR, vB);
    titikTuju = 0;
  }
}

void runKinematik(int x, int y, int t, int maxPWM) {
  float kpXY = 3.3, kiXY = 0.095, kdXY = 0;
  float kpT = 2.8, kiT = 0.07, kdT = 0, maxRPM = maxPWM;
  float eTheta, pTheta,mTheta, batasPlus, batasMin;

  errorX = x - jarX;
  errorY = y - jarY;

  if(Theta < 0){pTheta = 360 + Theta;}
  else{pTheta = Theta;}
  mTheta = Theta;
  
  if(t<0){ eTheta = 360 + t; }
  else{ eTheta = t; }

  if(pTheta > eTheta){errorT = pTheta - eTheta;}
  else {errorT = Theta - t;}
  
//  errorT = (int)(pTheta - eTheta);
  if(errorT>180){errorT =(int) (errorT - 360);}
      
  long r = sqrt(((errorX / 10) * (errorX / 10)) + ((errorY / 10) * (errorY / 10)));
  r *= 10;

  if (r < 15) {
    iX += errorX;
    iY += errorY;
    if (errorT < 15 && errorT > -15) { iT += errorT; }
  }else{
    iX=iY=iT=0;
  }

  vX = errorX * kpXY + (iX * kiXY);
  vY = errorY * kpXY + (iY * kiXY);
  vT = errorT * kpT + (iT * kiT);

  if(vX>0 && vY>0){ //Kuadran I
    if(vX>maxPWM && vX>vY){
      vY=maxPWM*vY/vX;
      vX=maxPWM;
    }else if(vY>maxPWM && vY>vX){
      vX=maxPWM*vX/vY;
      vY=maxPWM;
    } 
  }else if(vX<0 && vY>0){ //Kuadran II
    if(-vX>maxPWM && -vX>vY){
      vY=maxPWM*vY/-vX;
      vX=-maxPWM;
    }else if(vY>maxPWM && vY>-vX){
      vX=maxPWM*vX/vY;
      vY=maxPWM;
    } 
  }else if(vX>0 && vY<0){ //Kuadran III
    if(vX>maxPWM && vX>-vY){
      vY=maxPWM*vY/vX;
      vX=maxPWM;
    }else if(-vY>maxPWM && -vY>vX){
      vX=maxPWM*vX/-vY;
      vY=-maxPWM;
    }
  }else if(vX<0 && vY<0){ //Kuadran IV
    if(-vX>maxPWM && -vX>-vY){
      vY=maxPWM*vY/-vX;
      vX=-maxPWM;
    }else if(-vX>maxPWM && -vY>-vX){
      vX=maxPWM*vX/-vY;
      vY=-maxPWM;
    } 
  }

  vB = ((vY) * (-sin(radians(Theta)))) + ((vX) * (cos(radians(Theta)))) + ((vT / 3));
  vL = ((vY) * (-sin(radians(60 - Theta)))) + ((vX) * (-cos(radians(60 - Theta)))) + ((vT / 3));
  vR = ((vY) * ( sin(radians(60 + Theta)))) + ((vX) * (-cos(radians(60 + Theta)))) + ((vT / 3));

  if (vL > maxRPM) {
    vL = maxRPM;
  }
  if (vR > maxRPM) {
    vR = maxRPM;
  }
  if (vB > maxRPM) {
    vB = maxRPM;
  }
  if (vL < -maxRPM) {
    vL = -maxRPM;
  }
  if (vR < -maxRPM) {
    vR = -maxRPM;
  }
  if (vB < -maxRPM) {
    vB = -maxRPM;
  }

  if ((errorX < 4 && errorX > -4) && (errorY < 4 && errorY > -4) && (errorT < 2 && errorT > -2)) {
    motor(0, 0, 0);
    iX = iY = iT = 0;
    titikTuju = 1;
  }
  else {
    motor(vL, vR, vB);
    titikTuju = 0;
  }

//  lcd.setCursor(0,2);
//  lcd.print("e"); lcd.print((int)errorT);  lcd.print(" e"); lcd.print((int)eTheta);  lcd.print(" p"); lcd.print((int)pTheta);  lcd.print("   "); 
}

void testRunKinematik() {
  while (!C()) {
    lcd.clear();
    while (!C() && titikTuju != 1) {
      runKinematik(0, 200, 0, 50);
      displayJarak();
    } titikTuju = 0;
    while (!C() && titikTuju != 1) {
      runKinematik(200, 200, 0, 50);
      displayJarak();
    } titikTuju = 0;
    while (!C() && titikTuju != 1) {
      runKinematik(0, 200, 0, 50);
      displayJarak();
    } titikTuju = 0;
    while (!C() && titikTuju != 1) {
      runKinematik(-200, 200, 0, 50);
      displayJarak();
    } titikTuju = 0;
    while (!C() && titikTuju != 1) {
      runKinematik(0, 200, 0, 50);
      displayJarak();
    } titikTuju = 0;
    while (!C() && titikTuju != 1) {
      runKinematik(0, 0, 0, 50);
      displayJarak();
    } titikTuju = 0;
  }
  motor(0, 0, 0); delay(300); lcd.clear();
}

void testRunKinematik1() {
    while (!C()) {
    lcd.clear();
    while (!C() && titikTuju != 1) {
      runKinematik(0, 450, 0, 100);
      displayJarak();
    } titikTuju = 0;
//    while (!C() && titikTuju != 1) {
//      runKinematik(200, 200, 0, 50);
//      displayJarak();
//    } titikTuju = 0;
//    while (!C() && titikTuju != 1) {
//      runKinematik(0, 200, 0, 50);
//      displayJarak();
//    } titikTuju = 0;
//    while (!C() && titikTuju != 1) {
//      runKinematik(-200, 200, 0, 50);
//      displayJarak();
//    } titikTuju = 0;
//    while (!C() && titikTuju != 1) {
//      runKinematik(0, 200, 0, 50);
//      displayJarak();
//    } titikTuju = 0;
    while (!C() && titikTuju != 1) {
      runKinematik(0, 0, 0, 100);
      displayJarak();
    } titikTuju = 0;
  }
  motor(0, 0, 0); delay(300); lcd.clear();
}

void setting() {
  delay(300); lcd.clear();
  int mm = 0, p = 0;
  char ok;

  while (!C()) {
    errorIL = 0;
    errorIR = 0;
    errorIB = 0;
    countDetek = 0;
    countGaDetek = 0;

    lcd.setCursor(0, a); lcd.print(">");

    lcd.setCursor(1, 0); lcd.print("Set PID Bola");
    lcd.setCursor(1, 1); lcd.print("Set Jarak");
    lcd.setCursor(1, 2); lcd.print("Time "); lcd.print(tampilPlayer[p]);
    lcd.setCursor(1, 3); lcd.print("Test : "); lcd.print(tampilTest[mm]);

    lcd.setCursor(13, 2);
    if (ok == 1) {
      if (p == 0) {
        lcd.print(":[");
        lcd.print(timeSerang);
      }
      else if (p == 1) {
        lcd.print(":[");
        lcd.print(timeTengah);
      }
      else if (p == 2) {
        lcd.print(":[");
        lcd.print(timeBlkang);
      }
      lcd.print("] ");

      if (P() && p == 0) {
        timeSerang++;
        delay(120);
      }
      if (M() && p == 0) {
        timeSerang--;
        delay(120);
      }
      if (P() && p == 1) {
        timeTengah++;
        delay(120);
      }
      if (M() && p == 1) {
        timeTengah--;
        delay(120);
      }
      if (P() && p == 2) {
        timeBlkang++;
        delay(120);
      }
      if (M() && p == 2) {
        timeBlkang--;
        delay(120);
      }

      EEPROM.write(addTimeSerang, timeSerang);
      EEPROM.write(addTimeTengah, timeTengah);
      EEPROM.write(addTimeBlkang, timeBlkang);
    }
    else {
      lcd.print(": ");
      if (p == 0) {
        lcd.print(timeSerang);
      }
      else if (p == 1) {
        lcd.print(timeTengah);
      }
      else if (p == 2) {
        lcd.print(timeBlkang);
      }
      lcd.print("  ");
    }


    if (U() && ok == 0) {
      lcd.setCursor(0, a);
      lcd.print(" ");
      a--;
      delay(150);
    }
    if (D() && ok == 0) {
      lcd.setCursor(0, a);
      lcd.print(" ");
      a++;
      delay(150);
    }
    if (P() && a == 2 && ok == 0) {
      p--;
      delay(150);
    }
    if (M() && a == 2 && ok == 0) {
      p++;
      delay(150);
    }
    if (P() && a == 3) {
      mm--;
      delay(150);
    }
    if (M() && a == 3) {
      mm++;
      delay(150);
    }
    if (a > 3) {
      a = 0;
    } if (a < 0) {
      a = 3;
    }
    if (p > 2) {
      p = 0;
    } if (p < 0) {
      p = 2;
    }
    if (mm > 8) {
      mm = 0;
    } if (mm < 0) {
      mm = 8;
    }

    if (O() && a == 2) {
      ok++;
      delay(150);
    }
    if (ok > 1) {
      ok = 0;
    }

    if (O() && a == 3 && mm == 0) {
      testPidBola();
    }
    if (O() && a == 3 && mm == 1) {
      testPidBolaXY();
    }
    if (O() && a == 3 && mm == 2) {
      testRunKinematik();
    }
    if (O() && a == 3 && mm == 3) {
      tendang1();
    }
    if (O() && a == 3 && mm == 4) {
      tendang2();
    }
    if (O() && a == 3 && mm == 5) {
      tendang3();
    }
    if (O() && a == 3 && mm == 6) {
      keTengahLapangan();
    }
    if (O() && a == 3 && mm == 7) {
      testKeDepan();
    }
    if (O() && a == 3 && mm == 8) {
      testKeBelakang();
    }
    if (O() && a == 3 && mm == 9) {
      manualKontrol();
    }

  }
  delay(300); lcd.clear();
}

void cekSensor() {
  delay(300); lcd.clear();
  while (!C()) {
    kameraHp();
    parseGyro();
    parseBola();
    lcd.setCursor(0, 0);
    lcd.print("D"); lcd.print(jarDeL1()); lcd.print(" ");  lcd.print(jarDribble()); lcd.print(" "); lcd.print(jarDeR1()); lcd.print("  ");

    lcd.setCursor(0, 1);
    lcd.print("D"); lcd.print(jarDeL()); lcd.print(" "); lcd.print(jarDeR()); lcd.print(" "); lcd.print(jarde()); lcd.print(" ");

    lcd.setCursor(0, 2);
    lcd.print("L"); lcd.print(jarki()); lcd.print(" B"); lcd.print(jarBeL()); lcd.print(" "); lcd.print(jarBeR()); lcd.print(" R"); lcd.print(jarka()); lcd.print("  ");

    lcd.setCursor(0, 3);
    lcd.print("L"); lcd.print((int)intL);
    lcd.print(" R"); lcd.print((int)intR);
    lcd.print(" B"); lcd.print((int)intB);  lcd.print("  ");
    //    lcd.print("L");lcd.print(jarL);
    //    lcd.print(" R");lcd.print(jarR);
    //    lcd.print(" B");lcd.print(jarB);  lcd.print("  ");
    if (P()) {
      dribble(150);
      delay(100);
    }
    else if (M()) {
      dribble(-150);
      delay(100);
    }
    else {
      dribble(0);
    }

  }
  delay(300); lcd.clear();
}

void manualKontrol() {
  int der, drib, statusMaju = 0;
  while (O()) {} lcd.clear();
  while (!C()) {
    //    lcd.setCursor(0,0); lcd.print("Manual Kontrol");

    Serial4.listen();

    int data = Serial4.read();
    lcd.setCursor(0, 0);

    if (P()) {
      jarX = 0;
      jarY = 0;
      Theta = 0;
      jarL = 0;
      jarR = 0;
      jarB = 0;
    }
    if (U()) {
      titikTuju = 0; der = Theta;
      while (!C() && titikTuju != 1) {
        runKinematik(0, 0, der, 250);
      } titikTuju = 0;
      while (!C() && titikTuju != 1) {
        runKinematik(0, 0, 0, 250);
      }
      //      while(!C() && titikTuju!=1){runKinematik(0,0,0,250,250);}
      lcd.clear(); delay(500);
    }

    if (data == 'F') {
      statusMaju = 1;
      dribble(255);
      runXYPwm(0, 180, 0, 255);
      lcd.print("Maju ");
    }
    else if (data == 'B') {
      statusMaju = 1;
      dribble(255);
      runXYPwm(0, -180, 0, 255);
      lcd.print("Mundur");
    }
    else if (data == 'L') {
      statusMaju = 1;
      dribble(255);
      runXYPwm(-180, 0, 0, 255);
      lcd.print("Geser kiri");
    }
    else if (data == 'R') {
      statusMaju = 1;
      dribble(255);
      runXYPwm(180, 0, 0, 255);
      lcd.print("Geser kanan");
    }
    else if (data == 'G') {
      statusMaju = 0;
      runXYPwm(0, 0, 100, 255);
      lcd.print("Putar kiri");
    }
    else if (data == 'I') {
      statusMaju = 0;
      runXYPwm(0, 0, -100, 255);
      lcd.print("Putar kanan");
    }
    else if (data == 'J') {
      statusMaju = 0;
      runXYPwm(0, 0, 100, 255);
      lcd.print("Putar kiri");
    }
    else if (data == 'H') {
      statusMaju = 0;
      runXYPwm(0, 0, -100, 255);
      lcd.print("Putar kanan");
    }
    else if (data == '0') {
      statusMaju = 0;
      drib = 0;
      lcd.print("Dribble off");
    }
    else if (data == '1') {
      statusMaju = 0;
      drib = 1;
      lcd.print("Dribble on");
    }
    else if (data == '2') {
      statusMaju = 0;
      drib = 2;
      lcd.print("Dribble on");
    }
    else if (data == '3') {
      statusMaju = 0;
      drib = 3;
      lcd.print("Dribble on");
    }
    else if (data == '4') {
      statusMaju = 0;
      drib = 4;
      lcd.print("Dribble on");
    }
    else if (data == '5') {
      statusMaju = 0;
      drib = 5;
      lcd.print("Dribble on");
    }
    else if (data == '6') {
      statusMaju = 0;
      drib = 6;
      lcd.print("Dribble on");
    }
    else if (data == '7') {
      statusMaju = 0;
      drib = 7;
      lcd.print("Dribble on");
    }
    else if (data == '8') {
      statusMaju = 0;
      drib = 8;
      lcd.print("Dribble on");
    }
    else if (data == '9') {
      statusMaju = 0;
      drib = 9;
      lcd.print("Dribble on");
    }
    else if (data == 'X') {
      statusMaju = 0;
      dribble(-10);
      tendang();
      lcd.print("Tendang   ");
    }
    else if (data == 'S') {
      motor(0, 0, 0);
      errorIL = 0;
      errorIR = 0;
      errorIB = 0;
      statusMaju = 0;
      lcd.print("stop         ");
    }

    if (drib == 0 && statusMaju == 0) {
      dribble(0);
    }
    else if (drib == 1 && statusMaju == 0) {
      dribble(0);
    }
    else if (drib == 2 && statusMaju == 0) {
      dribble(0);
    }
    else if (drib == 3 && statusMaju == 0) {
      dribble(0);
    }
    else if (drib == 4 && statusMaju == 0) {
      dribble(70);
    }
    else if (drib == 5 && statusMaju == 0) {
      dribble(100);
    }
    else if (drib == 6 && statusMaju == 0) {
      dribble(150);
    }
    else if (drib == 7 && statusMaju == 0) {
      dribble(170);
    }
    else if (drib == 8 && statusMaju == 0) {
      dribble(200);
    }
    else if (drib == 9 && statusMaju == 0) {
      dribble(255);
    }

    lcd.setCursor(0, 1);
    lcd.print("X"); lcd.print((int)jarX);
    lcd.print(" Y"); lcd.print((int)jarY);
    lcd.print(" T"); lcd.print((int)Theta);  lcd.print("   ");

    lcd.setCursor(0, 2);
    lcd.print("L"); lcd.print((int)jarL);
    lcd.print(" R"); lcd.print((int)jarR);
    lcd.print(" B"); lcd.print((int)jarB);  lcd.print("   ");

    lcd.setCursor(0, 3);
    lcd.print("L"); lcd.print((int)intL);
    lcd.print(" R"); lcd.print((int)intR);
    lcd.print(" B"); lcd.print((int)intB); lcd.print("   ");
  }
  lcd.clear(); motor(0, 0, 0); dribble(0);
  while (C()) {}
}

unsigned int jarDribble() {
  //  int cm, data;
  //  data = analogRead(A0);
  //  if(data>11){cm=2076/(data-11);}
  //  else {cm=500;}
  //
  //  return cm;

  unsigned int adc, jarak;
  adc = analogRead(adcJarBola);
  jarak = 2076 / (adc - 11);
  if (jarak > 99) {
    jarak = 99;
  }
  if (jarak < 1) {
    jarak = 99;
  }
  return jarak;
}
unsigned int jarDeL1() {
  //  unsigned int adc,jarak;
  //  adc = analogRead(adcJarL);
  //  if(adc>2){jarak=4800/(adc-2);}
  //  else {jarak=500;}
  //  return jarak;

  unsigned int adc, jarak;
  adc = analogRead(adcJarL);
  jarak = 4800 / (adc - 2);
  if (jarak > 99) {
    jarak = 99;
  }
  if (jarak < 1) {
    jarak = 99;
  }
  return jarak;
}
unsigned int jarDeR1() {
  //  unsigned int adc,jarak;
  //  adc = analogRead(adcJarR);
  //  if(adc>2){jarak=4800/(adc-2);}
  //  else {jarak=500;}
  //  return jarak;

  unsigned int adc, jarak;
  adc = analogRead(adcJarR);
  jarak = 4800 / (adc - 2);
  if (jarak > 99) {
    jarak = 99;
  }
  if (jarak < 1) {
    jarak = 99;
  }
  return jarak;
}
unsigned int jarde() {
  //  unsigned int adc,jarak;
  //  adc = analogRead(adcJarR);
  //  if(adc>2){jarak=4800/(adc-2);}
  //  else {jarak=500;}
  //  return jarak;

  unsigned int adc, jarak;
  adc = analogRead(adcJarD);
  jarak = 4800 / (adc - 2);
  if (jarak > 99) {
    jarak = 99;
  }
  if (jarak < 1) {
    jarak = 99;
  }
  return jarak;
}
unsigned int jarDeL() {
  int jar = ultdeL.distanceRead();
  if (jar == 0) {
    jar = 360;
  }
  return jar;
}
unsigned int jarDeR() {
  int jar = ultdeR.distanceRead();
  if (jar == 0) {
    jar = 360;
  }
  return jar;
}
unsigned int jarBeL() {
  int jar = ultbeL.distanceRead();
  if (jar == 0) {
    jar = 360;
  }
  return jar;
}
unsigned int jarBeR() {
  int jar = ultbeR.distanceRead();
  if (jar == 0) {
    jar = 360;
  }
  return jar;

}
unsigned int jarki() {
  int jar = ultki.distanceRead();
  if (jar == 0) {
    jar = 360;
  }
  return jar;
}
unsigned int jarka() {
  int jar = ultka.distanceRead();
  if (jar == 0) {
    jar = 360;
  }
  return jar;
}

void inMux(byte out_ke) {
  byte value1, value2, value3;

  value1 = 1 & (out_ke >> 0);
  value2 = 1 & (out_ke >> 1);
  value3 = 1 & (out_ke >> 2);

  digitalWrite(sel1, value1);
  digitalWrite(sel2, value2);
  digitalWrite(sel3, value3);
}

int tarX = 400, tarY = 600, tarT = 0, countPosisioning = 0;
//jarX = 400, jarY = 600;

void pidBola() {
  float kp = 1.3, kp1 = 0.6, kp2 = 1, ki = 0, kd = 0.5, kd1 = 0.6;
  int  maxSpd = 45, maxSpd1 = 400, maxPWM = 140, maxPWM1 = 200, maxSpd2 = 300, errorY, errorX, maxx = 200;
  int kir, kan, bel , PIDD;
  int runX, runY, runW;
  //  int spX=325, spY=250;

  parseBola();
  //  int JarL = jarki();
  //  int JarR = jarka();

  if (yBola >= 0) {
    errorIL = 0;
    errorIR = 0;
    errorIB = 0;
    detekBola = 1;
    countGaDetek = 0;
    errorY = spY - yBola;
  }
  else {
    errorY = 0;
  }
  if (xBola >= 0) {
    errorIL = 0;
    errorIR = 0;
    errorIB = 0;
    detekBola = 1;
    countGaDetek = 0;
    errorX = spX - xBola;
  }
  else {
    errorX = 0;
    detekBola = 0;
    countDetek = 0;
  }

  if (yBola <= 130 && yBola > -1) {
    maxx = maxPWM1;
    PID = (kp1 * errorX) + (ki * (errorX + lastError)) + (kd * (errorX - lastError));

    if (countDetek > 10) {
      kir = (150 - PID);
      kan = (150 + PID);
    }
    else if (countDetek > 7) {
      kir = (130 - PID);
      kan = (130 + PID);
    }
    else {
      kir = (100 - PID);
      kan = (100 + PID);
    }

    //    int jarKi = jarDeL1();
    //    int jarKa = jarDeR1();
    //
    //    if(jarKi<jarKa){
    //      if(jarKi<20){kir+=20; bel=90;}
    //      else{kir=kir; kan=kan; bel=0;}
    //    }
    //    else{
    //      if(jarKa<20){kan+=20; bel=-90;}
    //      else{kir=kir; kan=kan; bel=0;}
    //    }

    PIDD = kp * errorX;
  }
  else if (yBola > 130) {
    maxx = maxPWM;
    PID = (kp2 * errorX) + (ki * (errorX + lastError)) + (kd1 * (errorX - lastError));

    kir = (100 - PID);
    kan = (100 + PID);

    //    int jarKi = jarDeL1();
    //    int jarKa = jarDeR1();
    //
    //    if(jarKi<jarKa){
    //      if(jarKi<20){kir+=20; bel=90;}
    //      else{kir=kir; kan=kan; bel=0;}
    //    }
    //    else{
    //      if(jarKa<20){kan+=20; bel=-90;}
    //      else{kir=kir; kan=kan; bel=0;}
    //    }

    PIDD = kp * errorX;
  }

  if (kir > maxx) {
    kir = maxx;
  }
  if (kir < 0) {
    kir = 0;
  }
  if (kan > maxx) {
    kan = maxx;
  }
  if (kan < 0) {
    kan = 0;
  }

  if (PIDD > maxSpd) {
    PIDD = maxSpd;
  }
  if (PIDD < -maxSpd) {
    PIDD = -maxSpd;
  }

  if (jarDribble() < jarbo && xBola != -1) {
    countDetek = 0;
    dapatBola = 'Y';
    countPosisioning = 0;
  }
  else if (errorX >= -110 && errorX <= 110 && detekBola == 1) {
    motor(-kir, kan, bel);
    dapatBola = 'N';
  }
  else if (errorX < -110 || errorX > 110 && detekBola == 1) {
    kir = kan = bel = PIDD;
    motor(PIDD, PIDD, PIDD);
    dribble(0);
    countDetek = 0;
    dapatBola = 'N';
  }
  else if (countGaDetek > 100) {
    countGaDetek = 0;
    countPosisioning++;
    if (countPosisioning > 2) {
      countPosisioning = 0;
    }
  }
  else if (countGaDetek > 80) { 

    runKinematik(400, 200, 0, 100);

    //      lcd.setCursor(0,0);
    //      lcd.print("X");lcd.print((int)jarX);
    //      lcd.print(" Y");lcd.print((int)jarY);
    //      lcd.print(" T");lcd.print((int)Theta); lcd.print("   ");
    //      lcd.setCursor(0,1); lcd.print("Kine "); lcd.print(countGaDetek);  lcd.print("  ");
  }
  else if (countGaDetek > 40) {
    if (posisiBola == 'L') {
      motor(-50, -50, -50);
      dribble(0);
    }
    else {
      motor(50, 50, 50);
      dribble(0);
    }
    //    lcd.setCursor(0, 1); lcd.print("Scan2");
  }
  else if (countGaDetek > 10) {
    if (posisiBola == 'L') {
      motor(50, 50, 50);
      dribble(0);
    }
    else {
      motor(-50, -50, -50);
      dribble(0);
    }
    //    lcd.setCursor(12, 1); lcd.print("Scan1");
  }
  //  else if (countGaDetek > 2 && koreksiBola == 'Y') {
  //    if (posisiBola == 'L') {
  //      motor(30, 30, 30);
  //      dribble(0);
  //    }
  //    else {
  //      motor(-30, -30, -30);
  //      dribble(0);
  //    }
  //    lcd.setCursor(12, 1); lcd.print("Scan");
  //  }
  else {
    errorIL = errorIR = errorIB = 0;
    kir = kan = bel = 0;
    motor(0, 0, 0);
    dribble(0);
    dapatBola = 'N';
    //    lcd.setCursor(12, 1);lcd.print("    ");
  }

  if (errorX > 0) {
    posisiBola = 'L';
  }
  if (errorX < 0) {
    posisiBola = 'R';
  }
  if (errorX > 100 || errorX < -100) {
    koreksiBola = 'Y';
  }
  else {
    koreksiBola = 'N';
  }

  lastError = errorX;
    
    lcd.setCursor(0, 0); lcd.print("eX"); lcd.print((int)errorX); lcd.print(" eY"); lcd.print((int)errorY);
    lcd.setCursor(0, 2); lcd.print("x"); lcd.print((int)xBola); lcd.print(" y"); lcd.print((int)yBola); lcd.print("  ");
    lcd.setCursor(0, 3);
    lcd.print("L"); lcd.print((int)kir);
    lcd.print(" R"); lcd.print((int)kan);
    lcd.print(" B"); lcd.print((int)bel); lcd.print(" "); lcd.print(countGaDetek); lcd.print("  ");

  if (jarDribble() < 40) {
    dribble(150);
  }
  else {
    dribble(0);
  }
}

void pidBola360() {
  float kp = 1.3, kp1 = 0.2, kp2 = 0.3, kp3 = 1, kp4 = 2, kp5 = 3, ki = 0, ki1 = 0, kd = 0.5, kd1 = 0.6, kd2 = 0.6;
  int maxSpd = 45, maxSpd1 = 50, maxPWM0 = 180, maxPWM = 180, maxPWM1 = 200, maxSpd2 = 300, maxx = 200;
  int errorY, errorX, errorY1, errorX1;
  int kir, kan, bel = 0, PIDD, PIDDD, PIDDDD, speed1=130, speed2=170;
  int runX, runY, runW;
  //  int spX=325, spY=250;

  parseBola();
  
  if (yBola >= 0) {
    errorIL = 0;
    errorIR = 0;
    errorIB = 0;
    detekBola = 1;
    countGaDetek = 0;
    errorY = spY - yBola;
  }
  else {
    errorY = 0;
  }
  if (xBola >= 0) {
    errorIL = 0;
    errorIR = 0;
    errorIB = 0;
    detekBola = 1;
    countGaDetek = 0;
    errorX = spX - xBola;
  }
  else {
    errorX = 0;
    detekBola = 0;
    countDetek = 0;
  }

  if (yBola360 >= 0) {
    errorIL = 0;
    errorIR = 0;
    errorIB = 0;
    detekBola360 = 1;
    countGaDetek = 0;
    errorY1 = spY360 - yBola360;
  }
  else {
    errorY1 = 0;
  }
  if (xBola360 >= 0) {
    errorIL = 0;
    errorIR = 0;
    errorIB = 0;
    detekBola360 = 1;
    countGaDetek = 0;
    errorX1 = spX360 - xBola360;
  }
  else {
    errorX1 = 0;
    detekBola360 = 0;
    countDetek = 0;
  }

  if (yBola <= 300 && yBola > -1 && (yBola360 == -1 || yBola360 > -1)) {
    maxx = maxPWM1;
    PID = (kp1 * errorX) + (ki * (errorX + lastError)) + (kd * (errorX - lastError));

    if (countDetek > 10) {
      kir = (180 - PID);
      kan = (180 + PID);
    }
    else if (countDetek > 7) {
      kir = (140 - PID);
      kan = (140 + PID);
    }
    else {
      kir = (110 - PID);
      kan = (110 + PID);
    }

// ini penting
//    int jarKi = jarDeL1();
//    int jarKa = jarDeR1();
//
//    if (jarKi < jarKa) {
//      if (jarKi < 20) {
//        kir += 20;
//        bel = 90;
//      }
//      else {
//        kir = kir;
//        kan = kan;
//        bel = 0;
//      }
//    }
//    else {
//      if (jarKa < 20) {
//        kan += 20;
//        bel = -90;
//      }
//      else {
//        kir = kir;
//        kan = kan;
//        bel = 0;
//      }
//    }

    PIDD = kp * errorX;
  }
  else if (yBola > 300 && (yBola360 == -1 || yBola360 > -1)) {
    maxx = maxPWM;
    PID = (kp2 * errorX) + (ki * (errorX + lastError)) + (kd1 * (errorX - lastError));

    kir = (150 - PID);
    kan = (150 + PID);

//ini penting
//    int jarKi = jarDeL1();
//    int jarKa = jarDeR1();
//
//    if (jarKi < jarKa) {
//      if (jarKi < 20) {
//        kir += 20;
//        bel = 90;
//      }
//      else {
//        kir = kir;
//        kan = kan;
//        bel = 0;
//      }
//    }
//    else {
//      if (jarKa < 20) {
//        kan += 20;
//        bel = -90;
//      }
//      else {
//        kir = kir;
//        kan = kan;
//        bel = 0;
//      }
//    }

    PIDD = kp * errorX;
  }
  else if (yBola == -1 && yBola360 <= 230 && yBola360 > -1) {
    maxx = maxPWM0;
    PID = (kp3 * errorX1) + (ki1 * (errorX1 + lastError1)) + (kd2 * (errorX1 - lastError1));

    kir = (150 - PID);
    kan = (150 + PID);

//ini penting
//    int jarKi = jarDeL1();
//    int jarKa = jarDeR1();
//
//    if (jarKi < jarKa) {
//      if (jarKi < 20) {
//        kir += 20;
//        bel = 90;
//      }
//      else {
//        kir = kir;
//        kan = kan;
//        bel = 0;
//      }
//    }
//    else {
//      if (jarKa < 20) {
//        kan += 20;
//        bel = -90;
//      }
//      else {
//        kir = kir;
//        kan = kan;
//        bel = 0;
//      }
//    }

    PIDDD = kp4 * errorX1;
  }
  else if (yBola == -1 && yBola360 > 230) {   
    if (errorX1 >= -40 && errorX1 <= 40) {
      PIDDDD = 60;
    }
    else if (errorX1 < -40 || errorX1 > 40) {
      PIDDDD = kp5 * errorX1;
    }
  }
  
  if (kir > maxx) {
    kir = maxx;
  }
  if (kir < 0) {
    kir = 0;
  }
  if (kan > maxx) {
    kan = maxx;
  }
  if (kan < 0) {
    kan = 0;
  }

  if (PIDD > maxSpd) {
    PIDD = maxSpd;
  }
  if (PIDD < -maxSpd) {
    PIDD = -maxSpd;
  }

  if (PIDDD > maxSpd) {
    PIDDD = maxSpd;
  }
  if (PIDDD < -maxSpd) {
    PIDDD = -maxSpd;
  }

  if (PIDDDD > maxSpd1) {
    PIDDDD = maxSpd1;
  }
  if (PIDDDD < -maxSpd1) {
    PIDDDD = -maxSpd1;
  }


  if (jarDribble() <= jarbo && (xBola != -1 || xBola360 != -1)) {
    countDetek = 0;
    dapatBola = 'Y';
    countPosisioning = 0;
  }
  else if ((detekBola360 == 1 || detekBola360 == 0) && detekBola == 1 && (errorX >= -240 && errorX <= 240)) {
    motor(-kir, kan, bel);
    dapatBola = 'N';
  }
  else if ((detekBola360 == 1 || detekBola360 == 0) && detekBola == 1 && (errorX < -240 || errorX > 240)) {
    kir = kan = bel = PIDD;
    motor(PIDD, PIDD, PIDD);
    dribble(0);
    countDetek = 0;
    dapatBola = 'N';
  }
  else if ((detekBola360 == 1 && detekBola == 0) && (errorX1 >= -90 && errorX1 <= 90) && yBola360 <= 230) {
    motor(-kir, kan, bel);
    dapatBola = 'N';
  }
  else if ((detekBola360 == 1 && detekBola == 0) && (errorX1 < -90 || errorX1 > 90) && yBola360 <= 230) {
    kir = kan = bel = PIDDD;
    motor(PIDDD, PIDDD, PIDDD);
    dribble(0);
    countDetek = 0;
    dapatBola = 'N';
  }
  else if ((detekBola360 == 1 && detekBola == 0) && yBola360 > 230) {
    kir = kan = bel = PIDDDD;
    motor(PIDDDD, PIDDDD, PIDDDD);
    dribble(0);
    countDetek = 0;
    dapatBola = 'N';
  }
  else if (countGaDetek > 70) {
    countGaDetek = 0;
    countPosisioning++;
    if (countPosisioning > 1) {
      countPosisioning = 0;
    }
  }
  else if (countGaDetek > 30) { 

    if (countPosisioning == 0) {
      tarX = 300;
      tarY = 500;
      tarT = 0;
    }
    if (countPosisioning == 1) {
      tarX = 300;
      tarY = 300;
      tarT = 0;
    }

    runKinematik(tarX, tarY, tarT, 150);

    //      lcd.setCursor(0,0);
    //      lcd.print("X");lcd.print((int)jarX);
    //      lcd.print(" Y");lcd.print((int)jarY);
    //      lcd.print(" T");lcd.print((int)Theta); lcd.print("   ");
    //      lcd.setCursor(0,1); lcd.print("Kine "); lcd.print(countGaDetek);  lcd.print("  ");
  }
  else {
    errorIL = errorIR = errorIB = 0;
    kir = kan = bel = 0;
    motor(0, 0, 0);
    dribble(0);
    dapatBola = 'N';
    //lcd.setCursor(0, 1);
    //lcd.print("    ");
  }

  lastError = errorX;
  lastError1 = errorX1;

  lcd.setCursor(0, 0); 
  lcd.print("eX"); lcd.print((int)errorX); lcd.print(" erX"); lcd.print((int)errorX1); lcd.print("  ");
  lcd.setCursor(0, 1); 
  lcd.print("x"); lcd.print((int)xBola); lcd.print(" y"); lcd.print((int)yBola); lcd.print("  ");
  lcd.setCursor(0, 2);
  lcd.print("X"); lcd.print((int)xBola360); lcd.print(" Y"); lcd.print((int)yBola360); lcd.print("  ");
  lcd.setCursor(0, 3);
  lcd.print("L"); lcd.print((int)kir);
  lcd.print(" R"); lcd.print((int)kan);
  lcd.print(" B"); lcd.print((int)bel); lcd.print(" "); lcd.print(countGaDetek); lcd.print("  ");
  
  if (jarDribble() < 20) {
    dribble(255);
  }
  else {
    dribble(0);
  }
}

void pidBola360A() {
  float kp = 1.3, kp1 = 0.3, kp2 = 0.4, kp3 = 1, kp4 = 2, kp5 = 3, ki = 0, ki1 = 0, kd = 0.5, kd1 = 0.6, kd2 = 0.6;
  int maxSpd = 45, maxSpd1 = 50, maxPWM0 = 180, maxPWM = 200, maxPWM1 = 180, maxPWM2 = 140, maxSpd2 = 300, maxx = 200;
  int errorY, errorX, errorY1, errorX1;
  int kir, kan, bel = 0, PIDD, PIDDD, PIDDDD, speed1=130, speed2=170;
  int runX, runY, runW;
  //  int spX=325, spY=250;

  parseBola();
  
  if (yBola >= 0) {
    errorIL = 0;
    errorIR = 0;
    errorIB = 0;
    detekBola = 1;
    countGaDetek = 0;
    errorY = spY - yBola;
  }
  else {
    errorY = 0;
  }
  if (xBola >= 0) {
    errorIL = 0;
    errorIR = 0;
    errorIB = 0;
    detekBola = 1;
    countGaDetek = 0;
    errorX = spX - xBola;
  }
  else {
    errorX = 0;
    detekBola = 0;
    countDetek = 0;
  }

  if (yBola360 >= 0) {
    errorIL = 0;
    errorIR = 0;
    errorIB = 0;
    detekBola360 = 1;
    countGaDetek = 0;
    errorY1 = spY360 - yBola360;
  }
  else {
    errorY1 = 0;
  }
  if (xBola360 >= 0) {
    errorIL = 0;
    errorIR = 0;
    errorIB = 0;
    detekBola360 = 1;
    countGaDetek = 0;
    errorX1 = spX360 - xBola360;
  }
  else {
    errorX1 = 0;
    detekBola360 = 0;
    countDetek = 0;
  }

  if (yBola <= 300 && yBola > -1 && (yBola360 == -1 || yBola360 > -1)) {
    maxx = maxPWM1;
    PID = (kp1 * errorX) + (ki * (errorX + lastError)) + (kd * (errorX - lastError));

    if (countDetek > 10) {
      kir = (180 - PID);
      kan = (180 + PID);
    }
    else if (countDetek > 7) {
      kir = (130 - PID);
      kan = (130 + PID);
    }
    else {
      kir = (110 - PID);
      kan = (110 + PID);
    }

// ini penting
//    int jarKi = jarDeL1();
//    int jarKa = jarDeR1();
//
//    if (jarKi < jarKa) {
//      if (jarKi < 20) {
//        kir += 20;
//        bel = 90;
//      }
//      else {
//        kir = kir;
//        kan = kan;
//        bel = 0;
//      }
//    }
//    else {
//      if (jarKa < 20) {
//        kan += 20;
//        bel = -90;
//      }
//      else {
//        kir = kir;
//        kan = kan;
//        bel = 0;
//      }
//    }

    PIDD = kp * errorX;
  }
  else if (yBola > 300 && (yBola360 == -1 || yBola360 > -1)) {
    maxx = maxPWM2;
    PID = (kp2 * errorX) + (ki * (errorX + lastError)) + (kd1 * (errorX - lastError));

    kir = (100 - PID);
    kan = (100 + PID);

//ini penting
//    int jarKi = jarDeL1();
//    int jarKa = jarDeR1();
//
//    if (jarKi < jarKa) {
//      if (jarKi < 20) {
//        kir += 20;
//        bel = 90;
//      }
//      else {
//        kir = kir;
//        kan = kan;
//        bel = 0;
//      }
//    }
//    else {
//      if (jarKa < 20) {
//        kan += 20;
//        bel = -90;
//      }
//      else {
//        kir = kir;
//        kan = kan;
//        bel = 0;
//      }
//    }

    PIDD = kp * errorX;
  }
  else if (yBola == -1 && yBola360 <= 230 && yBola360 > -1) {
    maxx = maxPWM0;
    PID = (kp3 * errorX1) + (ki1 * (errorX1 + lastError1)) + (kd2 * (errorX1 - lastError1));

    kir = (150 - PID);
    kan = (150 + PID);

//ini penting
//    int jarKi = jarDeL1();
//    int jarKa = jarDeR1();
//
//    if (jarKi < jarKa) {
//      if (jarKi < 20) {
//        kir += 20;
//        bel = 90;
//      }
//      else {
//        kir = kir;
//        kan = kan;
//        bel = 0;
//      }
//    }
//    else {
//      if (jarKa < 20) {
//        kan += 20;
//        bel = -90;
//      }
//      else {
//        kir = kir;
//        kan = kan;
//        bel = 0;
//      }
//    }

    PIDDD = kp4 * errorX1;
  }
  else if (yBola == -1 && yBola360 > 230) {   
    if (errorX1 >= -40 && errorX1 <= 40) {
      PIDDDD = 60;
    }
    else if (errorX1 < -40 || errorX1 > 40) {
      PIDDDD = kp5 * errorX1;
    }
  }
  
  if (kir > maxx) {
    kir = maxx;
  }
  if (kir < 0) {
    kir = 0;
  }
  if (kan > maxx) {
    kan = maxx;
  }
  if (kan < 0) {
    kan = 0;
  }

  if (PIDD > maxSpd) {
    PIDD = maxSpd;
  }
  if (PIDD < -maxSpd) {
    PIDD = -maxSpd;
  }

  if (PIDDD > maxSpd) {
    PIDDD = maxSpd;
  }
  if (PIDDD < -maxSpd) {
    PIDDD = -maxSpd;
  }

  if (PIDDDD > maxSpd1) {
    PIDDDD = maxSpd1;
  }
  if (PIDDDD < -maxSpd1) {
    PIDDDD = -maxSpd1;
  }


  if (jarDribble() <= jarbo && (xBola != -1 || xBola360 != -1)) {
    countDetek = 0;
    dapatBola = 'Y';
    countPosisioning = 0;
  }
  else if ((detekBola360 == 1 || detekBola360 == 0) && detekBola == 1 && (errorX >= -240 && errorX <= 240)) {
    motor(-kir, kan, bel);
    dapatBola = 'N';
  }
  else if ((detekBola360 == 1 || detekBola360 == 0) && detekBola == 1 && (errorX < -240 || errorX > 240)) {
    kir = kan = bel = PIDD;
    motor(PIDD, PIDD, PIDD);
    dribble(0);
    countDetek = 0;
    dapatBola = 'N';
  }
  else if ((detekBola360 == 1 && detekBola == 0) && (errorX1 >= -90 && errorX1 <= 90) && yBola360 <= 230) {
    motor(-kir, kan, bel);
    dapatBola = 'N';
  }
  else if ((detekBola360 == 1 && detekBola == 0) && (errorX1 < -90 || errorX1 > 90) && yBola360 <= 230) {
    kir = kan = bel = PIDDD;
    motor(PIDDD, PIDDD, PIDDD);
    dribble(0);
    countDetek = 0;
    dapatBola = 'N';
  }
  else if ((detekBola360 == 1 && detekBola == 0) && yBola360 > 230) {
    kir = kan = bel = PIDDDD;
    motor(PIDDDD, PIDDDD, PIDDDD);
    dribble(0);
    countDetek = 0;
    dapatBola = 'N';
  }
  else if (countGaDetek > 70) {
    countGaDetek = 0;
    countPosisioning++;
    if (countPosisioning > 1) {
      countPosisioning = 0;
    }
  }
  else if (countGaDetek > 30) { 

    if (countPosisioning == 0) {
      tarX = 300;
      tarY = 500;
      tarT = 0;
    }
    if (countPosisioning == 1) {
      tarX = 300;
      tarY = 300;
      tarT = 0;
    }

    runKinematik(tarX, tarY, tarT, 150);

    //      lcd.setCursor(0,0);
    //      lcd.print("X");lcd.print((int)jarX);
    //      lcd.print(" Y");lcd.print((int)jarY);
    //      lcd.print(" T");lcd.print((int)Theta); lcd.print("   ");
    //      lcd.setCursor(0,1); lcd.print("Kine "); lcd.print(countGaDetek);  lcd.print("  ");
  }
  else {
    errorIL = errorIR = errorIB = 0;
    kir = kan = bel = 0;
    motor(0, 0, 0);
    dribble(0);
    dapatBola = 'N';
    //lcd.setCursor(0, 1);
    //lcd.print("    ");
  }

  lastError = errorX;
  lastError1 = errorX1;

//  lcd.setCursor(0, 0); 
//  lcd.print("eX"); lcd.print((int)errorX); lcd.print(" erX"); lcd.print((int)errorX1); lcd.print("  ");
//  lcd.setCursor(0, 1); 
//  lcd.print("x"); lcd.print((int)xBola); lcd.print(" y"); lcd.print((int)yBola); lcd.print("  ");
//  lcd.setCursor(0, 2);
//  lcd.print("X"); lcd.print((int)xBola360); lcd.print(" Y"); lcd.print((int)yBola360); lcd.print("  ");
//  lcd.setCursor(0, 3);
//  lcd.print("L"); lcd.print((int)kir);
//  lcd.print(" R"); lcd.print((int)kan);
//  lcd.print(" B"); lcd.print((int)bel); lcd.print(" "); lcd.print(countGaDetek); lcd.print("  ");
  
  if (jarDribble() < 20) {
    dribble(255);
  }
  else {
    dribble(0);
  }
}

void pidLawan360() {
  float kp = 1.3, kp1 = 0.3, kp2 = 0.2, kp3 = 0.5, kp4 = 2, kp5 = 3, ki = 0, ki1 = 0, kd = 0.5, kd1 = 0.6, kd2 = 0.6;
  int maxSpd = 45, maxSpd1 = 50, maxPWM0 = 140, maxPWM = 180, maxPWM1 = 200, maxSpd2 = 300, maxx = 200;
  int errorY, errorX, errorY1, errorX1;
  int kir, kan, bel = 0, PIDD, PIDDD, PIDDDD, speed1=130, speed2=170;
  int runX, runY, runW;
  //  int spX=325, spY=250;

  parseBola();
  
  if (yLawan >= 0) {
    errorIL = 0;
    errorIR = 0;
    errorIB = 0;
    detekLawan = 1;
    countGaDetek = 0;
    errorY = spYLawan - yLawan;
  }
  else {
    errorY = 0;
  }
  if (xLawan >= 0) {
    errorIL = 0;
    errorIR = 0;
    errorIB = 0;
    detekLawan = 1;
    countGaDetek = 0;
    errorX = spXLawan - xLawan;
  }
  else {
    errorX = 0;
    detekLawan = 0;
    countDetek = 0;
  }

  if (yLawan < 70 && yLawan > -1) {
    maxx = maxPWM;
    PID = (kp2 * errorX) + (ki * (errorX + lastError)) + (kd2 * (errorX - lastError));

    if (countDetek > 8) {
      kir = (180 - PID);
      kan = (180 + PID);
    }
    else if (countDetek > 5) {
      kir = (140 - PID);
      kan = (140 + PID);
    }
    else {
      kir = (110 - PID);
      kan = (110 + PID);
    
    }  

    /*kir = (150 - PID);
    kan = (150 + PID);*/
    

//ini penting
//    int jarKi = jarDeL1();
//    int jarKa = jarDeR1();
//
//    if (jarKi < jarKa) {
//      if (jarKi < 20) {
//        kir += 20;
//        bel = 90;
//      }
//      else {
//        kir = kir;
//        kan = kan;
//        bel = 0;
//      }
//    }
//    else {
//      if (jarKa < 20) {
//        kan += 20;
//        bel = -90;
//      }
//      else {
//        kir = kir;
//        kan = kan;
//        bel = 0;
//      }
//    }

    PIDD = kp4 * errorX;
  }
  else if (yLawan <= 230 && yLawan > 70) {
    maxx = maxPWM0;
    PID = (kp3 * errorX) + (ki1 * (errorX + lastError)) + (kd2 * (errorX - lastError));

    kir = (100 - PID);
    kan = (100 + PID);

//ini penting
//    int jarKi = jarDeL1();
//    int jarKa = jarDeR1();
//
//    if (jarKi < jarKa) {
//      if (jarKi < 20) {
//        kir += 20;
//        bel = 90;
//      }
//      else {
//        kir = kir;
//        kan = kan;
//        bel = 0;
//      }
//    }
//    else {
//      if (jarKa < 20) {
//        kan += 20;
//        bel = -90;
//      }
//      else {
//        kir = kir;
//        kan = kan;
//        bel = 0;
//      }
//    }

    PIDD = kp4 * errorX;
  }
  else if (yLawan > 230) {   
    if (errorX >= -40 && errorX <= 40) {
      PIDDD = 60;
    }
    else if (errorX < -40 || errorX > 40) {
      PIDDD = kp5 * errorX;
    }
  }
  
  if (kir > maxx) {
    kir = maxx;
  }
  if (kir < 0) {
    kir = 0;
  }
  if (kan > maxx) {
    kan = maxx;
  }
  if (kan < 0) {
    kan = 0;
  }

  if (PIDD > maxSpd) {
    PIDD = maxSpd;
  }
  if (PIDD < -maxSpd) {
    PIDD = -maxSpd;
  }

  if (PIDDD > maxSpd1) {
    PIDDD = maxSpd1;
  }
  if (PIDDD < -maxSpd1) {
    PIDDD = -maxSpd1;
  }

  if (jarDribble() <= jarbo && xLawan != -1) {
    countDetek = 0;
    dapatLawan = 'Y';
    countPosisioning = 0;
  }
  else if (detekLawan == 1 && (errorX >= -100 && errorX <= 100) && yLawan <= 230) {
    motor(-kir, kan, bel);
    dapatLawan = 'N';
  }
  else if (detekLawan == 1 && (errorX < -100 || errorX > 100) && yLawan <= 230) {
    kir = kan = bel = PIDD;
    motor(PIDD, PIDD, PIDD);
    dribble(0);
    countDetek = 0;
    dapatLawan = 'N';
  }
  else if (detekLawan == 1 && yLawan > 230) {
    kir = kan = bel = PIDDD;
    motor(PIDDD, PIDDD, PIDDD);
    dribble(0);
    countDetek = 0;
    dapatLawan = 'N';
  }
  else if (countGaDetek > 80) {
    countGaDetek = 0;
    countPosisioning++;
    if (countPosisioning > 2) {
      countPosisioning = 0;
    }
  }
  else if (countGaDetek > 40) {
    runKinematik(350, 200, 0, 100);
  }
  else {
    errorIL = errorIR = errorIB = 0;
    kir = kan = bel = 0;
    motor(0, 0, 0);
    dribble(0);
    dapatBola = 'N';
    //lcd.setCursor(0, 1);
    //lcd.print("    ");
  }

  lastError = errorX;

//  lcd.setCursor(0, 0); 
//  lcd.print("eX"); lcd.print((int)errorX); lcd.print(" eY"); lcd.print((int)errorY); lcd.print("  ");
//  lcd.setCursor(0, 1); 
//  lcd.print("x"); lcd.print((int)xLawan); lcd.print(" y"); lcd.print((int)yLawan); lcd.print("  ");
//  lcd.setCursor(0, 2);
//  lcd.print("L"); lcd.print((int)kir);
//  lcd.print(" R"); lcd.print((int)kan);
//  lcd.print(" B"); lcd.print((int)bel); lcd.print(" "); lcd.print(countGaDetek); lcd.print("  ");
  
  if (jarDribble() < 20) {
    dribble(0);
  }
  else {
    dribble(0);
  }
}

void pidBolaJaga() {
  float kp = 1.3, kp1 = 0.6, kp2 = 1, ki = 0, kd = 0.5, kd1 = 0.6;
  int  maxSpd = 45, maxSpd1 = 400, maxPWM = 160, maxPWM1 = 200, maxSpd2 = 300, errorY, errorX, maxx;
  int kir, kan, bel , PIDD;
  int runX, runY, runW;

  parseBola();

  if (yBola >= 0) {
    errorIL = 0;
    errorIR = 0;
    errorIB = 0;
    detekBola = 1;
    countGaDetek = 0;
    errorY = spY - yBola;
  }
  else {
    errorY = 0;
  }
  if (xBola >= 0) {
    errorIL = 0;
    errorIR = 0;
    errorIB = 0;
    detekBola = 1;
    countGaDetek = 0;
    errorX = spX - xBola;

    xBola1 = xBola - spX;
    yBola1 = spY - yBola;
    rBola = 10 * sqrt((xBola1 / 10 * xBola1 / 10) + (yBola1 / 10 * yBola1 / 10));

    if (xBola1 >= 0) {
      dBola = (acos(yBola1 / rBola) * 180 / PI);
    }
    else {
      dBola = -(acos(yBola1 / rBola) * 180 / PI);
    }
  }
  else {
    errorX = 0;
    detekBola = 0;
    countDetek = 0;
    xBola1 = 0;
    yBola1 = 0;
    rBola  = 0;
  }

  if ((xBola1 <= 70 && xBola1 > -70) && rBola <= 10 && xBola >= 0) {
    runXYPwm(0, -yBola1 * 3.3, Theta, 120);
  }
  else if ((xBola1 <= 70 && xBola1 > -70) && rBola <= 30 && xBola >= 0) {
    runXYPwm(0, -yBola1 * 2.2, Theta, 120);
  }
  else if ((xBola1 <= 70 && xBola1 > -70) && rBola <= 50 && xBola >= 0) {
    runXYPwm(0, -yBola1 * 2.1, Theta, 120);
  }
  else if ((xBola1 <= 70 && xBola1 > -70) && rBola <= 70 && xBola >= 0) {
    runXYPwm(0, -yBola1 * 1.7, Theta, 120);
  }
  else if (rBola > 90 && yBola1 >= 60 && xBola >= 0) {
    runXYPwm(xBola1 * 1.1, yBola1 * 1.1, -dBola * 1.1, 120);
  }

  else if ((Theta < -10)) {
    runXYPwm(-80, -30, -30, 130);
  }
  else if ((Theta > 10)) {
    runXYPwm(80, 30, 30, 130);
  }

  else if ((xBola1 >= 10 || xBola1 <= -10) && yBola1 >= 50 && xBola >= 0) {
    runXYPwm(xBola1 * 1.1, 0, Theta, 120);
  }
  else if (xBola1 > 10 && yBola1 < 50 && xBola >= 0) {
    runXYPwm(yBola1 * 1.1, -xBola1 * 1.1, Theta, 120);
  }
  else if (xBola1 < -10 && yBola1 < 50 && xBola >= 0) {
    runXYPwm(-yBola1 * 1.1, xBola1 * 1.1, Theta, 120);
  }
  else if (countGaDetek > 130) {
    countGaDetek = 0;
    countPosisioning++;
    if (countPosisioning > 2) {
      countPosisioning = 0;
    }
  }
  else if (countGaDetek > 90) { //60
    runKinematik(xTarget, yTarget, tTarget, 100);
  }
  else if (countGaDetek > 20) {
    if (posisiBola == 'L') {
      motor(-30, -30, -30);
      dribble(0);
    }
    else {
      motor(30, 30, 30);
      dribble(0);
    }
    lcd.setCursor(0, 1); lcd.print("Scan");
  }
  else if (countGaDetek > 2 && koreksiBola == 'Y') {
    if (posisiBola == 'L') {
      motor(30, 30, 30);
      dribble(0);
    }
    else {
      motor(-30, -30, -30);
      dribble(0);
    }
    lcd.setCursor(0, 1); lcd.print("Scan");
  }
  else {
    errorIL = errorIR = errorIB = 0;
    kir = kan = bel = 0;
    motor(0, 0, 0);
    dribble(0);
    dapatBola = 'N';
  }

  if (errorX > 0) {
    posisiBola = 'L';
  }
  if (errorX < 0) {
    posisiBola = 'R';
  }
  if (errorX > 100 || errorX < -100) {
    koreksiBola = 'Y';
  }
  else {
    koreksiBola = 'N';
  }

  //  lcd.setCursor(0, 0); lcd.print("x");
  //  lcd.print(xBola); lcd.print(" y");
  //  lcd.print(yBola); lcd.print("  ");
  //
  //  lcd.setCursor(0, 1); lcd.print("X");
  //  lcd.print((int)xBola1); lcd.print(" Y");
  //  lcd.print((int)yBola1); lcd.print(" R");
  //  lcd.print((int)rBola); lcd.print("  ");
  //
  //  lcd.setCursor(0, 2); lcd.print("L");
  //  lcd.print((int)kir); lcd.print(" R");
  //  lcd.print((int)kan); lcd.print(" B");
  //  lcd.print((int)bel); lcd.print(" ");
  //
  //  lcd.setCursor(0, 3); lcd.print("L");
  //  lcd.print((int)pwmL); lcd.print(" R");
  //  lcd.print((int)pwmR); lcd.print(" B");
  //  lcd.print((int)pwmB); lcd.print(" ");

  if (jarDribble() < 30) {
    dribble(0);
  }
  else {
    dribble(0);
  }
}

void pidBolaJaga(int der) {
  float kp = 1.3, kp1 = 0.6, kp2 = 1, ki = 0, kd = 0.5, kd1 = 0.6;
  int  maxSpd = 45, maxSpd1 = 400, maxPWM = 160, maxPWM1 = 200, maxSpd2 = 300, errorY, errorX, maxx;
  int kir, kan, bel , PIDD;
  int runX, runY, runW;

  parseBola();

  if (yBola >= 0) {
    errorIL = 0;
    errorIR = 0;
    errorIB = 0;
    detekBola = 1;
    countGaDetek = 0;
    errorY = spY - yBola;
  }
  else {
    errorY = 0;
  }
  if (xBola >= 0) {
    errorIL = 0;
    errorIR = 0;
    errorIB = 0;
    detekBola = 1;
    countGaDetek = 0;
    errorX = spX - xBola;

    xBola1 = xBola - spX;
    yBola1 = spY - yBola;
    rBola = 10 * sqrt((xBola1 / 10 * xBola1 / 10) + (yBola1 / 10 * yBola1 / 10));

    if (xBola1 >= 0) {
      dBola = (acos(yBola1 / rBola) * 180 / PI);
    }
    else {
      dBola = -(acos(yBola1 / rBola) * 180 / PI);
    }
  }
  else {
    errorX = 0;
    detekBola = 0;
    countDetek = 0;
    xBola1 = 0;
    yBola1 = 0;
    rBola  = 0;
  }

  int eTheta = Theta - der;

  if ((xBola1 <= 70 && xBola1 > -70) && rBola <= 10 && xBola >= 0) {
    runXYPwm(0, -yBola1 * 2.9, eTheta, 120);
  }
  else if ((xBola1 <= 70 && xBola1 > -70) && rBola <= 20 && xBola >= 0) {
    runXYPwm(0, -yBola1 * 2.3, eTheta, 120);
  }
  else if ((xBola1 <= 70 && xBola1 > -70) && rBola <= 30 && xBola >= 0) {
    runXYPwm(0, -yBola1 * 2.1, eTheta, 120);
  }
  else if ((xBola1 <= 70 && xBola1 > -70) && rBola <= 40 && xBola >= 0) {
    runXYPwm(0, -yBola1 * 1.7, eTheta, 120);
  }
  else if (rBola > 80 && yBola1 >= 60 && xBola >= 0) {
    runXYPwm(xBola1 * 1.1, yBola1 * 1.1, -dBola * 1.1, 120);
  }
  else if ((eTheta < -10)) {
    runXYPwm(-80, -30, -30, 130);
  }
  else if ((eTheta > 10)) {
    runXYPwm(80, 30, 30, 130);
  }

  else if ((xBola1 >= 10 || xBola1 <= -10) && yBola1 >= 50 && xBola >= 0) {
    runXYPwm(xBola1 * 1.1, 0, eTheta, 120);
  }
  else if (xBola1 > 10 && yBola1 < 50 && xBola >= 0) {
    runXYPwm(yBola1 * 1.1, -xBola1 * 1.1, eTheta, 120);
  }
  else if (xBola1 < -10 && yBola1 < 50 && xBola >= 0) {
    runXYPwm(-yBola1 * 1.1, xBola1 * 1.1, eTheta, 120);
  }
  else if (countGaDetek > 130) {
    countGaDetek = 0;
    countPosisioning++;
    if (countPosisioning > 2) {
      countPosisioning = 0;
    }
  }
  else if (countGaDetek > 90) { //60
    runKinematik(xTarget, yTarget, tTarget, 100);
  }
  else if (countGaDetek > 20) {
    if (posisiBola == 'L') {
      motor(-30, -30, -30);
      dribble(0);
    }
    else {
      motor(30, 30, 30);
      dribble(0);
    }
    lcd.setCursor(0, 1); lcd.print("Scan");
  }
  else if (countGaDetek > 2 && koreksiBola == 'Y') {
    if (posisiBola == 'L') {
      motor(30, 30, 30);
      dribble(0);
    }
    else {
      motor(-30, -30, -30);
      dribble(0);
    }
    lcd.setCursor(0, 1); lcd.print("Scan");
  }
  else {
    errorIL = errorIR = errorIB = 0;
    kir = kan = bel = 0;
    motor(0, 0, 0);
    dribble(0);
    dapatBola = 'N';
  }

  if (errorX > 0) {
    posisiBola = 'L';
  }
  if (errorX < 0) {
    posisiBola = 'R';
  }
  if (errorX > 100 || errorX < -100) {
    koreksiBola = 'Y';
  }
  else {
    koreksiBola = 'N';
  }

  //  lcd.setCursor(0, 0); lcd.print("x");
  //  lcd.print(xBola); lcd.print(" y");
  //  lcd.print(yBola); lcd.print("  ");
  //
  //  lcd.setCursor(0, 1); lcd.print("X");
  //  lcd.print((int)xBola1); lcd.print(" Y");
  //  lcd.print((int)yBola1); lcd.print(" R");
  //  lcd.print((int)rBola); lcd.print("  ");
  //
  //  lcd.setCursor(0, 2); lcd.print("L");
  //  lcd.print((int)kir); lcd.print(" R");
  //  lcd.print((int)kan); lcd.print(" B");
  //  lcd.print((int)bel); lcd.print(" ");
  //
  //  lcd.setCursor(0, 3); lcd.print("L");
  //  lcd.print((int)pwmL); lcd.print(" R");
  //  lcd.print((int)pwmR); lcd.print(" B");
  //  lcd.print((int)pwmB); lcd.print(" ");

  if (jarDribble() < 30) {
    dribble(0);
  }
  else {
    dribble(0);
  }
}

void pidBertahan1() {
  float kp = 1.3, kp1 = 0.6, kp2 = 1, ki = 0, kd = 0.5, kd1 = 0.6, der;
  int  maxSpd = 45, maxSpd1 = 400, maxPWM = 160, maxPWM1 = 200, maxSpd2 = 300, errorY, errorX, maxx;
  int kir, kan, bel , PIDD;
  int runX, runY, runW;

  parseBola();

  if (yBola >= 0) {
    errorIL = 0;
    errorIR = 0;
    errorIB = 0;
    detekBola = 1;
    countGaDetek = 0;
    errorY = spY - yBola;
  }
  else {
    errorY = 0;
  }
  if (xBola >= 0) {
    errorIL = 0;
    errorIR = 0;
    errorIB = 0;
    detekBola = 1;
    countGaDetek = 0;
    errorX = spX - xBola;

    xBola1 = xBola - spX; // 
    yBola1 = spY - yBola; //
    rBola = 10 * sqrt((xBola1 / 10 * xBola1 / 10) + (yBola1 / 10 * yBola1 / 10));

    if (xBola1 >= 0) {
      dBola = (acos(yBola1 / rBola) * 180 / PI);
    }
    else {
      dBola = -(acos(yBola1 / rBola) * 180 / PI);
    }
  }
  else {
    errorX = 0;
    detekBola = 0;
    countDetek = 0;
    xBola1 = 0;
    yBola1 = 0;
    rBola  = 0;
  }

  if (yBola360 >= 0) {
    errorIL = 0;
    errorIR = 0;
    errorIB = 0;
    detekBola360 = 1;
    countGaDetek = 0;
    errorY1 = spY1 - yBola360;
  }
  else {
    errorY1 = 0;
  }
  if (xBola360 >= 0) {
    errorIL = 0;
    errorIR = 0;
    errorIB = 0;
    detekBola360 = 1;
    countGaDetek = 0;
    errorX1 = spX360 - xBola360;

    xBola2 = xBola360 - spX360;
    yBola2 = spY360 - yBola360;
    rBola1 = 10 * sqrt((xBola2 / 10 * xBola2 / 10) + (yBola2 / 10 * yBola2 / 10));

    if (xBola2 >= 0) {
      dBola1 = (acos(yBola2 / rBola1) * 180 / PI); //derajat
    }
    else {
      dBola1 = -(acos(yBola2 / rBola1) * 180 / PI); //derajat
    }
  }
  else {
    errorX1 = 0;
    detekBola360 = 0;
    countDetek = 0;
    xBola2 = 0;
    yBola2 = 0;
    rBola1 = 0;
  }

  float jarX1;
  
  if (jarX >= 350) {
    jarX1 = (jarX + 100);
  }
  else if (jarX <= 250) {
    jarX1 = (jarX - 100);
  }
  else {
    jarX1 = jarX;
  }
  
  float xPos = jarX1 - 300;
  float yPos = jarY;
  float rPos = 10 * sqrt((xPos / 10 * xPos / 10) + (yPos / 10 * yPos / 10));

  if (xPos >= 0) {
    der = (acos(yPos / rPos) * 180 / PI);
  } else {
    der = -(acos(yPos / rPos) * 180 / PI);
  }

  int eTheta = Theta - der;

  if ((xBola1 <= 100 && xBola1 > -100) && rBola <= 200 && xBola >= 0 && detekBola == 1) {
    runXYPwm(0, -yBola1 * 0.71, eTheta, 150);
  }
  else if ((xBola1 <= 100 && xBola1 > -100) && rBola <= 210 && xBola >= 0 && detekBola == 1) {
    runXYPwm(0, -yBola1 * 0.65, eTheta, 150);
  }
  else if ((xBola1 <= 100 && xBola1 > -100) && rBola <= 220 && xBola >= 0 && detekBola == 1) {
    runXYPwm(0, -yBola1 * 0.58, eTheta, 150);
  }
  else if ((xBola1 <= 100 && xBola1 > -100) && rBola <= 230 && xBola >= 0 && detekBola == 1) {
    runXYPwm(0, -yBola1 * 0.52, eTheta, 150);
  }
  else if ((xBola1 <= 100 && xBola1 > -100) && rBola <= 250 && xBola >= 0 && detekBola == 1) {
    runXYPwm(0, -yBola1 * 0.32, eTheta, 150);
  }

  else if ((xBola1 >= 10 || xBola1 <= -10) && yBola1 >= 240 && xBola >= 0 && detekBola == 1) {
    runXYPwm(xBola1 * 0.6, 0, eTheta, 150);
  }
  else if (xBola1 > 10 && yBola1 < 240 && xBola >= 0 && detekBola == 1) {
    runXYPwm(yBola1 * 0.4, -xBola1 * 0.5, eTheta, 150);
  }
  else if (xBola1 < -10 && yBola1 < 240 && xBola >= 0 && detekBola == 1) {
    runXYPwm(-yBola1 * 0.4, xBola1 * 0.5, eTheta, 150);
  }

  else if (xBola2 > 10 && detekBola == 0 && detekBola360 == 1) {
    motor(-60, -60, -60);
  }
  else if (xBola2 < -10 && detekBola == 0 && detekBola360 == 1) {
    motor(60, 60, 60);
  }
  else if ((xBola2 <= 10 && xBola2 >= -10) && detekBola == 0 && detekBola360 == 1) {
    motor(60, 60, 60);
  }

  else if (countGaDetek > 80) {
    countGaDetek = 0;
    countPosisioning++;
    if (countPosisioning > 2) {
      countPosisioning = 0;
    }
  }
  else if (countGaDetek > 40) {
    runKinematik(350, 200, 0, 100);
  }
  else {
    errorIL = errorIR = errorIB = 0;
    kir = kan = bel = 0;
    motor(0, 0, 0);
    dribble(0);
    dapatBola = 'N';
  }


  //lcd.setCursor(16, 2); lcd.print(countGaDetek); lcd.print("  ");
//    lcd.setCursor(0,0); lcd.print("x");
//    lcd.print((int)xPos); lcd.print(" y");
//    lcd.print((int)yPos); lcd.print(" r");
//    lcd.print((int)rPos); lcd.print(" d");
//    lcd.print(der); lcd.print("   ");

//    lcd.setCursor(0,1); lcd.print("X");
//    lcd.print((int)xBola1); lcd.print(" Y");
//    lcd.print((int)yBola1); lcd.print(" R");
//    lcd.print((int)rBola); lcd.print("  ");
//
//    lcd.setCursor(0,2); lcd.print("x");
//    lcd.print((int)xBola); lcd.print(" y");
//    lcd.print((int)yBola); lcd.print(" d");
//    lcd.print((int)dBola); lcd.print("  ");

//    lcd.setCursor(0,1); lcd.print("X");
//    lcd.print((int)xBola2); lcd.print(" Y");
//    lcd.print((int)yBola2); lcd.print(" R");
//    lcd.print((int)rBola1); lcd.print("  ");
//
//    lcd.setCursor(0,2); lcd.print("x");
//    lcd.print((int)xBola360); lcd.print(" y");
//    lcd.print((int)yBola360); lcd.print(" d");
//    lcd.print((int)dBola1); lcd.print("  ");
  
//    lcd.setCursor(0,3); lcd.print("L");
//    lcd.print((int)pwmL); lcd.print(" R");
//    lcd.print((int)pwmR); lcd.print(" B");
//    lcd.print((int)pwmB); lcd.print(" "); lcd.print(countGaDetek); lcd.print("  ");

  if (jarDribble() < 10) {
    dribble(255);
  }
  else {
    dribble(0);
  }
}

void pidBertahan(int r) {
  float kp = 1.3, kp1 = 0.6, kp2 = 1, ki = 0, kd = 0.5, kd1 = 0.6, der;
  int  maxSpd = 45, maxSpd1 = 400, maxPWM = 160, maxPWM1 = 200, maxSpd2 = 300, errorY, errorX, maxx;
  int kir, kan, bel , PIDD;
  int runX, runY, runW;

  parseBola();

  if (yBola >= 0) {
    errorIL = 0;
    errorIR = 0;
    errorIB = 0;
    detekBola = 1;
    countGaDetek = 0;
    errorY = spY - yBola;
  }
  else {
    errorY = 0;
  }
  if (xBola >= 0) {
    errorIL = 0;
    errorIR = 0;
    errorIB = 0;
    detekBola = 1;
    countGaDetek = 0;
    errorX = spX - xBola;

    xBola1 = xBola - spX; // 
    yBola1 = spY - yBola; //
    rBola = 10 * sqrt((xBola1 / 10 * xBola1 / 10) + (yBola1 / 10 * yBola1 / 10));

    if (xBola1 >= 0) {
      dBola = (acos(yBola1 / rBola) * 180 / PI);
    }
    else {
      dBola = -(acos(yBola1 / rBola) * 180 / PI);
    }
  }
  else {
    errorX = 0;
    detekBola = 0;
    countDetek = 0;
    xBola1 = 0;
    yBola1 = 0;
    rBola  = 0;
  }

  if (yBola360 >= 0) {
    errorIL = 0;
    errorIR = 0;
    errorIB = 0;
    detekBola360 = 1;
    countGaDetek = 0;
    errorY1 = spY1 - yBola360;
  }
  else {
    errorY1 = 0;
  }
  if (xBola360 >= 0) {
    errorIL = 0;
    errorIR = 0;
    errorIB = 0;
    detekBola360 = 1;
    countGaDetek = 0;
    errorX1 = spX360 - xBola360;

    xBola2 = xBola360 - spX360;
    yBola2 = spY360 - yBola360;
    rBola1 = 10 * sqrt((xBola2 / 10 * xBola2 / 10) + (yBola2 / 10 * yBola2 / 10));

    if (xBola2 >= 0) {
      dBola1 = (acos(yBola2 / rBola1) * 180 / PI); //derajat
    }
    else {
      dBola1 = -(acos(yBola2 / rBola1) * 180 / PI); //derajat
    }
  }
  else {
    errorX1 = 0;
    detekBola360 = 0;
    countDetek = 0;
    xBola2 = 0;
    yBola2 = 0;
    rBola1 = 0;
  }

  float jarX1;
  
  if (jarX >= 350) {
    jarX1 = (jarX + 100);
  }
  else if (jarX <= 250) {
    jarX1 = (jarX - 100);
  }
  else {
    jarX1 = jarX;
  }

  float xPos = jarX1 - 300;
  float yPos = jarY;
  float rPos = 10 * sqrt((xPos / 10 * xPos / 10) + (yPos / 10 * yPos / 10));

  if (xPos >= 0) {
    der = (acos(yPos / rPos) * 180 / PI);
  } else {
    der = -(acos(yPos / rPos) * 180 / PI);
  }

  int eTheta = Theta - der;

  if (rBola > r && yBola1 >= 250 && xBola >= 0 && detekBola == 1){
    runXYPwm(xBola1 * 0.5, yBola1 * 0.4, -dBola * 0.7, 180);
  }
  else if((xBola1 <= 70 && xBola1 > -70 && xBola >=0) && (eTheta < -10) && detekBola == 1) {
    runXYPwm(-80, -30, -30, 130);
  }
  else if((xBola1 <= 70 && xBola1 > -70 && xBola >=0) && (eTheta > 10) && detekBola == 1) {
    runXYPwm(80, 30, 30, 130);
  }

  else if ((xBola1 <= 100 && xBola1 > -100) && rBola <= 200 && xBola >= 0 && detekBola == 1) {
    runXYPwm(0, -yBola1 * 0.71, eTheta, 150);
  }
  else if ((xBola1 <= 100 && xBola1 > -100) && rBola <= 210 && xBola >= 0 && detekBola == 1) {
    runXYPwm(0, -yBola1 * 0.65, eTheta, 150);
  }
  else if ((xBola1 <= 100 && xBola1 > -100) && rBola <= 220 && xBola >= 0 && detekBola == 1) {
    runXYPwm(0, -yBola1 * 0.58, eTheta, 150);
  }
  else if ((xBola1 <= 100 && xBola1 > -100) && rBola <= 230 && xBola >= 0 && detekBola == 1) {
    runXYPwm(0, -yBola1 * 0.52, eTheta, 150);
  }
  else if ((xBola1 <= 100 && xBola1 > -100) && rBola <= 250 && xBola >= 0 && detekBola == 1) {
    runXYPwm(0, -yBola1 * 0.32, eTheta, 150);
  }

  else if ((xBola1 >= 10 || xBola1 <= -10) && yBola1 >= 240 && xBola >= 0 && detekBola == 1) {
    runXYPwm(xBola1 * 0.6, 0, eTheta, 150);
  }
  else if (xBola1 > 10 && yBola1 < 240 && xBola >= 0 && detekBola == 1) {
    runXYPwm(yBola1 * 0.4, -xBola1 * 0.5, eTheta, 150);
  }
  else if (xBola1 < -10 && yBola1 < 240 && xBola >= 0 && detekBola == 1) {
    runXYPwm(-yBola1 * 0.4, xBola1 * 0.5, eTheta, 150);
  }

  else if (xBola2 > 10 && detekBola == 0 && detekBola360 == 1) {
    motor(-60, -60, -60);
  }
  else if (xBola2 < -10 && detekBola == 0 && detekBola360 == 1) {
    motor(60, 60, 60);
  }
  else if ((xBola2 <= 10 && xBola2 >= -10) && detekBola == 0 && detekBola360 == 1) {
    motor(60, 60, 60);
  }

  else if (countGaDetek > 80) {
    countGaDetek = 0;
    countPosisioning++;
    if (countPosisioning > 2) {
      countPosisioning = 0;
    }
  }
  else if (countGaDetek > 40) {
    runKinematik(350, 200, 0, 100);
  }
  else {
    errorIL = errorIR = errorIB = 0;
    kir = kan = bel = 0;
    motor(0, 0, 0);
    dribble(0);
    dapatBola = 'N';
  }


  //lcd.setCursor(16, 2); lcd.print(countGaDetek); lcd.print("  ");
//    lcd.setCursor(0,0); lcd.print("x");
//    lcd.print((int)xPos); lcd.print(" y");
//    lcd.print((int)yPos); lcd.print(" r");
//    lcd.print((int)rPos); lcd.print(" d");
//    lcd.print(eTheta); lcd.print("   ");
//
//    lcd.setCursor(0,1); lcd.print("X");
//    lcd.print((int)xBola1); lcd.print(" Y");
//    lcd.print((int)yBola1); lcd.print(" R");
//    lcd.print((int)rBola); lcd.print("  ");
//
//    lcd.setCursor(0,2); lcd.print("x");
//    lcd.print((int)xBola); lcd.print(" y");
//    lcd.print((int)yBola); lcd.print(" d");
//    lcd.print((int)dBola); lcd.print("  ");

//    lcd.setCursor(0,1); lcd.print("X");
//    lcd.print((int)xBola2); lcd.print(" Y");
//    lcd.print((int)yBola2); lcd.print(" R");
//    lcd.print((int)rBola1); lcd.print("  ");
//
//    lcd.setCursor(0,2); lcd.print("x");
//    lcd.print((int)xBola360); lcd.print(" y");
//    lcd.print((int)yBola360); lcd.print(" d");
//    lcd.print((int)dBola1); lcd.print("  ");
  
//    lcd.setCursor(0,3); lcd.print("L");
//    lcd.print((int)pwmL); lcd.print(" R");
//    lcd.print((int)pwmR); lcd.print(" B");
//    lcd.print((int)pwmB); lcd.print(" "); lcd.print(countGaDetek); lcd.print("  ");

  if (jarDribble() < 10) {
    dribble(255);
  }
  else {
    dribble(0);
  }
}

void pidBertahanA() {
  float kp = 1.3, kp1 = 0.6, kp2 = 1, ki = 0, kd = 0.5, kd1 = 0.6, der;
  int  maxSpd = 45, maxSpd1 = 400, maxPWM = 160, maxPWM1 = 200, maxSpd2 = 300, errorY, errorX, maxx;
  int kir, kan, bel , PIDD;
  int runX, runY, runW;

  parseBola();

  if (yBola >= 0) {
    errorIL = 0;
    errorIR = 0;
    errorIB = 0;
    detekBola = 1;
    countGaDetek = 0;
    errorY = spY - yBola;
  }
  else {
    errorY = 0;
  }
  if (xBola >= 0) {
    errorIL = 0;
    errorIR = 0;
    errorIB = 0;
    detekBola = 1;
    countGaDetek = 0;
    errorX = spX - xBola;

    xBola1 = xBola - spX; // 
    yBola1 = spY - yBola; //
    rBola = 10 * sqrt((xBola1 / 10 * xBola1 / 10) + (yBola1 / 10 * yBola1 / 10));

    if (xBola1 >= 0) {
      dBola = (acos(yBola1 / rBola) * 180 / PI);
    }
    else {
      dBola = -(acos(yBola1 / rBola) * 180 / PI);
    }
  }
  else {
    errorX = 0;
    detekBola = 0;
    countDetek = 0;
    xBola1 = 0;
    yBola1 = 0;
    rBola  = 0;
  }

  if (yBola360 >= 0) {
    errorIL = 0;
    errorIR = 0;
    errorIB = 0;
    detekBola360 = 1;
    countGaDetek = 0;
    errorY1 = spY1 - yBola360;
  }
  else {
    errorY1 = 0;
  }
  if (xBola360 >= 0) {
    errorIL = 0;
    errorIR = 0;
    errorIB = 0;
    detekBola360 = 1;
    countGaDetek = 0;
    errorX1 = spX360 - xBola360;

    xBola2 = xBola360 - spX360;
    yBola2 = spY360 - yBola360;
    rBola1 = 10 * sqrt((xBola2 / 10 * xBola2 / 10) + (yBola2 / 10 * yBola2 / 10));

    if (xBola2 >= 0) {
      dBola1 = (acos(yBola2 / rBola1) * 180 / PI); //derajat
    }
    else {
      dBola1 = -(acos(yBola2 / rBola1) * 180 / PI); //derajat
    }
  }
  else {
    errorX1 = 0;
    detekBola360 = 0;
    countDetek = 0;
    xBola2 = 0;
    yBola2 = 0;
    rBola1 = 0;
  }

  float jarX1;
  
  if (jarX >= 400) {
    jarX1 = (jarX + 100);
  }
  else if (jarX <= 200) {
    jarX1 = (jarX - 100);
  }
  else {
    jarX1 = jarX;
  }
  
  float xPos = jarX1 - 300;
  float yPos = jarY;
  float rPos = 10 * sqrt((xPos / 10 * xPos / 10) + (yPos / 10 * yPos / 10));

  if (xPos >= 0) {
    der = (acos(yPos / rPos) * 180 / PI);
  } else {
    der = -(acos(yPos / rPos) * 180 / PI);
  }

  int eTheta = Theta - der;

  if ((xBola1 <= 100 && xBola1 > -100) && rBola <= 140 && xBola >= 0 && detekBola == 1) {
    runXYPwm(0, -yBola1 * 0.68, eTheta, 130);
  }
  else if ((xBola1 <= 100 && xBola1 > -100) && rBola <= 150 && xBola >= 0 && detekBola == 1) {
    runXYPwm(0, -yBola1 * 0.70, eTheta, 130);
  }
  else if ((xBola1 <= 100 && xBola1 > -100) && rBola <= 170 && xBola >= 0 && detekBola == 1) {
    runXYPwm(0, -yBola1 * 0.75, eTheta, 130);
  }
  else if ((xBola1 <= 100 && xBola1 > -100) && rBola <= 180 && xBola >= 0 && detekBola == 1) {
    runXYPwm(0, -yBola1 * 0.72, eTheta, 130);
  }
  else if ((xBola1 <= 100 && xBola1 > -100) && rBola <= 200 && xBola >= 0 && detekBola == 1) {
    runXYPwm(0, -yBola1 * 0.41, eTheta, 130);
  }

  else if ((xBola1 >= 10 || xBola1 <= -10) && yBola1 >= 180 && xBola >= 0 && detekBola == 1) {
    runXYPwm(xBola1 * 0.35, 0, eTheta, 130);
  }
  else if (xBola1 > 10 && yBola1 < 180 && xBola >= 0 && detekBola == 1) {
    runXYPwm(yBola1 * 0.39, -xBola1 * 0.35, eTheta, 130);
  }
  else if (xBola1 < -10 && yBola1 < 180 && xBola >= 0 && detekBola == 1) {
    runXYPwm(-yBola1 * 0.39, xBola1 * 0.35, eTheta, 130);
  }

  else if (xBola2 > 10 && detekBola == 0 && detekBola360 == 1) {
    motor(-50, -50, -50);
  }
  else if (xBola2 < -10 && detekBola == 0 && detekBola360 == 1) {
    motor(50, 50, 50);
  }
  else if ((xBola2 <= 10 && xBola2 >= -10) && detekBola == 0 && detekBola360 == 1) {
    motor(50, 50, 50);
  }

  else if (countGaDetek > 80) {
    countGaDetek = 0;
    countPosisioning++;
    if (countPosisioning > 2) {
      countPosisioning = 0;
    }
  }
  else if (countGaDetek > 40) {
    runKinematik(350, 200, 0, 100);
  }
  else {
    errorIL = errorIR = errorIB = 0;
    kir = kan = bel = 0;
    motor(0, 0, 0);
    dribble(0);
    dapatBola = 'N';
  }


  //lcd.setCursor(16, 2); lcd.print(countGaDetek); lcd.print("  ");
//    lcd.setCursor(0,0); lcd.print("x");
//    lcd.print((int)xPos); lcd.print(" y");
//    lcd.print((int)yPos); lcd.print(" r");
//    lcd.print((int)rPos); lcd.print(" d");
//    lcd.print(der); lcd.print("   ");

//    lcd.setCursor(0,1); lcd.print("X");
//    lcd.print((int)xBola1); lcd.print(" Y");
//    lcd.print((int)yBola1); lcd.print(" R");
//    lcd.print((int)rBola); lcd.print("  ");
//
//    lcd.setCursor(0,2); lcd.print("x");
//    lcd.print((int)xBola); lcd.print(" y");
//    lcd.print((int)yBola); lcd.print(" d");
//    lcd.print((int)dBola); lcd.print("  ");

//    lcd.setCursor(0,1); lcd.print("X");
//    lcd.print((int)xBola2); lcd.print(" Y");
//    lcd.print((int)yBola2); lcd.print(" R");
//    lcd.print((int)rBola1); lcd.print("  ");
//
//    lcd.setCursor(0,2); lcd.print("x");
//    lcd.print((int)xBola360); lcd.print(" y");
//    lcd.print((int)yBola360); lcd.print(" d");
//    lcd.print((int)dBola1); lcd.print("  ");
  
//    lcd.setCursor(0,3); lcd.print("L");
//    lcd.print((int)pwmL); lcd.print(" R");
//    lcd.print((int)pwmR); lcd.print(" B");
//    lcd.print((int)pwmB); lcd.print("  "); lcd.print(countGaDetek); lcd.print("  ");

  if (jarDribble() < 10) {
    dribble(255);
  }
  else {
    dribble(0);
  }
}

void pidBertahanB(int r) {
  float kp = 1.3, kp1 = 0.6, kp2 = 1, ki = 0, kd = 0.5, kd1 = 0.6, der;
  int  maxSpd = 45, maxSpd1 = 400, maxPWM = 160, maxPWM1 = 200, maxSpd2 = 300, errorY, errorX, maxx;
  int kir, kan, bel , PIDD;
  int runX, runY, runW;

  parseBola();

  if (yBola >= 0) {
    errorIL = 0;
    errorIR = 0;
    errorIB = 0;
    detekBola = 1;
    countGaDetek = 0;
    errorY = spY - yBola;
  }
  else {
    errorY = 0;
  }
  if (xBola >= 0) {
    errorIL = 0;
    errorIR = 0;
    errorIB = 0;
    detekBola = 1;
    countGaDetek = 0;
    errorX = spX - xBola;

    xBola1 = xBola - spX; // 
    yBola1 = spY - yBola; //
    rBola = 10 * sqrt((xBola1 / 10 * xBola1 / 10) + (yBola1 / 10 * yBola1 / 10));

    if (xBola1 >= 0) {
      dBola = (acos(yBola1 / rBola) * 180 / PI);
    }
    else {
      dBola = -(acos(yBola1 / rBola) * 180 / PI);
    }
  }
  else {
    errorX = 0;
    detekBola = 0;
    countDetek = 0;
    xBola1 = 0;
    yBola1 = 0;
    rBola  = 0;
  }

  if (yBola360 >= 0) {
    errorIL = 0;
    errorIR = 0;
    errorIB = 0;
    detekBola360 = 1;
    countGaDetek = 0;
    errorY1 = spY1 - yBola360;
  }
  else {
    errorY1 = 0;
  }
  if (xBola360 >= 0) {
    errorIL = 0;
    errorIR = 0;
    errorIB = 0;
    detekBola360 = 1;
    countGaDetek = 0;
    errorX1 = spX360 - xBola360;

    xBola2 = xBola360 - spX360;
    yBola2 = spY360 - yBola360;
    rBola1 = 10 * sqrt((xBola2 / 10 * xBola2 / 10) + (yBola2 / 10 * yBola2 / 10));

    if (xBola2 >= 0) {
      dBola1 = (acos(yBola2 / rBola1) * 180 / PI); //derajat
    }
    else {
      dBola1 = -(acos(yBola2 / rBola1) * 180 / PI); //derajat
    }
  }
  else {
    errorX1 = 0;
    detekBola360 = 0;
    countDetek = 0;
    xBola2 = 0;
    yBola2 = 0;
    rBola1 = 0;
  }

  float jarX1;
  
  if (jarX >= 400) {
    jarX1 = (jarX + 100);
  }
  else if (jarX <= 200) {
    jarX1 = (jarX - 100);
  }
  else {
    jarX1 = jarX;
  }

  float xPos = jarX1 - 300;
  float yPos = jarY;
  float rPos = 10 * sqrt((xPos / 10 * xPos / 10) + (yPos / 10 * yPos / 10));

  if (xPos >= 0) {
    der = (acos(yPos / rPos) * 180 / PI);
  } else {
    der = -(acos(yPos / rPos) * 180 / PI);
  }

  int eTheta = Theta - der;

  if (rBola > r && yBola1 >= 200 && xBola >= 0 && detekBola == 1){
    runXYPwm(xBola1 * 0.4, yBola1 * 0.35, -dBola * 0.7, 150);
  }
  else if((xBola1 <= 70 && xBola1 > -70 && xBola >=0) && (eTheta < -10) && detekBola == 1) {
    runXYPwm(-80, -30, -30, 130);
  }
  else if((xBola1 <= 70 && xBola1 > -70 && xBola >=0) && (eTheta > 10) && detekBola == 1) {
    runXYPwm(80, 30, 30, 130);
  }

  else if ((xBola1 <= 100 && xBola1 > -100) && rBola <= 140 && xBola >= 0 && detekBola == 1) {
    runXYPwm(0, -yBola1 * 0.68, eTheta, 130);
  }
  else if ((xBola1 <= 100 && xBola1 > -100) && rBola <= 150 && xBola >= 0 && detekBola == 1) {
    runXYPwm(0, -yBola1 * 0.70, eTheta, 130);
  }
  else if ((xBola1 <= 100 && xBola1 > -100) && rBola <= 170 && xBola >= 0 && detekBola == 1) {
    runXYPwm(0, -yBola1 * 0.75, eTheta, 130);
  }
  else if ((xBola1 <= 100 && xBola1 > -100) && rBola <= 180 && xBola >= 0 && detekBola == 1) {
    runXYPwm(0, -yBola1 * 0.72, eTheta, 130);
  }
  else if ((xBola1 <= 100 && xBola1 > -100) && rBola <= 200 && xBola >= 0 && detekBola == 1) {
    runXYPwm(0, -yBola1 * 0.41, eTheta, 130);
  }

  else if ((xBola1 >= 10 || xBola1 <= -10) && yBola1 >= 180 && xBola >= 0 && detekBola == 1) {
    runXYPwm(xBola1 * 0.35, 0, eTheta, 130);
  }
  else if (xBola1 > 10 && yBola1 < 180 && xBola >= 0 && detekBola == 1) {
    runXYPwm(yBola1 * 0.39, -xBola1 * 0.35, eTheta, 130);
  }
  else if (xBola1 < -10 && yBola1 < 180 && xBola >= 0 && detekBola == 1) {
    runXYPwm(-yBola1 * 0.39, xBola1 * 0.35, eTheta, 130);
  }

  else if (xBola2 > 10 && detekBola == 0 && detekBola360 == 1) {
    motor(-50, -50, -50);
  }
  else if (xBola2 < -10 && detekBola == 0 && detekBola360 == 1) {
    motor(50, 50, 50);
  }
  else if ((xBola2 <= 10 && xBola2 >= -10) && detekBola == 0 && detekBola360 == 1) {
    motor(50, 50, 50);
  }

  else if (countGaDetek > 80) {
    countGaDetek = 0;
    countPosisioning++;
    if (countPosisioning > 2) {
      countPosisioning = 0;
    }
  }
  else if (countGaDetek > 40) {
    runKinematik(350, 200, 0, 100);
  }
  else {
    errorIL = errorIR = errorIB = 0;
    kir = kan = bel = 0;
    motor(0, 0, 0);
    dribble(0);
    dapatBola = 'N';
  }


  //lcd.setCursor(16, 2); lcd.print(countGaDetek); lcd.print("  ");
//    lcd.setCursor(0,0); lcd.print("x");
//    lcd.print((int)xPos); lcd.print(" y");
//    lcd.print((int)yPos); lcd.print(" r");
//    lcd.print((int)rPos); lcd.print(" d");
//    lcd.print(eTheta); lcd.print("   ");
//
//    lcd.setCursor(0,1); lcd.print("X");
//    lcd.print((int)xBola1); lcd.print(" Y");
//    lcd.print((int)yBola1); lcd.print(" R");
//    lcd.print((int)rBola); lcd.print("  ");
//
//    lcd.setCursor(0,2); lcd.print("x");
//    lcd.print((int)xBola); lcd.print(" y");
//    lcd.print((int)yBola); lcd.print(" d");
//    lcd.print((int)dBola); lcd.print("  ");

//    lcd.setCursor(0,1); lcd.print("X");
//    lcd.print((int)xBola2); lcd.print(" Y");
//    lcd.print((int)yBola2); lcd.print(" R");
//    lcd.print((int)rBola1); lcd.print("  ");
//
//    lcd.setCursor(0,2); lcd.print("x");
//    lcd.print((int)xBola360); lcd.print(" y");
//    lcd.print((int)yBola360); lcd.print(" d");
//    lcd.print((int)dBola1); lcd.print("  ");
  
//    lcd.setCursor(0,3); lcd.print("L");
//    lcd.print((int)pwmL); lcd.print(" R");
//    lcd.print((int)pwmR); lcd.print(" B");
//    lcd.print((int)pwmB); lcd.print(" "); lcd.print(countGaDetek); lcd.print("  ");

  if (jarDribble() < 10) {
    dribble(255);
  }
  else {
    dribble(0);
  }
}

void pidBertahanLawan() {
  float kp = 1.3, kp1 = 0.6, kp2 = 1, ki = 0, kd = 0.5, kd1 = 0.6, der;
  int  maxSpd = 45, maxSpd1 = 400, maxPWM = 160, maxPWM1 = 200, maxSpd2 = 300, errorY, errorX, maxx;
  int kir, kan, bel, PIDD;
  int runX, runY, runW;

  parseBola();

  if (yLawan >= 0) {
    errorIL = 0;
    errorIR = 0;
    errorIB = 0;
    detekLawan = 1;
    countGaDetek = 0;
    errorY = spYLawan - yLawan;
  }
  else {
    errorY = 0;
  }
  if (xLawan >= 0) {
    errorIL = 0;
    errorIR = 0;
    errorIB = 0;
    detekLawan = 1;
    countGaDetek = 0;
    errorX = spXLawan - xLawan;

    xLawan1 = xLawan - spXLawan;
    yLawan1 = spYLawan - yLawan;
    rLawan = 10 * sqrt((xLawan1 / 10 * xLawan1 / 10) + (yLawan1 / 10 * yLawan1 / 10));

    if (xLawan1 >= 0) {
      dLawan = (acos(yLawan1 / rLawan) * 180 / PI); //derajat
    }
    else {
      dLawan = -(acos(yLawan1 / rLawan) * 180 / PI); //derajat
    }
  }
  else {
    errorX = 0;
    detekLawan = 0;
    countDetek = 0;
    xLawan1 = 0;
    yLawan1 = 0;
    rLawan = 0;
  }

  float jarX1;
  
  if (jarX >= 400) {
    jarX1 = (jarX + 100);
  }
  else if (jarX <= 200) {
    jarX1 = (jarX - 100);
  }
  else {
    jarX1 = jarX;
  }
  
  float xPos = jarX - 300;
  float yPos = jarY;
  float rPos = 10 * sqrt((xPos / 10 * xPos / 10) + (yPos / 10 * yPos / 10));

  if (xPos >= 0) {
    der = (acos(yPos / rPos) * 180 / PI);
  } else {
    der = -(acos(yPos / rPos) * 180 / PI);
  }

  int eTheta = Theta - der;

  if ((xLawan1 <= 50 && xLawan1 > -50) && rLawan <= 80 && yLawan1 >= 0 && xLawan >= 0) {
    runXYPwm(0, -yLawan1 * 0.7, eTheta, 130);
  }
  else if ((xLawan1 <= 50 && xLawan1 > -50) && rLawan <= 90 && yLawan1 >= 0 && xLawan >= 0) {
    runXYPwm(0, -yLawan1 * 0.5, eTheta, 130);
  }
  else if ((xLawan1 <= 50 && xLawan1 > -50) && rLawan <= 100 && yLawan1 >= 0 && xLawan >= 0) {
    runXYPwm(0, -yLawan1 * 0.2, eTheta, 130);
  }

  else if ((xLawan1 >= 10 || xLawan1 <= -10) && yLawan1 >= 100 && xLawan >= 0) {
    runXYPwm(xLawan1 * 0.7, eTheta, 0, 150);
  }
  else if ((yLawan1 < 100 && yLawan1 >= 0) && xLawan >= 0) {
    runXYPwm(0, -yLawan1 * 0.6, eTheta, 150);
  }
  else if (yLawan1 < 0 && xLawan >= 0) {
    runXYPwm(0, yLawan1 * 0.6, eTheta, 150);
  }
  
  else if (countGaDetek > 80) {
    countGaDetek = 0;
    countPosisioning++;
    if (countPosisioning > 2) {
      countPosisioning = 0;
    }
  }
  else if (countGaDetek > 40) {
    runKinematik(350, 200, 0, 100);
  }
  else {
    errorIL = errorIR = errorIB = 0;
    kir = kan = bel = 0;
    motor(0, 0, 0);
    dribble(0);
    dapatBola = 'N';
  }


  //lcd.setCursor(16, 2); lcd.print(countGaDetek); lcd.print("  ");
//    lcd.setCursor(0,0); lcd.print("x");
//    lcd.print((int)xPos); lcd.print(" y");
//    lcd.print((int)yPos); lcd.print(" r");
//    lcd.print((int)rPos); lcd.print(" d");
//    lcd.print(der); lcd.print("   ");

//    lcd.setCursor(0,0); lcd.print("X");
//    lcd.print((int)xLawan1); lcd.print(" Y");
//    lcd.print((int)yLawan1); lcd.print(" R");
//    lcd.print((int)rLawan); lcd.print("  ");
//
//    lcd.setCursor(0,1); lcd.print("x");
//    lcd.print((int)xLawan); lcd.print(" y");
//    lcd.print((int)yLawan); lcd.print("  ");
//  
//    lcd.setCursor(0,2); lcd.print("L");
//    lcd.print((int)pwmL); lcd.print(" R");
//    lcd.print((int)pwmR); lcd.print(" B");
//    lcd.print((int)pwmB); lcd.print("  "); lcd.print(countGaDetek); lcd.print("  ");

  if (jarDribble() < 10) {
    dribble(0);
  }
  else {
    dribble(0);
  }
}

void pidBolaJaga1(int der) {
  float kp = 1.3, kp1 = 0.6, kp2 = 1, ki = 0, kd = 0.5, kd1 = 0.6;
  int  maxSpd = 45, maxSpd1 = 400, maxPWM = 160, maxPWM1 = 200, maxSpd2 = 300, errorY, errorX, maxx;
  int kir, kan, bel , PIDD;
  int runX, runY, runW;

  parseBola();

  if (yBola >= 0) {
    errorIL = 0;
    errorIR = 0;
    errorIB = 0;
    detekBola = 1;
    countGaDetek = 0;
    errorY = spY - yBola;
  }
  else {
    errorY = 0;
  }
  if (xBola >= 0) {
    errorIL = 0;
    errorIR = 0;
    errorIB = 0;
    detekBola = 1;
    countGaDetek = 0;
    errorX = spX - xBola;

    xBola1 = xBola - spX;
    yBola1 = spY - yBola;
    rBola = 10 * sqrt((xBola1 / 10 * xBola1 / 10) + (yBola1 / 10 * yBola1 / 10));

    if (xBola1 >= 0) {
      dBola = (acos(yBola1 / rBola) * 180 / PI);
    }
    else {
      dBola = -(acos(yBola1 / rBola) * 180 / PI);
    }
  }
  else {
    errorX = 0;
    detekBola = 0;
    countDetek = 0;
    xBola1 = 0;
    yBola1 = 0;
    rBola  = 0;
  }

  int eTheta = Theta - der;

  if ((xBola1 <= 70 && xBola1 > -70) && rBola <= 20 && xBola >= 0) {
    runXYPwm(0, -yBola1 * 2.9, eTheta, 120);
  }
  else if ((xBola1 <= 70 && xBola1 > -70) && rBola <= 40 && xBola >= 0) {
    runXYPwm(0, -yBola1 * 2.3, eTheta, 120);
  }
  else if ((xBola1 <= 70 && xBola1 > -70) && rBola <= 50 && xBola >= 0) {
    runXYPwm(0, -yBola1 * 2.1, eTheta, 120);
  }
  else if ((xBola1 <= 70 && xBola1 > -70) && rBola <= 70 && xBola >= 0) {
    runXYPwm(0, -yBola1 * 1.7, eTheta, 120);
  }
  else if (rBola > 90 && yBola1 >= 60 && xBola >= 0) {
    runXYPwm(xBola1 * 1.1, yBola1 * 1.1, -dBola * 1.1, 120);
  }
  else if ((eTheta < -10)) {
    runXYPwm(-80, -30, -30, 130);
  }
  else if ((eTheta > 10)) {
    runXYPwm(80, 30, 30, 130);
  }

  else if ((xBola1 >= 10 || xBola1 <= -10) && yBola1 >= 50 && xBola >= 0) {
    runXYPwm(xBola1 * 1.1, 0, eTheta, 120);
  }
  else if (xBola1 > 10 && yBola1 < 50 && xBola >= 0) {
    runXYPwm(yBola1 * 1.1, -xBola1 * 1.1, eTheta, 120);
  }
  else if (xBola1 < -10 && yBola1 < 50 && xBola >= 0) {
    runXYPwm(-yBola1 * 1.1, xBola1 * 1.1, eTheta, 120);
  }
  else if (countGaDetek > 130) {
    countGaDetek = 0;
    countPosisioning++;
    if (countPosisioning > 2) {
      countPosisioning = 0;
    }
  }
  else if (countGaDetek > 90) { //60
    runKinematik(xTarget, yTarget, tTarget, 100);
  }
  else if (countGaDetek > 20) {
    if (posisiBola == 'L') {
      motor(-30, -30, -30);
      dribble(0);
    }
    else {
      motor(30, 30, 30);
      dribble(0);
    }
    lcd.setCursor(0, 1); lcd.print("Scan");
  }
  else if (countGaDetek > 2) {
    if (posisiBola == 'L') {
      motor(30, 30, 30);
      dribble(0);
    }
    else {
      motor(-30, -30, -30);
      dribble(0);
    }
    lcd.setCursor(0, 1); lcd.print("Scan");
  }
  else {
    errorIL = errorIR = errorIB = 0;
    kir = kan = bel = 0;
    motor(0, 0, 0);
    dribble(0);
    dapatBola = 'N';
  }

  if (errorX > 0) {
    posisiBola = 'L';
  }
  if (errorX < 0) {
    posisiBola = 'R';
  }
  if (errorX > 100 || errorX < -100) {
    koreksiBola = 'Y';
  }
  else {
    koreksiBola = 'N';
  }

  lcd.setCursor(0, 0); lcd.print("x");
  lcd.print(xBola); lcd.print(" y");
  lcd.print(yBola); lcd.print("  ");

  lcd.setCursor(0, 1); lcd.print("X");
  lcd.print((int)xBola1); lcd.print(" Y");
  lcd.print((int)yBola1); lcd.print(" R");
  lcd.print((int)rBola); lcd.print("  ");

  lcd.setCursor(0, 2); lcd.print("L");
  lcd.print((int)kir); lcd.print(" R");
  lcd.print((int)kan); lcd.print(" B");
  lcd.print((int)bel); lcd.print(" ");

  lcd.setCursor(0, 3); lcd.print("L");
  lcd.print((int)pwmL); lcd.print(" R");
  lcd.print((int)pwmR); lcd.print(" B");
  lcd.print((int)pwmB); lcd.print(" ");

  if (jarDribble() < 30) {
    dribble(0);
  }
  else {
    dribble(0);
  }
}

void pidJagaGawang() {
  float kp = 3.5, kp1 = 0.6, kp2 = 1, ki = 0, kd = 0.5, kd1 = 0.6;
  int  maxSpd = 45, maxSpd1 = 400, maxPWM = 160, maxPWM1 = 200, maxSpd2 = 300, errorY, errorX, maxx;
  int kir, kan, bel , PIDD;
  int runX, runY, runW;

  parseBola();

  if (yBola >= 0) {
    errorIL = 0;
    errorIR = 0;
    errorIB = 0;
    detekBola = 1;
    countGaDetek = 0;
    errorY = spY - yBola;
  }
  else {
    errorY = 0;
  }
  if (xBola >= 0) {
    errorIL = 0;
    errorIR = 0;
    errorIB = 0;
    detekBola = 1;
    countGaDetek = 0;
    errorX = spX - xBola;

    xBola1 = xBola - spX;
    yBola1 = spY - yBola;
    rBola = 10 * sqrt((xBola1 / 10 * xBola1 / 10) + (yBola1 / 10 * yBola1 / 10));

    if (xBola1 >= 0) {
      dBola = (acos(yBola1 / rBola) * 180 / PI);
    }
    else {
      dBola = -(acos(yBola1 / rBola) * 180 / PI);
    }
  }
  else {
    errorX = 0;
    detekBola = 0;
    countDetek = 0;
    xBola1 = 0;
    yBola1 = 0;
    rBola  = 0;
  }

  float speedX = xBola1 * kp;

  if (speedX > 70) {
    speedX = 70;
  }
  if (speedX < -70) {
    speedX = -70;
  }

  if ((xBola1 >= 2) && jarX < 300 + 50) {
    runKinematik(300 + speedX, 10, 0, 200);
  }
  else if ((xBola1 <= -2) && jarX > 300 - 50) {
    runKinematik(300 + speedX, 10, 0, 200);
  }
  else if ((xBola > 0) && jarX > 300 + 40) {
    runKinematik(300 + 40, 10, 0, 200);
  }
  else if ((xBola > 0) && jarX < 300 - 40) {
    runKinematik(300 - 40, 10, 0, 200);
  }
  else {
    errorIL = errorIR = errorIB = 0;
    kir = kan = bel = 0;
    motor(0, 0, 0);
    titikTuju = 0;
  }

  if (countGaDetek > 10) {
    runKinematik(300, 10, 0, 100);
  }

  lcd.setCursor(0, 0); lcd.print("x");
  lcd.print(xBola); lcd.print(" y");
  lcd.print(yBola); lcd.print(" j");
  lcd.print(rBola); lcd.print("  ");

  lcd.setCursor(0, 1); lcd.print("X");
  lcd.print((int)xBola1); lcd.print(" Y");
  lcd.print((int)yBola1); lcd.print(" R");
  lcd.print((int)rBola); lcd.print("  ");
}

void pidBolaXY() {
  int kir, kan, bel;
  int maxSpd = 45, maxSpd1 = 400, maxPWM = 160, maxPWM1 = 200, maxSpd2 = 300,
      sumbuX, sumbuY, errorY, errorX, maxx;
  float x, y, r, sudutTuju;
  float kp = 1.3, kp1 = 0.6, kp2 = 1, ki = 0, kd = 0.5, kd1 = 0.6;

  parseBola();

  if (xBola >= 0) {
    sumbuX = xBola - spX;
    sumbuY = spY - yBola;
    detekBola = 1;
  } else {
    detekBola = 0;
  }

  if (detekBola == 1) {
    x = sumbuX;
    y = sumbuY;
    r = sqrt((x * x) + (y * y));

    if (x >= 0) {
      sudutTuju = (acos(y / r) * 180 / PI);
    }
    else {
      sudutTuju = -(acos(y / r) * 180 / PI);
    }

    lcd.setCursor(0, 2);
    lcd.print(sudutTuju); lcd.print("  ");

    if (x < -130 || x > 130) {
      runXYPwm(x * 1.2, y * 1.2, -sudutTuju, 180);
    }
    else if (r > 80) {
      runXYPwm(x * 1.2, y * 1.2, -sudutTuju, 120);
    }
    else if ((Theta > 10 && Theta < 180) || Theta < -180) {
      runXYPwm(y * 1.2, -x * 1.2, -sudutTuju * 1.2, 180);
    }
    else if ((Theta < -10 && Theta > -180) || Theta > 180) {
      runXYPwm(-y * 1.2, x * 1.2, -sudutTuju * 1.2, 180);
    }
    //    else if(x < -15 || x > 15){motor(-sudutTuju*1.1,-sudutTuju*1.1,-sudutTuju*1.1);}
    else {
      errorIL = 0;
      errorIR = 0;
      errorIB = 0;
      motor(0, 0, 0); lcd.print("stop");
    }
  }
  else {
    errorIL = 0;;
    errorIR = 0;
    errorIB = 0;
    motor(0, 0, 0); lcd.print("stop");
  }

  lcd.setCursor(0, 0); lcd.print("x");
  lcd.print(xBola); lcd.print(" y");
  lcd.print(yBola); lcd.print("  ");

  lcd.setCursor(0, 1); lcd.print("X");
  lcd.print((int)x); lcd.print(" Y");
  lcd.print((int)y); lcd.print(" R");
  lcd.print((int)r); lcd.print("  ");

  lcd.setCursor(0, 3); lcd.print("L");
  lcd.print((int)pwmL); lcd.print(" R");
  lcd.print((int)pwmR); lcd.print(" B");
  lcd.print((int)pwmB); lcd.print(" ");


}

void testPidBola360() {
  delay(300); lcd.clear();
  while (!C()) {
    parseBola();
    if (dapatBola == 'N') {
      sudahMundur = 'N';
      pidBola360();
      //pidBola360A();
      //pidLawan360();
    }
    else if (jarDribble() > jarbo) {
      dapatBola = 'N';
      dribble(0);
    }
    else {
      dribble(255);
      motor(0, 0, 0);
//      if (sudahMundur == 'N') {
//        motor(50, -50, 0);
//        delay(200);
//        motor(0, 0, 0);
//        sudahMundur = 'Y';
//      }
//      if ((Theta > 5 && Theta <= 180) || Theta < -180) {
//        motor(55, 55, 65);
//        lcd.setCursor(13, 1);
//        lcd.print("roL");
//      }
//      else if ((Theta < -5 && Theta >= -180) || Theta > 180) {
//        motor(-55, -55, -65);
//        lcd.setCursor(13, 1);
//        lcd.print("roR");
//      }
//      else {
//        //          motor(0,0,0); dribble(5);
//        int jarKi = jarki();
//        int jarKa = jarka();
//        int jarDe = jarDeL();
//        int jarBe = jarBeL();
//
//        if (jarKi > 250 && jarKa > 250) {
//          lcd.setCursor(13, 1); lcd.print("Cen");
//          tendang();
//        }
//        else if (jarKi < 250 && jarKa > 250) {
//          lcd.setCursor(13, 1); lcd.print("Kir");
//          if (jarKi < 150) {
//            motor(-55, -65, -65); delay(100); motor(0, 0, 0);
//            tendang();
//          }
//          else {
//            motor(-55, -65, -65); delay(70); motor(0, 0, 0);
//            tendang();
//          }
//        }
//        else if (jarKi > 250 && jarKa < 250) {
//          lcd.setCursor(13, 1); lcd.print("Kan");
//          if (jarKi < 150) {
//            motor(65, 55, 65); delay(100); motor(0, 0, 0);
//            tendang();
//          }
//          else {
//            motor(65, 55, 65); delay(70); motor(0, 0, 0);
//            tendang();
//          }
//        }
//        else {
//          if (jarBe < 300 && jarKa > 100 && jarKi > 100) {
//            motor(-130, 130, 0); delay(700);
//          }
//          else if (jarDe < 100 && jarBe > 100 && jarKa < 200 && jarKi < 200) {
//            motor(130, -130, 0); delay(700);
//          }
//          else if (jarDe < 100 && jarBe > 100 && jarKa < 200 && jarKi > 100) {
//            motor(130, 0, -130); delay(700);
//          }
//          else if (jarDe < 100 && jarBe > 100 && jarKa > 100 && jarKi < 200) {
//            motor(0, -130, 130); delay(700);
//          }
//          else {
//            motor(120, -120, 0); delay(700);
//            motor(0, 0, 0);
//            if (jarKi < jarKa) {
//              motor(-55, -65, -65); delay(200); motor(0, 0, 0);
//              tendang();
//            }
//            else {
//              motor(65, 55, 65); delay(200); motor(0, 0, 0);
//              tendang();
//            }
//          }
//        }
//
//      }

    }
  } motor(0, 0, 0); delay(300); lcd.clear();
}

void testPidBola() {
  lcd.clear();
  while (!C()) {
//    parseBola();
//    lcd.setCursor(0, 0);
//    lcd.print("x"); lcd.print((int)xBola);
//    lcd.print(" y"); lcd.print((int)yBola); lcd.print("  ");
//    lcd.setCursor(0, 1);
//    lcd.print("X"); lcd.print((int)jarX);
//    lcd.print(" Y"); lcd.print((int)jarY);
//    lcd.print(" T"); lcd.print((int)Theta); lcd.print("  ");
      
    if (dapatBola == 'N') {
      sudahMundur = 'N';
      pidBola();
    }
    else if (jarDribble() > jarbo) {
      dapatBola = 'N';
      dribble(0);
    }
    else {
      dribble(255);
//      if (sudahMundur == 'N') {
//        motor(50, -50, 0);
//        delay(200);
//        motor(0, 0, 0);
//        sudahMundur = 'Y';
//      }
//      if ((Theta > 5 && Theta <= 180) || Theta < -180) {
//        motor(55, 55, 65);
//        lcd.setCursor(13, 1);
//        lcd.print("roL");
//      }
//      else if ((Theta < -5 && Theta >= -180) || Theta > 180) {
//        motor(-55, -55, -65);
//        lcd.setCursor(13, 1);
//        lcd.print("roR");
//      }
//      else {
//        //          motor(0,0,0); dribble(5);
//        int jarKi = jarki();
//        int jarKa = jarka();
//        int jarDe = jarDeL();
//        int jarBe = jarBeL();
//
//        if (jarKi > 250 && jarKa > 250) {
//          lcd.setCursor(13, 1); lcd.print("Cen");
//          tendang();
//        }
//        else if (jarKi < 250 && jarKa > 250) {
//          lcd.setCursor(13, 1); lcd.print("Kir");
//          if (jarKi < 150) {
//            motor(-55, -65, -65); delay(100); motor(0, 0, 0);
//            tendang();
//          }
//          else {
//            motor(-55, -65, -65); delay(70); motor(0, 0, 0);
//            tendang();
//          }
//        }
//        else if (jarKi > 250 && jarKa < 250) {
//          lcd.setCursor(13, 1); lcd.print("Kan");
//          if (jarKi < 150) {
//            motor(65, 55, 65); delay(100); motor(0, 0, 0);
//            tendang();
//          }
//          else {
//            motor(65, 55, 65); delay(70); motor(0, 0, 0);
//            tendang();
//          }
//        }
//        else {
//          if (jarBe < 300 && jarKa > 100 && jarKi > 100) {
//            motor(-130, 130, 0); delay(700);
//          }
//          else if (jarDe < 100 && jarBe > 100 && jarKa < 200 && jarKi < 200) {
//            motor(130, -130, 0); delay(700);
//          }
//          else if (jarDe < 100 && jarBe > 100 && jarKa < 200 && jarKi > 100) {
//            motor(130, 0, -130); delay(700);
//          }
//          else if (jarDe < 100 && jarBe > 100 && jarKa > 100 && jarKi < 200) {
//            motor(0, -130, 130); delay(700);
//          }
//          else {
//            motor(120, -120, 0); delay(700);
//            motor(0, 0, 0);
//            if (jarKi < jarKa) {
//              motor(-55, -65, -65); delay(200); motor(0, 0, 0);
//              tendang();
//            }
//            else {
//              motor(65, 55, 65); delay(200); motor(0, 0, 0);
//              tendang();
//            }
//          }
//        }
//
//      }

    }
  } motor(0, 0, 0); delay(300); lcd.clear();
}

void testPidBolaBertahan() {
  delay(300); lcd.clear();
  while (!C()) {
    //runXYPwm(-80, -30, -30, 130);
    //runXYPwm(80, 30, 30, 130);
    //runXYPwm(-100, 70, 55, 180);
    //lcd.setCursor(0,3); lcd.print("L");
    //lcd.print((int)pwmL); lcd.print(" R");
    //lcd.print((int)pwmR); lcd.print(" B");
    //lcd.print((int)pwmB); lcd.print(" ");
    //pidBertahanA();
    pidBertahan(300);
    //pidBertahanLawan();
    /*parseBola();
    if (dapatBola == 'N') {
      sudahMundur = 'N';
      //pidBola360();
      pidBola360A();
      //pidLawan360();
      //pidLawan360();
    }
    else if (jarDribble() > jarbo) {
      dapatBola = 'N';
      dribble(0);
    }
    else {
      dribble(255);
      motor(0, 0, 0);
    }*/
  }
  motor(0, 0, 0);
  delay(300); lcd.clear();
}

void testPidBolaXY() {
  delay(300); lcd.clear();
  while (!C()) {
    pidBolaJaga();
    //    pidBolaXY();
  }
  motor(0, 0, 0);
  delay(300); lcd.clear();
}

void keTengahLapanganS() {
  if      ((Theta <= 180 && Theta > 90 + 10) || (Theta <= -180 && Theta > -270 + 10)) {
    motor(55, 55, 55);
    lcd.setCursor (15, 1);
    lcd.print("roL");
  }
  else if ((Theta <= 0 && Theta > -90 + 10) || (Theta <= 360 && Theta > 270 + 10)) {
    motor(55, 55, 55);
    lcd.setCursor (15, 1);
    lcd.print("roL");
  }
  else if ((Theta > 0 && Theta < 90 - 10) || (Theta >= -360 && Theta < -270 - 10)) {
    motor(-55, -55, -55);
    lcd.setCursor(15, 1);
    lcd.print("roR");
  }
  else if ((Theta > 180 && Theta < 270 - 10) || (Theta > -180 && Theta < -90 - 10)) {
    motor(-55, -55, -55);
    lcd.setCursor(15, 1);
    lcd.print("roR");
  }
  else {
    motor(0, 0, 0);
  }
}

void keTitikLapangan180(int sX, int jarakbelakang) {
  int jarakbe, jarakki, jarakde, jarakka, pwmX, pwmY, sumbuX, sumbuY;

  if      ((Theta > 180 + 10 && Theta <= 360) || (Theta > -180 + 10 && Theta < 0)) {
    dribble(255);
    motor(40, 40, 40);
    lcd.setCursor (0, 2);
    lcd.print("l");
  }
  else if ((Theta < 180 - 10 && Theta >= 0) || (Theta < -180 - 10 && Theta >= -360)) {
    dribble(255);
    motor(-40, -40, -40);
    lcd.setCursor (0, 2);
    lcd.print("r");
    kir = kan = bel = 0;
    countPos = 21;
  }
  else {
    countPos++;
    if (countPos > 40) {
      jarakki = jarki();
      jarakka = jarka();
      jarakbe = jarBeL() + jarBeR() / 2;
      jarakde = jarDeL() + jarDeR() / 2;

      sumbuX = (jarakki) - (jarakka + sX);
      sumbuY = jarakbe - jarakbelakang;

      if (pwmX > 255) {
        pwmX = 255;
      } if (pwmX < -255) {
        pwmX = -255;
      }
      if (pwmY > 255) {
        pwmY = 255;
      } if (pwmY < -255) {
        pwmY = -255;
      }

      if (jarakbe > 360 && jarakka > 100 && jarakki > 100) {
        dribble(255);
        kir = 140;
        kan = -140;
        bel = 0;
        tengahX = 0;
        tengahY = 0;
        lcd.print("B");
      }
      else if (jarakbe > 360 && jarakka > 100 && jarakki < 100) {
        dribble(255);
        kir = 70;
        kan = -140;
        bel = 70;
        tengahX = 0;
        tengahY = 0;
        lcd.print("B");
      }
      else if (jarakde > 360 && jarakka < 100 && jarakki > 100) {
        dribble(255);
        kir = 140;
        kan = -70;
        bel = -70;
        tengahX = 0;
        tengahY = 0;
        lcd.print("B");
      }
      else if (jarakbe < 150 && jarakde > 150) {
        dribble(10);
        kir = -140;
        kan = 140;
        bel = 0;
        tengahX = 0;
        tengahY = 1;
        lcd.print("F");
      }

      else if (jarakde > 360 && jarakka > 200 && jarakki < 200) {
        dribble(255);
        kir = 0;
        kan = -100;
        bel = 100;
        tengahX = 0;
        tengahY = 0;
        lcd.print("B");
      }
      else if (jarakde > 360 && jarakka < 200 && jarakki > 200) {
        dribble(255);
        kir = 100;
        kan = -0;
        bel = -100;
        tengahX = 0;
        tengahY = 0;
        lcd.print("B");
      }

      else if (jarakbe < 10) {
        dribble(10);
        kir = -150;
        kan = 150;
        bel = 0;
        tengahX = 0;
        tengahY = 1;
        lcd.print("F");
      }
      //      else if(jarakbe > jarakbelakang-10 && jarakde < 10) {dribble(255); kir=100; kan=-100; bel=0; tengahX=0; tengahY=0; lcd.print("B");}
      //      else if(jarakbe < jarakbelakang-10) {dribble(10); kir=-150; kan=150; bel=0; tengahX=0; tengahY=1; lcd.print("F");}
      //      else if(jarakbe > jarakbelakang+10) {dribble(255); kir=100; kan=-100; bel=0;tengahX=0; tengahY=0; lcd.print("B");}

      else if (jarakbe > 350 - 10 && jarakde < 10) {
        dribble(255);
        kir = 100;
        kan = -100;
        bel = 0;
        tengahX = 0;
        tengahY = 0;
        lcd.print("B");
      }
      else if (jarakbe < 350 - 10) {
        dribble(10);
        kir = -150;
        kan = 150;
        bel = 0;
        tengahX = 0;
        tengahY = 1;
        lcd.print("F");
      }
      else if (jarakbe > 350 + 10) {
        dribble(255);
        kir = 100;
        kan = -100;
        bel = 0;
        tengahX = 0;
        tengahY = 0;
        lcd.print("B");
      }

      else if (jarakki > 300 && jarakka < 300) {
        dribble(255);
        kir = 50;
        kan = 50;
        bel = -100;
        tengahX = 0;
        tengahY = 1;
        lcd.print("L");
      }
      else if (jarakki < 300 && jarakka > 300) {
        dribble(255);
        kir = -50;
        kan = -50;
        bel = 100;
        tengahX = 0;
        tengahY = 1;
        lcd.print("R");
      }
      else if (jarakki < 300 && jarakka < 300 && sumbuX > sX + 80) {
        dribble(255);
        kir = 50;
        kan = 50;
        bel = -100;
        tengahX = 0;
        tengahY = 1;
        lcd.print("L");
      }
      else if (jarakki < 300 && jarakka < 300 && sumbuX < sX - 80) {
        dribble(255);
        kir = -50;
        kan = -50;
        bel = 100;
        tengahX = 0;
        tengahY = 1;
        lcd.print("R");
      }


      else {
        dribble(255);
        kir = 0;
        kan = 0;
        bel = 0;
        tengahX = 1;
        tengahY = 1;
        lcd.print("S");
      }

      lcd.setCursor(0, 0); lcd.print(sumbuX);
      lcd.print(" T"); lcd.print((int)Theta); lcd.print(" B");
      lcd.print(jarakbe); lcd.print(" ");

      lcd.setCursor(0, 1); lcd.print("L");
      lcd.print(jarakki); lcd.print(" R");
      lcd.print(jarakka); lcd.print(" D");
      lcd.print(jarakde); lcd.print("  ");

      countPos = 0;
    }
    else {
      dribble(255);
      motor(kir, kan, bel);
    }

    lcd.setCursor(0, 3);
    lcd.print(kir); lcd.print(" ");
    lcd.print(kan); lcd.print(" ");
    lcd.print(bel); lcd.print(" ");
    lcd.print(countPos); lcd.print("   ");
  }
}

void keTitikLapangan0(int sX, int jarakdepan) {
  int jarakbe, jarakki, jarakde, jarakka, pwmX, pwmY, sumbuX, sumbuY;

  if      (Theta > 10 && Theta <= 180) {
    dribble(255);
    motor(40, 40, 40);
    lcd.setCursor (0, 2);
    lcd.print("l");
  }
  else if (Theta < -10 && Theta >= -180) {
    dribble(255);
    motor(-40, -40, -40);
    lcd.setCursor (0, 2);
    lcd.print("r");
    kir = kan = bel = 0;
    countPos = 21;
  }
  else {
    countPos++;
    if (countPos > 30) {
      jarakki = jarki();
      jarakka = jarka();
      jarakbe = jarBeL() + jarBeR() / 2;
      jarakde = jarDeL() + jarDeR() / 2;

      sumbuX = (jarakki) - (jarakka + sX);
      sumbuY = jarakbe - jarakdepan;

      if (pwmX > 255) {
        pwmX = 255;
      } if (pwmX < -255) {
        pwmX = -255;
      }
      if (pwmY > 255) {
        pwmY = 255;
      } if (pwmY < -255) {
        pwmY = -255;
      }

      if (jarakbe > 330 && jarakka > 100 && jarakki > 100) {
        dribble(255);
        kir = 140;
        kan = -140;
        bel = 0;
        tengahX = 0;
        tengahY = 0;
        lcd.print("B");
      }
      else if (jarakbe > 330 && jarakka > 100 && jarakki < 100) {
        dribble(255);
        kir = 70;
        kan = -140;
        bel = 70;
        tengahX = 0;
        tengahY = 0;
        lcd.print("B");
      }
      else if (jarakde > 330 && jarakka < 100 && jarakki > 100) {
        dribble(255);
        kir = 140;
        kan = -70;
        bel = -70;
        tengahX = 0;
        tengahY = 0;
        lcd.print("B");
      }
      else if (jarakde > 330 && jarakka < 100 && jarakki < 100) {
        dribble(255);
        kir = 140;
        kan = -70;
        bel = -70;
        tengahX = 0;
        tengahY = 0;
        lcd.print("B");
      }
      else if (jarakbe < 150 && jarakde > 150) {
        dribble(10);
        kir = -140;
        kan = 140;
        bel = 0;
        tengahX = 0;
        tengahY = 1;
        lcd.print("F");
      }

      else if (jarakde > 330 && jarakka > 250 && jarakki < 250) {
        dribble(255);
        kir = 50;
        kan = -100;
        bel = 50;
        tengahX = 0;
        tengahY = 0;
        lcd.print("B");
      }
      else if (jarakde > 330 && jarakka < 250 && jarakki > 250) {
        dribble(255);
        kir = 100;
        kan = -50;
        bel = -50;
        tengahX = 0;
        tengahY = 0;
        lcd.print("B");
      }

      else if (jarakbe < 30) {
        dribble(10);
        kir = -150;
        kan = 150;
        bel = 0;
        tengahX = 0;
        tengahY = 1;
        lcd.print("F");
      }
      else if (jarakde < jarakdepan + 10 && jarakde < 10) {
        dribble(255);
        kir = 100;
        kan = -100;
        bel = 0;
        tengahX = 0;
        tengahY = 0;
        lcd.print("B");
      }
      else if (jarakde > jarakdepan + 10) {
        dribble(10);
        kir = -150;
        kan = 150;
        bel = 0;
        tengahX = 0;
        tengahY = 1;
        lcd.print("F");
      }
      else if (jarakde < jarakdepan - 10) {
        dribble(255);
        kir = 100;
        kan = -100;
        bel = 0;
        tengahX = 0;
        tengahY = 0;
        lcd.print("B");
      }

      else if (jarakki > 200 && jarakka < 200) {
        dribble(255);
        kir = 50;
        kan = 50;
        bel = -100;
        tengahX = 0;
        tengahY = 1;
        lcd.print("L");
      }
      else if (jarakki < 200 && jarakka > 200) {
        dribble(255);
        kir = -50;
        kan = -50;
        bel = 100;
        tengahX = 0;
        tengahY = 1;
        lcd.print("R");
      }
      else if (jarakki < 200 && jarakka < 200 && sumbuX > sX + 70) {
        dribble(255);
        kir = 50;
        kan = 50;
        bel = -100;
        tengahX = 0;
        tengahY = 1;
        lcd.print("L");
      }
      else if (jarakki < 200 && jarakka < 200 && sumbuX < sX - 70) {
        dribble(255);
        kir = -50;
        kan = -50;
        bel = 100;
        tengahX = 0;
        tengahY = 1;
        lcd.print("R");
      }

      else {
        dribble(255);
        kir = 0;
        kan = 0;
        bel = 0;
        tengahX = 1;
        tengahY = 1;
        lcd.print("S");
      }

      lcd.setCursor(0, 0); lcd.print(sumbuX);
      lcd.print(" T"); lcd.print((int)Theta); lcd.print(" B");
      lcd.print(jarakbe); lcd.print(" ");

      lcd.setCursor(0, 1); lcd.print("L");
      lcd.print(jarakki); lcd.print(" R");
      lcd.print(jarakka); lcd.print(" D");
      lcd.print(jarakde); lcd.print("  ");

      countPos = 0;
    }
    else {
      dribble(255);
      motor(kir, kan, bel);
    }

    lcd.setCursor(0, 3);
    lcd.print(kir); lcd.print(" ");
    lcd.print(kan); lcd.print(" ");
    lcd.print(bel); lcd.print(" ");
    lcd.print(countPos); lcd.print("   ");
  }
}

void keTitikBelakang(int sX, int jarakbelakang) {
  int jarakbe, jarakki, jarakde, jarakka, pwmX, pwmY, sumbuX, sumbuY;

  if      (Theta > 10 && Theta <= 180) {
    dribble(255);
    motor(40, 40, 40);
    lcd.setCursor (0, 2);
    lcd.print("l");
  }
  else if (Theta < -10 && Theta >= -180) {
    dribble(255);
    motor(-40, -40, -40);
    lcd.setCursor (0, 2);
    lcd.print("r");
    kir = kan = bel = 0;
    countPos = 21;
  }
  else {
    countPos++;
    if (countPos > 40) {
      jarakki = jarki();
      jarakka = jarka();
      jarakbe = jarBeL() + jarBeR() / 2;
      jarakde = jarDeL() + jarDeR() / 2;

      sumbuX = (jarakki) - (jarakka + sX);
      sumbuY = jarakbe - jarakbelakang;

      if (pwmX > 255) {
        pwmX = 255;
      } if (pwmX < -255) {
        pwmX = -255;
      }
      if (pwmY > 255) {
        pwmY = 255;
      } if (pwmY < -255) {
        pwmY = -255;
      }

      if (jarakbe > jarakbelakang + 10 && jarakka > 100 && jarakki > 100) {
        dribble(255);
        kir = 60;
        kan = -60;
        bel = 0;
        tengahX = 0;
        tengahY = 0;
        lcd.print("B");
      }
      else if (jarakbe > jarakbelakang + 10 && jarakka > 100 && jarakki < 100) {
        dribble(255);
        kir = 30;
        kan = -60;
        bel = 30;
        tengahX = 0;
        tengahY = 0;
        lcd.print("B");
      }
      else if (jarakde > jarakbelakang + 10 && jarakka < 100 && jarakki > 100) {
        dribble(255);
        kir = 60;
        kan = -30;
        bel = -30;
        tengahX = 0;
        tengahY = 0;
        lcd.print("B");
      }
      else if (jarakbe < 150 && jarakde > 150) {
        dribble(10);
        kir = -60;
        kan = 60;
        bel = 0;
        tengahX = 0;
        tengahY = 1;
        lcd.print("F");
      }

      else if (jarakbe > jarakbelakang + 10 && jarakka > 200 && jarakki < 200) {
        dribble(255);
        kir = 30;
        kan = -60;
        bel = 30;
        tengahX = 0;
        tengahY = 0;
        lcd.print("B");
      }
      else if (jarakde > jarakbelakang + 10 && jarakka < 200 && jarakki > 200) {
        dribble(255);
        kir = 60;
        kan = -30;
        bel = -30;
        tengahX = 0;
        tengahY = 0;
        lcd.print("B");
      }

      else if (jarakbe < 10) {
        dribble(10);
        kir = -60;
        kan = 60;
        bel = 0;
        tengahX = 0;
        tengahY = 1;
        lcd.print("F");
      }
      else if (jarakbe > jarakbelakang - 10 && jarakde < 10) {
        dribble(255);
        kir = 60;
        kan = -60;
        bel = 0;
        tengahX = 0;
        tengahY = 0;
        lcd.print("B");
      }
      else if (jarakbe < jarakbelakang - 10) {
        dribble(10);
        kir = -60;
        kan = 60;
        bel = 0;
        tengahX = 0;
        tengahY = 1;
        lcd.print("F");
      }
      else if (jarakbe > jarakbelakang + 10) {
        dribble(255);
        kir = 60;
        kan = -60;
        bel = 0;
        tengahX = 0;
        tengahY = 0;
        lcd.print("B");
      }

      else if (jarakki > 200 && jarakka < 200) {
        dribble(255);
        kir = 30;
        kan = 60;
        bel = -60;
        tengahX = 0;
        tengahY = 1;
        lcd.print("L");
      }
      else if (jarakki < 200 && jarakka > 200) {
        dribble(255);
        kir = -30;
        kan = -30;
        bel = 60;
        tengahX = 0;
        tengahY = 1;
        lcd.print("R");
      }
      else if (jarakki < 200 && jarakka < 200 && sumbuX > sX + 80) {
        dribble(255);
        kir = 30;
        kan = 30;
        bel = -60;
        tengahX = 0;
        tengahY = 1;
        lcd.print("L");
      }
      else if (jarakki < 200 && jarakka < 200 && sumbuX < sX - 80) {
        dribble(255);
        kir = -30;
        kan = -30;
        bel = 60;
        tengahX = 0;
        tengahY = 1;
        lcd.print("R");
      }

      else {
        dribble(255);
        kir = 0;
        kan = 0;
        bel = 0;
        tengahX = 1;
        tengahY = 1;
        lcd.print("S");
      }

      lcd.setCursor(0, 0); lcd.print(sumbuX);
      lcd.print(" T"); lcd.print((int)Theta); lcd.print(" B");
      lcd.print(jarakbe); lcd.print(" ");

      lcd.setCursor(0, 1); lcd.print("L");
      lcd.print(jarakki); lcd.print(" R");
      lcd.print(jarakka); lcd.print(" D");
      lcd.print(jarakde); lcd.print("  ");

      countPos = 0;
    }
    else {
      dribble(255);
      motor(kir, kan, bel);
    }

    lcd.setCursor(0, 3);
    lcd.print(kir); lcd.print(" ");
    lcd.print(kan); lcd.print(" ");
    lcd.print(bel); lcd.print(" ");
    lcd.print(countPos); lcd.print("   ");
  }
}

void hitungKeTengahLapangan() {
  int kir, kan, bel;

  if      ((Theta > 180 + 10 && Theta <= 360) || (Theta > -180 + 10 && Theta < 0)) {
    motor(55, 55, 55);
    lcd.setCursor (15, 1);
    lcd.print("roL");
  }
  else if ((Theta < 180 - 10 && Theta >= 0) || (Theta < -180 - 10 && Theta >= -360)) {
    motor(-55, -55, -55);
    lcd.setCursor (15, 1);
    lcd.print("roR");
  }
  else {
    int jarKi = jarki();
    int jarKa = jarka();
    int jarBe = jarBeL();
    int jarDe = jarDeL();

    lcd.setCursor (15, 1);
    //    if(jarKi<jarKa-80){kir=-60; kan=-60; bel=120; lcd.print("toR");}
    //    else if(jarKa<jarKi-80){kir=60; kan=60; bel=-120; lcd.print("toL");}
    //    else {kir=0; kan=0; bel=0;}
    //
    //    if(jarBe>300){kir+=60; kan+=-60; lcd.print("Mun");}
    //    else if(jarBe<100 && jarDe>100){kir+=-60; kan+=60;  lcd.print("Maj");}
    //    else {kir=kir; kan=kan; lcd.print("Stp");}
    if (jarBe > 300) {
      kir = 100;
      kan = -100;
      lcd.print("Mun");
    }
    else if (jarBe < 100 && jarDe > 100) {
      kir = -100;
      kan = 100;
      lcd.print("Maj");
    }
    else {
      kir = kir;
      kan = kan;
      lcd.print("Stp");
    }

    motor(kir, kan, bel);
  }
}

void keTengahLapangan() {
  lcd.clear();
  while (!C()) {
    keTitikLapangan180(300, 350);
  }
  lcd.clear(); berhenti();
}

void testKeBelakang() {
  lcd.clear();
  while (!C()) {
    keTitikLapangan0(300, 150);
  }
  lcd.clear(); berhenti();
}
void testKeDepan() {
  lcd.clear();
  while (!C()) {
    keTitikBelakang(300, 250);
  }
  lcd.clear(); berhenti();
}

void tendangJarakDekat() {
  dribble(255);
  if ((Theta > 5 && Theta <= 180) || Theta < -180) {
    motor(55, 55, 65);
    lcd.setCursor(13, 1);
    lcd.print("roL");
  }
  else if ((Theta < -5 && Theta >= -180) || Theta > 180) {
    motor(-55, -55, -65);
    lcd.setCursor(13, 1);
    lcd.print("roR");
  }
  else {
    int jarKi = jarki();
    int jarKa = jarka();
    int jarDe = jarDeL();
    int jarBe = jarBeL();

    dribble(255);
    if (jarKi > 250 && jarKa > 250) {
      lcd.setCursor(13, 1); lcd.print("Cen");
      tendang();
    }
    else if (jarKi < 250 && jarKa > 250) {
      lcd.setCursor(13, 1); lcd.print("Kir");
      if (jarKi < 150) {
        motor(-55, -65, -65); delay(70); motor(0, 0, 0);
        tendang();
      }
      else {
        motor(-55, -65, -65); delay(50); motor(0, 0, 0);
        tendang();
      }
    }
    else if (jarKi > 250 && jarKa < 250) {
      lcd.setCursor(13, 1); lcd.print("Kan");
      if (jarKi < 150) {
        motor(65, 55, 65); delay(70); motor(0, 0, 0);
        tendang();
      }
      else {
        motor(65, 55, 65); delay(50); motor(0, 0, 0);
        tendang();
      }
    }
    else {
      if (jarKi < jarKa) {
        motor(-55, -65, -65); delay(200); motor(0, 0, 0);
        tendang();
      }
      else {
        motor(65, 55, 65); delay(200); motor(0, 0, 0);
        tendang();
      }
    }

  }
}

void runFullAuto() {
  delay(300); lcd.clear();
  int count, countKoreksi;
  char pernahStart = 'N';
  enableScanBola = 'N';
  enableKickOff = 'N';


  if (team == 'c') {
    lcd.setCursor(0, 0);
    lcd.print("CYN");
  }
  if (team == 'm') {
    lcd.setCursor(0, 0);
    lcd.print("MGT");
  }
  lcd.print(" S:"); lcd.print(strategi); lcd.print(" ");

  xTarget = 300; yTarget = 250; tTarget = 0;

  while (!C()) {
    parseBola();

    if (intruksiPlay == 'R' && titikTuju == 0) {
      if (enableReadThetaNow == 1) {
        thetaNow = Theta;
        enableReadThetaNow = 0;
      }
      runKinematik(xTarget, yTarget, tTarget, 130);
    }

/*    else if((xBola>-1 && titikTuju==1) &&
      ((team=='c' && (intruksiFull=='N' || intruksiFull=='K' || intruksiFull=='F' || intruksiFull=='G' || intruksiFull=='C' || intruksiFull=='P'))||
      (team=='m' && (intruksiFull=='N' || intruksiFull=='k' || intruksiFull=='f' || intruksiFull=='g' || intruksiFull=='c' || intruksiFull=='p')))){
    
       if (intruksiFull == 'K' || intruksiFull == 'k') {
          pidBolaJaga(90);
       } else{
          pidBolaJaga(0);
       }
          countKoreksi=0;
          //titikTuju=1;
       }
       else if(xBola==-1){
          countKoreksi++;
       if(countKoreksi>100){countKoreksi=0; titikTuju=0;}
       }*/

    if (team == 'c') {
      lcd.setCursor(0, 0);
      lcd.print("CYN");
    }
    if (team == 'm') {
      lcd.setCursor(0, 0);
      lcd.print("MGT");
    }
    lcd.print(" S:"); lcd.print(strategi); lcd.print(" A:"); lcd.write(enableBertahan);
    lcd.print(" B:"); lcd.write(enableSerang); lcd.print(" C:"); lcd.write(enableBertahanSerang); lcd.print(" ");

    lcd.setCursor(0, 1);
    lcd.print("X"); lcd.print((int)jarX);
    lcd.print(" Y"); lcd.print((int)jarY);
    lcd.print(" T"); lcd.print((int)Theta); lcd.print("  ");
    lcd.setCursor(16, 1); lcd.print("T:"); lcd.write(sudahTendang); lcd.print(" ");
    
    //    if(bolehMain=='N'){
    parseBola();
    lcd.setCursor(0, 2);
    lcd.print("X"); lcd.print(xBola);
    lcd.print(" Y"); lcd.print(yBola); lcd.print("  ");

    lcd.setCursor(0, 3);
    //lcd.print("Play Now: ");
    lcd.print("X"); lcd.print(xBola360);
    lcd.print(" Y"); lcd.print(yBola360); lcd.print("  ");
    //    }

    listenCoach();

    if (enableInisial == 'Y') {
      //inisial Tim kita
      if (team == 'c' && (intruksiFull == 'N' || intruksiFull == 'K' || intruksiFull == 'F' || intruksiFull == 'G' || intruksiFull == 'C' || intruksiFull == 'P')) {
        enableKickOff = 'N'; enablePenalty = 'N';
        if (intruksiFull == 'N') {
          lcd.setCursor(12, 2);
          lcd.print("DropBall");
          xTarget = 300;
          yTarget = 280;
          tTarget = 0;
        }
        if (intruksiFull == 'K') {
          lcd.setCursor(12, 2);
          lcd.print("KickOff ");
          xTarget = 330;
          yTarget = 300;
          tTarget = 0;
          enableKickOff = 'Y';
        }
        if (intruksiFull == 'F') {
          lcd.setCursor(12, 2);
          lcd.print("FreeKick");
          xTarget = 300;
          yTarget = 420;
          tTarget = 0;
        }
        if (intruksiFull == 'G') {
          lcd.setCursor(12, 2);
          lcd.print("GoalKick");
          xTarget = 300;
          yTarget = 100;
          tTarget = 0;
          enableGoalKick = 'Y';
        }
        if (intruksiFull == 'C') {
          lcd.setCursor(12, 2);
          lcd.print("Corner  ");
          xTarget = 300;
          yTarget = 400;
          tTarget = 0;
        }
        if (intruksiFull == 'P') {
          lcd.setCursor(12, 2);
          lcd.print("Penalty ");
          xTarget = 300;
          yTarget = 250;
          tTarget = 0;
          enablePenalty = 'Y';
        }
      }
      else if (team == 'm' && (intruksiFull == 'N' || intruksiFull == 'k' || intruksiFull == 'f' || intruksiFull == 'g' || intruksiFull == 'c' || intruksiFull == 'p')) {
        enableKickOff = 'N';  enablePenalty = 'N';
        if (intruksiFull == 'N') {
          lcd.setCursor(12, 2);
          lcd.print("DropBall");
          xTarget = 300;
          yTarget = 280;
          tTarget = 0;
        }
        if (intruksiFull == 'k') {
          lcd.setCursor(12, 2);
          lcd.print("KickOff ");
          xTarget = 330;
          yTarget = 300;
          tTarget = 0;
          enableKickOff = 'Y';
        }
        if (intruksiFull == 'f') {
          lcd.setCursor(12, 2);
          lcd.print("FreeKick");
          xTarget = 300;
          yTarget = 420;
          tTarget = 0;
        }
        if (intruksiFull == 'g') {
          lcd.setCursor(12, 2);
          lcd.print("GoalKick");
          xTarget = 300;
          yTarget = 100;
          tTarget = 0;
          enableGoalKick = 'Y';
        }
        if (intruksiFull == 'c') {
          lcd.setCursor(12, 2);
          lcd.print("Corner  ");
          xTarget = 300;
          yTarget = 400;
          tTarget = 0;
        }
        if (intruksiFull == 'p') {
          lcd.setCursor(12, 2);
          lcd.print("Penalty ");
          xTarget = 300;
          yTarget = 250;
          tTarget = 0;
          enablePenalty = 'Y';
        }
      }

      //inisial Tim Lawan
      else if (team == 'c' && (intruksiFull == 'k' || intruksiFull == 'f' || intruksiFull == 'g' || intruksiFull == 'c' || intruksiFull == 'p')) {
        enableKickOff = 'N';  enablePenalty = 'N';
        if (intruksiFull == 'k') {
          lcd.setCursor(12, 2);
          lcd.print("KickOff ");
          xTarget = 280;
          yTarget = 320;
          tTarget = 0;
        }
        if (intruksiFull == 'f') {
          lcd.setCursor(12, 2);
          lcd.print("FreeKick");
          xTarget = 280;
          yTarget = 300;
          tTarget = 0;
        }
        if (intruksiFull == 'g') {
          lcd.setCursor(12, 2);
          lcd.print("GoalKick");
          xTarget = 250;
          yTarget = 300;
          tTarget = 0;
        }
        if (intruksiFull == 'c') {
          lcd.setCursor(12, 2);
          lcd.print("Corner  ");
          xTarget = 400;
          yTarget = 100;
          tTarget = 0;
        }
        if (intruksiFull == 'p') {
          lcd.setCursor(12, 2);
          lcd.print("Penalty ");
          xTarget = 300;
          yTarget = 450;
          tTarget = 0;
          enablePenalty = 'Y';
        }
      }
      else if (team == 'm' && (intruksiFull == 'K' || intruksiFull == 'F' || intruksiFull == 'G' || intruksiFull == 'C' || intruksiFull == 'P')) {
        enableKickOff = 'N';  enablePenalty = 'N';
        if (intruksiFull == 'K') {
          lcd.setCursor(12, 2);
          lcd.print("KickOff ");
          xTarget = 280;
          yTarget = 320;
          tTarget = 0;
        }
        if (intruksiFull == 'F') {
          lcd.setCursor(12, 2);
          lcd.print("FreeKick");
          xTarget = 280;
          yTarget = 300;
          tTarget = 0;
        }
        if (intruksiFull == 'G') {
          lcd.setCursor(12, 2);
          lcd.print("GoalKick");
          xTarget = 250;
          yTarget = 300;
          tTarget = 0;
        }
        if (intruksiFull == 'C') {
          lcd.setCursor(12, 2);
          lcd.print("Corner  ");
          xTarget = 400;
          yTarget = 100;
          tTarget = 0;
        }
        if (intruksiFull == 'P') {
          lcd.setCursor(12, 2);
          lcd.print("Penalty ");
          xTarget = 300;
          yTarget = 450;
          tTarget = 0;
          enablePenalty = 'Y';
        }
      }
    }

    if (intruksiR == 'R') {
      motor(-30, -30, -30);
    }
    else if (intruksiR == 'L') {
      motor(30, 30, 30);
    }
    else if (intruksiPlay == 'S') {
      lcd.setCursor(12, 2); lcd.print("       ");
      motor(0, 0, 0);
      errorIL = 0;
      errorIR = 0;
      errorIB = 0;
      dribble(0);
      titikTuju = 0;
      enableReadThetaNow = 1;
      runIntruksi = 'N';
      runBertahan = 'N';
      bolehMain = 'N';
      sudahTendang = 'N';
      enableMain = 'N';
      belumMundur = 'N';
      bertahanKickOff = 'N';
      countDetek = 0;
      countGaDetek = 0;
    }
    if (intruksiPlay == 's') {

      if (enableBertahan == 'Y' && enableSerang == 'Y' && enableBertahanSerang == 'N') {      
        parseBola();
        if (yBola >= 420 && 330 <= xBola <= 490 && jarY < 400) {
          if (dapatBola == 'N') {
            sudahMundur = 'N';
            pidBola360();
          }
          else if (jarDribble() > jarbo) {
            dapatBola = 'N';
            //dribble(0);
          }
          else {
            dribble(255);
            tendanganMacan();
          }
        }     
        else {
          parseBola();
          if (countGaDetek > 100) {
            countGaDetek = 0;  
          } else if ((xBola < 0 && xBola360 < 0) && countGaDetek < 10){
            motor(0, 0, 0);
          } else if ((xBola >= 0 || xBola360 >= 0) && jarY >= 400) {
            runKinematik(280, 350, 0, 120);
          } else if ((xBola >= 0 || xBola360 >= 0) && jarY >= 350) {
            pidBertahan1(); //pidBertahanA();
          } else if ((xBola >= 0 || xBola360 >= 0) && jarY < 350) {
            pidBertahan(300); //pidBertahanB(250);
          } else {
            runKinematik(280, 150, 0, 120);
          }
        }
      }
      else if (enableBertahan == 'Y' && enableSerang == 'N' && enableBertahanSerang == 'N') {
        parseBola();
        if (countGaDetek > 100) {
          countGaDetek = 0;  
        } else if ((xBola < 0 && xBola360 < 0) && countGaDetek < 10){
          motor(0, 0, 0);
        } else if ((xBola >= 0 || xBola360 >= 0) && jarY >= 400) {
          runKinematik(280, 350, 0, 120);
        } else if ((xBola >= 0 || xBola360 >= 0) && jarY >= 350) {
          pidBertahan1();
        } else if ((xBola >= 0 || xBola360 >= 0) && jarY < 350) {
          pidBertahan(300);
        } else {
          runKinematik(280, 150, 0, 120);
        }

        /*if (countGaDetek > 100) {
          countGaDetek = 0;  
        } else if (xLawan < 0 && countGaDetek < 20){
          motor(0, 0, 0);
        } else if (xLawan >= 0 && jarY >= 500) {
          runKinematik(280, 450, 0, 120);
        } else if (xLawan >= 0 && jarY >= 450) {
          pidBertahanLawan();
        } else if (xLawan >= 0 && jarY < 450) {
          pidLawan360();
        } else {
          runKinematik(280, 150, 0, 120);
        }*/
      } 
      else if (enableBertahan == 'Y' && enableSerang == 'Y' && enableBertahanSerang == 'Y') {
        //enableGoalKick = 'N';
        //sudahTendang = 'N';
        //run Tim kita
        if (team == 'c' && (intruksi == 'N' || intruksi == 'K' || intruksi == 'F' || intruksi == 'G' || intruksi == 'C' || intruksi == 'P')) {
          if (intruksi == 'N') {
            lcd.setCursor(12, 2);
            lcd.print("DropBall");
            bolehMain = 'N';
          }
          if (intruksi == 'K') {
            lcd.setCursor(12, 2);
            lcd.print("KickOff ");
            bolehMain = 'Y';
          }
          if (intruksi == 'F') {
            lcd.setCursor(12, 2);
            lcd.print("FreeKick");
            bolehMain = 'Y';
          }
          if (intruksi == 'G') {
            lcd.setCursor(12, 2);
            lcd.print("GoalKick");
            bolehMain = 'Y';
          }
          if (intruksi == 'C') {
            lcd.setCursor(12, 2);
            lcd.print("Corner  ");
            bolehMain = 'N';
          }
          if (intruksi == 'P') {
            lcd.setCursor(12, 2);
            lcd.print("Penalty ");
            bolehMain = 'Y';
          }
        }
        else if (team == 'm' && (intruksi == 'N' || intruksi == 'k' || intruksi == 'f' || intruksi == 'g' || intruksi == 'c' || intruksi == 'p')) {
          if (intruksi == 'N') {
            lcd.setCursor(12, 2);
            lcd.print("DropBall");
            bolehMain = 'N';
          }
          if (intruksi == 'k') {
            lcd.setCursor(12, 2);
            lcd.print("KickOff ");
            bolehMain = 'Y';
          }
          if (intruksi == 'f') {
            lcd.setCursor(12, 2);
            lcd.print("FreeKick");
            bolehMain = 'Y';
          }
          if (intruksi == 'g') {
            lcd.setCursor(12, 2);
            lcd.print("GoalKick");
            bolehMain = 'Y';
          }
          if (intruksi == 'c') {
            lcd.setCursor(12, 2);
            lcd.print("Corner  ");
            bolehMain = 'N';
          }
          if (intruksi == 'p') {
            lcd.setCursor(12, 2);
            lcd.print("Penalty ");
            bolehMain = 'Y';
          }
        }

        //run Tim Lawan
        else if (team == 'c' && (intruksi == 'k' || intruksi == 'f' || intruksi == 'g' || intruksi == 'c' || intruksi == 'p')) {
          if (intruksi == 'k') {
              lcd.setCursor(12, 2);
              lcd.print("KickOff ");
              bertahanKickOff = 'Y';
              bolehMain = 'N';
          }
          if (intruksi == 'f') {
              lcd.setCursor(12, 2);
              lcd.print("FreeKick");
              bertahanKickOff = 'Y';
              bolehMain = 'N';
          }
          if (intruksi == 'g') {
              lcd.setCursor(12, 2);
              lcd.print("GoalKick");
              bertahanKickOff = 'N';
              bolehMain = 'N';
          }
          if (intruksi == 'c') {
              lcd.setCursor(12, 2);
              lcd.print("Corner  ");
              bertahanKickOff = 'N';
              bolehMain = 'N';
          }
          if (intruksi == 'p') {
              lcd.setCursor(12, 2);
              lcd.print("Penalty ");\
              bertahanKickOff = 'N';
              bolehMain = 'N';
         }
        }
        else if (team == 'm' && (intruksi == 'K' || intruksi == 'F' || intruksi == 'G' || intruksi == 'C' || intruksi == 'P')) {
          if (intruksi == 'K') {
              lcd.setCursor(12, 2);
              lcd.print("KickOff ");
              bertahanKickOff = 'Y';
              bolehMain = 'N';
          }
          if (intruksi == 'F') {
              lcd.setCursor(12, 2);
              lcd.print("FreeKick");
              bertahanKickOff = 'Y';
              bolehMain = 'N';
          }
          if (intruksi == 'G') {
              lcd.setCursor(12, 2);
              lcd.print("GoalKick");
              bertahanKickOff = 'N';
              bolehMain = 'N';
          }
          if (intruksi == 'C') {
              lcd.setCursor(12, 2);
              lcd.print("Corner  ");
              bertahanKickOff = 'N';
              bolehMain = 'N';
          }
          if (intruksi == 'P') {
              lcd.setCursor(12, 2);
              lcd.print("Penalty ");
              bertahanKickOff = 'N';
              bolehMain = 'N';
          }
        }

        if (bolehMain == 'Y') {
          if (sudahTendang == 'Y') {
            if (runBertahan == 'N') {
              detikBertahan = 0;
              runBertahan = 'Y';
            } 
            if (belumMundur == 'N' && detikBertahan < 3) {
              runKinematik(300, 250, 0, 150);
            }  
            if (detikBertahan >= 2) {  
              parseBola();
              if (yBola >= 420 && 330 <= xBola <= 490 && jarY < 400) {
                if (dapatBola == 'N') {
                  sudahMundur = 'N';
                  pidBola360();
                }
                else if (jarDribble() > jarbo) {
                  dapatBola = 'N';
                  //dribble(0);
                }
                else {
                  dribble(255);
                  tendanganMacan();
                }
              }     
              else {
                parseBola();
                if (countGaDetek > 100) {
                  countGaDetek = 0;  
                } else if ((xBola < 0 && xBola360 < 0) && countGaDetek < 10){
                  motor(0, 0, 0);
                } else if ((xBola >= 0 || xBola360 >= 0) && jarY >= 400) {
                  runKinematik(280, 350, 0, 120);
                } else if ((xBola >= 0 || xBola360 >= 0) && jarY >= 350) {
                  pidBertahan1(); //pidBertahanA();
                } else if ((xBola >= 0 || xBola360 >= 0) && jarY < 350) {
                  pidBertahan(300); //pidBertahanB(250);
                } else {
                  runKinematik(280, 150, 0, 120);
                }
              }
            }
          }

          if (pernahStart == 'N' && sudahTendang == 'N') {
            countDetek = 0;
            countGaDetek = 0;
            pernahStart = 'Y';
          }
          if (dapatBola == 'N' && sudahTendang == 'N') {
            sudahMundur = 'N';
            pidBola360A();
          }
          else if (jarDribble() > jarbo && sudahTendang == 'N') {
            dapatBola = 'N';
            //dribble(0);
          }
          else if ((jarDribble() <= jarbo || dapatBola == 'Y') && sudahTendang == 'N') {
            dribble(255);
            runToGoal();
            pernahStart = 'N';
          }
        }
        else if (bolehMain == 'N') {
          parseBola();
          bolehMain = 'N';
          if (bertahanKickOff == 'Y') {
            if (runIntruksi == 'N') {
              detikRun = 0;
              runIntruksi = 'Y';
            }
            lcd.setCursor(12, 2); lcd.print(detikRun); lcd.print("   ");

            if (detikRun <= 6) {
              pidBertahan1();
            }

            if (detikRun > 7) {
              parseBola();
              if (yBola >= 420 && 330 <= xBola <= 490 && jarY < 400) {
                if (dapatBola == 'N') {
                  sudahMundur = 'N';
                  pidBola360();
                }
                else if (jarDribble() > jarbo) {
                  dapatBola = 'N';
                  //dribble(0);
                }
                else {
                  dribble(255);
                  tendanganMacan();
                }
              }     
              else {
                parseBola();
                if (countGaDetek > 100) {
                  countGaDetek = 0;  
                } else if ((xBola < 0 && xBola360 < 0) && countGaDetek < 20){
                  motor(0, 0, 0);
                } else if ((xBola >= 0 || xBola360 >= 0) && jarY >= 400) {
                  runKinematik(280, 350, 0, 120);
                } else if ((xBola >= 0 || xBola360 >= 0) && jarY >= 350) {
                  pidBertahan1(); //pidBertahanA();
                } else if ((xBola >= 0 || xBola360 >= 0) && jarY < 350) {
                  pidBertahan(300); //pidBertahanB(250);
                } else {
                  runKinematik(280, 150, 0, 120);
                }
              }
            }
          }
          else if (yBola >= 420 && 330 <= xBola <= 490 && jarY < 400) {
            if (dapatBola == 'N') {
              sudahMundur = 'N';
              pidBola360();
            }
            else if (jarDribble() > jarbo) {
              dapatBola = 'N';
              //dribble(0);
            }
            else {
              dribble(255);
              tendanganMacan();
            }
          }     
          else {
            parseBola();
            if (countGaDetek > 100) {
              countGaDetek = 0;  
            } else if ((xBola < 0 && xBola360 < 0) && countGaDetek < 10){
              motor(0, 0, 0);
            } else if ((xBola >= 0 || xBola360 >= 0) && jarY >= 400) {
              runKinematik(280, 350, 0, 120);
            } else if ((xBola >= 0 || xBola360 >= 0) && jarY >= 350) {
              pidBertahan1(); //pidBertahanA();
            } else if ((xBola >= 0 || xBola360 >= 0) && jarY < 350) {
              pidBertahan(300); //pidBertahanB(250);
            } else {
              runKinematik(280, 150, 0, 120);
            }
          }
        }
      }
      else if (enableBertahan == 'Y' && enableSerang == 'N' && enableBertahanSerang == 'Y') {
        //enableGoalKick = 'N';
        //sudahTendang = 'N';
        //run Tim kita
        if (team == 'c' && (intruksi == 'N' || intruksi == 'K' || intruksi == 'F' || intruksi == 'G' || intruksi == 'C' || intruksi == 'P')) {
          if (intruksi == 'N') {
            lcd.setCursor(12, 2);
            lcd.print("DropBall");
            bolehMain = 'N';
          }
          if (intruksi == 'K') {
            lcd.setCursor(12, 2);
            lcd.print("KickOff ");
            bolehMain = 'Y';
          }
          if (intruksi == 'F') {
            lcd.setCursor(12, 2);
            lcd.print("FreeKick");
            bolehMain = 'Y';
          }
          if (intruksi == 'G') {
            lcd.setCursor(12, 2);
            lcd.print("GoalKick");
            bolehMain = 'Y';
          }
          if (intruksi == 'C') {
            lcd.setCursor(12, 2);
            lcd.print("Corner  ");
            bolehMain = 'N';
          }
          if (intruksi == 'P') {
            lcd.setCursor(12, 2);
            lcd.print("Penalty ");
            bolehMain = 'Y';
          }
        }
        else if (team == 'm' && (intruksi == 'N' || intruksi == 'k' || intruksi == 'f' || intruksi == 'g' || intruksi == 'c' || intruksi == 'p')) {
          if (intruksi == 'N') {
            lcd.setCursor(12, 2);
            lcd.print("DropBall");
            bolehMain = 'N';
          }
          if (intruksi == 'k') {
            lcd.setCursor(12, 2);
            lcd.print("KickOff ");
            bolehMain = 'Y';
          }
          if (intruksi == 'f') {
            lcd.setCursor(12, 2);
            lcd.print("FreeKick");
            bolehMain = 'Y';
          }
          if (intruksi == 'g') {
            lcd.setCursor(12, 2);
            lcd.print("GoalKick");
            bolehMain = 'Y';
          }
          if (intruksi == 'c') {
            lcd.setCursor(12, 2);
            lcd.print("Corner  ");
            bolehMain = 'N';
          }
          if (intruksi == 'p') {
            lcd.setCursor(12, 2);
            lcd.print("Penalty ");
            bolehMain = 'Y';
          }
        }

        //run Tim Lawan
        else if (team == 'c' && (intruksi == 'k' || intruksi == 'f' || intruksi == 'g' || intruksi == 'c' || intruksi == 'p')) {
          if (intruksi == 'k') {
              lcd.setCursor(12, 2);
              lcd.print("KickOff ");
              bertahanKickOff = 'Y';
              bolehMain = 'N';
          }
          if (intruksi == 'f') {
              lcd.setCursor(12, 2);
              lcd.print("FreeKick");
              bertahanKickOff = 'Y';
              bolehMain = 'N';
          }
          if (intruksi == 'g') {
              lcd.setCursor(12, 2);
              lcd.print("GoalKick");
              bertahanKickOff = 'N';
              bolehMain = 'N';
          }
          if (intruksi == 'c') {
              lcd.setCursor(12, 2);
              lcd.print("Corner  ");
              bertahanKickOff = 'N';
              bolehMain = 'N';
          }
          if (intruksi == 'p') {
              lcd.setCursor(12, 2);
              lcd.print("Penalty ");\
              bertahanKickOff = 'N';
              bolehMain = 'N';
         }
        }
        else if (team == 'm' && (intruksi == 'K' || intruksi == 'F' || intruksi == 'G' || intruksi == 'C' || intruksi == 'P')) {
          if (intruksi == 'K') {
              lcd.setCursor(12, 2);
              lcd.print("KickOff ");
              bertahanKickOff = 'Y';
              bolehMain = 'N';
          }
          if (intruksi == 'F') {
              lcd.setCursor(12, 2);
              lcd.print("FreeKick");
              bertahanKickOff = 'Y';
              bolehMain = 'N';
          }
          if (intruksi == 'G') {
              lcd.setCursor(12, 2);
              lcd.print("GoalKick");
              bertahanKickOff = 'N';
              bolehMain = 'N';
          }
          if (intruksi == 'C') {
              lcd.setCursor(12, 2);
              lcd.print("Corner  ");
              bertahanKickOff = 'N';
              bolehMain = 'N';
          }
          if (intruksi == 'P') {
              lcd.setCursor(12, 2);
              lcd.print("Penalty ");
              bertahanKickOff = 'N';
              bolehMain = 'N';
          }
        }

        if (bolehMain == 'Y') {
          if (sudahTendang == 'Y') {
            if (runBertahan == 'N') {
              detikBertahan = 0;
              runBertahan = 'Y';
            } 
            if (belumMundur == 'N' && detikBertahan < 3) {
              runKinematik(300, 250, 0, 150);
            }  
            if (detikBertahan >= 2) {  
              parseBola();
              if (countGaDetek > 100) {
                countGaDetek = 0;  
              } else if ((xBola < 0 && xBola360 < 0) && countGaDetek < 10){
                motor(0, 0, 0);
              } else if ((xBola >= 0 || xBola360 >= 0) && jarY >= 400) {
                runKinematik(280, 350, 0, 120);
              } else if ((xBola >= 0 || xBola360 >= 0) && jarY >= 350) {
                pidBertahan1();
              } else if ((xBola >= 0 || xBola360 >= 0) && jarY < 350) {
                pidBertahan(300);
              } else {
                runKinematik(280, 150, 0, 120);
              }
            }
          }

          if (pernahStart == 'N' && sudahTendang == 'N') {
            countDetek = 0;
            countGaDetek = 0;
            pernahStart = 'Y';
          }
          if (dapatBola == 'N' && sudahTendang == 'N') {
            sudahMundur = 'N';
            pidBola360A();
          }
          else if (jarDribble() > jarbo && sudahTendang == 'N') {
            dapatBola = 'N';
            //dribble(0);
          }
          else if ((jarDribble() <= jarbo || dapatBola == 'Y') && sudahTendang == 'N') {
            dribble(255);
            runToGoal();
            pernahStart = 'N';
          }
        }
        else if (bolehMain == 'N') {
          parseBola();
          bolehMain = 'N';
          if (bertahanKickOff == 'Y') {
            if (runIntruksi == 'N') {
              detikRun = 0;
              runIntruksi = 'Y';
            }
            lcd.setCursor(12, 2); lcd.print(detikRun); lcd.print("   ");

            if (detikRun <= 6) {
              pidBertahan1();
            }

            if (detikRun > 7) {
              parseBola();
              if (countGaDetek > 100) {
                countGaDetek = 0;  
              } else if ((xBola < 0 && xBola360 < 0) && countGaDetek < 10){
                motor(0, 0, 0);
              } else if ((xBola >= 0 || xBola360 >= 0) && jarY >= 400) {
                runKinematik(280, 350, 0, 120);
              } else if ((xBola >= 0 || xBola360 >= 0) && jarY >= 350) {
                pidBertahan1();
              } else if ((xBola >= 0 || xBola360 >= 0) && jarY < 350) {
                pidBertahan(300);
              } else {
                runKinematik(280, 150, 0, 120);
              }
            }
          }
          else {
            parseBola();
            if (countGaDetek > 100) {
              countGaDetek = 0;  
            } else if ((xBola < 0 && xBola360 < 0) && countGaDetek < 10){
              motor(0, 0, 0);
            } else if ((xBola >= 0 || xBola360 >= 0) && jarY >= 400) {
              runKinematik(280, 350, 0, 120);
            } else if ((xBola >= 0 || xBola360 >= 0) && jarY >= 350) {
              pidBertahan1();
            } else if ((xBola >= 0 || xBola360 >= 0) && jarY < 350) {
              pidBertahan(300);
            } else {
              runKinematik(280, 150, 0, 120);
            }
          }
        }
      }
      else if (enableBertahan == 'N' && enableSerang == 'Y' && enableBertahanSerang == 'Y') {
        //run Tim kita
        if (team == 'c' && (intruksi == 'N' || intruksi == 'K' || intruksi == 'F' || intruksi == 'G' || intruksi == 'C' || intruksi == 'P')) {
          if (runIntruksi == 'N') {
            detikRun = 0;
            runIntruksi = 'Y';
          }
          lcd.setCursor(12, 2); lcd.print(detikRun); lcd.print("   ");
          
          if (bolehMain == 'N' || detikRun <= 1 && (intruksi == 'N' || intruksi == 'K' || intruksi == 'C')) {
            motor(0, 0, 0); 
          }

          if (intruksi == 'F') {
            lcd.setCursor(12, 2);
            lcd.print("FreeKick");
            bolehMain = 'Y';
          }
          if (intruksi == 'G') {
            lcd.setCursor(12, 2);
            lcd.print("GoalKick");
            bolehMain = 'Y';
          }
          if (intruksi == 'P') {
              lcd.setCursor(12, 2);
              lcd.print("Penalty ");
              bolehMain = 'Y';
          }

          if (detikRun > 1) {
            if (intruksi == 'N') {
              lcd.setCursor(12, 2);
              lcd.print("DropBall");
              bolehMain = 'Y';
            }
            if (intruksi == 'K') {
              lcd.setCursor(12, 2);
              lcd.print("KickOff ");
              bolehMain = 'Y';
            }  
          }
          if (detikRun >= 2) {
            if (intruksi == 'C') {
              lcd.setCursor(12, 2);
              lcd.print("Corner  ");
              bolehMain = 'Y';
            } 
          }
        }
        else if (team == 'm' && (intruksi == 'N' || intruksi == 'k' || intruksi == 'f' || intruksi == 'g' || intruksi == 'c' || intruksi == 'p')) {
          if (runIntruksi == 'N') {
            detikRun = 0;
            runIntruksi = 'Y';
          }
          lcd.setCursor(12, 2); lcd.print(detikRun); lcd.print("   ");
          
          if (bolehMain == 'N' || detikRun <= 1 && (intruksi == 'N' || intruksi == 'k' || intruksi == 'c')) {
            motor(0, 0, 0); 
          }

          if (intruksi == 'f') {
            lcd.setCursor(12, 2);
            lcd.print("FreeKick");
            bolehMain = 'Y';
          }
          if (intruksi == 'g') {
            lcd.setCursor(12, 2);
            lcd.print("GoalKick");
            bolehMain = 'Y';
          }
          if (intruksi == 'p') {
              lcd.setCursor(12, 2);
              lcd.print("Penalty ");
              bolehMain = 'Y';
          }

          if (detikRun > 1) {
            if (intruksi == 'N') {
              lcd.setCursor(12, 2);
              lcd.print("DropBall");
              bolehMain = 'Y';
            }
            if (intruksi == 'k') {
              lcd.setCursor(12, 2);
              lcd.print("KickOff ");
              bolehMain = 'Y';
            }
          }
          if (detikRun >= 2) {
            if (intruksi == 'c') {
              lcd.setCursor(12, 2);
              lcd.print("Corner  ");
              bolehMain = 'Y';
            }
          }
        }

        //run Tim Lawan
        else if (team == 'c' && (intruksi == 'k' || intruksi == 'f' || intruksi == 'g' || intruksi == 'c' || intruksi == 'p')) {
          if (runIntruksi == 'N') {
            detikRun = 0;
            runIntruksi = 'Y';
          }
          lcd.setCursor(12, 2); lcd.print(detikRun); lcd.print("   ");

          if (bolehMain == 'N' && detikRun <= 6) {
            pidBertahan1();
          }

          if (detikRun > 6) {
            if (intruksi == 'k') {
              lcd.setCursor(12, 2);
              lcd.print("KickOff ");
              bolehMain = 'Y';
            }
            if (intruksi == 'f') {
              lcd.setCursor(12, 2);
              lcd.print("FreeKick");
              bolehMain = 'Y';
            }
            if (intruksi == 'g') {
              lcd.setCursor(12, 2);
              lcd.print("GoalKick");
              bolehMain = 'Y';
            }
            if (intruksi == 'c') {
              lcd.setCursor(12, 2);
              lcd.print("Corner  ");
              bolehMain = 'Y';
            }
            if (intruksi == 'p') {
              lcd.setCursor(12, 2);
              lcd.print("Penalty ");
              bolehMain = 'Y';
            }
          }
        }
        else if (team == 'm' && (intruksi == 'K' || intruksi == 'F' || intruksi == 'G' || intruksi == 'C' || intruksi == 'P')) {
          if (runIntruksi == 'N') {
            detikRun = 0;
            runIntruksi = 'Y';
          }
          lcd.setCursor(12, 2); lcd.print(detikRun); lcd.print("   ");

          if (bolehMain == 'N' && detikRun <= 6) {
            pidBertahan1();
          }

          if (detikRun > 6) {
            if (intruksi == 'K') {
              lcd.setCursor(12, 2);
              lcd.print("KickOff ");
              bolehMain = 'Y';
            }
            if (intruksi == 'F') {
              lcd.setCursor(12, 2);
              lcd.print("FreeKick");
              bolehMain = 'Y';
            }
            if (intruksi == 'G') {
              lcd.setCursor(12, 2);
              lcd.print("GoalKick");
              bolehMain = 'Y';
            }
            if (intruksi == 'C') {
              lcd.setCursor(12, 2);
              lcd.print("Corner  ");
              bolehMain = 'Y';
            }
            if (intruksi == 'P') {
              lcd.setCursor(12, 2);
              lcd.print("Penalty ");
              bolehMain = 'Y';
            }
          }
        }

        if (bolehMain == 'Y') {
          if (pernahStart == 'N') {
            countDetek = 0;
            countGaDetek = 0;
            pernahStart = 'Y';
          }

          if (dapatBola == 'N') {
            sudahMundur = 'N';
            pidBola360();
          }
          else if (jarDribble() > jarbo) {
            dapatBola = 'N';
            //dribble(0);
          }
          else {
            dribble(255);
            runToGoal();
            pernahStart = 'N';
          }
        }
      }
      else if (enableBertahan == 'N' && enableSerang == 'Y' && enableBertahanSerang == 'N') {
        //run Tim kita
        if (team == 'c' && (intruksi == 'N' || intruksi == 'K' || intruksi == 'F' || intruksi == 'G' || intruksi == 'C' || intruksi == 'P')) {
          if (intruksi == 'N') {
            lcd.setCursor(12, 2);
            lcd.print("DropBall");
            bolehMain = 'Y';
          }
          if (intruksi == 'K') {
            lcd.setCursor(12, 2);
            lcd.print("KickOff ");
            bolehMain = 'Y';
          }
          if (intruksi == 'F') {
            lcd.setCursor(12, 2);
            lcd.print("FreeKick");
            bolehMain = 'Y';
          }
          if (intruksi == 'G') {
            lcd.setCursor(12, 2);
            lcd.print("GoalKick");
            bolehMain = 'Y';
          }
          if (intruksi == 'C') {
            lcd.setCursor(12, 2);
            lcd.print("Corner  ");
            bolehMain = 'Y';
          }
          if (intruksi == 'P') {
            lcd.setCursor(12, 2);
            lcd.print("Penalty ");
            bolehMain = 'Y';
          }
        }
        else if (team == 'm' && (intruksi == 'N' || intruksi == 'k' || intruksi == 'f' || intruksi == 'g' || intruksi == 'c' || intruksi == 'p')) {
          if (intruksi == 'N') {
            lcd.setCursor(12, 2);
            lcd.print("DropBall");
            bolehMain = 'Y';
          }
          if (intruksi == 'k') {
            lcd.setCursor(12, 2);
            lcd.print("KickOff ");
            bolehMain = 'Y';
          }
          if (intruksi == 'f') {
            lcd.setCursor(12, 2);
            lcd.print("FreeKick");
            bolehMain = 'Y';
          }
          if (intruksi == 'g') {
            lcd.setCursor(12, 2);
            lcd.print("GoalKick");
            bolehMain = 'Y';
          }
          if (intruksi == 'c') {
            lcd.setCursor(12, 2);
            lcd.print("Corner  ");
            bolehMain = 'Y';
          }
          if (intruksi == 'p') {
            lcd.setCursor(12, 2);
            lcd.print("Penalty ");
            bolehMain = 'Y';
          }
        }

        //run Tim Lawan
        else if (team == 'c' && (intruksi == 'k' || intruksi == 'f' || intruksi == 'g' || intruksi == 'c' || intruksi == 'p')) {
          if (runIntruksi == 'N') {
            detikRun = 0;
            runIntruksi = 'Y';
          }
          lcd.setCursor(12, 2); lcd.print(detikRun); lcd.print("   ");

          if (bolehMain == 'N' && detikRun <= 6) {
            pidBertahan(300);
          }

          if (detikRun > 6) {
            if (intruksi == 'k') {
              lcd.setCursor(12, 2);
              lcd.print("KickOff ");
              bolehMain = 'Y';
            }
            if (intruksi == 'f') {
              lcd.setCursor(12, 2);
              lcd.print("FreeKick");
              bolehMain = 'Y';
            }
            if (intruksi == 'g') {
              lcd.setCursor(12, 2);
              lcd.print("GoalKick");
              bolehMain = 'Y';
            }
            if (intruksi == 'c') {
              lcd.setCursor(12, 2);
              lcd.print("Corner  ");
              bolehMain = 'Y';
            }
            if (intruksi == 'p') {
              lcd.setCursor(12, 2);
              lcd.print("Penalty ");
              bolehMain = 'Y';
            }
          }
        }
        else if (team == 'm' && (intruksi == 'K' || intruksi == 'F' || intruksi == 'G' || intruksi == 'C' || intruksi == 'P')) {
          if (runIntruksi == 'N') {
            detikRun = 0;
            runIntruksi = 'Y';
          }
          lcd.setCursor(12, 2); lcd.print(detikRun); lcd.print("   ");

          if (bolehMain == 'N' && detikRun <= 6) {
            pidBertahan(300);
          }

          if (detikRun > 6) {
            if (intruksi == 'K') {
              lcd.setCursor(12, 2);
              lcd.print("KickOff ");
              bolehMain = 'Y';
            }
            if (intruksi == 'F') {
              lcd.setCursor(12, 2);
              lcd.print("FreeKick");
              bolehMain = 'Y';
            }
            if (intruksi == 'G') {
              lcd.setCursor(12, 2);
              lcd.print("GoalKick");
              bolehMain = 'Y';
            }
            if (intruksi == 'C') {
              lcd.setCursor(12, 2);
              lcd.print("Corner  ");
              bolehMain = 'Y';
            }
            if (intruksi == 'P') {
              lcd.setCursor(12, 2);
              lcd.print("Penalty ");
              bolehMain = 'Y';
            }
          }
        }

        if (bolehMain == 'Y') {
          if (pernahStart == 'N') {
            countDetek = 0;
            countGaDetek = 0;
            pernahStart = 'Y';
          }

          if (dapatBola == 'N') {
            sudahMundur = 'N';
            pidBola360();
          }
          else if (jarDribble() > jarbo) {
            dapatBola = 'N';
            //dribble(0);
          }
          else {
            dribble(255);
            runToGoal();
            pernahStart = 'N';
          }
        }
      }
    }

    count++;
    if (count > 10) {
      //      Serial2.print(String("#?,")+(int)jarX+String(",")+(int)jarY+String(",")+(int)Theta+String(",\n"));
      Serial2.print(String("#?,") + (int)jarX + String(",") + (int)jarY + String(",") + (int)Theta + String(",") + (int)xBola + String(",") + (int)yBola  /*(int)lihatBola*/ + String(",\n"));
      Serial2.print(String("#!,") + (int)xTarget + String(",") + (int)yTarget + String(",") + (int)tTarget + String(",\n"));
      count = 0;
    }
  }

  delay(300); lcd.clear();
}

void runToGoal() {
  int acuanX = 400, acuanY = 800, acuanB = 350;

  if (enableKickOff == 'Y') {

    //    dribble(100);
    //    motor(0, 0, 0);
    //    delay(100);
    dribble(-120);
    delay(250);
    
    // dribble(-120);
    // delay(120);
    // if (strategiKick % 2 == 0) {runXYPwm(-130, -20, -20, 200); delay(1000); motor(0, 0, 0);}
    // else {runXYPwm(130, 20, 20, 200); delay(1000); motor(0, 0, 0);}

    // strategiKick++;
    enableKickOff = 'N';
    enableInisial = 'N';
  }
  else if (enableGoalKick == 'Y') {
    dribble(200); motor(0, 0, 0); delay(500); 
    
    dribble(-100);
    delay(120);

    enableGoalKick = 'N';
    enableInisial = 'N';
  }
  /*else if (enableGoalKick == 'Y') {
    dribble(200); 
    motor(0, 0, 0); delay(150);
    if (jarX < 300) { 
      motor(-70, -70, -70); delay(850);
      motor(0, 0, 0);
      dribble(-100);
      delay(250);
    }
    else {
      motor(70, 70, 70); delay(850);
      motor(0, 0, 0);
      dribble(-100);
      delay(250);      
    }
    
    enableGoalKick = 'N';
    enableInisial = 'N';
  }*/
  else if (enablePenalty == 'Y') {
    dribble(150); motor(0, 0, 0); delay(1000);
    if (strategi == 1) {
      motor(30, 30, 30);
      delay(150);
      motor(0, 0, 0);
      tendanganMacan();
    }
    else {
      motor(-30, -30, -30);
      delay(150);
      motor(0, 0, 0);
      tendanganMacan();
    }
    enablePenalty = 'N';
    enableInisial = 'N';
  }
  else if (strategi == 1 && jarY > 870) {
    runKinematik(acuanX, acuanY, 0, 110);

    if (jarX > acuanX - 60 && jarX < acuanX + 60 && jarY < 830) {
      tendanganMacan();
    }
  }
  else if (strategi == 1) {
    tendanganMacan();
  }
  else if (strategi == 2 && jarY < 350) {
    tendanganMacan(); 
  }
  else if (strategi == 2 && jarY > 550) {
    tendanganMacan(); 
  }
  else if (strategi == 2 && jarY > 870) {
    runKinematik(acuanX, acuanY, 0, 110);

    if (jarX > acuanX - 60 && jarX < acuanX + 60 && jarY < 830) {
      tendanganMacan();
    }
  }
  else if (strategi == 2) {
    if (jarX >= 300) {
      runKinematik(450, 450, -150, 130);
      if ((jarX > 420 && jarX < 480) && (jarY > 420 && jarY < 480)) {
        tendanganMacan();
      }
    }
    else {
      runKinematik(150, 450, 150, 130);
      if ((jarX > 120 && jarX < 180) && (jarY > 420 && jarY < 480)) {
        tendanganMacan();
      }
    }
  }
  else if (strategi == 3) {
    tendangUmpan1();
  }
  else {
    runKinematik(acuanX, acuanY, 180, 160);

    if (jarX > acuanX - 60 && jarX < acuanX + 60 && jarY > 600 && jarY < 900) {
      tendanganMacan();
    }
  }
}

void tendanganMacan() {
  float x, y, r, target, sudutTuju, eT;

  if(jarX>400){
    target = 400-10;
  }else {
    target = 400+10;
  }

  x = (target - jarX) / 100;
  y = (1200 - jarY) / 100;
  
//  x = (target - jarX) / 100;
//  y = (900 - jarY) / 100;
    
  r = sqrt((x * x) + (y * y));
  sudutTuju;

  if (x > 0) {
    sudutTuju = 90 - (asin(y / r) * 180 / PI);
  }
  else {
    sudutTuju = (asin(y / r) * 180 / PI) - 90;
  }

  eT = Theta - sudutTuju;

  int spd = eT * 1.3;
  if (spd > 90) {
    spd = 90;
  } if (spd < -90) {
    spd = -90;
  }
  if (eT > 5 || eT < -5) {
    dribble(255);
    motor(spd, spd, spd);
  }
  //    if(Theta>sudutTuju+10){
  //      dribble(255);
  //      motor(55,55,55);
  //    }
  //    else if(Theta<sudutTuju-7){
  //      dribble(255);
  //      motor(-55,-55,-55);
  //    }
  else if (irSw() && jarY < 900) {
    dribble(255);
    if (jarX <= 250) {
      runXYPwm(130, 30, 10, 200); delay(1000);
    }
    else if (jarX <= 350 && jarX >= 300) {
      runXYPwm(130, 30, 10, 200); delay(1000);
    }
    else if (jarX >= 250 && jarX <= 300) {
      runXYPwm(-130, 30, -10, 200); delay(1000);
    }
    else if (jarX >= 350) {
      runXYPwm(-130, 30, -10, 200); delay(1000);
    }
  }
  else {
    motor(0, 0, 0);
    //      if(Theta<0 && Theta>-20){
    //        motor(40,40,40); delay(100); motor(0,0,0);
    //      }
    //      else if(Theta>0 && Theta<20){
    //        motor(-40,-40,-40); delay(100); motor(0,0,0);
    //      }
    dribble(150);
    tendang();
    sudahTendang ='Y';
  }
}

void tendangUmpan1() {
  float x, y, r, target, sudutTuju;

  if(jarX>300){
    target = 300-10;
  }else {
    target = 300+10;
  }
  
  x = (target - jarX) / 100;
  y = (1200 - jarY) / 100;
    
  r = sqrt((x * x) + (y * y));
  sudutTuju;

  if (x > 0) {
    sudutTuju = 25 - (asin(y / r) * 180 / PI);
  }
  else {
    sudutTuju = (-(asin(y / r) * 180 / PI)) + 35;
  }

  int errorT = Theta - sudutTuju;

  int spd = errorT * 2.8;
  if (spd > 90) {
    spd = 90;
  } if (spd < -90) {
    spd = -90;
  }
  if (errorT > 5 || errorT < -5) {
    dribble(255);
    motor(spd, spd, spd);
  }

  //    if(Theta>sudutTuju+10){
  //      dribble(255);
  //      motor(55,55,55);
  //    }
  //    else if(Theta<sudutTuju-7){
  //      dribble(255);
  //      motor(-55,-55,-55);
  //    }
  else if (irSw() && jarY < 900) {
    dribble(255);
    runKinematik(400, 400, 0, 200);
    tendanganMacan();
//    if (jarX <= 350) {
//      runXYPwm(130, 10, 0, 200); delay(1200);
//    }
//    else if (jarX <= 450 && jarX >= 400) {
//      runXYPwm(130, 10, 0, 200); delay(1200);
//    }
//    else if (jarX >= 350 && jarX <= 400) {
//      runXYPwm(-130, -10, 0, 200); delay(1200);
//    }
//    else if (jarX >= 450) {
//      runXYPwm(-130, -10, 0, 200); delay(1200);
//    }

    //      if(jarX<=300){
    //        runXYPwm(130,30,10,200); delay(1000);
    //      }
    //      else{
    //        runXYPwm(-130,30,-10,200); delay(1000);
    //      }
  }
  else {
    motor(0, 0, 0);
    //      if(Theta<0 && Theta>-20){
    //        motor(40,40,40); delay(100); motor(0,0,0);
    //      }
    //      else if(Theta>0 && Theta<20){
    //        motor(-40,-40,-40); delay(100); motor(0,0,0);
    //      }
    dribble(120);
    tendang3();
    sudahTendang = 'Y';
  }
  //  }
}

void tendangUmpan2() {
  float x, y, r, target, sudutTuju;

  if(jarX>300){
    target = 300-10;
  }else {
    target = 300+10;
  }
  
  x = (target - jarX) / 100;
  y = (1200 - jarY) / 100;
    
  r = sqrt((x * x) + (y * y));
  sudutTuju;

  if (x > 0) {
    sudutTuju = 145 - (asin(y / r) * 180 / PI);
  }
  else {
    sudutTuju = (-(asin(y / r) * 180 / PI)) + 155;
  }

  int errorT = Theta - sudutTuju;

  int spd = errorT * 2.8;
    if (spd > 90) {
    spd = 90;
  } if (spd < -90) {
    spd = -90;
  }
  if (errorT > 5 || errorT < -5) {
    dribble(255);
    motor(spd, spd, spd);
  }

  //    if(Theta>sudutTuju+10){
  //      dribble(255);
  //      motor(55,55,55);
  //    }
  //    else if(Theta<sudutTuju-7){
  //      dribble(255);
  //      motor(-55,-55,-55);
  //    }
  else if (irSw() && jarY < 900) {
    dribble(255);
    runKinematik(200, 400, 0, 200);
    tendanganMacan();
//    if (jarX <= 350) {
//      runXYPwm(130, 10, 0, 200); delay(1200);
//    }
//    else if (jarX <= 450 && jarX >= 400) {
//      runXYPwm(130, 10, 0, 200); delay(1200);
//    }
//    else if (jarX >= 350 && jarX <= 400) {
//      runXYPwm(-130, -10, 0, 200); delay(1200);
//    }
//    else if (jarX >= 450) {
//      runXYPwm(-130, -10, 0, 200); delay(1200);
//    }

    //      if(jarX<=300){
    //        runXYPwm(130,30,10,200); delay(1000);
    //      }
    //      else{
    //        runXYPwm(-130,30,-10,200); delay(1000);
    //      }
  }
  else {
    motor(0, 0, 0);
    //      if(Theta<0 && Theta>-20){
    //        motor(40,40,40); delay(100); motor(0,0,0);
    //      }
    //      else if(Theta>0 && Theta<20){
    //        motor(-40,-40,-40); delay(100); motor(0,0,0);
    //      }
    dribble(120);
    tendang3();
    sudahTendang = 'Y';
  }
  //  }
}

void listenCoach() {
  if (Serial2.available() > 0) {
    char data = Serial2.read();

    if (data == 'S') {
      intruksiPlay = 'S';
      intruksiFull = 'S';
      intruksiR = 'S';
      lcd.setCursor(10, 3);
      lcd.print("Stop      ");
      enableMain = 'N';
      enableInisial = 'Y';
    }
    else if (data == 's') {
      intruksiPlay = 's';
      intruksiFull = 'S';
      intruksiR = 'S';
      lcd.setCursor(10, 3);
      lcd.print("Start     ");
      enableMain = 'N';
      enableInisial = 'Y';
    }
    else if (data == 'R') {
      intruksiPlay = 'R';
      intruksiFull = 'S';
      intruksiR = 'S';
      lcd.setCursor(10, 3);
      lcd.print("Reset Posisi");
      enableMain = 'N';
      enableInisial = 'Y';
      titikTuju = 0;
    }
    else if (data == 'r') {
      intruksiPlay = 'r';
      intruksiR = 'S';
      lcd.setCursor(10, 3);
      lcd.print("Krksi Bola");
      enableMain = 'N';
      enableInisial = 'Y';
    }

    else if (data == 'N') {
      intruksi = 'N';
      intruksiFull = 'N';
      intruksiR = 'S';
      lcd.setCursor(10, 3);
      lcd.print("DropBall  ");
      enableMain = 'Y';
      enableInisial = 'Y';
    }

    else if (data == 'K') {
      intruksi = 'K';
      intruksiFull = 'K';
      intruksiR = 'S';
      lcd.setCursor(10, 3);
      lcd.print("KickOff C ");
      enableMain = 'Y';
      enableInisial = 'Y';
    }
    else if (data == 'F') {
      intruksi = 'F';
      intruksiFull = 'F';
      intruksiR = 'S';
      lcd.setCursor(10, 3);
      lcd.print("FreeKick C");
      enableMain = 'Y';
      enableInisial = 'Y';
    }
    else if (data == 'G') {
      intruksi = 'G';
      intruksiFull = 'G';
      intruksiR = 'S';
      lcd.setCursor(10, 3);
      lcd.print("GoalKick C");
      enableMain = 'Y';
      enableInisial = 'Y';
    }
    else if (data == 'C') {
      intruksi = 'C';
      intruksiFull = 'C';
      intruksiR = 'S';
      lcd.setCursor(10, 3);
      lcd.print("Corner C  ");
      enableMain = 'Y';
      enableInisial = 'Y';
    }
    else if (data == 'P') {
      intruksi = 'P';
      intruksiFull = 'P';
      intruksiR = 'S';
      lcd.setCursor(10, 3);
      lcd.print("Penalty C ");
      enableMain = 'Y';
      enableInisial = 'Y';
    }

    else if (data == 'k') {
      intruksi = 'k';
      intruksiFull = 'k';
      intruksiR = 'S';
      lcd.setCursor(10, 3);
      lcd.print("KickOff M ");
      enableMain = 'Y';
      enableInisial = 'Y';
    }
    else if (data == 'f') {
      intruksi = 'f';
      intruksiFull = 'f';
      intruksiR = 'S';
      lcd.setCursor(10, 3);
      lcd.print("FreeKick M");
      enableMain = 'Y';
      enableInisial = 'Y';
    }
    else if (data == 'g') {
      intruksi = 'g';
      intruksiFull = 'g';
      intruksiR = 'S';
      lcd.setCursor(10, 3);
      lcd.print("GoalKick M");
      enableMain = 'Y';
      enableInisial = 'Y';
    }
    else if (data == 'c') {
      intruksi = 'c';
      intruksiFull = 'c';
      intruksiR = 'S';
      lcd.setCursor(10, 3);
      lcd.print("Corner M  ");
      enableMain = 'Y';
      enableInisial = 'Y';
    }
    else if (data == 'p') {
      intruksi = 'p';
      intruksiFull = 'p';
      intruksiR = 'S';
      lcd.setCursor(10, 3);
      lcd.print("Penalty M ");
      enableMain = 'Y';
      enableInisial = 'Y';
    }

    else if (data == 'q') {
      intruksiR = 'L';
      intruksiFull = 'S';
      lcd.setCursor(10, 3);
      lcd.print("Rotasi L  ");
      enableMain = 'N';
    }
    else if (data == 'e') {
      intruksiR = 'R';
      intruksiFull = 'S';
      lcd.setCursor(10, 3);
      lcd.print("Rotasi R  ");
      enableMain = 'N';
    }

    // aktifin ini untuk terima data Roso

    else if (data == '?') {
      data = Serial2.read();
      if (data == 'x') {
        int dataX = Serial2.parseInt();
        if (Serial2.read() == 'y') {
          int dataY = Serial2.parseInt();
          if (Serial2.read() == '#') {
            jarX = (int)dataX;
            jarY = (int)dataY;
            jarL = jarR = jarB = 0;
          }
        }
      }
      else if (data == 'w') {
        int dataW = Serial2.parseInt();
        if (Serial2.read() == '#') {
          Serial3.println(dataW + String("#"));
        }
      }
      enableInisial = 'N';
    }
    else if (data == '!') {
      data = Serial2.read();
      if (data == 'x') {
        int dataX = Serial2.parseInt();
        if (Serial2.read() == 'y') {
          int dataY = Serial2.parseInt();
          if (Serial2.read() == '#') {
            xTarget = (int)dataX;
            yTarget = (int)dataY;
            titikTuju = 0;
          }
        }
      }
      else if (data == 'w') {
        int dataW = Serial2.parseInt();
        if (Serial2.read() == '#') {
          tTarget = dataW;
          titikTuju = 0;
        }
      }
      enableInisial = 'N';
    }
    else if (data == '@') {
      int dataIn = Serial2.parseInt();
      if (Serial2.read() == '#') {
        if (dataIn < 4) {
          strategi = dataIn;
        }
        if (dataIn == 4) {
          enableBertahan = 'Y'; enableSerang = 'Y'; enableBertahanSerang = 'N';
        }
        if (dataIn == 5) {
          enableBertahan = 'Y'; enableSerang = 'N'; enableBertahanSerang = 'N'; 
        }
        if (dataIn == 6) {
          enableBertahan = 'Y'; enableSerang = 'Y'; enableBertahanSerang = 'Y';
        }
        if (dataIn == 7) {
          enableBertahan = 'Y'; enableSerang = 'N'; enableBertahanSerang = 'Y';
        }
        if (dataIn == 8) {
          enableBertahan = 'N'; enableSerang = 'Y'; enableBertahanSerang = 'Y';
        }
        if (dataIn == 9) {
          enableBertahan = 'N'; enableSerang = 'Y'; enableBertahanSerang = 'N';
        }
      }
      enableInisial = 'N';
    }
  }
}

//////////////////////////////////Trash
/*
   void listenCoach() {
  //  Serial5.listen();
  //  if(Serial5.available()>0){
  //    char data = Serial5.read();

  if (Serial2.available() > 0) {
    char data = Serial2.read();

    if (data == 'S') {
      intruksiPlay = 'S';
      intruksiFull = 'S';
      intruksiR = 'S';
      lcd.setCursor(10, 3);
      lcd.print("Stop      ");
      enableMain = 'N';
      enableInisial = 'Y';
    }
    else if (data == 's') {
      intruksiPlay = 's';
      intruksiFull = 'S';
      intruksiR = 'S';
      lcd.setCursor(10, 3);
      lcd.print("Start     ");
      enableMain = 'N';
      enableInisial = 'Y';
    }
    else if (data == 'R') {
      intruksiPlay = 'R';
      intruksiFull = 'S';
      intruksiR = 'S';
      lcd.setCursor(10, 3);
      lcd.print("Reset Posisi");
      enableMain = 'N';
      enableInisial = 'Y';
    }

    else if (data == 'N') {
      intruksi = 'N';
      intruksiFull = 'N';
      intruksiR = 'S';
      lcd.setCursor(10, 3);
      lcd.print("DropBall  ");
      enableMain = 'Y';
      enableInisial = 'Y';
    }

    else if (data == 'K') {
      intruksi = 'K';
      intruksiFull = 'K';
      intruksiR = 'S';
      lcd.setCursor(10, 3);
      lcd.print("KickOff C ");
      enableMain = 'Y';
      enableInisial = 'Y';
    }
    else if (data == 'F') {
      intruksi = 'F';
      intruksiFull = 'F';
      intruksiR = 'S';
      lcd.setCursor(10, 3);
      lcd.print("FreeKick C");
      enableMain = 'Y';
      enableInisial = 'Y';
    }
    else if (data == 'G') {
      intruksi = 'G';
      intruksiFull = 'G';
      intruksiR = 'S';
      lcd.setCursor(10, 3);
      lcd.print("GoalKick C");
      enableMain = 'Y';
      enableInisial = 'Y';
    }
    else if (data == 'C') {
      intruksi = 'C';
      intruksiFull = 'C';
      intruksiR = 'S';
      lcd.setCursor(10, 3);
      lcd.print("Corner C  ");
      enableMain = 'Y';
      enableInisial = 'Y';
    }
    else if (data == 'P') {
      intruksi = 'P';
      intruksiFull = 'P';
      intruksiR = 'S';
      lcd.setCursor(10, 3);
      lcd.print("Penalty C ");
      enableMain = 'Y';
      enableInisial = 'Y';
    }

    else if (data == 'k') {
      intruksi = 'k';
      intruksiFull = 'k';
      intruksiR = 'S';
      lcd.setCursor(10, 3);
      lcd.print("KickOff M ");
      enableMain = 'Y';
      enableInisial = 'Y';
    }
    else if (data == 'f') {
      intruksi = 'f';
      intruksiFull = 'f';
      intruksiR = 'S';
      lcd.setCursor(10, 3);
      lcd.print("FreeKick M");
      enableMain = 'Y';
      enableInisial = 'Y';
    }
    else if (data == 'g') {
      intruksi = 'g';
      intruksiFull = 'g';
      intruksiR = 'S';
      lcd.setCursor(10, 3);
      lcd.print("GoalKick M");
      enableMain = 'Y';
      enableInisial = 'Y';
    }
    else if (data == 'c') {
      intruksi = 'c';
      intruksiFull = 'c';
      intruksiR = 'S';
      lcd.setCursor(10, 3);
      lcd.print("Corner M  ");
      enableMain = 'Y';
      enableInisial = 'Y';
    }
    else if (data == 'p') {
      intruksi = 'p';
      intruksiFull = 'p';
      intruksiR = 'S';
      lcd.setCursor(10, 3);
      lcd.print("Penalty M ");
      enableMain = 'Y';
      enableInisial = 'Y';
    }
    else if (data == 'Q') {
      intruksiR = 'L';
      intruksiFull = 'S';
      lcd.setCursor(10, 3);
      lcd.print("Rotasi L  ");
      enableMain = 'N';
    }
    else if (data == 'E') {
      intruksiR = 'R';
      intruksiFull = 'S';
      lcd.setCursor(10, 3);
      lcd.print("Rotasi R  ");
      enableMain = 'N';
    }

    // aktifin ini untuk terima data Roso

    else if (data == '?') {
      //aktifin ini untuk terima data rosi posisi x dan y sekarang
      //      data = Serial5.read();
      //      if(data=='X'){
      //        int dataX = Serial5.parseInt();
      //        if(Serial5.read()=='Y'){
      //          int dataY = Serial5.parseInt();
      //          if(Serial5.read()=='#'){
      data = Serial2.read();
      if (data == 'x') {
        int dataX = Serial2.parseInt();
        if (Serial2.read() == 'y') {
          int dataY = Serial2.parseInt();
          if (Serial2.read() == '#') {
            jarX = (int)dataX;
            jarY = (int)dataY;
            jarL = jarR = jarB = 0;
          }
        }
      }
      else if (data == 'w') {
        //        int dataW = Serial5.parseInt();
        //        if(Serial5.read()=='#'){
        int dataW = Serial2.parseInt();
        if (Serial2.read() == '#') {
          Serial3.println(dataW + String("#"));
        }
      }
      enableInisial = 'N';
    }
    else if (data == '!') {
      //      data = Serial5.read();
      //      if(data=='X'){
      //        int dataX = Serial5.parseInt();
      //        if(Serial5.read()=='Y'){
      //          int dataY = Serial5.parseInt();
      //          if(Serial5.read()=='#'){
      data = Serial2.read();
      if (data == 'x') {
        int dataX = Serial2.parseInt();
        if (Serial2.read() == 'y') {
          int dataY = Serial2.parseInt();
          if (Serial2.read() == '#') {
            xTarget = (int)dataX;
            yTarget = (int)dataY;
            titikTuju = 0;
          }
        }
      }
      else if (data == 'w') {
        //        int dataW = Serial5.parseInt();
        //        if(Serial5.read()=='#'){
        int dataW = Serial2.parseInt();
        if (Serial2.read() == '#') {
          tTarget = dataW;
          titikTuju = 0;
        }
      }
      enableInisial = 'N';
    }
    else if (data == '@') {
      //      int dataIn = Serial5.parseInt();
      //      if(Serial5.read()=='#'){
      int dataIn = Serial2.parseInt();
      if (Serial2.read() == '#') {
        strategi = dataIn;
        //        strategiKick=dataIn;
      }
      enableInisial = 'N';
    }
  //    if (data == '^') {
  //      data = Serial2.read();
  //      if (data == 'X') {
  //        int dataX = Serial2.parseInt();
  //        if (Serial2.read() == 'Y') {
  //          int dataY = Serial2.parseInt();
  //          if (Serial2.read() == '#') {
  //            jarXRosi = (int)dataX;
  //            jarYRosi = (int)dataY;
  //          }
  //        }
  //      }
  //    }
  //
  }
  }

   void runManualPlay() {
  int maxCount, timeDetekBola = 0, countDetekBola = 0;
  int acuanX = 300, acuanB = 350;
  int count;
  char pernahStart = 'N';

  lcd.clear();
  lcd.setCursor(0, 0);
  if (strategi == 0) {
    lcd.print("Serang Play");
    maxCount = timeSerang;
  }
  if (strategi == 1) {
    lcd.print("Tengah Play ");
    maxCount = timeTengah;
  }
  if (strategi == 2) {
    lcd.print("Blkang Play");
    maxCount = timeBlkang;
  }
  countManual = 0;
  errorIL = 0;
  errorIR = 0;
  errorIB = 0;
  countDetek = 0;
  countGaDetek = 0;
  while (countManual < maxCount) {
    lcd.setCursor(12, 0); lcd.print(countManual); lcd.print("  ");
    lcd.setCursor(0, 1); lcd.print("x"); lcd.print(xBola); lcd.print(" y"); lcd.print(yBola); lcd.print(" ");
    lcd.print(countDetekBola); lcd.print("  ");
    parseBola();
    if (xBola >= 0 && yBola > 50) {
      countDetekBola++;
    }
    //    else{countDetekBola=0;}

    if (countDetekBola > 10) {
      break;
    }

    if (countManual > 6) {
      motor(-200, 200, 0);
    }
    else if (countManual > 3) {
      motor(-180, 180, 0);
    }
    else {
      motor(-110, 110, 0);
    }

  }
  motor(0, 0, 0);
  errorIL = 0;
  errorIR = 0;
  errorIB = 0;
  countDetek = 0;
  countGaDetek = 0;
  while (!C()) {
    lcd.setCursor(0, 2); lcd.print("Scan Bola");

    if (pernahStart == 'N') {
      countDetek = 0;
      countGaDetek = 0;
      pernahStart = 'Y';
    }

    if (dapatBola == 'N') {
      sudahMundur = 'N';
      pidBola();
    }
    else if (jarDribble() > jarbo) {
      dapatBola = 'N';
      dribble(0);
    }
    else {
      keTitikLapangan180(acuanX, acuanB); pernahStart = 'N';
      if (tengahY == 1) {
        tendangJarakDekat();
      }
    }

  }
  delay(300); lcd.clear(); motor(0, 0, 0);
  }

  void runAutoPlay() {
  delay(300); lcd.clear();
  int count;
  char pernahStart = 'N';

  if (team == 'C') {
    lcd.setCursor(0, 0);
    lcd.print("AUTO PLAY C");
  }
  if (team == 'M') {
    lcd.setCursor(0, 0);
    lcd.print("AUTO PLAY M");
  }
  lcd.print(" S:"); lcd.print(strategi); lcd.print(" ");

  xTarget = 310; yTarget = 300; tTarget = 0;
  //  xTarget = 250; yTarget = 200; tTarget=0;

  while (!C()) {
    parseBola();

    if (team == 'C') {
      lcd.setCursor(0, 0);
      lcd.print("AUTO PLAY C");
    }
    else if (team == 'M') {
      lcd.setCursor(0, 0);
      lcd.print("AUTO PLAY M");
    }
    lcd.print(" Strgi:"); lcd.print(strategi); lcd.print(" ");

    lcd.setCursor(0, 1);
    lcd.print("X"); lcd.print((int)jarX);
    lcd.print(" Y"); lcd.print((int)jarY);
    lcd.print(" T"); lcd.print((int)Theta); lcd.print("  ");

    if (bolehMain == 'N') {
      lcd.setCursor(0, 2);
      lcd.print("X"); lcd.print(xBola);
      lcd.print(" Y"); lcd.print(yBola); lcd.print("  ");

      lcd.setCursor(0, 3);
      lcd.print("Play Now: ");
    }

    listenCoach();

    if (intruksiR == 'R') {
      motor(-30, -30, -30);
    }
    else if (intruksiR == 'L') {
      motor(30, 30, 30);
    }
    //    if(intruksiR=='R'){runXYPwm(80, 30, 30, 130);}
    //    else if(intruksiR=='L'){runXYPwm(-80, -30, -30, 130);}

    else if (intruksiPlay == 'S') {
      lcd.setCursor(12, 2); lcd.print("       ");
      motor(0, 0, 0);
      errorIL = 0;
      errorIR = 0;
      errorIB = 0;
      dribble(0);
      titikTuju = 0;
      iX = iY = iT = 0;
      enableReadThetaNow = 1;
      runIntruksi = 'N';
      bolehMain = 'N';
      countDetek = 0;
      countGaDetek = 0;
    }

    if (intruksiPlay == 's') {

      //run Tim kita
      if (team == 'C' && (intruksi == 'N' || intruksi == 'K' || intruksi == 'F' || intruksi == 'G' || intruksi == 'C')) {
        if (intruksi == 'N') {
          lcd.setCursor(12, 2);
          lcd.print("DropBall");
          bolehMain = 'Y';
        }
        if (intruksi == 'K') {
          lcd.setCursor(12, 2);
          lcd.print("KickOff ");
          bolehMain = 'Y';
        }
        if (intruksi == 'F') {
          lcd.setCursor(12, 2);
          lcd.print("FreeKick");
          bolehMain = 'Y';
        }
        if (intruksi == 'G') {
          lcd.setCursor(12, 2);
          lcd.print("GoalKick");
          bolehMain = 'Y';
        }
        if (intruksi == 'C') {
          lcd.setCursor(12, 2);
          lcd.print("Corner  ");
          bolehMain = 'Y';
        }
        if (intruksi == 'P') {
          lcd.setCursor(12, 2);
          lcd.print("Penalty ");
          bolehMain = 'Y';
        }
      }
      else if (team == 'M' && (intruksi == 'N' || intruksi == 'k' || intruksi == 'f' || intruksi == 'g' || intruksi == 'c')) {
        if (intruksi == 'N') {
          lcd.setCursor(12, 2);
          lcd.print("DropBall");
          bolehMain = 'Y';
        }
        if (intruksi == 'k') {
          lcd.setCursor(12, 2);
          lcd.print("KickOff ");
          bolehMain = 'Y';
        }
        if (intruksi == 'f') {
          lcd.setCursor(12, 2);
          lcd.print("FreeKick");
          bolehMain = 'Y';
        }
        if (intruksi == 'g') {
          lcd.setCursor(12, 2);
          lcd.print("GoalKick");
          bolehMain = 'Y';
        }
        if (intruksi == 'c') {
          lcd.setCursor(12, 2);
          lcd.print("Corner  ");
          bolehMain = 'Y';
        }
        if (intruksi == 'p') {
          lcd.setCursor(12, 2);
          lcd.print("Penalty ");
          bolehMain = 'Y';
        }
      }

      //run Tim Lawan
      else if (team == 'C' && (intruksi == 'k' || intruksi == 'f' || intruksi == 'g' || intruksi == 'c')) {
        if (runIntruksi == 'N') {
          detikRun = 0;
          runIntruksi = 'Y';
        }
        lcd.setCursor(12, 2); lcd.print(detikRun); lcd.print("   ");

        if (bolehMain == 'N' && detikRun <= 6) {
          pidBolaJaga();
        }

        if (detikRun > 6) {
          if (intruksi == 'k') {
            lcd.setCursor(12, 2);
            lcd.print("KickOff ");
            bolehMain = 'Y';
          }
          if (intruksi == 'f') {
            lcd.setCursor(12, 2);
            lcd.print("FreeKick");
            bolehMain = 'Y';
          }
          if (intruksi == 'g') {
            lcd.setCursor(12, 2);
            lcd.print("GoalKick");
            bolehMain = 'Y';
          }
          if (intruksi == 'c') {
            lcd.setCursor(12, 2);
            lcd.print("Corner  ");
            bolehMain = 'Y';
          }
          if (intruksi == 'p') {
            lcd.setCursor(12, 2);
            lcd.print("Penalty ");
            bolehMain = 'Y';
          }
        }
      }
      else if (team == 'M' && (intruksi == 'K' || intruksi == 'F' || intruksi == 'G' || intruksi == 'C')) {
        if (runIntruksi == 'N') {
          detikRun = 0;
          runIntruksi = 'Y';
        }
        lcd.setCursor(12, 2); lcd.print(detikRun); lcd.print("   ");

        if (bolehMain == 'N' && detikRun <= 6) {
          pidBolaJaga();
        }

        if (detikRun > 6) {
          if (intruksi == 'K') {
            lcd.setCursor(12, 2);
            lcd.print("KickOff ");
            bolehMain = 'Y';
          }
          if (intruksi == 'F') {
            lcd.setCursor(12, 2);
            lcd.print("FreeKick");
            bolehMain = 'Y';
          }
          if (intruksi == 'G') {
            lcd.setCursor(12, 2);
            lcd.print("GoalKick");
            bolehMain = 'Y';
          }
          if (intruksi == 'C') {
            lcd.setCursor(12, 2);
            lcd.print("Corner  ");
            bolehMain = 'Y';
          }
          if (intruksi == 'P') {
            lcd.setCursor(12, 2);
            lcd.print("Penalty ");
            bolehMain = 'Y';
          }
        }
      }

      if (bolehMain == 'Y') {
        if (pernahStart == 'N') {
          countDetek = 0;
          countGaDetek = 0;
          pernahStart = 'Y';
        }

        if (dapatBola == 'N') {
          sudahMundur = 'N';
          pidBola();
        }
        else if (jarDribble() > jarbo) {
          dapatBola = 'N';
          dribble(0);
        }
        else {
          runToGoal();
          pernahStart = 'N';
        }
      }
    }
    if (intruksiPlay == 'R' && titikTuju == 0) {
      if (enableReadThetaNow == 1) {
        thetaNow = Theta;
        enableReadThetaNow = 0;
      }
      //        runKinematik(xTarget,yTarget,tTarget,130);
      runKinematik(xTarget, yTarget, tTarget, 120);
    }

    count++;
    if (count > 10) {

      Serial2.print(String("#?,") + (int)jarX + String(",") + (int)jarY + String(",") + (int)Theta + String(",\n"));
      Serial2.print(String("#!,") + (int)xTarget + String(",") + (int)yTarget + String(",") + (int)tTarget + String(",\n"));
      count = 0;
    }
  }

  delay(300); lcd.clear();
  }

   void pidBola1() {
  float kp = 1.3, kp1 = 0.6, kp2 = 1, ki = 0, kd = 0.5, kd1 = 0.6;
  int  maxSpd = 45, maxSpd1 = 400, maxPWM = 160, maxPWM1 = 200, maxSpd2 = 300, errorY, errorX, maxx;
  int kir, kan, bel , PIDD;
  //  int spX=325, spY=250;

  parseBola();

  if (yBola >= 0) {
    detekBola = 1;
    countGaDetek = 0;
    errorY = spY - yBola;
  }
  else {
    errorY = 0;
  }
  if (xBola >= 0) {
    detekBola = 1;
    countGaDetek = 0;
    errorX = spX - xBola;
  }
  else {
    errorIL = 0;
    errorIR = 0;
    errorIB = 0;
    errorX = 0;
    detekBola = 0;
  }

  //  if(detekBola==0 && countGaDetek>10){
  //      if(posisiBola=='L'){
  //        errorX=70; dribble(0);
  //      }
  //      if(posisiBola=='R'){
  //        errorX=-70; dribble(0);
  //      }
  //  }

  //  PID = (kp*errorY) + (ki*(errorY + lastError)) + (kd*(errorY-lastError));
  //  PID = errorY;

  if (yBola <= 120 && yBola > -1) {
    //    maxx = maxSpd1;
    maxx = maxPWM1;
    PID = (kp1 * errorX) + (ki * (errorX + lastError)) + (kd * (errorX - lastError));
    kir = (140 - PID);
    kan = (140 + PID);

    PIDD = kp * errorX;
  }
  else if (yBola > 120) {
    //    maxx = maxSpd2;
    maxx = maxPWM;
    PID = (kp2 * errorX) + (ki * (errorX + lastError)) + (kd1 * (errorX - lastError));
    kir = (110 - PID);
    kan = (110 + PID);

    PIDD = kp * errorX;
  }

  if (kir > maxx) {
    kir = maxx;
  }
  if (kir < 0) {
    kir = 0;
  }
  if (kan > maxx) {
    kan = maxx;
  }
  if (kan < 0) {
    kan = 0;
  }

  if (PIDD > maxSpd) {
    PIDD = maxSpd;
  }
  if (PIDD < -maxSpd) {
    PIDD = -maxSpd;
  }

  //  if(jarDribble()<40){dribble(250); motor(0,0,0);}
  //  else if(xBola>=spX-40 && xBola<=spX && errorY>-20 && errorY<20){motor(0,0,0);}
  //  else if(errorY>-40 && errorY<40 && xBola<=spX ){kir=-(100+(errorX*kp1))-PID; kan=(100+(kp1*errorX))-PID; bel=0; runRPM1(kir,kan,bel,maxSpd1);}
  //  else if(xBola>spX  && errorY<spY){kir=kan=bel=-(kp*xBola); runRPM1(kir,kan,bel,maxSpd);}
  //  else if(xBola>spX  && errorY>spY){kir=kan=bel= (kp*xBola); runRPM1(kir,kan,bel,maxSpd);}
  //  else {kir=kan=bel=-(errorY); runRPM1(kir,kan,bel,maxSpd); dribble(0);}

  //  if(jarDribble()<jarbo){errorIL=errorIR=errorIB=0; dribble(150); motor(0,0,0); dapatBola='Y';}
  //  else if(yBola<10 && errorX>=-40 && errorX<=40){runRPM1(-kir,kan,0,maxSpd1); lcd.setCursor(0,3); lcd.print("Maju   "); dapatBola='N';}
  if (errorX >= -110 && errorX <= 110 && detekBola == 1) {
    motor(-kir, kan, 0);
    dapatBola = 'N';
  }
  else if (errorX < -110 || errorX > 110 && detekBola == 1) {
    motor(PIDD, PIDD, PIDD);
    dribble(0);
    dapatBola = 'N';
  }
  else if (countGaDetek > 55) {
    runKinematik(300, 450, 0, 130);
  }
  else if (countGaDetek > 10) {
    if (posisiBola == 'L') {
      motor(30, 30, 30);
      dribble(0);
    }
    if (posisiBola == 'R') {
      motor(-30, -30, -30);
      dribble(0);
    }
  }
  else if (countGaDetek > 2 && koreksiBola == 'Y') {
    if (posisiBola == 'L') {
      motor(30, 30, 30);
      dribble(0);
    }
    if (posisiBola == 'R') {
      motor(-30, -30, -30);
      dribble(0);
    }
  }

  else {
    errorIL = errorIR = errorIB = 0;
    kir = kan = bel = 0;
    motor(0, 0, 0);
    dribble(0);
    dapatBola = 'N';
  }

  if (errorX > 0) {
    posisiBola = 'L';
  }
  if (errorX < 0) {
    posisiBola = 'R';
  }
  if (errorX > 100 || errorX < -100) {
    koreksiBola = 'Y';
  }
  else {
    koreksiBola = 'N';
  }

  lastError = errorX;

  lcd.setCursor(0, 2); lcd.print("x "); lcd.print((int)xBola); lcd.print(" y "); lcd.print((int)yBola); lcd.print("  ");
  //  lcd.setCursor(0,1); lcd.print(errorX); lcd.print(" "); lcd.print(errorY); lcd.print(" ");
  //  lcd.print((int)kir); lcd.print(" "); lcd.print((int)kan); lcd.print(" "); lcd.print((int)PID); lcd.print("   ");
  lcd.setCursor(0, 3);
  lcd.print("L"); lcd.print((int)kir);
  lcd.print(" R"); lcd.print((int)kan);
  lcd.print(" B"); lcd.print((int)pwmB); lcd.print("   ");

  if (jarDribble() < 30) {
    dribble(200);
  }
  else {
    dribble(0);
  }
  }

   void runRPM0(int L, int R, int B) {
  float kp = 0.001, ki = 0.01, kd = 0;
  //  float kp=0.003, ki=0.01, kd=0;

  errorL = L - rpmL;
  errorR = R - rpmR;
  errorB = B - rpmB;

  errorIL += errorL;
  errorIR += errorR;
  errorIB += errorB;

  pwmL = (kp * errorL) + (ki * (errorIL)) + (kd * (errorL - errorL1));
  pwmR = (kp * errorR) + (ki * (errorIR)) + (kd * (errorR - errorR1));
  pwmB = (kp * errorB) + (ki * (errorIB)) + (kd * (errorB - errorB1));

  errorL1 = errorL;
  errorR1 = errorR;
  errorB1 = errorB;

  if (pwmL > 255) {
    pwmL = 255;
  }
  if (pwmL < -255) {
    pwmL = -255;
  }
  if (pwmR > 255) {
    pwmR = 255;
  }
  if (pwmR < -255) {
    pwmR = -255;
  }
  if (pwmB > 255) {
    pwmB = 255;
  }
  if (pwmB < -255) {
    pwmB = -255;
  }

  motor(pwmL, pwmR, pwmB);
  }

  void runRPM(int L, int R, int B) {
  float kp = 0.005, ki = 0.03, kd = 0;
  //  float kp=0.003, ki=0.01, kd=0;

  errorL = L - rpmL;
  errorR = R - rpmR;
  errorB = B - rpmB;

  errorIL += errorL;
  errorIR += errorR;
  errorIB += errorB;

  pwmL = (kp * errorL) + (ki * (errorIL)) + (kd * (errorL - errorL1));
  pwmR = (kp * errorR) + (ki * (errorIR)) + (kd * (errorR - errorR1));
  pwmB = (kp * errorB) + (ki * (errorIB)) + (kd * (errorB - errorB1));

  errorL1 = errorL;
  errorR1 = errorR;
  errorB1 = errorB;

  if (pwmL > 255) {
    pwmL = 255;
  }
  if (pwmL < -255) {
    pwmL = -255;
  }
  if (pwmR > 255) {
    pwmR = 255;
  }
  if (pwmR < -255) {
    pwmR = -255;
  }
  if (pwmB > 255) {
    pwmB = 255;
  }
  if (pwmB < -255) {
    pwmB = -255;
  }

  motor(pwmL, pwmR, pwmB);
  }

  void runRPM1(int L, int R, int B, int maxPWM) {
  float kp = 0.005, ki = 0.02, kd = 0;
  //  float kp=0.004, ki=0.02, kd=0;
  //  float kp=0.003, ki=0.01, kd=0;

  errorL = L - rpmL;
  errorR = R - rpmR;
  errorB = B - rpmB;

  errorIL += errorL;
  errorIR += errorR;
  errorIB += errorB;

  pwmL = (kp * errorL) + (ki * (errorIL)) + (kd * (errorL - errorL1));
  pwmR = (kp * errorR) + (ki * (errorIR)) + (kd * (errorR - errorR1));
  pwmB = (kp * errorB) + (ki * (errorIB)) + (kd * (errorB - errorB1));

  errorL1 = errorL;
  errorR1 = errorR;
  errorB1 = errorB;

  if (pwmL > maxPWM) {
    pwmL = maxPWM;
  }
  if (pwmL < -maxPWM) {
    pwmL = -maxPWM;
  }
  if (pwmR > maxPWM) {
    pwmR = maxPWM;
  }
  if (pwmR < -maxPWM) {
    pwmR = -maxPWM;
  }
  if (pwmB > maxPWM) {
    pwmB = maxPWM;
  }
  if (pwmB < -maxPWM) {
    pwmB = -maxPWM;
  }

  motor(pwmL, pwmR, pwmB);
  }

  void runXY0(int X, int Y, int W) {
  //
  //  vL=-(sin(radians(150))*X) + (cos(radians(150))*Y) + (W*rRobot);
  //  vR=-(sin(radians(30))*X) + (cos(radians(30))*Y) + (W*rRobot);
  //  vB=-(sin(radians(270))*X) + (cos(radians(270))*Y) + (W*rRobot);

  vL = -(sin(radians(150)) * X) + (cos(radians(150)) * Y) + (W);
  vR = -(sin(radians(30)) * X) + (cos(radians(30)) * Y) + (W);
  vB = -(sin(radians(270)) * X) + (cos(radians(270)) * Y) + (W);

  //  Serial.println(cos(radians(180))*X);

  if (vL > maxRPM) {
    vL = maxRPM;
  }
  if (vR > maxRPM) {
    vR = maxRPM;
  }
  if (vB > maxRPM) {
    vB = maxRPM;
  }
  if (vL < -maxRPM) {
    vL = -maxRPM;
  }
  if (vR < -maxRPM) {
    vR = -maxRPM;
  }
  if (vB < -maxRPM) {
    vB = -maxRPM;
  }

  runRPM0(vL, vR, vB);
  }

  void runXY(int X, int Y, int W) {
  //
  //  vL=-(sin(radians(150))*X) + (cos(radians(150))*Y) + (W*rRobot);
  //  vR=-(sin(radians(30))*X) + (cos(radians(30))*Y) + (W*rRobot);
  //  vB=-(sin(radians(270))*X) + (cos(radians(270))*Y) + (W*rRobot);

  vL = -(sin(radians(150)) * X) + (cos(radians(150)) * Y) + (W);
  vR = -(sin(radians(30)) * X) + (cos(radians(30)) * Y) + (W);
  vB = -(sin(radians(270)) * X) + (cos(radians(270)) * Y) + (W);

  //  Serial.println(cos(radians(180))*X);

  if (vL > maxRPM) {
    vL = maxRPM;
  }
  if (vR > maxRPM) {
    vR = maxRPM;
  }
  if (vB > maxRPM) {
    vB = maxRPM;
  }
  if (vL < -maxRPM) {
    vL = -maxRPM;
  }
  if (vR < -maxRPM) {
    vR = -maxRPM;
  }
  if (vB < -maxRPM) {
    vB = -maxRPM;
  }

  runRPM(vL, vR, vB);
  }

  void runXYJar(int X, int Y, int W, int jar) {
  //
  //  vL=-(sin(radians(150))*X) + (cos(radians(150))*Y) + (W*rRobot);
  //  vR=-(sin(radians(30))*X) + (cos(radians(30))*Y) + (W*rRobot);
  //  vB=-(sin(radians(270))*X) + (cos(radians(270))*Y) + (W*rRobot);

  vL = -(sin(radians(150)) * X) + (cos(radians(150)) * Y) + (W * rRobot);
  vR = -(sin(radians(30)) * X) + (cos(radians(30)) * Y) + (W * rRobot);
  vB = -(sin(radians(270)) * X) + (cos(radians(270)) * Y) + (W * rRobot);

  //  Serial.println(cos(radians(180))*X);

  if (vL > maxRPM) {
    vL = maxRPM;
  }
  if (vR > maxRPM) {
    vR = maxRPM;
  }
  if (vB > maxRPM) {
    vB = maxRPM;
  }
  if (vL < -maxRPM) {
    vL = -maxRPM;
  }
  if (vR < -maxRPM) {
    vR = -maxRPM;
  }
  if (vB < -maxRPM) {
    vB = -maxRPM;
  }

  runRPM(vL, vR, vB);
  }


   void runKinematik1(int x, int y, int t, int maxPWM){
  //  float vX,vY,w;
  //  float acuan=1.072; //diameter=37cm
  float kpXY=1.1, kiXY=0.06, kdXY=0;
  float kpT=2.3, kiT=0.05, kdT=0;

  errorX = x - jarX;
  errorY = y - jarY;
  errorT = Theta - t;

  if(errorX>maxPWM || errorX<-maxPWM){errorX /=1.5;}
  if(errorY>maxPWM || errorY<-maxPWM){errorY /=1.5;}
  if(errorT>maxPWM || errorT<-maxPWM){errorY /=1.5;}
  if(errorX>maxPWM*2 || errorX<-maxPWM*2){errorX /=1.8;}
  if(errorY>maxPWM*2 || errorY<-maxPWM*2){errorY /=1.8;}
  if(errorT>maxPWM*2 || errorT<-maxPWM*2){errorY /=1.8;}
  if(errorX>maxPWM*3 || errorX<-maxPWM*3){errorX /=2.3;}
  if(errorY>maxPWM*3 || errorY<-maxPWM*3){errorY /=2.3;}
  if(errorT>maxPWM*3 || errorT<-maxPWM*3){errorY /=2.3;}

  //    vB =(errorY*(-sin(radians(Theta)))) + (errorX*(cos(radians(Theta)))) + ((errorT));
  //    vL =(errorY*(-sin(radians(60-Theta)))) + (errorX*(-cos(radians(60-Theta)))) +((errorT));
  //    vR =(errorY*(sin(radians(60+Theta)))) + (errorX*(-cos(radians(60+Theta)))) +((errorT));

  //  vB =(errorY*(-sin(radians(Theta)))) + (errorX*(cos(radians(Theta)))) + ((errorT*rRobot));
  //  vL =(errorY*(-sin(radians(60-Theta)))) + (errorX*(-cos(radians(60-Theta)))) +((errorT*rRobot));
  //  vR =(errorY*( sin(radians(60+Theta)))) + (errorX*(-cos(radians(60+Theta)))) +((errorT*rRobot));

  ///fix
  //  vB =(errorY*(-sin(radians(Theta)))) + (errorX*(cos(radians(Theta)))) + ((errorT/3));
  //  vL =(errorY*(-sin(radians(60-Theta)))) + (errorX*(-cos(radians(60-Theta)))) +((errorT/3));
  //  vR =(errorY*( sin(radians(60+Theta)))) + (errorX*(-cos(radians(60+Theta)))) +((errorT/3));

  //  float vXY = (errorX*kpXY) + (kiXY*errorIX) + kd

  vB =((kpXY*errorY)*(-sin(radians(Theta)))) + ((kpXY*errorX)*(cos(radians(Theta)))) + ((kpT*errorT/3));
  vL =((kpXY*errorY)*(-sin(radians(60-Theta)))) + ((kpXY*errorX)*(-cos(radians(60-Theta)))) +((kpT*errorT/3));
  vR =((kpXY*errorY)*( sin(radians(60+Theta)))) + ((kpXY*errorX)*(-cos(radians(60+Theta)))) +((kpT*errorT/3));

  //  vB =(errorY*(-sin(radians(Theta)))) + (errorX*(cos(radians(Theta)))) + ((errorT));
  //  vL =(errorY*(-sin(radians(Theta+120)))) + (errorX*(cos(radians(Theta+120)))) +((errorT));
  //  vR =(errorY*(-sin(radians(Theta+240)))) + (errorX*(-cos(radians(Theta+240)))) +((errorT));

  //  vX += ((2*rpsB - rpsL - rpsR)/5.85);
  //  vY += ((sqrt(3)*rpsR - sqrt(3)*rpsL)/5.85);
  //  w +=  ((rpsL + rpsB + rpsR)/acuan);
  //
  //  vB = errorX*vX + errorT*w*dRobot;
  //  vL = -errorX*vX/2 - errorY*sqrt(3)*vY/2 + errorT*w*dRobot;
  //  vR = -errorX*vX/2 + errorY*sqrt(3)*vY/2 + errorT*w*dRobot;

  if(vL>maxRPM){vL=maxRPM;}
  if(vR>maxRPM){vR=maxRPM;}
  if(vB>maxRPM){vB=maxRPM;}
  if(vL<-maxRPM){vL=-maxRPM;}
  if(vR<-maxRPM){vR=-maxRPM;}
  if(vB<-maxRPM){vB=-maxRPM;}

  //  if((errorX<10 && errorX>-10) && (errorY<10 && errorY>-10) && (errorT<7 && errorT>-7)){runRPM1(0,0,0,0);  titikTuju=1;}
  //  else {runRPM1(vL,vR,vB,maxPWM); titikTuju=0;}

  if((errorX<10 && errorX>-10) && (errorY<10 && errorY>-10) && (errorT<7 && errorT>-7)){runRPM1(0,0,0,0);  titikTuju=1;}
  else {runRPM1(vL,vR,vB,maxPWM); titikTuju=0;}

  //  lcd.setCursor(0,0);
  //  lcd.print("L"); lcd.print((int)vL);
  //  lcd.print(" R"); lcd.print((int)vR);
  //  lcd.print(" B"); lcd.print((int)vB);
  //  lcd.print("  ");
  //
  //  lcd.setCursor(0,1);
  //  lcd.print("X");lcd.print((int)jarX);
  //  lcd.print(" Y");lcd.print((int)jarY);
  //  lcd.print(" T");lcd.print((int)Theta); lcd.print("   ");
  //
  //  lcd.setCursor(0,2);
  //  lcd.print("X");lcd.print((int)errorX);
  //  lcd.print(" Y");lcd.print((int)errorY);
  //  lcd.print(" T");lcd.print((int)errorT); lcd.print("   ");
  //
  //  lcd.setCursor(0,3);
  //  lcd.print("L");lcd.print((int)rpmL);
  //  lcd.print(" R");lcd.print((int)rpmR);
  //  lcd.print(" B");lcd.print((int)rpmB); lcd.print("   ");
  }

  void runKinematik2(int x, int y, int t, int maxPWM){
  //  float vX,vY,w;
  //  float acuan=1.072; //diameter=37cm

  //  float kpXY=1.1, kiXY=0.06, kdXY=0;
  //  float kpT=2.3, kiT=0.05, kdT=0, maxRPM=255;

  float kpXY=1.1, kiXY=0.06, kdXY=0;
  float kpT=5.3, kiT=0.05, kdT=0, maxRPM=maxPWM;
  //  int Theta_;
  //  if(Theta>=0){
  //    Theta_=Theta;
  //  }
  //  else{
  //    Theta_=360+Theta;
  //  }
  //
  errorX = x - jarX;
  errorY = y - jarY;
  errorT = Theta - t;

  //  if(errorX>maxPWM || errorX<-maxPWM){errorX /=1.5;}
  //  if(errorY>maxPWM || errorY<-maxPWM){errorY /=1.5;}
  //  if(errorT>maxPWM || errorT<-maxPWM){errorY /=1.5;}
  //  if(errorX>maxPWM*2 || errorX<-maxPWM*2){errorX /=1.8;}
  //  if(errorY>maxPWM*2 || errorY<-maxPWM*2){errorY /=1.8;}
  //  if(errorT>maxPWM*2 || errorT<-maxPWM*2){errorY /=1.8;}
  //  if(errorX>maxPWM*3 || errorX<-maxPWM*3){errorX /=2.3;}
  //  if(errorY>maxPWM*3 || errorY<-maxPWM*3){errorY /=2.3;}
  //  if(errorT>maxPWM*3 || errorT<-maxPWM*3){errorY /=2.3;}

  if(errorX<13 || errorX<-13){errorX *=2.9;}
  if(errorY<13 || errorY<-13){errorY *=2.9;}
  if(errorX<15 || errorX<-15){errorX *=2.1;}
  if(errorY<15 || errorY<-15){errorY *=2.1;}
  if(errorX<20 || errorX<-20){errorX *=1.8;}
  if(errorY<20 || errorY<-20){errorY *=1.8;}
  if(errorX<30 || errorX<-30){errorX *=1.5;}
  if(errorY<30 || errorY<-30){errorY *=1.5;}
  if(errorX>maxPWM || errorX<-maxPWM){errorX /=1.5;}
  if(errorY>maxPWM || errorY<-maxPWM){errorY /=1.5;}
  if(errorT>maxPWM || errorT<-maxPWM){errorT /=1.5;}
  if(errorX>maxPWM/2 || errorX<-maxPWM/2){errorX /=0.5;}
  if(errorY>maxPWM/2 || errorY<-maxPWM/2){errorY /=0.5;}
  if(errorT>maxPWM/2 || errorT<-maxPWM/2){errorT /=0.5;}
  if(errorX>maxPWM*2 || errorX<-maxPWM*2){errorX /=1.9;}
  if(errorY>maxPWM*2 || errorY<-maxPWM*2){errorY /=1.9;}
  if(errorT>maxPWM*2 || errorT<-maxPWM*2){errorT /=1.9;}
  if(errorX>maxPWM*3 || errorX<-maxPWM*3){errorX /=2.6;}
  if(errorY>maxPWM*3 || errorY<-maxPWM*3){errorY /=2.6;}
  if(errorT>maxPWM*3 || errorT<-maxPWM*3){errorT /=2.6;}
  if(errorX>maxPWM*4 || errorX<-maxPWM*4){errorX /=4.6;}
  if(errorY>maxPWM*4 || errorY<-maxPWM*4){errorY /=4.6;}
  if(errorT>maxPWM*4 || errorT<-maxPWM*4){errorT /=4.6;}

  vB =((kpXY*errorY)*(-sin(radians(Theta)))) + ((kpXY*errorX)*(cos(radians(Theta)))) + ((kpT*errorT/3));
  vL =((kpXY*errorY)*(-sin(radians(60-Theta)))) + ((kpXY*errorX)*(-cos(radians(60-Theta)))) +((kpT*errorT/3));
  vR =((kpXY*errorY)*( sin(radians(60+Theta)))) + ((kpXY*errorX)*(-cos(radians(60+Theta)))) +((kpT*errorT/3));

  if(vL>maxRPM){vL=maxRPM;}
  if(vR>maxRPM){vR=maxRPM;}
  if(vB>maxRPM){vB=maxRPM;}
  if(vL<-maxRPM){vL=-maxRPM;}
  if(vR<-maxRPM){vR=-maxRPM;}
  if(vB<-maxRPM){vB=-maxRPM;}

  //  if((errorX<10 && errorX>-10) && (errorY<10 && errorY>-10) && (errorT<7 && errorT>-7)){runRPM1(0,0,0,0);  titikTuju=1;}
  //  else {runRPM1(vL,vR,vB,maxPWM); titikTuju=0;}

  //  if((errorX<10 && errorX>-10) && (errorY<20 && errorY>-20) && (errorT<7 && errorT>-7)){motor(0,0,0);  titikTuju=1;}
  //  else {runXYPwm(errorX,errorY,errorT,255); titikTuju=0;}

  if((errorX<10 && errorX>-10) && (errorY<10 && errorY>-10) && (errorT<7 && errorT>-7)){motor(0,0,0);  titikTuju=1;}
  else {motor(vL,vR,vB); titikTuju=0;}

  }


   //peke 3 rotary
  void updateJarak(){
  float vX,vY,w;
  //  float acuan=1.05; //diameter=37cm
  //  float acuan=1.071; //diameter=37cm
  float acuan=1.11; //diameter=37cm

  //fix
  //  vX = ((2*rpsB - rpsL - rpsR)/5.85);
  //  vY = ((sqrt(3)*rpsR - sqrt(3)*rpsL)/5.85);
  //  w =  (-(rpsL + rpsB + rpsR)/acuan)/2;

  //  vX = ((2*rpsB - rpsL - rpsR)/6.85);
  //  vY = ((sqrt(3)*rpsR - sqrt(3)*rpsL)/6.85);
  //  w =  (-(rpsL + rpsB + rpsR)/acuan)/2;

  vX = ((2*rpsB - rpsL - rpsR)/6.85);
  vY = ((sqrt(3)*rpsR - sqrt(3)*rpsL)/6.85);

  jarX+=(cos(radians(Theta))*vX + sin(radians(Theta))*vY)*acuan;
  jarY+=(-sin(radians(Theta))*vX + cos(radians(Theta))*vY)*acuan;

  //  if(enableGyro==0){Theta+=w;}
  //  else{parseGyro(); Theta=dataGyro;}
  parseGyro(); Theta=dataGyro;
  }

  //peke 3 rotary
  void updateJarak2(){
  float vX,vY,w;
  float offset=1.11;

  vX = ((2*rpsB - rpsL - rpsR)/2);
  vY = ((sqrt(3)*rpsR/2 - sqrt(3)*rpsL/2));

  jarX+=(cos(radians(Theta))*vX + sin(radians(Theta))*vY);
  jarY+=(-sin(radians(Theta))*vX + cos(radians(Theta))*vY);

  parseGyro(); //Theta=dataGyro;
  if(dataGyro<0){
    Theta=360+dataGyro;
  }else{
    Theta=dataGyro;
  }
  if(Theta>180){
    Theta=Theta-360;
  }
  }

  //peke 2 rotary(kiri dan kanan)
  void updateJarak11(){
  float vX,vY,w;
  float acuan=0.714; //diameter=37cm
  //  float rRotary=9.2;
  float rRotary=9.2;

  //fix
  vX = (3*(rpsL + rpsR)/(rRotary));
  vY = ((sqrt(3)*rpsR - sqrt(3)*rpsL)/(rRotary));
  //  vX = (3*(radpsL + radpsR)/(rRotary));
  //  vY = ((sqrt(3)*radpsR - sqrt(3)*radpsL)/(rRotary));

  jarX+=(cos(radians(Theta))*vX + sin(radians(Theta))*vY);
  jarY+=(-sin(radians(Theta))*vX + cos(radians(Theta))*vY);

  //  vX = (cos(radians(30-Theta))*jarR1 + cos(radians(30+Theta))*jarL1);
  //  vY = (sin(radians(30-Theta))*jarR1 - sin(radians(30+Theta))*jarL1);
  //
  //  jarX += vX;
  //  jarY += vY;

  //  jarX = (jarL*-cos(radians(210+Theta)))+(jarR*-cos(radians(330+Theta)));
  //  jarY = (jarL*-sin(radians(210+Theta)))+(jarR*-sin(radians(330+Theta)));



  parseGyro(); //Theta=dataGyro;
  if(dataGyro<0){
    Theta=360+dataGyro;
  }else{
    Theta=dataGyro;
  }
  if(Theta>180){
    Theta=Theta-360;
  }
  }

  //peke 2 rotary(kiri dan kanan)
  void updateJarak1(){
  float vX,vY,w;
  float acuan=1.02;
  float acuan1=0.98;
  //  float rRotary=9.2;
  float rRotary=9.2;

  vX = (-cos(radians(45))*jarR - cos(radians(45))*jarL);
  vY = (sin(radians(45))*jarR - sin(radians(45))*jarL);

  jarX+=(cos(radians(Theta))*vX + sin(radians(Theta))*vY)*acuan1;
  jarY+=(-sin(radians(Theta))*vX + cos(radians(Theta))*vY)*acuan;

  //  jarX = (jarL*-cos(radians(210+Theta)))+(jarR*-cos(radians(330+Theta)));
  //  jarY = (jarL*-sin(radians(210+Theta)))+(jarR*-sin(radians(330+Theta)));

  parseGyro(); //Theta=dataGyro;
  if(dataGyro<0){
    Theta=360+dataGyro;
  }else{
    Theta=dataGyro;
  }
  if(Theta>180){
    Theta=Theta-360;
  }
  }


   void interrupt0(){
  //  rpmL=frekL*600/perRotasi;
  //  rpmR=frekR*600/perRotasi;
  //  rpmB=frekB*600/perRotasi;
  //  jarL+=jarPerPulsa * frekL;
  //  jarR+=jarPerPulsa * frekR;
  //  jarB+=jarPerPulsa * frekB;
  //  mmpsL=kelRoda * rpmL/60;
  //  mmpsR=kelRoda * rpmR/60;
  //  mmpsB=kelRoda * rpmB/60;

  rotpsL=frekL*10/perRotasi1;
  rotpsR=frekR*10/perRotasi1;

  rpmL=frekL*600/perRotasi1;
  rpmR=frekR*600/perRotasi1;
  rpmB=frekB*600/perRotasi;
  jarL=jarPerPulsa1 * frekL;
  jarR=jarPerPulsa1 * frekR;
  jarB=jarPerPulsa * frekB;
  mmpsL=kelRoda1 * rpmL/60;
  mmpsR=kelRoda1 * rpmR/60;
  mmpsB=kelRoda  * rpmB/60;

  //1 rpm = 0.104719755 rps rpm(rotasi per menit) rps(radian per seconds)
  //1 rotasi = 360 derajat = 2 pi radian
  //1 rpm = 2 pi/60 radian/sec
  //1 rpm = 0.104719755 rps

  radpsL = rotpsL*6.2857142857142857142857142857143;
  radpsR = rotpsR*6.2857142857142857142857142857143;

  rpsL=rpmL*0.104719755;
  rpsR=rpmR*0.104719755;
  rpsB=rpmB*0.104719755;

  //  updateJarak();
  updateJarak1();
  //  updateJarak2();

  frekL=frekR=frekB=0;

  countRun++;
  if(countRun>10){detikRun++; countRun=0;}

  countManual++;
  countDetek++;
  countGaDetek++;
  countTendang++;
  }


  void pidBola(){
  float kp=1.3, kp1=0.6, kp2=1, ki=0, kd=0.5, kd1=0.6;
  int  maxSpd=45,maxSpd1=400,maxPWM=160,maxPWM1=200,maxSpd2=300, errorY, errorX, maxx;
  int kir, kan, bel ,PIDD;
  int runX,runY,runW;
  //  int spX=325, spY=250;

  parseBola();
  //  int JarL = jarki();
  //  int JarR = jarka();

  if(yBola>=0){
    errorIL=0;
    errorIR=0;
    errorIB=0;
    detekBola=1;
    countGaDetek=0;
    errorY = spY - yBola;
  }
  else {
    errorY=0;
  }
  if(xBola>=0){
    errorIL=0;
    errorIR=0;
    errorIB=0;
    detekBola=1;
    countGaDetek=0;
    errorX = spX - xBola;
  }
  else {
    errorX=0;
    detekBola=0;
    countDetek=0;
  }

  if(yBola<=140 && yBola>-1){
    maxx = maxPWM1;
    PID = (kp1*errorX) + (ki*(errorX + lastError)) + (kd*(errorX-lastError));

    if(countDetek>10){
      kir = (150-PID);
      kan = (150+PID);
    }
    else if(countDetek>7){
      kir = (130-PID);
      kan = (130+PID);
    }
    else{
      kir = (100-PID);
      kan = (100+PID);
    }

    int jarKi = jarDeL1();
    int jarKa = jarDeR1();

    if(jarKi<jarKa){
      if(jarKi<20){kir+=20; bel=90;}
      else{kir=kir; kan=kan; bel=0;}
    }
    else{
      if(jarKa<20){kan+=20; bel=-90;}
      else{kir=kir; kan=kan; bel=0;}
    }

    PIDD = kp*errorX;
  }
  else if(yBola>140){
    maxx = maxPWM;
    PID = (kp2*errorX) + (ki*(errorX + lastError)) + (kd1*(errorX-lastError));

    kir = (100-PID);
    kan = (100+PID);

    int jarKi = jarDeL1();
    int jarKa = jarDeR1();

    if(jarKi<jarKa){
      if(jarKi<20){kir+=20; bel=90;}
      else{kir=kir; kan=kan; bel=0;}
    }
    else{
      if(jarKa<20){kan+=20; bel=-90;}
      else{kir=kir; kan=kan; bel=0;}
    }

    PIDD = kp*errorX;
  }

  if(kir > maxx) {kir= maxx;}
  if(kir <0) {kir=0;}
  if(kan > maxx) {kan= maxx;}
  if(kan <0) {kan=0;}

  if(PIDD > maxSpd) {PIDD= maxSpd;}
  if(PIDD <-maxSpd) {PIDD=-maxSpd;}

  if(jarDribble()<jarbo && xBola!=-1){countDetek=0; dapatBola='Y';}
  else if(errorX>=-110 && errorX<=110 && detekBola==1){motor(-kir,kan,bel); dapatBola='N';}
  else if(errorX<-110 || errorX>110 && detekBola==1){kir=kan=bel=PIDD; motor(PIDD,PIDD,PIDD); dribble(0); countDetek=0; dapatBola='N';}
  else if(countGaDetek>90){countGaDetek=0; countPosisioning++; if(countPosisioning>2){countPosisioning=0;}}
  else if(countGaDetek>50){ //60

      if(countPosisioning==0){tarX=300; tarY=450; tarT=0;}
      if(countPosisioning==1){tarX=300; tarY=350; tarT=0;}
      if(countPosisioning==2){tarX=300; tarY=500; tarT=0;}

      runKinematik(tarX,tarY,tarT,100);

  //      lcd.setCursor(0,0);
  //      lcd.print("X");lcd.print((int)jarX);
  //      lcd.print(" Y");lcd.print((int)jarY);
  //      lcd.print(" T");lcd.print((int)Theta); lcd.print("   ");
  //      lcd.setCursor(0,1); lcd.print("Kine "); lcd.print(countGaDetek);  lcd.print("  ");
  }
  else if(countGaDetek>10){
      if(posisiBola=='L'){motor(30,30,30);dribble(0);}
      else {motor(-30,-30,-30); dribble(0);}
      lcd.setCursor(0,1); lcd.print("Scan");
  }
  else if(countGaDetek>2 && koreksiBola=='Y'){
      if(posisiBola=='L'){motor(30,30,30);dribble(0);}
      else {motor(-30,-30,-30); dribble(0);}
      lcd.setCursor(0,1); lcd.print("Wait");
  }

  else {errorIL=errorIR=errorIB=0; kir=kan=bel=0; motor(0,0,0);  dribble(0); dapatBola='N'; lcd.setCursor(0,1); lcd.print("    "); }

  if(errorX>0){posisiBola='L';}
  if(errorX<0){posisiBola='R';}
  if(errorX>100 || errorX<-100){koreksiBola='Y';}
  else {koreksiBola='N';}

  lastError=errorX;

  lcd.setCursor(0,2); lcd.print("x"); lcd.print((int)xBola); lcd.print(" y"); lcd.print((int)yBola); lcd.print("  ");
  lcd.setCursor(0,3);
  lcd.print("L");lcd.print((int)kir);
  lcd.print(" R");lcd.print((int)kan);
  lcd.print(" B");lcd.print((int)bel); lcd.print(" "); lcd.print(countGaDetek); lcd.print(" ");

  if(jarDribble()<30){dribble(200);}
  else {dribble(0);}
  }

   parseGyro();
  //  kameraHp();
  //  lcd.setCursor(0,0); lcd.print("URT-ROS0 2018");
  //  lcd.setCursor(0,1); lcd.print(dataGyro); lcd.print(" ");
  //  lcd.print(xBola); lcd.print(" "); lcd.print(yBola); lcd.print(" ");

  //  lcd.setCursor(0,2);
  //  if(O()){ lcd.print("ENTER");}
  //  else if(U()){ lcd.print("UP");}
  //  else if(D()){ lcd.print("DOWN");}
  //  else if(P()){ lcd.print("PLUS");}
  //  else if(M()){ lcd.print("MINUS");}
  //  else if(C()){ lcd.print("Blkang");}
  //  else {lcd.print("     ");}

  //  lcd.setCursor(0,2);
  //  lcd.print(pwmL); lcd.print(" ");
  //  lcd.print(pwmR); lcd.print(" ");
  //  lcd.print(pwmB); lcd.print("  ");
  //  lcd.setCursor(0,2);
  //  lcd.print(jarX); lcd.print(" ");
  //  lcd.print(jarY); lcd.print(" ");
  //  lcd.print(Theta); lcd.print("  ");
  //
  //  lcd.setCursor(0,3);
  //  lcd.print(rpmL); lcd.print(" ");
  //  lcd.print(rpmR); lcd.print(" ");
  //  lcd.print(rpmB); lcd.print("  ");

  //  runRPM(-100,100,0);
  //  motor(100,50,255);
  //  dribble(100);


  void updateJarak1(){
  float vX,vY,w;
  float acuan=1.071; //diameter=37cm
  //  jarX =(( cos(radians(Theta))/1.5)*jarB) + (((-sin(radians(Theta))/sqrt(3))+(-cos(radians(Theta))/3))*jarL) + (((sin(radians(Theta))/sqrt(3))+(-cos(radians(Theta))/3))*jarR);
  //  jarY =((-sin(radians(Theta))/1.5)*jarB) + (((-cos(radians(Theta))/sqrt(3))+( sin(radians(Theta))/3))*jarL) + (((cos(radians(Theta))/sqrt(3))+( sin(radians(Theta))/3))*jarR);

  //jarX = sin(radians(120+Theta))*jarR + sin(radians(240+Theta))*jarL + sin(radians(360+Theta)*jarB);
  //jarY = cos(radians(120+Theta))*jarR + cos(radians(240+Theta))*jarL + cos(radians(360+Theta)*jarB);

  //  if(enableGyro==0){Theta=-(jarL/acuan)-(jarR/acuan)-(jarB/acuan);}
  //  else{Theta=dataGyro();}

  //  if(T>360){Theta=T-360;}
  //  else if(T<0){Theta=360+T;}
  //  else {Theta=T;}
  //  vX = (2*jarB - jarL - jarR)/3;
  //  vY = (sqrt(3)*jarR - sqrt(3)*jarL)/3;
  //  w =  (jarL + jarB + jarR)/acuan;

  //fix
  vX = ((2*rpsB - rpsL - rpsR)/5.85);
  vY = ((sqrt(3)*rpsR - sqrt(3)*rpsL)/5.85);
  w =  (-(rpsL + rpsB + rpsR)/acuan)/2;

  jarX+=(cos(radians(Theta))*vX + sin(radians(Theta))*vY)*1.05;
  jarY+=(-sin(radians(Theta))*vX + cos(radians(Theta))*vY)*1.05;
  if(enableGyro==0){Theta+=w;}
  else{parseGyro(); Theta=dataGyro;}
  //  parseGyro();
  //  Theta=dataGyro;


  //  jarY=(-sin(radians(60+Theta))*jarL) + (sin(radians(60-Theta))*jarR);
  //  jarX=(-cos(radians(225+Theta))*jarL) + (-cos(radians(135+Theta))*jarR);
  //  jarY=(-sin(radians(Theta-60))*jarL) + (-sin(radians(60+Theta))*jarR);
  //  jarY=(-sin(radians(225+Theta))*jarL) + (-sin(radians(135+Theta))*jarR);
  //  Theta+=w;

  //  Theta=(0.5882352941176471*jarL)+(0.5882352941176471*jarR)+(0.5882352941176471*jarB);

  //  derajat = Theta * 180/3.142857142857143;
  //  Theta=(dRobot/3)*jarL + (dRobot/3)*jarR + (dRobot/3)*jarB;

  }

    if(data=='*'){
        data = Serial5.parseInt();
  //      lcd.setCursor(15,3); lcd.write("*");
  //      if(data==1){dribble(10); lcd.setCursor(15,3); lcd.write("A");}
  //      else if(data==2){dribble(40); lcd.setCursor(15,3); lcd.write("S");}
  //      else if(data==3){dribble(100); lcd.setCursor(15,3); lcd.write("D");}
  //      else if(data==4){dribble(150); lcd.setCursor(15,3); lcd.write("Q");}
  //      else if(data==5){dribble(200); lcd.setCursor(15,3); lcd.write("W");}
  //      else if(data==6){dribble(250); lcd.setCursor(15,3); lcd.write("E");}
  //      else if(data==0){dribble(0); lcd.setCursor(15,3); lcd.write(" ");}

        if(data==1){motor(50,50,-100); lcd.setCursor(15,3); lcd.write("A");}
        else if(data==2){motor(100,-100,0); lcd.setCursor(15,3); lcd.write("S");}
        else if(data==3){motor(-50,-50,100); lcd.setCursor(15,3); lcd.write("D");}
        else if(data==4){motor(50,50,50); lcd.setCursor(15,3); lcd.write("Q");}
        else if(data==5){motor(-100,100,0); lcd.setCursor(15,3); lcd.write("W");}
        else if(data==6){motor(-50,-50,-50); lcd.setCursor(15,3); lcd.write("E");}
        else if(data==0){motor(0,0,0); lcd.setCursor(15,3); lcd.write(" ");}
      }

  //  else if(data=='*'){
  //        data = Serial5.parseInt();
  //  else if(data=='Q'){motor(50,50,50); lcd.setCursor(15,3); lcd.write("Q");}
  //  else if(data=='E'){motor(-50,-50,-50); lcd.setCursor(15,3); lcd.write("E");}
  //  else if(data=='Z'){motor(0,0,0); lcd.setCursor(15,3); lcd.write("Z");}
  //      }
  //  lcd.setCursor(13,3); lcd.write(data);

  //  else if(data=='X'){
  //    int dataX = Serial5.parseInt();
  //    if(Serial5.read()=='Y'){
  //        int dataY = Serial5.parseInt();
  //        if(Serial5.read()=='#'){
  //          jarX=(int)dataX;
  //          jarY=(int)dataY;
  //          jarL=jarR=jarB=0;
  //        }
  //    }
  //  }

  //  // aktifin ini untuk terima data Theta Rosi
  //  else if(data=='W'){
  //    int dataW = Serial5.parseInt();
  //    if(Serial5.read()=='#'){
  //      lcd.setCursor(13,2); lcd.print(dataW); lcd.print("   ");
  //      Serial2.println(dataW + String("#"));
  //    }
  //   }
  //
  //  // aktifin ini untuk terima data Theta Roso
  ////  else if(data=='w'){
  ////    int dataW = Serial5.parseInt();
  ////    if(Serial5.read()=='#'){
  ////      lcd.setCursor(13,2); lcd.print(dataW); lcd.print("   ");
  ////      Serial2.println(dataW + String("#"));
  ////    }
  ////   }
  //
  //
  //  else if(data=='?'){
  //    //aktifin ini untuk terima data rosi posisi x dan y sekarang
  //    if(Serial5.read()=='X'){
  //      int dataX = Serial5.parseInt();
  //      if(Serial5.read()=='Y'){
  //          int dataY = Serial5.parseInt();
  //          if(Serial5.read()=='#'){
  //            jarX=(int)dataX;
  //            jarY=(int)dataY;
  //            jarL=jarR=jarB=0;
  //          }
  //        }
  //      }
  //
  //    //aktifin ini untuk terima data roso posisi x dan y sekarang
  ////    if(Serial5.read()=='x'){
  ////      int dataX = Serial5.parseInt();
  ////      if(Serial5.read()=='y'){
  ////          int dataY = Serial5.parseInt();
  ////          if(Serial5.read()=='#'){
  ////            jarX=(int)dataX;
  ////            jarY=(int)dataY;
  ////            jarL=jarR=jarB=0;
  ////          }
  ////        }
  ////      }
  //   }
  //   else if(data=='!'){
  //    //aktifin ini untuk terima data rosi posisi x dan y sekarang
  //    if(Serial5.read()=='X'){
  //      int dataX = Serial5.parseInt();
  //      if(Serial5.read()=='Y'){
  //          int dataY = Serial5.parseInt();
  //          if(Serial5.read()=='#'){
  //            xTarget=(int)dataX;
  //            yTarget=(int)dataY;
  //          }
  //        }
  //      }
  //   }
  //
  //   //aktifkan ini untuk terima data roso posisi x dan y sekarang
  ////   if(Serial5.read()=='x'){
  ////      int dataX = Serial5.parseInt();
  ////      if(Serial5.read()=='y'){
  ////          int dataY = Serial5.parseInt();
  ////          if(Serial5.read()=='#'){
  ////            xTarget=(int)dataX;
  ////            yTarget=(int)dataY;
  ////          }
  ////        }
  ////      }
  ////   }
  //  }

     //run KickOff Tim kita
      if((intruksi=='K' && team=='C') || (intruksi=='k' && team=='M') || (intruksi=='F' && team=='C') || (intruksi=='f' && team=='M') ||
         (intruksi=='G' && team=='C') || (intruksi=='g' && team=='M') || (intruksi=='C' && team=='C') || (intruksi=='c' && team=='M')){
        lcd.setCursor(12,2); lcd.print("KickOff"); bolehMain='Y';
      }
      //run KickOff Tim Lawan
      if((intruksi=='k' && team=='C') || (intruksi=='K' && team=='M')){

        if(runIntruksi=='N') {detikRun=0; runIntruksi='Y';}
        lcd.setCursor(12,2); lcd.print(detikRun); lcd.print("   ");
        if(detikRun>3){
          lcd.setCursor(12,2); lcd.print("KickOff");
          bolehMain='Y';
        }
      }

         while(!C()){

      if(dapatBola=='N'){sudahMundur='N'; pidBola();}
      else if(jarDribble()>jarbo){dapatBola='N'; dribble(0);}
      else{
        dribble(255);
        if(sudahMundur=='N'){motor(50,-50,0); delay(200); motor(0,0,0); sudahMundur='Y';}
        if((Theta>5 && Theta<=180) || Theta<-180){motor(55,55,65); lcd.setCursor(13,1); lcd.print("roL");}
        else if((Theta<-5 && Theta>=-180) || Theta>180){motor(-55,-55,-65); lcd.setCursor(13,1); lcd.print("roR");}
        else{
  //          motor(0,0,0); dribble(5);
          int jarKi = jarki();
          int jarKa = jarka();
          int jarDe = jarDeL();
          int jarBe = jarBeL();

          if(jarKi>250 && jarKa>250){
            lcd.setCursor(13,1); lcd.print("Cen");
            tendang();
          }
          else if(jarKi<250 && jarKa>250){
            lcd.setCursor(13,1); lcd.print("Kir");
            if(jarKi<150){
              motor(-55,-65,-65); delay(100); motor(0,0,0);
              tendang();
            }
            else{
              motor(-55,-65,-65); delay(70); motor(0,0,0);
              tendang();
            }
          }
          else if(jarKi>250 && jarKa<250){
            lcd.setCursor(13,1); lcd.print("Kan");
            if(jarKi<150){
              motor(65,55,65); delay(100); motor(0,0,0);
              tendang();
            }
            else{
              motor(65,55,65); delay(70); motor(0,0,0);
              tendang();
            }
          }
          else {
            if(jarBe<300 && jarKa>100 && jarKi>100){
              motor(-130,130,0); delay(700);
            }
            else if(jarDe<100 && jarBe>100 && jarKa<200 && jarKi<200){
              motor(130,-130,0); delay(700);
            }
            else if(jarDe<100 && jarBe>100 && jarKa<200 && jarKi>100){
              motor(130,0,-130); delay(700);
            }
            else if(jarDe<100 && jarBe>100 && jarKa>100 && jarKi<200){
              motor(0,-130,130); delay(700);
            }
            else{
              motor(120,-120,0); delay(700);
              motor(0,0,0);
              if(jarKi<jarKa){
                motor(-55,-65,-65); delay(200); motor(0,0,0);
                tendang();
              }
              else{
                motor(65,55,65); delay(200); motor(0,0,0);
                tendang();
              }
            }
          }

        }

      }
    } motor(0,0,0); lcd.clear();

  //      lcd.setCursor(0,2);
  //      if(jarakbe > 300 && jarakka > 200 && jarakki < 200) {dribble(255); motor(50,-100,50); tengahX=0; tengahY=0; lcd.print("B");}
  //      else if(jarakde > 300 && jarakka < 200 && jarakki > 200) {dribble(255); motor(100,-50,-50); tengahX=0; tengahY=0; lcd.print("B");}
  //
  //      else if(jarakbe < 10) {dribble(150); maju(170,170); tengahX=0; tengahY=0; lcd.print("F");}
  //      else if(jarakbe > jarakbelakang-10 && jarakde < 10) {dribble(255); mundur(100,100); tengahX=0; tengahY=0; lcd.print("B");}
  //      else if(jarakbe < jarakbelakang-10) {dribble(150); maju(100,100); tengahX=0; tengahY=0; lcd.print("F");}
  //      else if(jarakbe > jarakbelakang+10) {dribble(255); mundur(100,100); tengahX=0; tengahY=0; lcd.print("B");}
  //
  //      else if(jarakki > 200 && jarakka < 200) {dribble(255); geserKiri(100); tengahX=0; tengahY=1; lcd.print("L");}
  //      else if(jarakki < 200 && jarakka > 200) {dribble(255); geserKanan(100); tengahX=0; tengahY=1; lcd.print("R");}
  //      else if(jarakki < 200 && jarakka < 200 && sumbuX > sX+80) {dribble(255); geserKiri(100); tengahX=0; tengahY=1; lcd.print("L");}
  //      else if(jarakki < 200 && jarakka < 200 && sumbuX < sX-80) {dribble(255); geserKanan(100); tengahX=0; tengahY=1; lcd.print("R");}
  //
  //      else {dribble(255); berhenti(); tengahX=1; tengahY=1; lcd.print("S");}

  void keTengahLapanganS() {
  if      ((Theta <= 180 && Theta > 90 + 10) || (Theta <= -180 && Theta > -270 + 10)) {
    motor(55, 55, 55);
    lcd.setCursor (15, 1);
    lcd.print("roL");
  }
  else if ((Theta <= 0 && Theta > -90 + 10) || (Theta <= 360 && Theta > 270 + 10)) {
    motor(55, 55, 55);
    lcd.setCursor (15, 1);
    lcd.print("roL");
  }
  else if ((Theta > 0 && Theta < 90 - 10) || (Theta >= -360 && Theta < -270 - 10)) {
    motor(-55, -55, -55);
    lcd.setCursor(15, 1);
    lcd.print("roR");
  }
  else if ((Theta > 180 && Theta < 270 - 10) || (Theta > -180 && Theta < -90 - 10)) {
    motor(-55, -55, -55);
    lcd.setCursor(15, 1);
    lcd.print("roR");
  }
  else {
    motor(0, 0, 0);
  }
  }

  void keTitikLapangan180(int sX, int jarakbelakang) {
  int jarakbe, jarakki, jarakde, jarakka, pwmX, pwmY, sumbuX, sumbuY;

  if      ((Theta > 180 + 10 && Theta <= 360) || (Theta > -180 + 10 && Theta < 0)) {
    dribble(255);
    motor(40, 40, 40);
    lcd.setCursor (0, 2);
    lcd.print("l");
  }
  else if ((Theta < 180 - 10 && Theta >= 0) || (Theta < -180 - 10 && Theta >= -360)) {
    dribble(255);
    motor(-40, -40, -40);
    lcd.setCursor (0, 2);
    lcd.print("r");
    kir = kan = bel = 0;
    countPos = 21;
  }
  else {
    countPos++;
    if (countPos > 40) {
      jarakki = jarki();
      jarakka = jarka();
      jarakbe = jarBeL() + jarBeR() / 2;
      jarakde = jarDeL() + jarDeR() / 2;

      sumbuX = (jarakki) - (jarakka + sX);
      sumbuY = jarakbe - jarakbelakang;

      if (pwmX > 255) {
        pwmX = 255;
      } if (pwmX < -255) {
        pwmX = -255;
      }
      if (pwmY > 255) {
        pwmY = 255;
      } if (pwmY < -255) {
        pwmY = -255;
      }

      if (jarakbe > 360 && jarakka > 100 && jarakki > 100) {
        dribble(255);
        kir = 140;
        kan = -140;
        bel = 0;
        tengahX = 0;
        tengahY = 0;
        lcd.print("B");
      }
      else if (jarakbe > 360 && jarakka > 100 && jarakki < 100) {
        dribble(255);
        kir = 70;
        kan = -140;
        bel = 70;
        tengahX = 0;
        tengahY = 0;
        lcd.print("B");
      }
      else if (jarakde > 360 && jarakka < 100 && jarakki > 100) {
        dribble(255);
        kir = 140;
        kan = -70;
        bel = -70;
        tengahX = 0;
        tengahY = 0;
        lcd.print("B");
      }
      else if (jarakbe < 150 && jarakde > 150) {
        dribble(10);
        kir = -140;
        kan = 140;
        bel = 0;
        tengahX = 0;
        tengahY = 1;
        lcd.print("F");
      }

      else if (jarakde > 360 && jarakka > 200 && jarakki < 200) {
        dribble(255);
        kir = 0;
        kan = -100;
        bel = 100;
        tengahX = 0;
        tengahY = 0;
        lcd.print("B");
      }
      else if (jarakde > 360 && jarakka < 200 && jarakki > 200) {
        dribble(255);
        kir = 100;
        kan = -0;
        bel = -100;
        tengahX = 0;
        tengahY = 0;
        lcd.print("B");
      }

      else if (jarakbe < 10) {
        dribble(10);
        kir = -150;
        kan = 150;
        bel = 0;
        tengahX = 0;
        tengahY = 1;
        lcd.print("F");
      }
      //      else if(jarakbe > jarakbelakang-10 && jarakde < 10) {dribble(255); kir=100; kan=-100; bel=0; tengahX=0; tengahY=0; lcd.print("B");}
      //      else if(jarakbe < jarakbelakang-10) {dribble(10); kir=-150; kan=150; bel=0; tengahX=0; tengahY=1; lcd.print("F");}
      //      else if(jarakbe > jarakbelakang+10) {dribble(255); kir=100; kan=-100; bel=0;tengahX=0; tengahY=0; lcd.print("B");}

      else if (jarakbe > 350 - 10 && jarakde < 10) {
        dribble(255);
        kir = 100;
        kan = -100;
        bel = 0;
        tengahX = 0;
        tengahY = 0;
        lcd.print("B");
      }
      else if (jarakbe < 350 - 10) {
        dribble(10);
        kir = -150;
        kan = 150;
        bel = 0;
        tengahX = 0;
        tengahY = 1;
        lcd.print("F");
      }
      else if (jarakbe > 350 + 10) {
        dribble(255);
        kir = 100;
        kan = -100;
        bel = 0;
        tengahX = 0;
        tengahY = 0;
        lcd.print("B");
      }

      else if (jarakki > 300 && jarakka < 300) {
        dribble(255);
        kir = 50;
        kan = 50;
        bel = -100;
        tengahX = 0;
        tengahY = 1;
        lcd.print("L");
      }
      else if (jarakki < 300 && jarakka > 300) {
        dribble(255);
        kir = -50;
        kan = -50;
        bel = 100;
        tengahX = 0;
        tengahY = 1;
        lcd.print("R");
      }
      else if (jarakki < 300 && jarakka < 300 && sumbuX > sX + 80) {
        dribble(255);
        kir = 50;
        kan = 50;
        bel = -100;
        tengahX = 0;
        tengahY = 1;
        lcd.print("L");
      }
      else if (jarakki < 300 && jarakka < 300 && sumbuX < sX - 80) {
        dribble(255);
        kir = -50;
        kan = -50;
        bel = 100;
        tengahX = 0;
        tengahY = 1;
        lcd.print("R");
      }


      else {
        dribble(255);
        kir = 0;
        kan = 0;
        bel = 0;
        tengahX = 1;
        tengahY = 1;
        lcd.print("S");
      }

      lcd.setCursor(0, 0); lcd.print(sumbuX);
      lcd.print(" T"); lcd.print((int)Theta); lcd.print(" B");
      lcd.print(jarakbe); lcd.print(" ");

      lcd.setCursor(0, 1); lcd.print("L");
      lcd.print(jarakki); lcd.print(" R");
      lcd.print(jarakka); lcd.print(" D");
      lcd.print(jarakde); lcd.print("  ");

      countPos = 0;
    }
    else {
      dribble(255);
      motor(kir, kan, bel);
    }

    lcd.setCursor(0, 3);
    lcd.print(kir); lcd.print(" ");
    lcd.print(kan); lcd.print(" ");
    lcd.print(bel); lcd.print(" ");
    lcd.print(countPos); lcd.print("   ");
  }
  }

  void keTitikLapangan0(int sX, int jarakdepan) {
  int jarakbe, jarakki, jarakde, jarakka, pwmX, pwmY, sumbuX, sumbuY;

  if      (Theta > 10 && Theta <= 180) {
    dribble(255);
    motor(40, 40, 40);
    lcd.setCursor (0, 2);
    lcd.print("l");
  }
  else if (Theta < -10 && Theta >= -180) {
    dribble(255);
    motor(-40, -40, -40);
    lcd.setCursor (0, 2);
    lcd.print("r");
    kir = kan = bel = 0;
    countPos = 21;
  }
  else {
    countPos++;
    if (countPos > 30) {
      jarakki = jarki();
      jarakka = jarka();
      jarakbe = jarBeL() + jarBeR() / 2;
      jarakde = jarDeL() + jarDeR() / 2;

      sumbuX = (jarakki) - (jarakka + sX);
      sumbuY = jarakbe - jarakdepan;

      if (pwmX > 255) {
        pwmX = 255;
      } if (pwmX < -255) {
        pwmX = -255;
      }
      if (pwmY > 255) {
        pwmY = 255;
      } if (pwmY < -255) {
        pwmY = -255;
      }

      if (jarakbe > 330 && jarakka > 100 && jarakki > 100) {
        dribble(255);
        kir = 140;
        kan = -140;
        bel = 0;
        tengahX = 0;
        tengahY = 0;
        lcd.print("B");
      }
      else if (jarakbe > 330 && jarakka > 100 && jarakki < 100) {
        dribble(255);
        kir = 70;
        kan = -140;
        bel = 70;
        tengahX = 0;
        tengahY = 0;
        lcd.print("B");
      }
      else if (jarakde > 330 && jarakka < 100 && jarakki > 100) {
        dribble(255);
        kir = 140;
        kan = -70;
        bel = -70;
        tengahX = 0;
        tengahY = 0;
        lcd.print("B");
      }
      else if (jarakde > 330 && jarakka < 100 && jarakki < 100) {
        dribble(255);
        kir = 140;
        kan = -70;
        bel = -70;
        tengahX = 0;
        tengahY = 0;
        lcd.print("B");
      }
      else if (jarakbe < 150 && jarakde > 150) {
        dribble(10);
        kir = -140;
        kan = 140;
        bel = 0;
        tengahX = 0;
        tengahY = 1;
        lcd.print("F");
      }

      else if (jarakde > 330 && jarakka > 250 && jarakki < 250) {
        dribble(255);
        kir = 50;
        kan = -100;
        bel = 50;
        tengahX = 0;
        tengahY = 0;
        lcd.print("B");
      }
      else if (jarakde > 330 && jarakka < 250 && jarakki > 250) {
        dribble(255);
        kir = 100;
        kan = -50;
        bel = -50;
        tengahX = 0;
        tengahY = 0;
        lcd.print("B");
      }

      else if (jarakbe < 30) {
        dribble(10);
        kir = -150;
        kan = 150;
        bel = 0;
        tengahX = 0;
        tengahY = 1;
        lcd.print("F");
      }
      else if (jarakde < jarakdepan + 10 && jarakde < 10) {
        dribble(255);
        kir = 100;
        kan = -100;
        bel = 0;
        tengahX = 0;
        tengahY = 0;
        lcd.print("B");
      }
      else if (jarakde > jarakdepan + 10) {
        dribble(10);
        kir = -150;
        kan = 150;
        bel = 0;
        tengahX = 0;
        tengahY = 1;
        lcd.print("F");
      }
      else if (jarakde < jarakdepan - 10) {
        dribble(255);
        kir = 100;
        kan = -100;
        bel = 0;
        tengahX = 0;
        tengahY = 0;
        lcd.print("B");
      }

      else if (jarakki > 200 && jarakka < 200) {
        dribble(255);
        kir = 50;
        kan = 50;
        bel = -100;
        tengahX = 0;
        tengahY = 1;
        lcd.print("L");
      }
      else if (jarakki < 200 && jarakka > 200) {
        dribble(255);
        kir = -50;
        kan = -50;
        bel = 100;
        tengahX = 0;
        tengahY = 1;
        lcd.print("R");
      }
      else if (jarakki < 200 && jarakka < 200 && sumbuX > sX + 70) {
        dribble(255);
        kir = 50;
        kan = 50;
        bel = -100;
        tengahX = 0;
        tengahY = 1;
        lcd.print("L");
      }
      else if (jarakki < 200 && jarakka < 200 && sumbuX < sX - 70) {
        dribble(255);
        kir = -50;
        kan = -50;
        bel = 100;
        tengahX = 0;
        tengahY = 1;
        lcd.print("R");
      }

      else {
        dribble(255);
        kir = 0;
        kan = 0;
        bel = 0;
        tengahX = 1;
        tengahY = 1;
        lcd.print("S");
      }

      lcd.setCursor(0, 0); lcd.print(sumbuX);
      lcd.print(" T"); lcd.print((int)Theta); lcd.print(" B");
      lcd.print(jarakbe); lcd.print(" ");

      lcd.setCursor(0, 1); lcd.print("L");
      lcd.print(jarakki); lcd.print(" R");
      lcd.print(jarakka); lcd.print(" D");
      lcd.print(jarakde); lcd.print("  ");

      countPos = 0;
    }
    else {
      dribble(255);
      motor(kir, kan, bel);
    }

    lcd.setCursor(0, 3);
    lcd.print(kir); lcd.print(" ");
    lcd.print(kan); lcd.print(" ");
    lcd.print(bel); lcd.print(" ");
    lcd.print(countPos); lcd.print("   ");
  }
  }

  void keTitikBelakang(int sX, int jarakbelakang) {
  int jarakbe, jarakki, jarakde, jarakka, pwmX, pwmY, sumbuX, sumbuY;

  if      (Theta > 10 && Theta <= 180) {
    dribble(255);
    motor(40, 40, 40);
    lcd.setCursor (0, 2);
    lcd.print("l");
  }
  else if (Theta < -10 && Theta >= -180) {
    dribble(255);
    motor(-40, -40, -40);
    lcd.setCursor (0, 2);
    lcd.print("r");
    kir = kan = bel = 0;
    countPos = 21;
  }
  else {
    countPos++;
    if (countPos > 40) {
      jarakki = jarki();
      jarakka = jarka();
      jarakbe = jarBeL() + jarBeR() / 2;
      jarakde = jarDeL() + jarDeR() / 2;

      sumbuX = (jarakki) - (jarakka + sX);
      sumbuY = jarakbe - jarakbelakang;

      if (pwmX > 255) {
        pwmX = 255;
      } if (pwmX < -255) {
        pwmX = -255;
      }
      if (pwmY > 255) {
        pwmY = 255;
      } if (pwmY < -255) {
        pwmY = -255;
      }

      if (jarakbe > jarakbelakang + 10 && jarakka > 100 && jarakki > 100) {
        dribble(255);
        kir = 60;
        kan = -60;
        bel = 0;
        tengahX = 0;
        tengahY = 0;
        lcd.print("B");
      }
      else if (jarakbe > jarakbelakang + 10 && jarakka > 100 && jarakki < 100) {
        dribble(255);
        kir = 30;
        kan = -60;
        bel = 30;
        tengahX = 0;
        tengahY = 0;
        lcd.print("B");
      }
      else if (jarakde > jarakbelakang + 10 && jarakka < 100 && jarakki > 100) {
        dribble(255);
        kir = 60;
        kan = -30;
        bel = -30;
        tengahX = 0;
        tengahY = 0;
        lcd.print("B");
      }
      else if (jarakbe < 150 && jarakde > 150) {
        dribble(10);
        kir = -60;
        kan = 60;
        bel = 0;
        tengahX = 0;
        tengahY = 1;
        lcd.print("F");
      }

      else if (jarakbe > jarakbelakang + 10 && jarakka > 200 && jarakki < 200) {
        dribble(255);
        kir = 30;
        kan = -60;
        bel = 30;
        tengahX = 0;
        tengahY = 0;
        lcd.print("B");
      }
      else if (jarakde > jarakbelakang + 10 && jarakka < 200 && jarakki > 200) {
        dribble(255);
        kir = 60;
        kan = -30;
        bel = -30;
        tengahX = 0;
        tengahY = 0;
        lcd.print("B");
      }

      else if (jarakbe < 10) {
        dribble(10);
        kir = -60;
        kan = 60;
        bel = 0;
        tengahX = 0;
        tengahY = 1;
        lcd.print("F");
      }
      else if (jarakbe > jarakbelakang - 10 && jarakde < 10) {
        dribble(255);
        kir = 60;
        kan = -60;
        bel = 0;
        tengahX = 0;
        tengahY = 0;
        lcd.print("B");
      }
      else if (jarakbe < jarakbelakang - 10) {
        dribble(10);
        kir = -60;
        kan = 60;
        bel = 0;
        tengahX = 0;
        tengahY = 1;
        lcd.print("F");
      }
      else if (jarakbe > jarakbelakang + 10) {
        dribble(255);
        kir = 60;
        kan = -60;
        bel = 0;
        tengahX = 0;
        tengahY = 0;
        lcd.print("B");
      }

      else if (jarakki > 200 && jarakka < 200) {
        dribble(255);
        kir = 30;
        kan = 60;
        bel = -60;
        tengahX = 0;
        tengahY = 1;
        lcd.print("L");
      }
      else if (jarakki < 200 && jarakka > 200) {
        dribble(255);
        kir = -30;
        kan = -30;
        bel = 60;
        tengahX = 0;
        tengahY = 1;
        lcd.print("R");
      }
      else if (jarakki < 200 && jarakka < 200 && sumbuX > sX + 80) {
        dribble(255);
        kir = 30;
        kan = 30;
        bel = -60;
        tengahX = 0;
        tengahY = 1;
        lcd.print("L");
      }
      else if (jarakki < 200 && jarakka < 200 && sumbuX < sX - 80) {
        dribble(255);
        kir = -30;
        kan = -30;
        bel = 60;
        tengahX = 0;
        tengahY = 1;
        lcd.print("R");
      }

      else {
        dribble(255);
        kir = 0;
        kan = 0;
        bel = 0;
        tengahX = 1;
        tengahY = 1;
        lcd.print("S");
      }

      lcd.setCursor(0, 0); lcd.print(sumbuX);
      lcd.print(" T"); lcd.print((int)Theta); lcd.print(" B");
      lcd.print(jarakbe); lcd.print(" ");

      lcd.setCursor(0, 1); lcd.print("L");
      lcd.print(jarakki); lcd.print(" R");
      lcd.print(jarakka); lcd.print(" D");
      lcd.print(jarakde); lcd.print("  ");

      countPos = 0;
    }
    else {
      dribble(255);
      motor(kir, kan, bel);
    }

    lcd.setCursor(0, 3);
    lcd.print(kir); lcd.print(" ");
    lcd.print(kan); lcd.print(" ");
    lcd.print(bel); lcd.print(" ");
    lcd.print(countPos); lcd.print("   ");
  }
  }

  void hitungKeTengahLapangan() {
  int kir, kan, bel;

  if      ((Theta > 180 + 10 && Theta <= 360) || (Theta > -180 + 10 && Theta < 0)) {
    motor(55, 55, 55);
    lcd.setCursor (15, 1);
    lcd.print("roL");
  }
  else if ((Theta < 180 - 10 && Theta >= 0) || (Theta < -180 - 10 && Theta >= -360)) {
    motor(-55, -55, -55);
    lcd.setCursor (15, 1);
    lcd.print("roR");
  }
  else {
    int jarKi = jarki();
    int jarKa = jarka();
    int jarBe = jarBeL();
    int jarDe = jarDeL();

    lcd.setCursor (15, 1);
    //    if(jarKi<jarKa-80){kir=-60; kan=-60; bel=120; lcd.print("toR");}
    //    else if(jarKa<jarKi-80){kir=60; kan=60; bel=-120; lcd.print("toL");}
    //    else {kir=0; kan=0; bel=0;}
    //
    //    if(jarBe>300){kir+=60; kan+=-60; lcd.print("Mun");}
    //    else if(jarBe<100 && jarDe>100){kir+=-60; kan+=60;  lcd.print("Maj");}
    //    else {kir=kir; kan=kan; lcd.print("Stp");}
    if (jarBe > 300) {
      kir = 100;
      kan = -100;
      lcd.print("Mun");
    }
    else if (jarBe < 100 && jarDe > 100) {
      kir = -100;
      kan = 100;
      lcd.print("Maj");
    }
    else {
      kir = kir;
      kan = kan;
      lcd.print("Stp");
    }

    motor(kir, kan, bel);
  }
  }

  void keTengahLapangan() {
  lcd.clear();
  while (!C()) {
    keTitikLapangan180(300, 350);
  }
  lcd.clear(); berhenti();
  }

  void testKeBelakang() {
  lcd.clear();
  while (!C()) {
    keTitikLapangan0(300, 150);
  }
  lcd.clear(); berhenti();
  }
  void testKeDepan() {
  lcd.clear();
  while (!C()) {
    keTitikBelakang(300, 250);
  }
  lcd.clear(); berhenti();
  }

  void tendangJarakDekat() {
  dribble(255);
  if ((Theta > 5 && Theta <= 180) || Theta < -180) {
    motor(55, 55, 65);
    lcd.setCursor(13, 1);
    lcd.print("roL");
  }
  else if ((Theta < -5 && Theta >= -180) || Theta > 180) {
    motor(-55, -55, -65);
    lcd.setCursor(13, 1);
    lcd.print("roR");
  }
  else {
    int jarKi = jarki();
    int jarKa = jarka();
    int jarDe = jarDeL();
    int jarBe = jarBeL();

    dribble(255);
    if (jarKi > 250 && jarKa > 250) {
      lcd.setCursor(13, 1); lcd.print("Cen");
      tendang();
    }
    else if (jarKi < 250 && jarKa > 250) {
      lcd.setCursor(13, 1); lcd.print("Kir");
      if (jarKi < 150) {
        motor(-55, -65, -65); delay(70); motor(0, 0, 0);
        tendang();
      }
      else {
        motor(-55, -65, -65); delay(50); motor(0, 0, 0);
        tendang();
      }
    }
    else if (jarKi > 250 && jarKa < 250) {
      lcd.setCursor(13, 1); lcd.print("Kan");
      if (jarKi < 150) {
        motor(65, 55, 65); delay(70); motor(0, 0, 0);
        tendang();
      }
      else {
        motor(65, 55, 65); delay(50); motor(0, 0, 0);
        tendang();
      }
    }
    else {
      if (jarKi < jarKa) {
        motor(-55, -65, -65); delay(200); motor(0, 0, 0);
        tendang();
      }
      else {
        motor(65, 55, 65); delay(200); motor(0, 0, 0);
        tendang();
      }
    }

  }
  }


*/
