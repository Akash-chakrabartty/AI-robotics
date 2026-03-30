// ============================================================
//  R O B O T   F I R M W A R E   v4.9
// ============================================================

#include <Servo.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT  64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire);

const int ENA=5,ENB=6,IN1=7,IN2=8,IN3=9,IN4=10;
const int SERVO_PIN=3,TRIG_PIN=A0,ECHO_PIN=A1;
const int IR_LEFT=4,IR_RIGHT=A2,IR_BACK=12;
const int BUZZER=11,VBAT_PIN=A3,MODE_BTN=2;

volatile bool btnPressed=false;
unsigned long lastBtnTime=0;
const unsigned long DEBOUNCE=1000;
void onModeBtn(){btnPressed=true;}

const float VBAT_FULL=8.4f,VBAT_LOW=6.6f,VBAT_DIVIDER=2.0f;
float batteryVolts=0.0f;
int   batteryPct=100;
bool  batteryLow=false;
unsigned long lastVbatRead=0,lastLowBeep=0;
const int VBAT_READ_MS=2000,LOW_BEEP_MS=5000;

const float LEFT_FACTOR=1.0f,RIGHT_FACTOR=1.37f;
Servo radarServo;

char  command='S';
bool  autoMode=false,lineMode=false,scanMode=false,distMode=false;
int   speedValue=110,lastSeen=0;
bool  diagMode=false,obstacleBeep=true,rearGuard=false,cruiseBoost=false;

enum AutoState{AUTO_FORWARD,AUTO_BACKUP,AUTO_TURN};
AutoState autoState=AUTO_FORWARD;
unsigned long autoStateStart=0;
int turnDirection=0;
const int STOP_DIST=25;

const int SERVO_CENTER=90,AUTO_SWEEP_MIN=30,AUTO_SWEEP_MAX=150;
const int LINE_SWEEP_MIN=75,LINE_SWEEP_MAX=105;
int servoAngle=SERVO_CENTER,servoStep=10;
unsigned long lastServoTime=0;

unsigned long lastPingTime=0;
long currentDist=999;

enum FaceID{
  FACE_IDLE,FACE_HAPPY,FACE_FROWN,FACE_ANGRY,
  FACE_PANIC_FAST,FACE_SPOOKED,FACE_SMIRK,
  FACE_DEAD,FACE_SEARCHING,FACE_LOWBAT
};
FaceID currentFace=FACE_IDLE;
unsigned long faceChangedAt=0;
int searchFrame=0;
unsigned long searchFrameTime=0;
unsigned long lastDisplayUpdate=0;
const int DISPLAY_MS=60;


// ── startup beep — short rising tones only ───────────────────
void playCarRev(){
  int notes[]={200,300,450,650,900,650};
  int durs[] ={ 60, 55, 55, 60, 90, 80};
  for(int i=0;i<6;i++){tone(BUZZER,notes[i],durs[i]);delay(durs[i]+20);}
  noTone(BUZZER);
}

// ── mode chime ────────────────────────────────────────────────
void playModeChime(int m){
  switch(m){
    case 0:tone(BUZZER,600,80);delay(100);break;
    case 1:tone(BUZZER,800,70);delay(100);tone(BUZZER,1100,70);delay(80);break;
    case 2:tone(BUZZER,900,55);delay(75);tone(BUZZER,1100,55);delay(75);tone(BUZZER,1300,55);delay(80);break;
    case 3:tone(BUZZER,1000,55);delay(70);tone(BUZZER,1200,55);delay(70);tone(BUZZER,1000,55);delay(80);break;
    case 4:tone(BUZZER,1400,80);delay(100);break;
  }
  noTone(BUZZER);
}

// ── battery ───────────────────────────────────────────────────
void readBattery(){
  int raw=analogRead(VBAT_PIN);
  batteryVolts=(raw/1023.0f)*5.0f*VBAT_DIVIDER;
  batteryPct=(int)(((batteryVolts-VBAT_LOW)/(VBAT_FULL-VBAT_LOW))*100.0f);
  batteryPct=constrain(batteryPct,0,100);
  batteryLow=(batteryVolts<VBAT_LOW+0.3f);
}

void drawBatteryIcon(int x,int y){
  display.drawRect(x,y,22,10,WHITE);
  display.fillRect(x+22,y+3,3,4,WHITE);
  int w=constrain((int)(18.0f*batteryPct/100.0f),0,18);
  if(w>0)display.fillRect(x+2,y+2,w,6,WHITE);
  if(batteryLow){display.drawLine(x+2,y+2,x+19,y+8,WHITE);display.drawLine(x+2,y+8,x+19,y+2,WHITE);}
}

// ── motors ────────────────────────────────────────────────────
void setMotors(int sL,int sR,bool d1,bool d2,bool d3,bool d4){
  analogWrite(ENA,constrain((int)(sL*LEFT_FACTOR),0,255));
  analogWrite(ENB,constrain((int)(sR*RIGHT_FACTOR),0,255));
  digitalWrite(IN1,d1);digitalWrite(IN2,d2);digitalWrite(IN3,d3);digitalWrite(IN4,d4);
}
void moveForward(int s){setMotors(s,s,LOW,HIGH,LOW,HIGH);}
void moveBackward(int s){setMotors(s,s,HIGH,LOW,HIGH,LOW);}
void turnLeft(int s){setMotors(s,s,HIGH,LOW,LOW,HIGH);}
void turnRight(int s){setMotors(s,s,LOW,HIGH,HIGH,LOW);}
void pivotLeft(int s){setMotors(0,s,LOW,LOW,LOW,HIGH);}
void pivotRight(int s){setMotors(s,0,LOW,HIGH,LOW,LOW);}
void pivotBackLeft(int s){setMotors(0,s,LOW,LOW,HIGH,LOW);}
void pivotBackRight(int s){setMotors(s,0,HIGH,LOW,LOW,LOW);}
void stopBot(){setMotors(0,0,LOW,LOW,LOW,LOW);}

// ── ultrasonic ────────────────────────────────────────────────
long readUltrasonic(){
  digitalWrite(TRIG_PIN,LOW);delayMicroseconds(2);
  digitalWrite(TRIG_PIN,HIGH);delayMicroseconds(10);
  digitalWrite(TRIG_PIN,LOW);
  long dur=pulseIn(ECHO_PIN,HIGH,20000);
  return dur==0?999:dur*0.034/2;
}

// ── servo ─────────────────────────────────────────────────────
void sweepRadar(int mn,int mx,int spd){
  unsigned long now=millis();
  if(now-lastServoTime>spd){
    lastServoTime=now;servoAngle+=servoStep;
    if(servoAngle>=mx){servoAngle=mx;servoStep=-abs(servoStep);}
    if(servoAngle<=mn){servoAngle=mn;servoStep=abs(servoStep);}
    radarServo.write(servoAngle);
  }
}
void parkServo(){
  if(servoAngle!=SERVO_CENTER){servoAngle=SERVO_CENTER;radarServo.write(SERVO_CENTER);}
}

// ── object width scan (slope-based for SR04 cone blur) ────────
// ── sample distance: average of 3 readings ───────────────────
long sampleDist(){
  long s=0;for(int i=0;i<3;i++){s+=readUltrasonic();delay(10);}
  return s/3;
}

// ── crawl servo one direction until the distance inflection ──
// Returns the angle of the steepest slope change (true edge).
// dir=+1 crawl right, dir=-1 crawl left. Starts from startAng.
// OLED shows live angle+dist while crawling.
float crawlToEdge(int startAng, int dir){
  const int   STEP     = 1;    // 1 deg per step — slow and precise
  const int   SETTLE   = 70;   // ms to settle after each move
  const int   MAX_STEPS= 80;   // don't wander more than 80 deg
  const float MIN_SLOPE= 4.0f; // ignore tiny noise

  long  prev = 0, curr = 0;
  float bestSlope = 0;
  int   bestAng   = startAng;
  int   ang       = startAng;

  radarServo.write(ang); delay(SETTLE);
  prev = sampleDist();

  for(int s=0; s<MAX_STEPS; s++){
    ang += dir * STEP;
    ang = constrain(ang, 30, 150);
    radarServo.write(ang); delay(SETTLE);
    curr = sampleDist();

    float slope = (float)(curr - prev) * dir; // positive = moving away from object
    if(slope > bestSlope){ bestSlope = slope; bestAng = ang; }

    // Live OLED feedback
    display.clearDisplay();display.setTextSize(1);display.setTextColor(WHITE);
    display.setCursor(0,0);display.print(F("Scanning..."));
    display.setCursor(0,12);display.print(ang);display.print(F("deg "));
    display.print(curr);display.println(F("cm"));
    display.setCursor(0,24);display.print(F("slope:"));display.print(slope,1);
    display.setCursor(0,36);display.print(F("best:"));display.print(bestAng);display.print(F("d"));
    display.display();

    prev = curr;

    // Stop early once we've clearly passed the peak and slope is falling
    if(bestSlope > MIN_SLOPE && slope < bestSlope * 0.3f) break;
  }
  return bestSlope >= MIN_SLOPE ? (float)bestAng : -1;
}

// ── main measure function: 3 passes, averaged ────────────────
void scanObjectWidth(){
  stopBot();
  Serial.println(F("MEAS start"));

  const int PASSES = 3;
  float widths[PASSES];
  int   good = 0;

  for(int p=0; p<PASSES; p++){
    // Start from centre each pass
    int centre = SERVO_CENTER;
    radarServo.write(centre); servoAngle=centre; delay(300);

    // Show pass number
    display.clearDisplay();display.setTextSize(1);display.setTextColor(WHITE);
    display.setCursor(0,0);display.print(F("Pass "));display.print(p+1);
    display.print(F("/"));display.println(PASSES);
    display.display(); delay(200);

    // Crawl LEFT to find edge A
    float angA = crawlToEdge(centre, -1);
    if(angA < 0){ Serial.println(F("no left edge")); continue; }
    long dA = sampleDist();
    tone(BUZZER, 1000, 60); delay(80); // beep: found edge

    // Go back to centre
    radarServo.write(centre); delay(300);

    // Crawl RIGHT to find edge B
    float angB = crawlToEdge(centre, +1);
    if(angB < 0){ Serial.println(F("no right edge")); continue; }
    long dB = sampleDist();
    tone(BUZZER, 1200, 60); delay(80); // beep: found edge

    // Sanity: edges must be on opposite sides
    if(angA >= angB){ Serial.println(F("edges crossed")); continue; }

    float theta = (angB - angA) * PI / 180.0f;
    float W = sqrt((float)(dA*dA + dB*dB) - 2.0f*dA*dB*cos(theta));
    widths[good++] = W;

    Serial.print(F("Pass "));Serial.print(p+1);
    Serial.print(F(": A="));Serial.print(angA,1);
    Serial.print(F("d B="));Serial.print(angB,1);
    Serial.print(F("d dA="));Serial.print(dA);
    Serial.print(F(" dB="));Serial.print(dB);
    Serial.print(F(" W="));Serial.print(W,1);Serial.println(F("cm"));

    radarServo.write(SERVO_CENTER); servoAngle=SERVO_CENTER;
    delay(400);
  }

  if(good == 0){ Serial.println(F("MEAS failed")); return; }

  // Average valid passes
  float sum=0; for(int i=0;i<good;i++) sum+=widths[i];
  float avg = sum/good;

  // Std dev for confidence
  float sq=0; for(int i=0;i<good;i++) sq+=(widths[i]-avg)*(widths[i]-avg);
  float sd = good>1 ? sqrt(sq/(good-1)) : 0;

  Serial.print(F("RESULT: "));Serial.print(avg,1);
  Serial.print(F("cm  sd="));Serial.print(sd,1);
  Serial.print(F("cm  passes="));Serial.println(good);

  // Final OLED display — stays until next button press
  display.clearDisplay();display.setTextSize(1);display.setTextColor(WHITE);
  display.setCursor(0,0);display.println(F("=== RESULT ==="));
  display.setTextSize(2);
  display.setCursor(0,14);display.print(avg,1);display.println(F("cm"));
  display.setTextSize(1);
  display.setCursor(0,36);display.print(F("sd: "));display.print(sd,1);display.println(F("cm"));
  display.setCursor(0,46);display.print(good);display.print(F("/"));display.print(PASSES);display.println(F(" passes OK"));
  display.setCursor(0,56);display.print(F("btn=next mode"));
  display.display();

  tone(BUZZER,800,60);delay(80);tone(BUZZER,1000,60);delay(80);tone(BUZZER,1200,80);
}

// ── scan/dist display (shared function saves flash) ───────────
void showSensorDisplay(){
  display.clearDisplay();display.setTextSize(1);display.setTextColor(WHITE);
  display.setCursor(0,1);display.print(batteryVolts,1);display.print(F("V"));
  display.setCursor(36,1);
  if(scanMode)display.print(F("[SCAN]"));
  else        display.print(F("[DIST]"));
  drawBatteryIcon(100,1);

  display.setTextSize(3);
  if(currentDist>=999){display.setCursor(20,20);display.print(F("---"));}
  else{display.setCursor(currentDist<100?28:10,20);display.print(currentDist);display.print(F("cm"));}

  // proximity bar
  long bFill=map(constrain(currentDist,0,200),200,0,0,110);
  display.drawRect(8,56,110,6,WHITE);
  if(bFill>0)display.fillRect(8,56,(int)bFill,6,WHITE);

  if(distMode){
    long bFill=map(constrain(currentDist,0,200),200,0,0,110);
    display.drawRect(8,56,110,6,WHITE);
    if(bFill>0)display.fillRect(8,56,(int)bFill,6,WHITE);
  }
  display.display();
}

// ── apply mode ────────────────────────────────────────────────
void applyMode(int m){
  stopBot();
  autoMode=lineMode=scanMode=distMode=false;
  switch(m){
    case 0:command='S';currentFace=FACE_DEAD;parkServo();Serial.println(F("[M]MAN"));break;
    case 1:autoMode=true;autoState=AUTO_FORWARD;servoAngle=SERVO_CENTER;radarServo.write(SERVO_CENTER);currentFace=FACE_SEARCHING;Serial.println(F("[M]AUTO"));break;
    case 2:lineMode=true;lastSeen=0;parkServo();currentFace=FACE_HAPPY;Serial.println(F("[M]LINE"));break;
    case 3:scanMode=true;parkServo();currentFace=FACE_SEARCHING;Serial.println(F("[M]SCAN"));break;
    case 4:distMode=true;parkServo();currentFace=FACE_IDLE;Serial.println(F("[M]DIST"));break;
  }
  faceChangedAt=millis();
  playModeChime(m);
  if(diagMode)showDiag();
  else if(distMode)showSensorDisplay();
  else if(scanMode)scanObjectWidth();  // run immediately on entry
  else drawFace();
}

int currentModeIndex(){
  if(autoMode)return 1;if(lineMode)return 2;
  if(scanMode)return 3;if(distMode)return 4;
  return 0;
}

// ── face selector ─────────────────────────────────────────────
void updateFace(long dist){
  if(scanMode||distMode)return;
  FaceID n=currentFace;
  if(batteryLow)n=FACE_LOWBAT;
  else if(dist==999)n=FACE_PANIC_FAST;
  else if(digitalRead(IR_BACK)==LOW)n=FACE_SPOOKED;
  else if(autoMode){
    if(autoState==AUTO_BACKUP)n=FACE_ANGRY;
    else if(autoState==AUTO_TURN)n=FACE_FROWN;
    else if(dist<STOP_DIST+15)n=FACE_SPOOKED;
    else n=FACE_SEARCHING;
  }else if(lineMode){
    if(dist<STOP_DIST)n=FACE_SPOOKED;
    else if(lastSeen==0)n=FACE_HAPPY;
    else n=FACE_FROWN;
  }else{
    switch(command){
      case 'F':case 'G':case 'H':
        n=(speedValue<120)?FACE_HAPPY:(speedValue<180)?FACE_FROWN:FACE_ANGRY;break;
      case 'B':case 'I':case 'J':n=FACE_ANGRY;break;
      case 'L':case 'R':n=FACE_SMIRK;break;
      case 'S':n=FACE_DEAD;break;
      default:n=FACE_IDLE;break;
    }
  }
  if(n!=currentFace){currentFace=n;faceChangedAt=millis();}
}

// ── draw face ─────────────────────────────────────────────────
void drawFace(){
  display.clearDisplay();
  display.setTextSize(1);display.setTextColor(WHITE);
  display.setCursor(0,1);display.print(batteryVolts,1);display.print(F("V"));
  display.setCursor(36,1);
  if(autoMode)display.print(F("[AUTO]"));
  else if(lineMode)display.print(F("[LINE]"));
  else display.print(F("[MAN] "));
  drawBatteryIcon(100,1);
  switch(currentFace){
    case FACE_IDLE:
      display.fillRect(20,26,88,6,WHITE);display.fillRect(20,36,88,6,WHITE);display.fillRect(20,46,88,6,WHITE);break;
    case FACE_DEAD:
      display.fillRect(14,36,100,5,WHITE);break;
    case FACE_HAPPY:
      display.fillRect(18,18,12,34,WHITE);display.fillRect(98,18,12,34,WHITE);
      display.fillRect(18,48,92,10,WHITE);display.fillRect(30,18,68,30,BLACK);break;
    case FACE_FROWN:
      display.fillRect(18,28,12,34,WHITE);display.fillRect(98,28,12,34,WHITE);display.fillRect(18,18,92,10,WHITE);break;
    case FACE_PANIC_FAST:{
        int p=((millis()-faceChangedAt)/40)%3,pad=p*5;
        int ox=constrain(30-pad,0,30),oy=constrain(16-pad,14,20),ow=68+pad*2,oh=44+pad*2;
        display.fillRect(ox,oy,ow,oh,WHITE);display.fillRect(ox+8,oy+8,ow-16,oh-16,BLACK);}break;
    case FACE_SPOOKED:
      display.fillRect(16,18,40,38,WHITE);display.fillRect(72,18,40,38,WHITE);
      display.fillRect(28,28,16,18,BLACK);display.fillRect(84,28,16,18,BLACK);break;
    case FACE_ANGRY:
      display.fillRect(8,18,20,14,WHITE);display.fillRect(26,40,20,14,WHITE);
      display.fillRect(54,18,20,14,WHITE);display.fillRect(82,40,20,14,WHITE);display.fillRect(100,18,20,14,WHITE);
      display.fillRect(26,30,8,12,WHITE);display.fillRect(44,30,12,12,WHITE);
      display.fillRect(72,30,12,12,WHITE);display.fillRect(98,30,8,12,WHITE);break;
    case FACE_SMIRK:
      display.fillRect(10,40,44,10,WHITE);display.fillRect(60,32,18,10,WHITE);
      display.fillRect(78,22,18,10,WHITE);display.fillRect(96,14,20,10,WHITE);display.fillRect(58,32,6,18,WHITE);break;
    case FACE_SEARCHING:
      display.fillRect(10,50,108,8,WHITE);
      {unsigned long n=millis();
       if(n-searchFrameTime>60){searchFrameTime=n;searchFrame++;if(searchFrame>17)searchFrame=0;}
       int fr=searchFrame>8?17-searchFrame:searchFrame,dx=14+fr*12;
       display.fillRect(dx,32,12,12,WHITE);
       if(dx-14>=14)display.fillRect(dx-14,35,8,6,WHITE);}break;
    case FACE_LOWBAT:{
        int ph=(int)((millis()/28)%108),bw=108-ph;
        display.fillRect(10,34,bw,12,WHITE);
        if(bw<40){display.setTextSize(2);display.setCursor(86,28);display.print(F("!!"));}}break;
  }
  display.display();
}

// ── line follow (exact working version) ──────────────────────
void doLineFollow(){
  if(currentDist>0&&currentDist<STOP_DIST){
    stopBot();if(obstacleBeep){tone(BUZZER,900,80);delay(100);tone(BUZZER,600,80);}return;
  }
  int L=digitalRead(IR_LEFT),R=digitalRead(IR_RIGHT);
  const int DRIVE_PWM=135,TURN_PWM=125,BURST_MS=50;
  const float LN=0.852f,RB=1.05f;
  auto lf=[&](int p){
    analogWrite(ENA,constrain((int)(p*LEFT_FACTOR*LN),0,255));
    analogWrite(ENB,constrain((int)(p*RIGHT_FACTOR*RB),0,255));
    digitalWrite(IN1,LOW);digitalWrite(IN2,HIGH);digitalWrite(IN3,LOW);digitalWrite(IN4,HIGH);
  };
  if(L==HIGH&&R==LOW){pivotLeft(TURN_PWM);delay(BURST_MS);stopBot();lastSeen=1;}
  else if(L==LOW&&R==HIGH){pivotRight(TURN_PWM);delay(BURST_MS);stopBot();lastSeen=2;}
  else if(L==LOW&&R==LOW){
    if(lastSeen==1){pivotLeft(TURN_PWM);delay(BURST_MS);}
    else if(lastSeen==2){pivotRight(TURN_PWM);delay(BURST_MS);}
    else{lf(DRIVE_PWM);delay(BURST_MS);}
    stopBot();
  }else{lf(DRIVE_PWM);delay(BURST_MS);stopBot();}
}

// ── autonomous ────────────────────────────────────────────────
void doAutonomous(){
  unsigned long now=millis();
  switch(autoState){
    case AUTO_FORWARD:
      sweepRadar(AUTO_SWEEP_MIN,AUTO_SWEEP_MAX,30);
      if(currentDist>0&&currentDist<STOP_DIST){
        stopBot();if(obstacleBeep)tone(BUZZER,700,80);
        turnDirection=(servoAngle>90)?-1:1;
        radarServo.write(SERVO_CENTER);autoState=AUTO_BACKUP;autoStateStart=now;
      }else moveForward(cruiseBoost?min(speedValue+30,210):speedValue);
      break;
    case AUTO_BACKUP:
      if(digitalRead(IR_BACK)==LOW){stopBot();if(obstacleBeep)tone(BUZZER,1200,150);autoState=AUTO_TURN;autoStateStart=now;}
      else moveBackward(speedValue);
      if(now-autoStateStart>350){stopBot();autoState=AUTO_TURN;autoStateStart=now;}
      break;
    case AUTO_TURN:
      if(turnDirection>0)turnLeft(speedValue);else turnRight(speedValue);
      if(now-autoStateStart>400){stopBot();autoState=AUTO_FORWARD;}
      break;
  }
}

// ── diagnostic display ────────────────────────────────────────
void showDiag(){
  display.clearDisplay();display.setTextSize(1);display.setTextColor(WHITE);display.setCursor(0,0);
  display.print(F("BAT:"));display.print(batteryVolts,1);display.print(F("V "));display.print(batteryPct);display.println(F("%"));
  if(lineMode){
    display.println(F("LINE"));
    display.print(F("L:"));display.print(digitalRead(IR_LEFT)==HIGH?F("BLK"):F("wht"));
    display.print(F(" R:"));display.println(digitalRead(IR_RIGHT)==HIGH?F("BLK"):F("wht"));
    display.print(F("DIST:"));display.print(currentDist);display.println(F("cm"));
    display.print(F("SEEN:"));
    if(lastSeen==0)display.println(F("C"));
    else if(lastSeen==1)display.println(F("L"));
    else display.println(F("R"));
  }else if(autoMode){
    display.println(F("AUTO"));
    display.print(F("D:"));display.print(currentDist);display.print(F("cm S:"));display.print(servoAngle);display.println(F("d"));
    display.print(F("ST:"));
    if(autoState==AUTO_FORWARD)display.println(F("FWD"));
    else if(autoState==AUTO_BACKUP)display.println(F("BCK"));
    else display.println(F("TRN"));
  }else{
    display.println(F("MAN"));
    display.print(F("CMD:"));display.print(command);display.print(F(" SPD:"));display.println(speedValue);
    display.print(F("D:"));display.print(currentDist);display.println(F("cm"));
  }
  display.setCursor(0,56);
  display.print(obstacleBeep?F("BP:Y"):F("BP:n"));
  display.print(rearGuard?F(" GD:Y"):F(" GD:n"));
  display.print(cruiseBoost?F(" BS:Y"):F(" BS:n"));
  display.display();
}

// ════════════════════════════════════════════════════════════
//  SETUP
// ════════════════════════════════════════════════════════════
void setup(){
  Serial.begin(9600);
  pinMode(ENA,OUTPUT);pinMode(ENB,OUTPUT);pinMode(IN1,OUTPUT);pinMode(IN2,OUTPUT);
  pinMode(IN3,OUTPUT);pinMode(IN4,OUTPUT);pinMode(TRIG_PIN,OUTPUT);pinMode(ECHO_PIN,INPUT);
  pinMode(IR_LEFT,INPUT);pinMode(IR_RIGHT,INPUT);pinMode(IR_BACK,INPUT);pinMode(BUZZER,OUTPUT);
  pinMode(MODE_BTN,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(MODE_BTN),onModeBtn,FALLING);
  radarServo.attach(SERVO_PIN);radarServo.write(SERVO_CENTER);servoAngle=SERVO_CENTER;

  if(!display.begin(SSD1306_SWITCHCAPVCC,0x3C)){Serial.println(F("OLED ERR"));while(true);}
  display.clearDisplay();display.setTextColor(WHITE);
  readBattery();
  display.setTextSize(2);display.setCursor(14,8);display.println(F("ROBO v4.9"));
  display.setTextSize(1);display.setCursor(10,36);display.println(F("systems online..."));
  display.setCursor(10,50);display.print(F("BAT:"));display.print(batteryVolts,1);display.println(F("V"));
  display.display();

  playCarRev();
  if(batteryLow){tone(BUZZER,400,200);delay(250);tone(BUZZER,400,200);}

  currentFace=FACE_HAPPY;faceChangedAt=millis();drawFace();delay(600);
  currentFace=FACE_IDLE;faceChangedAt=millis();

  Serial.println(F("ROBO v4.9 | MAN>AUTO>LINE>SCAN>DIST | W=width D=diag P=beep V=guard C=boost +/-=spd"));
}

// ════════════════════════════════════════════════════════════
//  MAIN LOOP
// ════════════════════════════════════════════════════════════
void loop(){
  unsigned long now=millis();

  if(now-lastPingTime>50){currentDist=readUltrasonic();lastPingTime=now;}

  if(now-lastVbatRead>VBAT_READ_MS){
    lastVbatRead=now;readBattery();
    Serial.print(F("BAT:"));Serial.print(batteryVolts,1);Serial.print(F("V "));Serial.println(batteryPct);
  }

  if(batteryLow&&now-lastLowBeep>LOW_BEEP_MS){lastLowBeep=now;tone(BUZZER,440,80);}

  if(btnPressed){
    btnPressed=false;
    if(now-lastBtnTime>DEBOUNCE){lastBtnTime=now;applyMode((currentModeIndex()+1)%5);}
  }

  if(Serial.available()){
    char c=toupper(Serial.read());command=c;
    switch(c){
      case 'A':applyMode(1);break;case 'T':applyMode(2);break;case 'M':applyMode(0);break;
      case 'W':scanObjectWidth();break;
      case 'D':diagMode=!diagMode;break;
      case 'P':obstacleBeep=!obstacleBeep;break;
      case 'V':rearGuard=!rearGuard;break;
      case 'C':cruiseBoost=!cruiseBoost;break;
      case '+':speedValue=min(speedValue+20,250);Serial.print(F("SPD:"));Serial.println(speedValue);break;
      case '-':speedValue=max(speedValue-20,80);Serial.print(F("SPD:"));Serial.println(speedValue);break;
      case 'F':case 'B':case 'L':case 'R':case 'S':
      case 'G':case 'H':case 'I':case 'J':break;
    }
  }

  if(rearGuard&&digitalRead(IR_BACK)==LOW){
    stopBot();tone(BUZZER,1200,50);delay(60);tone(BUZZER,1500,50);
    moveForward(255);delay(150);stopBot();command='S';
  }

  if(autoMode)doAutonomous();
  else if(lineMode)doLineFollow();
  else if(scanMode)parkServo();  // result already on OLED, just wait
  else{parkServo();
    if((command=='F'||command=='G'||command=='H')&&currentDist>0&&currentDist<STOP_DIST){
      stopBot();command='S';if(obstacleBeep)tone(BUZZER,800,50);
    }else switch(command){
      case 'F':moveForward(speedValue);break;case 'B':moveBackward(speedValue);break;
      case 'L':turnLeft(speedValue);break;case 'R':turnRight(speedValue);break;
      case 'G':pivotLeft(speedValue);break;case 'H':pivotRight(speedValue);break;
      case 'I':pivotBackLeft(speedValue);break;case 'J':pivotBackRight(speedValue);break;
      case 'S':stopBot();break;
    }
  }

  updateFace(currentDist);
  if(now-lastDisplayUpdate>DISPLAY_MS){
    lastDisplayUpdate=now;
    if(diagMode)showDiag();
    else if(distMode)showSensorDisplay();
    else if(scanMode){} // result stays on OLED until mode changes
    else drawFace();
  }
}
