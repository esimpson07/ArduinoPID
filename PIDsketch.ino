#define ena D3
#define in1 D4
#define in2 D5
#define enc1 D1
#define enc2 D2
byte encRaw1 = 0;
byte encRaw2 = 0;
bool encFlag = false;
double count = 0;
double ppr = 1176;
double mDegrees = 0;
double mSpeed = 0;

double retVal = 0;
double ttime = 0;
double previousTime = 0;
double timeInterval = 0;
double integral = 0;
double derivative = 0;
double setPoint = 0;
double error = 0;
double previousError = 0;
double P = 0;
double I = 0;
double D = 0;
double val = 0;

void ICACHE_RAM_ATTR encCount() {
  if(digitalRead(enc1) > digitalRead(enc2)){
    count --;
  } else if(digitalRead(enc1) < digitalRead(enc2)){
    count ++;
  }
  mDegrees = 360 * (count / ppr);
}

void setup() {
  pinMode(ena,OUTPUT);
  pinMode(in1,OUTPUT);
  pinMode(in2,OUTPUT);
  pinMode(enc1,INPUT);
  pinMode(enc2,INPUT);
  attachInterrupt(digitalPinToInterrupt(enc1), encCount, RISING);
  attachInterrupt(digitalPinToInterrupt(enc2), encCount, RISING);
  PIDController(0.0912,0.00002,0.00001);
  Serial.begin(115200);
}

void PIDController(double P, double I, double D) {
  setPID(P,I,D);
  setSetPoint(720);
}

void setPID(double a, double b, double c) {
  P = a;
  I = b;
  D = c;
}

void setSetPoint(double value){
  setPoint = value;
}

double calculate(double actual) {
  ttime = millis();
  
  timeInterval = ttime - previousTime;

  if(timeInterval > 2) {
    error = (setPoint - actual);
    integral += (error * timeInterval);
    derivative = (error - previousError) / timeInterval;
    retVal = (P * error) + (I * integral) + (D * derivative);
    previousError = error;
    previousTime = ttime;
  }

  return(retVal);
}

void drive(double mSpeed) {
  if(mSpeed > 100){
    mSpeed = 100;
  } else if(mSpeed < -100){
    mSpeed = -100;
  }
  if(mSpeed > 0 ) {
    analogWrite(ena,(int)(mSpeed * 2.55));
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
  } else if(mSpeed < 0) {
    analogWrite(ena,(int)(mSpeed * 2.55));
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
  } else {
    analogWrite(ena,0);
    digitalWrite(in1,HIGH);
    digitalWrite(in2,HIGH);
  }
}

void loop() {
  Serial.println(mDegrees);
  drive(calculate(mDegrees));
}
