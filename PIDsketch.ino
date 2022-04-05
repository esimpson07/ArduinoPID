double retVal = 0;
double time = 0;
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

void setup() {
  PIDController(0.0912,0.0002,0.001);
  Serial.begin(115200);
  Serial.println("Starting");
}

void PIDController(double P, double I, double D) {
  setPID(P,I,D);
  setSetPoint(1000);
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
  time = millis();
  
  timeInterval = time - previousTime;

  if(timeInterval > 20) {
    error = (setPoint - actual);
    integral += (error * timeInterval);
    derivative = (error - previousError) / timeInterval;
    retVal = (P * error) + (I * integral) + (D * derivative);
    previousError = error;
    previousTime = time;
  }

  return(retVal);
}

void readSer() {
  if(Serial.available() > 0){
    val = Serial.read();
  }
}

void loop() {
  Serial.println(calculate(val));
}
