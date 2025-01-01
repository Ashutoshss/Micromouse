class AngularPID{
  private:
    float kp, kd, ki,umax;  // kp,kd,ki are the gain terms,
    float eprev, eintegral;  // store the previous error and the sum of the previous error that is the integral

  public:
    // Constructor
    AngularPID() : kp(1), kd(0), ki(0), umax(255), eprev(0.0), eintegral(0.0){}
  void setParams(float )
}

void setup() {
  
}

void loop() {
  
}

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2){
  analogWrite(pwm,pwmVal);
  if(dir == 1){
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
  }
  else if(dir == -1){
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
  }
  else{
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);
  }  
}
