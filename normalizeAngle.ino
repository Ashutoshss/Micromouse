class SimplePID{
  private:
    float kp, kd, ki,umax;  // kp,kd,ki are the gain terms,
    float eprev, eintegral;  // store the previous error and the sum of the previous error that is the integral

  public:
    // Constructor
    AngularPID() : kp(1), kd(0), ki(0), umax(255), eprev(0.0), eintegral(0.0){}

  void setParams(float kpIn, float kiIn, float kdIn, float uMaxIn){
    //this method spwap the default values with the user defined values
    kp = kpIn;
    ki = kiIn;
    kd = kdIn;
    umax = uMaxIn;
  }

  // A function to compute the control signal
  void evalu(int value, int target, float deltaT, int &pwr, int &dir){
    // error
    int e = target - value;
  
    // derivative
    float dedt = (e-eprev)/(deltaT);
  
    // integral
    eintegral = eintegral + e*deltaT;
  
    // control signal
    float u = kp*e + kd*dedt + ki*eintegral;
  
    // motor power
    pwr = (int) fabs(u);
    if( pwr > umax ){
      pwr = umax;
    }
  
    // motor direction
    dir = 1;
    if(u<0){
      dir = -1;
    }
  
    // store previous error
    eprev = e;
  }
};

SimplePID Angular;

void setup() {
  
  Angular.setParams(1,0,0,255);
}

void loop() {
  
}

