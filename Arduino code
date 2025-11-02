#include <Servo.h>

const int joystick_Lx_pin = A0; //Left joystick X pin
const int joystick_Ly_pin = A1; //Left joystick Y pin
const int joystick_Lsw_pin = 2;   // Left joystick press pin (digital)

const int joystick_By_pin = A2; //Right joystick Y pin 
const int joystick_Bsw_pin = 3;   // Right joystick press pin (digital)

//component pins
const int thrusterA_pin = 9;
const int thrusterB_pin = 10;
const int thrusterC_pin = 11;
const int servo0_pin   = 6; 

//calibration values
const int centerValue = 530;
const int deadZone = 20; //Ignore jitter in the middle

// servo orientation angles (can be adjusted)
const int servoHorizontal = 0;
const int servoVertical   = 90;

//components
Servo thrusterA;
Servo thrusterB;
Servo thrusterC;
Servo servo0;

// current servo orientation variable
int currentServoAngle = servoHorizontal;

//change angle function for simplicity
void servoChangeAngle(int targetAngle) {
  if (currentServoAngle != targetAngle) {
    servo0.write(targetAngle);
    delay(1000); // give time to physically move (MUST CALIBRATE !!!!!!)
    currentServoAngle = targetAngle;
  }
}

void setup() {
  Serial.begin(115200); //monitor controller values to check for any defected components

  pinMode(joystick_Lsw_pin, INPUT_PULLUP); //activate the arduino's internal pullup resistor to make continous float sw values discrete (HIGH OR LOW)
  pinMode(joystick_Bsw_pin, INPUT_PULLUP);

// declare servos, attach ESC signal pins
  thrusterA.attach(thrusterA_pin);
  thrusterB.attach(thrusterB_pin);
  thrusterC.attach(thrusterC_pin);
  servo0.attach(servo0_pin);

// send neutral signal
  thrusterA.writeMicroseconds(1500);
  thrusterB.writeMicroseconds(1500);
  thrusterC.writeMicroseconds(1500);

// make the servo horizontal during setup
  servo0.write(servoHorizontal);
  currentServoAngle = servoHorizontal;

  delay(2000); //allow time to initialize
}

void loop() {
  // read switches (LOW = pressed)
  bool LswPressed = (digitalRead(joystick_Lsw_pin) == LOW); 
  bool BswPressed = (digitalRead(joystick_Bsw_pin) == LOW); 

  // monitor
  if (BswPressed) Serial.println("Bsw pressed - ASCEND");
  if (LswPressed) Serial.println("Lsw pressed - DESCEND");

  // Override all directional movement(no reading of joystick x or y) -> switch alignment -> and then thrust to move vertically
  if (BswPressed || LswPressed) {
    servoChangeAngle(servoVertical); //the delay in this function makes sure that the realignnment finishes before thrusters r powered

    int pwm = 1500;
    if (BswPressed) {
      pwm = 1700; // forward thrust (ascend)
    } else if (LswPressed) {
      pwm = 1300; // reverse thrust (descend)
      //pwm values can be adjusted for faster or slower ascent/descent
    }

    //all thrusters will move in the same direction when ascending / descending vertically
    thrusterA.writeMicroseconds(pwm);
    thrusterB.writeMicroseconds(pwm);
    thrusterC.writeMicroseconds(pwm);

    delay(20);
    return; // joystick does NOT detect any directional joystick input when moving
  }

  //normal joystick control :
  servoChangeAngle(servoHorizontal); //realign first to horizontal, every single time

  // read directional input from joystick
  int Lx = analogRead(joystick_Lx_pin);
  int Ly = analogRead(joystick_Ly_pin);
  int By = analogRead(joystick_By_pin);

  int deviationX = Lx - centerValue;
  int deviationY = Ly - centerValue;
  int deviationBy = By - centerValue;
  
  //monitor values
  Serial.print("Lx: "); Serial.print(Lx);
  Serial.print("  Ly: "); Serial.print(Ly);
  Serial.print("  By: "); Serial.println(By);

  int pwmA = 1500;
  int pwmB = 1500;
  int pwmC = 1500;

  if (abs(deviationX) >= deadZone) {
    if (deviationX > 0) {
      pwmA = map(deviationX, deadZone, 530, 1510, 2000); //proportional speed control OWO
    } else {
      pwmB = map(deviationX, -530, -deadZone, 2000, 1510);
    }
  }

  if (abs(deviationY) >= deadZone) {
    if (deviationY > 0) {
      int reversePWM = map(deviationY, deadZone, 530, 1490, 1000); 
      pwmA = reversePWM;
      pwmB = reversePWM;
    } else {
      int forwardPWM = map(deviationY, -530, -deadZone, 2000, 1510);
      pwmA = forwardPWM;
      pwmB = forwardPWM;
    }
  }

  if (abs(deviationBy) >= deadZone) {
    if (deviationBy > 0) {
      pwmC = map(deviationBy, deadZone, 530, 1490, 1000);
    } else {
      pwmC = map(deviationBy, -530, -deadZone, 2000, 1510);
    }
  }

  thrusterA.writeMicroseconds(pwmA);
  thrusterB.writeMicroseconds(pwmB);
  thrusterC.writeMicroseconds(pwmC);

  delay(20);
}
