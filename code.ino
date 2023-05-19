#include <Servo.h>
#include <NewPing.h>

// 0 = Tennis, 1 = Squash
//byte selectedBallType = 0;
//boolean collectedBalls = false;
//boolean touchingWall = false;
//boolean platformExtended = false;
//byte ballsDeposited = 0;
volatile static bool running = false;

#define TRIGGER_PING 5 // Pin 12 UltraSonic Trigger Pin
#define ECHO_PIN 6 // Pin 2 Ultra Sonic Echo input
#define MAX_DISTANCE 400 // Maximum distance we want to measure (in centimeters).

#define POWER_PIN 21

volatile long echo_start = 0;
volatile long echo_end = 0;
volatile long echo_duration = 0;
volatile int trigger_time_count = 0;

#define SERVO_BASE_PIN 13 // baseServo PMW pin
#define SERVO_SHOULDER_PIN 12 // Shoulder PMW pin
#define SERVO_ELBOW_PIN 11 // joint2Servo PMW pin
#define SERVO_WRIST_PIN 10 // Wrist PMW pin 
#define SERVO_GRIP_PIN 9 // Grip PMW pin 
#define HUMERUS 200
#define HUMERUS_SQR 40000
#define ULNA 230
#define ULNA_SQR 52900

#define BASE_HGT 8.5
#define GRIPPER 6.94

#define MOTORS_R_PWM 7
#define MOTORS_R_PIN1 37
#define MOTORS_R_PIN2 35

#define MOTORS_L_PWM 6
#define MOTORS_L_PIN1 36
#define MOTORS_L_PIN2 34

#define MOTORS_P_PWM 5
#define MOTORS_P_PIN1 31
#define MOTORS_P_PIN2 33

#define STATE_MOVETO_BALL 0
#define STATE_PICKUP_BALL 1
#define STATE_MOVETO_SILO 2
#define STATE_PLATFORM_EXTEND 3
#define STATE_DEPOSIT_BALLS 4
#define STATE_PLATFORM_RETRACTED 5


Servo baseServo;
Servo shoulderServo;
Servo elbowServo;
Servo wristServo;
Servo gripServo;

NewPing sonar(TRIGGER_PING, ECHO_PIN, MAX_DISTANCE);
bool tennis = false;

enum wheelMotion {
 STOP,
 FORWARD,
 BACKWARD,
 LEFT,
 RIGHT
};

uint8_t state = 0;
//
//typedef struct {
//  // Setup Distance Sensor data struct to allow storage
//} distanceSensorData;

// Function Prototypes
void move_to_balls(void);
void park_servos(void);
void stop_all_motors(void);
void setup_timer1(void);
void setup_motors(void);
void setup_servos(void);

void setup() {
  // Connect to io pins
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  pinMode(POWER_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(POWER_PIN), powerSwitch, CHANGE);

  // pinMode(trigPin, OUTPUT);                        // Trigger pin set to output
  // pinMode(echoPin, INPUT);                        // Echo pin set to input
  
  setup_motors();

  // pinMode(4, INPUT_PULLUP);

  // pinMode(MOTORS_TR_PWM, OUTPUT);
  // pinMode(MOTORS_TR_PIN1, OUTPUT);
  // pinMode(MOTORS_TR_PIN2, OUTPUT);

  setup_timer1();

  park_servos();
  // Connect to Sensors


  // Setup initial State

  // Store System Start time
  Serial.begin(9600);
  // Serial.println("START");
  // startTime = millis();

  // Flash led to show system start
}

void setup_motors(void){
  pinMode(MOTORS_R_PWM, OUTPUT);
  pinMode(MOTORS_R_PIN1, OUTPUT);
  pinMode(MOTORS_R_PIN2, OUTPUT);

  pinMode(MOTORS_L_PWM, OUTPUT);
  pinMode(MOTORS_L_PIN1, OUTPUT);
  pinMode(MOTORS_L_PIN2, OUTPUT);

  pinMode(MOTORS_P_PWM, OUTPUT);
  pinMode(MOTORS_P_PIN1, OUTPUT);
  pinMode(MOTORS_P_PIN2, OUTPUT);

  // analogWrite(MOTORS_L_PWM, 255);
  // analogWrite(MOTORS_R_PIN2, 255);
  // analogWrite(MOTORS_P_PIN1, 255);
}

void setup_servos(void){
  baseServo.attach(SERVO_BASE_PIN, 500, 2500);
  shoulderServo.attach(SERVO_SHOULDER_PIN, 500, 2500);
  elbowServo.attach(SERVO_ELBOW_PIN, 500, 2500);
  wristServo.attach(SERVO_WRIST_PIN);
  gripServo.attach(SERVO_GRIP_PIN);
}

void setup_timer1(void){

  TCNT1 = 0; // Clear Timer

  // 65536 = 40 millisecond seconds 16 bit timer
  OCR1A = 65536;// = (16*10^6) / (4*1024) - 1 (must be <65536)

  // Setup CTC mode
  TCCR1B = (1<<WGM12);

  // 1024 prescaler
  TCCR1B |= (1<<CS12)|(1<<CS10);

  // Enable interrupt;
  TIMSK1 = (1<<OCIE1A);

  sei(); // Allow intterupts
}

#define ELAPSED_MAX 7258
volatile static uint16_t elapsed_count = 0;

ISR(TIMER1_COMPA_vect){
  if(!running) return;
  elapsed_count++;
  if(elapsed_count >= ELAPSED_MAX){
    Serial.println(buffer);
    stopAll();
    running = false;
  }
}

#define BOUNCET	25
volatile static unsigned long bounceTime = 0;

void powerSwitch(void){
  if((millis() - bounceTime) > BOUNCET){
    running = !running;
    if(running){
      digitalWrite(LED_BUILTIN, HIGH);
      TCNT1 = 0;
      Serial.println("ON");
    }else{
      stopAll();
      running = false;
      Serial.println("Off");
    }
    bounceTime = millis();
  }
}

void stopAll(void){
  stop_all_motors();
}

bool printed = false;
int pos = 0;
void loop(void) {
  if(!running) return;

  // park_servos();
  // elapsedTime = millis();
  // if(elapsedTime <= endTime){
  //   running = false;
  //   stopAll();
  //   return;
  // }

  // switch (state){
  //   case STATE_MOVETO_BALL:
  //     move_to_balls();
  //     break;
  //   case STATE_PICKUP_BALL:
  //     break;
  //   case STATE_MOVETO_SILO:
  //     break;
  //   case STATE_PLATFORM_EXTEND:
  //     break;
  //   case STATE_PLATFORM_RETRACTED:
  //     break;
  //   case default:
  //     Serial.println("Invalid State");
  //     break;
  // }
  // baseServo.write(90);
  // Serial.println("Rotated");
  // if(!running){
  //   if(false){
  //     running = true;
  //     digitalWrite(LED_BUILTIN, HIGH);
  //     startTime = millis();
  //   }
  //   return;
  // };
  // elapsedTime = millis();
  // Stop all system functionality after 120000 milliseconds (120 seconds)
  // Serial.println((elapsedTime - startTime), DEC);
  // if ((elapsedTime - startTime) > 120000) {
    // Flash led to show system stop
    // Serial.println("END");
    // exit(0);
  // }

  // delay(5000);
  // float theta1;
  // float theta2;
  // float theta3;
  // set_arm(10, 10, 20, theta1, theta2, theta3);
  // char buffer[100];
  // sprintf(buffer, "%d, %d, %d", theta1, theta2, theta3);
  // Serial.println(buffer);
  // baseServo.write(theta1);
  // Shoulder.write(theta2);
  // joint2Servo.write(theta3);
  // delay(5000);
  // baseServo.write(45);
  // Shoulder.write(155);
  // joint2Servo.write(55);
  // delay(5000);
  // Shoulder.write(90);
  // char buffer[100];
  // pos = baseServo.read();
  // sprintf(buffer, "%d", pos);
  // Serial.println(buffer);
  // for (pos = baseServo.read(); pos <= 170; pos += 1) { // goes from 0 degrees to 180 degrees
  //   // in steps of 1 degree
  //   baseServo.write(pos);
  //   sprintf(buffer, "%d", pos);
  //   Serial.println(buffer);
  //   // Shoulder.write(pos);
  //   // joint2Servo.write(90);              // tell servo to go to position in variable 'pos'
  //   delay(15);                       // waits 15 ms for the servo to reach the position
  // }
  // for (pos = 170; pos >= 10; pos -= 1) { // goes from 180 degrees to 0 degrees
  //   baseServo.write(pos);
  //   sprintf(buffer, "%d", pos);
  //   Serial.println(buffer);
  //   // Shoulder.write(pos);
  //   // joint2Servo.write(pos);              // tell servo to go to position in variable 'pos'
  //   delay(15);                       // waits 15 ms for the servo to reach the position
  // }

  
  // driveDirection(FORWARD);
  // delay(1000);
  // driveDirection(BACKWARD);
  // delay(1000);
  // driveDirection(LEFT);
  // delay(1000);
  // driveDirection(RIGHT);
  // delay(1000);
  // driveDirection(STOP);

  // movePlatform(FORWARD);
  // delay(1000);
  // movePlatform(BACKWARD);
  // delay(1000);
  // movePlatform(STOP);
  // controlMotor(MOTORS_BL_PIN1, MOTORS_BL_PIN2, MOTORS_BL_PWM, 100, wheelMotion(1));
  // controlMotor(MOTORS_TR_PIN1, MOTORS_TR_PIN2, MOTORS_TR_PWM, 100, wheelMotion(1));
  // controlMotor(MOTORS_TL_PIN1, MOTORS_TL_PIN2, MOTORS_TL_PWM, 100, wheelMotion(1));
  // controlMotor(MOTORS_BR_PIN1, MOTORS_BR_PIN2, MOTORS_BR_PWM, 100, wheelMotion(0));
  // controlMotor(MOTORS_BL_PIN1, MOTORS_BL_PIN2, MOTORS_BL_PWM, 100, wheelMotion(0));
  // controlMotor(MOTORS_TR_PIN1, MOTORS_TR_PIN2, MOTORS_TR_PWM, 100, wheelMotion(0));
  // controlMotor(MOTORS_TL_PIN1, MOTORS_TL_PIN2, MOTORS_TL_PWM, 100, wheelMotion(0));

  //  if(ballsDeposited >= 6){
  //     // Ensure system has returned to home area
  //     // Flash led to show system stop
  //     exit(0);
  //  }
  //
  //  if(collectedBalls){
  //    if(handleBallDeposit()){
  //      collectedBalls = false;
  //      selectedBallType = 1;
  //    }else{
  //      // We have balls but haven't sucessfuly deposited them yet
  //      // so we want to keep
  //      // looping through the handleBallDeposit function
  //      return;
  //    }
  //  }
  //
  //  // Determine distance
  //  distanceSensorData latestData;
  //  if(withinRangeOfBalls(latestData)){
  //    if(handleBallCollection(latestData)){
  //      collectedBalls = true;
  //    }else{
  //      // Reposition in an attempt to sucessfully pick up on the next pass
  //    }
  //  }else {
  //    // Depending on selected Ball type drive to top-right/top-left of
  //    // lowered section
  //    // latestData will be used to determing distance and angle
  //    // needed to move for best chance of being within range in next loop
  //  }
  // delay(5000);
  // She likes a little bit of delay when theres nothing to do
  // delay(50);
}

void move_to_balls(void){
  if(tennis){
    // Move to Tennis balls
  }else{
    // Move to Squash balls
  }
}

void park_servos(void){

  baseServo.write(120);
  shoulderServo.write(0);
  elbowServo.write(10);
  wristServo.write(10);
  gripServo.write(90);
}

/* arm positioning routine utilizing inverse kinematics */
/* z is height, y is distance from base center out, x is side to side. y,z can only be positive */
void set_arm(float x, float y, float z, float grip_angle_d){
  if(!running) return;
  float grip_angle_r = radians( grip_angle_d );

  float bas_angle_r = radians(atan2(y, x));
  float rdist = sqrt((x*x) * (y*y));
  y = rdist;

  // Grip offset Calculations
  float grip_off_z = (sin(grip_angle_r)) * GRIPPER;
  float grip_off_y = (cos(grip_angle_r)) * GRIPPER;

  float wrist_z = (z - grip_off_z) - BASE_HGT;
  float wrist_y = y - grip_off_y;

  float s_w = (wrist_z * wrist_z) + (wrist_y * wrist_y);
  float s_w_sqrt = sqrt(s_w);

  float a1 = atan2(wrist_z, wrist_y);
  float a2 = acos((( HUMERUS_SQR - ULNA_SQR ) + s_w) / (2 * HUMERUS * s_w_sqrt));

  float shl_angle_r = a1 + a2;
  float shl_angle_d = degrees( shl_angle_r );

  float elb_angle_r = acos((HUMERUS_SQR + ULNA_SQR - s_w) / (2 * HUMERUS * ULNA));
  float elb_angle_d = degrees( elb_angle_r );
  float elb_angle_dn = -(180.0 - elb_angle_d);

  float wri_angle_d = (grip_angle_d - elb_angle_dn) - shl_angle_d;
}


//boolean withinRangeOfBalls(distanceSensorData &returnData){
//  // Collect Sensor data and correlate it to determine
//  // if the system is capable of picking up selected balltype.
//  // If outof range update returnData to allow system to navigate
//  // return true if withinrange false otherwise
//}
//
//// Handle ball deposit logic including driving to platform
//boolean handleBallDeposit(){
//  // Check ball type
//  // Depending on ball type drive to top-left/bottom-right
//  // sections of areana
//  // Position system until touching wall
//  // Extend platform
//  // Deposit balls
//  // Increment number of ballsDepositied variable
//  // retract platform
//  // return True if balls deposited, false otherwise
//}
//
//// Handle ball collection once within range
//boolean handleBallCollection(distanceSensorData data){
//  // Use sensor data to position arm
//  // Use ball type to determin how much the claw needs to close
//  // Sensor to determin if balls were sucessfully collected
//  // (possible limit switch/tracking opening of claw/feedback based on resistance)
//  // return true if balls collected, false otherwise
//}

void movePlatform(enum wheelMotion m){
  if(m == LEFT || m == RIGHT){
    Serial.println("Error! Attempted to drive platform sideways D:");
    return;
  }
  if(!running) return;
  controlMotor(MOTORS_P_PIN1, MOTORS_P_PIN2, MOTORS_P_PWM, 40, m);
}

void driveDirection(enum wheelMotion m){
  if(!running) return;
  switch(m){
      case LEFT:
      controlMotor(MOTORS_L_PIN1, MOTORS_L_PIN2, MOTORS_L_PWM, 100, BACKWARD);
      controlMotor(MOTORS_R_PIN1, MOTORS_R_PIN2, MOTORS_R_PWM, 100, FORWARD);
      break;
   case RIGHT:
      controlMotor(MOTORS_L_PIN1, MOTORS_L_PIN2, MOTORS_L_PWM, 100, FORWARD);
      controlMotor(MOTORS_R_PIN1, MOTORS_R_PIN2, MOTORS_R_PWM, 100, BACKWARD);
      break;
    default:
      controlMotor(MOTORS_L_PIN1, MOTORS_L_PIN2, MOTORS_L_PWM, 100, m);
      controlMotor(MOTORS_R_PIN1, MOTORS_R_PIN2, MOTORS_R_PWM, 100, m);
      break;
  }
}

void stop_all_motors(void){
  driveDirection(STOP);
  movePlatform(STOP);
}

//// Pin1 & Pin2 specify pins to control motor Direction,
//// PwnPin specifies pin to control speed, pwn is speed value,
//// wheelMotion specifices motor Direction
void controlMotor(int Pin1, int Pin2, int PwnPin, float pwm, enum wheelMotion m){
  if(!running) return;
  int pwmSpeed = map(pwm, 0, 100, 0, 255);
  analogWrite(PwnPin, pwmSpeed);
 switch(m){
   case FORWARD:
        digitalWrite(Pin1, HIGH);
        digitalWrite(Pin2, LOW);
     break;
   case BACKWARD:
        digitalWrite(Pin1, LOW);
        digitalWrite(Pin2, HIGH);
     break;
   default:
        digitalWrite(Pin1, LOW);
        digitalWrite(Pin2, LOW);
     break;
 }
}
