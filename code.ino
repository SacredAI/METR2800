#include <VarSpeedServo.h>
#include <NewPing.h>

// 0 = Tennis, 1 = Squash
//byte selectedBallType = 0;
//boolean collectedBalls = false;
//boolean touchingWall = false;
//boolean platformExtended = false;
//byte ballsDeposited = 0;
volatile static bool running = false;

#define TRIGGER_PING 5    // Pin 12 UltraSonic Trigger Pin
#define ECHO_PIN 6        // Pin 2 Ultra Sonic Echo input
#define MAX_DISTANCE 400  // Maximum distance we want to measure (in centimeters).

#define POWER_PIN 21

volatile long echo_start = 0;
volatile long echo_end = 0;
volatile long echo_duration = 0;
volatile int trigger_time_count = 0;

#define SERVO_BASE_PIN 13      // baseServo PMW pin
#define SERVO_SHOULDER_PIN 12  // Shoulder PMW pin
#define SERVO_ELBOW_PIN 11     // joint2Servo PMW pin
#define SERVO_WRIST_PIN 10     // Wrist PMW pin
#define SERVO_GRIP_PIN 8       // Grip PMW pin
#define HUMERUS 20.6
#define HUMERUS_SQR 424.36
#define ULNA 14.5
#define ULNA_SQR 210.25

#define BASE_HGT 10.4
#define GRIPPER 7.5

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
#define STATE_RETURN_HOME 6
#define STATE_FINISHED 7


VarSpeedServo baseServo;
VarSpeedServo shoulderServo;
VarSpeedServo elbowServo;
VarSpeedServo wristServo;
VarSpeedServo gripServo;

NewPing sonar(TRIGGER_PING, ECHO_PIN, MAX_DISTANCE);
bool tennis = false;

enum wheelMotion {
  STOP,
  FORWARD,
  BACKWARD,
  LEFT,
  RIGHT
};

uint8_t state = 1;
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
void set_arm_angles(uint16_t bas_ang, uint16_t shl_ang, uint16_t elb_ang, uint16_t wri_ang);
void set_arm(unsigned long x, unsigned long y, unsigned long z, unsigned long grip_angle_d);
void pickup_balls(void);
void move_to_silo(void);
void extend_platform(void);
void deposit_balls(void);
void return_home(void);
void retract_platform(void);

void setup() {
  // Connect to io pins
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  pinMode(POWER_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(POWER_PIN), powerSwitch, CHANGE);

  // pinMode(trigPin, OUTPUT);                        // Trigger pin set to output
  // pinMode(echoPin, INPUT);                        // Echo pin set to input

  setup_motors();

  setup_servos();

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

void setup_motors(void) {
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

void setup_servos(void) {
  baseServo.attach(SERVO_BASE_PIN, 500, 2500);
  shoulderServo.attach(SERVO_SHOULDER_PIN, 500, 2500);
  elbowServo.attach(SERVO_ELBOW_PIN, 500, 2500);
  wristServo.attach(SERVO_WRIST_PIN);
  gripServo.attach(SERVO_GRIP_PIN);
}

void setup_timer1(void) {

  TCNT1 = 0;  // Clear Timer

  // 65536 = 40 millisecond seconds 16 bit timer
  OCR1A = 65536;  // = (16*10^6) / (4*1024) - 1 (must be <65536)

  // Setup CTC mode
  TCCR1B = (1 << WGM12);

  // 1024 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);

  // Enable interrupt;
  TIMSK1 = (1 << OCIE1A);

  sei();  // Allow intterupts
}

#define ELAPSED_MAX 7258
volatile static uint16_t elapsed_count = 0;

ISR(TIMER1_COMPA_vect) {
  if (!running) return;
  elapsed_count++;
  if (elapsed_count >= ELAPSED_MAX) {
    stopAll();
    running = false;
  }
}

#define BOUNCET 25
volatile static unsigned long bounceTime = 0;

void powerSwitch(void) {
  if ((millis() - bounceTime) > BOUNCET) {
    running = !running;
    if (running) {
      digitalWrite(LED_BUILTIN, HIGH);
      TCNT1 = 0;
      Serial.println("ON");
    } else {
      stopAll();
      park_servos();
      running = false;
      Serial.println("Off");
    }
    bounceTime = millis();
  }
}

void stopAll(void) {
  stop_all_motors();
}

bool printed = false;
int pos = 0;
void loop(void) {
  if (!running) return;

  switch (state) {
    case STATE_MOVETO_BALL:
      // TODO: Move to tennis balls
      break;
    case STATE_PICKUP_BALL:
      pickup_balls();
      state++;
      break;
    case STATE_MOVETO_SILO:
      move_to_silo();
      state++;
      break;
    case STATE_PLATFORM_EXTEND:
      extend_platform();
      state++;
      break;
    case STATE_DEPOSIT_BALLS:
      deposit_balls();
      state++;
      break;
    case STATE_PLATFORM_RETRACTED:
      retract_platform();
      state++;
      break;
    case STATE_RETURN_HOME:
      return_home();
      state++;
      break
    case default:
      break;
  }
}

void pickup_balls(void){
  if(tennis){
    // TODO: Tennis
  }else{
    baseServo.write(90, 30, true);
    set_arm_angles(90, 150, 85, 110);
    driveDirection(FORWARD);
    delay(2000);
    driveDirection(STOP);
    elbowServo.write(150, 30, true);
    set_arm_angles(90, 80, 10, 10);
    gripServo.write(0);
  }
}

void move_to_silo(void){
  if(tennis){
    // TODO: Tennis
  }else{
    driveDirection(RIGHT);
    delay(3000);
    driveDirection(FORWARD);
    delay(1500);
    driveDirection(STOP);
  }
}

void extend_platform(void){
  if(tennis){
    // TODO: Tennis
  }else{
    movePlatform(FORWARD);
    delay(1500);
    movePlatform(STOP);
  }
}

void deposit_balls(void){
  if(tennis){
    // TODO: Tennis
  }else{
    elbowServo.write(110, 30);
    wristServo.write(50, 30);
    elbowServo.wait();
    wristServo.wait();
    set_arm_angles(90, 110, 145, 55);
    gripServo.write(180);
    delay(5000);
    gripServo.write(0);
    set_arm_angles(90, 80, 105, 50);
    set_arm_angles(120, 80, 10, 10);
  }
}

void retract_platform(void){
  if(tennis){

  }else{
    movePlatform(BACKWARD);
    delay(1500);
  }
}

void return_home(void){
  if(tennis){
    // TODO: Tennis
  }else{
    driveDirection(BACKWARD);
    delay(500);
    movePlatform(STOP);
    delay(1000);
    driveDirection(STOP);
  }
}

void park_servos(void){
  baseServo.write(120);
  shoulderServo.write(80);
  elbowServo.write(10);
  wristServo.write(10);
  gripServo.write(0);
}

void set_arm_angles(uint16_t bas_ang, uint16_t shl_ang, uint16_t elb_ang, uint16_t wri_ang){
  if((shl_ang) && shl_ang >= 20 && shl_ang <= 160){
    shoulderServo.write(shl_ang, 30);
  }
  if((elb_ang || elb_ang == 0) && elb_ang >= 0 && elb_ang <= 180){
    elbowServo.write(elb_ang, 30);
  }
  if((bas_ang || bas_ang == 0) && bas_ang <= 160 && bas_ang >= 20){
    baseServo.write(bas_ang, 30);
  }
  
  if((wri_ang || wri_ang == 0) && wri_ang >= 0 && wri_ang <= 180){
    wristServo.write(wri_ang, 30);
  }
  shoulderServo.wait();
  elbowServo.wait();
  baseServo.wait();
  wristServo.wait();
}

/* arm positioning routine utilizing inverse kinematics */
/* z is height, y is distance from base center out, x is side to side. y,z can only be positive */
// TODO: Maths is wrong need to redo it
void set_arm(unsigned long x, unsigned long y, unsigned long z, unsigned long grip_angle_d){
  if(!running) return;
  unsigned long grip_angle_r = radians( grip_angle_d );

  unsigned long bas_angle_d = atan2(x, y);
  unsigned long bas_angle_r = radians(bas_angle_d);
  unsigned long rdist = sqrt((x*x) * (y*y));
  y = rdist;

  // Grip offset Calculations
  unsigned long grip_off_z = (sin(grip_angle_r)) * GRIPPER;
  unsigned long grip_off_y = (cos(grip_angle_r)) * GRIPPER;

  unsigned long wrist_z = (z - grip_off_z) - BASE_HGT;
  unsigned long wrist_y = y - grip_off_y;

  unsigned long s_w = (wrist_z * wrist_z) + (wrist_y * wrist_y);
  unsigned long s_w_sqrt = sqrt(s_w);

  unsigned long a1 = atan2(wrist_z, wrist_y);
  unsigned long a2 = acos((( HUMERUS_SQR - ULNA_SQR ) + s_w) / (2 * HUMERUS * s_w_sqrt));

  unsigned long shl_angle_r = a1 + a2;
  unsigned long shl_angle_d = degrees( shl_angle_r );

  unsigned long elb_angle_r = acos((HUMERUS_SQR + ULNA_SQR - s_w) / (2 * HUMERUS * ULNA));
  unsigned long elb_angle_d = degrees( elb_angle_r );
  unsigned long elb_angle_dn = -(180.0 - elb_angle_d);

  unsigned long wri_angle_d = (grip_angle_d - elb_angle_dn) - shl_angle_d;

  char buffer[100];
  sprintf(buffer, "%lu, %lu, %lu, %lu", bas_angle_d, shl_angle_d, elb_angle_dn, wri_angle_d);
  Serial.println(buffer);
}

void movePlatform(enum wheelMotion m){
  if(m == LEFT || m == RIGHT){
    Serial.println("Error! Attempted to drive platform sideways D:");
    return;
  }
  if(!running) return;
  controlMotor(MOTORS_P_PIN1, MOTORS_P_PIN2, MOTORS_P_PWM, 50, m);
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

void stop_all_motors(void) {
  driveDirection(STOP);
  movePlatform(STOP);
}

//// Pin1 & Pin2 specify pins to control motor Direction,
//// PwnPin specifies pin to control speed, pwn is speed value,
//// wheelMotion specifices motor Direction
void controlMotor(int Pin1, int Pin2, int PwnPin, float pwm, enum wheelMotion m) {
  if (!running) return;
  int pwmSpeed = map(pwm, 0, 100, 0, 255);
  analogWrite(PwnPin, pwmSpeed);
  switch (m) {
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
