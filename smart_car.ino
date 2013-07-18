///////////////////////// //////////////////////////////////
//  Author: Remond <hello_ma@yeah.net>
//  
/////////////////////////////////////////////////////////////
#include <Wire.h>
#include <LiquidCrystal_I2C.h>


#include <Servo.h> 
#include <IRremote.h>
#include <AFMotor.h>

#include <avr/sleep.h>
#include <avr/wdt.h>

////define pins
#define IR_RECEIVER_PIN  2 // the IR receiver pin
#define SERVO_PIN        9 // the servo motor pin to carry ultrasonic detector
#define LED_PIN         13 // define led pin
#define ULTRASONIC_IN   A0 // define ultrasonic signal receiver pin  ECHO
#define ULTRASONIC_OUT  A1 // define ultrasonic signal transmitter pin  TRIG

//#define RUN_MODE_LED             A6 // indicate run mode. on-auto, off-manual, blink-follow path.
#define ENABLE_FOLLOW_PATH_PIN   A7 // give follow_path_board power or not..

#define PATH_DETECTOR_1_PIN   A2
#define PATH_DETECTOR_2_PIN   A3
#define PATH_DETECTOR_3_PIN   A4
#define PATH_DETECTOR_4_PIN   A5

#define MEASURE_VOLTAGE_PIN   A10

/*
  definitions
 */
#define FORWARD_ANGLE                  90-2
#define STEP_ANGLE_DEGREE              (180/5+5)
#define MIN_car_move_forward_DISTANCE  30 // the min distance in CM to move forward.
#define MIN_SIDE_DISTANCE              30 // the min safe side distance when go forward.

unsigned char MOTOR_FAST_SPEED = 180;
unsigned char MOTOR_SLOW_SPEED = 150;

////end define


//// global variables..

Servo ultrasonic_Servo;  // create servo object to control a servo 

// car motors.
AF_DCMotor motorLeft1 (1, MOTOR12_1KHZ);
AF_DCMotor motorRight1(2, MOTOR12_1KHZ);
AF_DCMotor motorLeft2 (3, MOTOR34_1KHZ);
AF_DCMotor motorRight2(4, MOTOR34_1KHZ);

// LCD display.
LiquidCrystal_I2C lcd(0x27,16,2);  // set the LCD address to 0x27 for a 16 chars and 2 line display
char lcd_buffer[16];

// IR reciever.
IRrecv irrecv(IR_RECEIVER_PIN);

//// end global variables.

//forward function define.
void car_move_forward(unsigned char speed, unsigned long delay_ms=0);
void car_stop(unsigned long msecond=0);
void car_move_backward(unsigned char speed, unsigned long delay_ms=0);
void car_turn_right(unsigned char left_speed, unsigned char right_speed, unsigned long delay_ms=0);
void car_turn_left(unsigned char left_speed, unsigned char right_speed, unsigned long delay_ms=0);
//end

void car_move_forward(unsigned char speed, unsigned long delay_ms)
{
  motorLeft1.setSpeed(speed);
  motorLeft2.setSpeed(speed);
  motorRight1.setSpeed(speed);
  motorRight2.setSpeed(speed);

  motorLeft1.run(FORWARD);    
  motorRight1.run(FORWARD);     
  motorRight2.run(FORWARD);     
  motorLeft2.run(FORWARD);    

  if(delay_ms)
    delay(delay_ms);
}
void car_stop(unsigned long delay_ms)
{
  motorLeft1.run(RELEASE);    
  motorRight1.run(RELEASE);      
  motorRight2.run(RELEASE);  
  motorLeft2.run(RELEASE);    

  if(delay_ms)
    delay(delay_ms);
}
void car_move_backward(unsigned char speed, unsigned long delay_ms)
{
  motorLeft1.setSpeed(speed);
  motorLeft2.setSpeed(speed);
  motorRight1.setSpeed(speed);
  motorRight2.setSpeed(speed);

  motorLeft1.run(BACKWARD);    
  motorRight1.run(BACKWARD); 
  motorRight2.run(BACKWARD);   
  motorLeft2.run(BACKWARD);  

  if(delay_ms)
    delay(delay_ms);  
}
void car_turn_right(unsigned char left_speed, unsigned char right_speed, unsigned long delay_ms)
{
  motorLeft1.setSpeed(left_speed);
  motorLeft2.setSpeed(left_speed);
  motorRight1.setSpeed(right_speed);
  motorRight2.setSpeed(right_speed);

  motorLeft1.run(FORWARD);    
  motorLeft2.run(FORWARD);      
  motorRight1.run(BACKWARD);     
  motorRight2.run(BACKWARD);     

  if(delay_ms)
    delay(delay_ms);  
}
void car_turn_left(unsigned char left_speed, unsigned char right_speed, unsigned long delay_ms)
{
  motorLeft1.setSpeed(left_speed);
  motorLeft2.setSpeed(left_speed);
  motorRight1.setSpeed(right_speed);
  motorRight2.setSpeed(right_speed);

  motorRight1.run(FORWARD);  
  motorRight2.run(FORWARD);  
  motorLeft1.run(BACKWARD);    
  motorLeft2.run(BACKWARD);     

  if(delay_ms)
    delay(delay_ms);
}

#define MOVE_BACK_TIME        180
#define TURN_20_DEGREE_TIME   200
#define TURN_45_DEGREE_TIME   300
#define TURN_90_DEGREE_TIME   480

void turn_right_45angle() 
{
  car_turn_right(MOTOR_FAST_SPEED, MOTOR_FAST_SPEED, TURN_45_DEGREE_TIME);
}

void turn_right_90angle() 
{
  car_turn_right(MOTOR_FAST_SPEED, MOTOR_FAST_SPEED, TURN_90_DEGREE_TIME);
}

void turn_left_45angle() 
{
  car_turn_left(MOTOR_FAST_SPEED, MOTOR_FAST_SPEED, TURN_45_DEGREE_TIME);
}

void turn_left_90angle() 
{
  car_turn_left(MOTOR_FAST_SPEED, MOTOR_FAST_SPEED, TURN_90_DEGREE_TIME);
}

/* for ultrasonic detector and servo motor */
// DO NOT change the order !!!!
enum DIRECTION
{
  MOVE_FORWARD, MOVE_RIGHT_FORWARD, MOVE_RIGHT, MOVE_LEFT_FORWARD, MOVE_LEFT, MOVE_BACK, MOVE_STOP
};

struct ultrasonicData
{
  DIRECTION      direction; //where to turn
  unsigned char  angle;   //angle for each turn
  char           needMeasure; //whether to measure distance.
  unsigned short delayBeforeMeasure; //delay before each measure.
};

// the data order must be aligned with enum DIRECTION{}!!!
struct ultrasonicData ultraData[] = 
{
  //forward
  {   
    MOVE_FORWARD, FORWARD_ANGLE, true, 100        }  
  ,
  {  
    //right forward
    MOVE_RIGHT_FORWARD, FORWARD_ANGLE - STEP_ANGLE_DEGREE, true, 150        }
  ,
  {  
    //right
    MOVE_RIGHT, FORWARD_ANGLE - STEP_ANGLE_DEGREE*2, true, 150        }
  ,
  {  
    //left forward
    MOVE_LEFT_FORWARD, FORWARD_ANGLE + STEP_ANGLE_DEGREE, true, 300        }
  ,
  {  
    //left
    MOVE_LEFT, FORWARD_ANGLE + STEP_ANGLE_DEGREE*2, true, 150        }
  ,
};

//return distance in mm.
unsigned short getDistance() 
{
  digitalWrite(ULTRASONIC_OUT, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_OUT, HIGH); // Pulse for 10μs to trigger ultrasonic detection
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_OUT, LOW);  
  unsigned short distance = pulseIn(ULTRASONIC_IN, HIGH);  // Read receiver pulse time
  distance= distance/58;   // Transform pulse time to distance

  lcd.setCursor(8,0);
  sprintf(lcd_buffer,"%5d cm",distance); //ocupy 8 chars.
  lcd.print(lcd_buffer);  
  
  delay(60);    
  return distance;
}

int getDirection(unsigned short &distance)
{
  //get distant
  DIRECTION direction = MOVE_FORWARD;
  distance = getDistance();
  unsigned short maxDistance = distance;

  if(distance > MIN_car_move_forward_DISTANCE)
  {
    //detect whether run into the position which is about 45 degree with the wall.
    ultrasonic_Servo.write(ultraData[MOVE_RIGHT_FORWARD].angle+10);
    delay(130);
    unsigned short r_distance = getDistance();
    if(r_distance<MIN_SIDE_DISTANCE)
    {
      car_stop(100);
    }
      
    ultrasonic_Servo.write(ultraData[MOVE_FORWARD].angle);
    delay(130);
    unsigned short f_distance = getDistance();
    if(f_distance<MIN_car_move_forward_DISTANCE)
    {
      car_stop(100);     
      goto NEED_TO_CHECK_AROUND;
    }
    
    //Serial.print("r_distance:"); 
    //Serial.println(r_distance);   //Ourput distance  
    ultrasonic_Servo.write(ultraData[MOVE_LEFT_FORWARD].angle-10);
    delay(130);
    unsigned short l_distance = getDistance();
    if(l_distance<MIN_SIDE_DISTANCE)
    {
       car_stop(100);
    }
    
    //Serial.print("l_distance:"); 
    //Serial.println(l_distance);   //Ourput distance  
    if(l_distance<MIN_SIDE_DISTANCE && l_distance < r_distance)
    {
      car_turn_right(MOTOR_FAST_SPEED, MOTOR_FAST_SPEED, TURN_20_DEGREE_TIME);  
    }
    else if(r_distance<MIN_SIDE_DISTANCE && r_distance < l_distance)
    {  
      car_turn_left(MOTOR_FAST_SPEED, MOTOR_FAST_SPEED, TURN_20_DEGREE_TIME);  
    }

    ultrasonic_Servo.write(ultraData[MOVE_FORWARD].angle); //reset to middle position.
    //delay(100);
    return MOVE_FORWARD;
  }

NEED_TO_CHECK_AROUND:

  // detect the direction.
  car_stop(0);//stop for path detection 
  for(unsigned char i=0; i<sizeof(ultraData)/sizeof(struct ultrasonicData); i++)  // goes from 0 degrees to 180 degrees 
  {        
    ultrasonic_Servo.write(ultraData[i].angle);         // tell servo to go to position in variable 'pos' 
    delay(ultraData[i].delayBeforeMeasure);
    if(ultraData[i].needMeasure)
    {
      distance = getDistance();
      // Serial.print(directionName[ultraData[i].direction]); 
      // Serial.print(" distance:"); Serial.println(distance);
      if (distance > maxDistance)
      {
        maxDistance = distance;
        direction = ultraData[i].direction;
      }      
    }
  }
  ultrasonic_Servo.write(ultraData[MOVE_FORWARD].angle);  //reset the servo position.
  delay(200);

  return direction;
}

void autoPilot()
{  
  unsigned short distance;
  switch(getDirection(distance))
  {
  case  MOVE_FORWARD: 
    {
      //Serial.println("Move FORWARD:");
      unsigned char speed = MOTOR_FAST_SPEED; //180
      distance =  getDistance();   
      if(distance > 500)
        ; //180
      if(distance > 300)
        speed -= 20;  //160
      else if(distance > 200)
        speed -= 40; //140;
      else if(distance > 100)
        speed -= 60; //120
      else if(distance > 50)
        speed -= 70;    //110    
      else
        speed -= 80;    //100    

      lcd.setCursor(12,1);        
      sprintf(lcd_buffer,"%4d",speed); //ocupy 8 chars.
      lcd.print(lcd_buffer);      
        
      if(distance > MIN_car_move_forward_DISTANCE)
         car_move_forward(speed, 200);     
      else 
         car_move_forward(speed, 100); 
    }
    break;

  case  MOVE_LEFT:
    {
      //Serial.println("Move to LEFT:"); 
      //turn left
      car_move_backward(MOTOR_SLOW_SPEED, MOVE_BACK_TIME);
      car_stop(200);
      turn_left_90angle();      
      car_stop(200);
    }
    break;

  case  MOVE_LEFT_FORWARD:
    {
      //Serial.println("Move to LEFT FORWARD:"); 
      //turn left forward
      car_move_backward(MOTOR_SLOW_SPEED, MOVE_BACK_TIME);
      car_stop(200);
      turn_left_45angle();      
      //car_turn_left(MOTOR_FAST_SPEED, MOTOR_FAST_SPEED, TURN_20_DEGREE_TIME);
      car_stop(200);
    }
    break;     

  case  MOVE_RIGHT:
    {
      //Serial.println("Move to RIGHT:");
      //turn right
      car_move_backward(MOTOR_SLOW_SPEED, MOVE_BACK_TIME);
      car_stop(200);
      turn_right_90angle();  
      car_stop(200);
    }
    break;

  case  MOVE_RIGHT_FORWARD:
    {
      //Serial.println("Move to RIGHT FORWARD:");     
      //turn right
      car_move_backward(MOTOR_SLOW_SPEED, MOVE_BACK_TIME);
      car_stop(200);
      turn_right_45angle();
      //car_turn_left(MOTOR_FAST_SPEED, MOTOR_FAST_SPEED, TURN_20_DEGREE_TIME);     
      car_stop(200);
    }
    break;     

  case  MOVE_BACK:
    {
      //Serial.println("Move BACKWARD:");
      car_move_backward(MOTOR_SLOW_SPEED, 1000);
      car_stop(100);
    }
    break;

  case  MOVE_STOP:
    break;    
  }

}

void followThePath()
{
#define FOLLOW_PATH_SPEED 100

  char val1 = digitalRead(PATH_DETECTOR_1_PIN);
  char val2 = digitalRead(PATH_DETECTOR_2_PIN);
  char val3 = digitalRead(PATH_DETECTOR_3_PIN);
  char val4 = digitalRead(PATH_DETECTOR_4_PIN);
  //Serial.print("follow data:");
  //Serial.print(val1,HEX);Serial.print(val2,HEX);Serial.print(val3,HEX);Serial.println(val4,HEX);
  if(val1 == 1)      // The left-1st-detector found black line.
  {
    car_turn_left(FOLLOW_PATH_SPEED, FOLLOW_PATH_SPEED, TURN_20_DEGREE_TIME/2);
    //car_stop(200);
  }
  else if(val4 == 1) // 
  {
    car_turn_right(FOLLOW_PATH_SPEED, FOLLOW_PATH_SPEED, TURN_20_DEGREE_TIME/2);
    //car_stop(200);
  }
  else if(val2 == 1 || val3 == 1)
  {
    car_move_forward(FOLLOW_PATH_SPEED, 100);
    //car_stop(200);
  }
  else if(!val1&&!val2&&!val3&&!val4)
  {
    car_stop(2000);
  }
}

void test_led(unsigned long ms)
{
  //test timing, should flash led every 2 seconds.  
  digitalWrite(LED_PIN, HIGH);
  delay(ms/2);
  digitalWrite(LED_PIN,LOW);
  delay(ms/2);
}

void test_turn()
{
  turn_right_45angle();
  car_stop(1000);

  turn_left_45angle();
  car_stop(1000);

  turn_right_90angle();
  car_stop(1000);

  turn_left_90angle();
  car_stop(2000);
}

enum runMode{
  AUTO_PILOT_MODE=0,
  MANUAL_CONTROL_MODE,
  FOLLOW_THE_PATH,
  CAR_STOP_MODE
};

const char *runMode_str[]={"Auto Pilot","Manually", "Follow Path", "CAR STOP"};

#define SWITCH_MODE_SIGNAL      0x807FC03F
#define car_move_forward_SIGNAL 0x807FE01F
#define TURN_RIGHT_SIGNAL       0x807FD02F
#define TURN_LEFT_SIGNAL        0x807F0AF5
#define MOVE_BACKWARD_SIGNAL    0x807FF00F

long previous_cmd = -1;
char run_mode = CAR_STOP_MODE; // refer to enum runMode.
void run_the_car()
{  
  decode_results results={   0  };
  if (irrecv.decode(&results)) 
  {
    Serial.print(results.value, HEX);
    Serial.print(" : ");
    Serial.println(results.decode_type);    
    digitalWrite(LED_PIN, HIGH);

    if (results.decode_type == 1)
    {
      if( results.value == SWITCH_MODE_SIGNAL)
      {
        Serial.println("Switch auto-pilot or manual control...");    
        run_mode ++;  
        run_mode = (run_mode == CAR_STOP_MODE)?AUTO_PILOT_MODE:run_mode;
        previous_cmd = -1; //don't support duplicate.
        car_stop();        
        
        lcd.setCursor(0,1);        
        sprintf(lcd_buffer,"%11s;",runMode_str[run_mode]); //ocupy 12 chars.
        lcd.print(lcd_buffer);        

      } 
    }
    else
    {
      previous_cmd = -1;
    }
    delay(200);  
    irrecv.resume(); // receive next  
    digitalWrite(LED_PIN, LOW);

  }

  if(run_mode == AUTO_PILOT_MODE)
  {
    //Serial.println("Auto-pilot...");  
    //delay(500); 
    digitalWrite(ENABLE_FOLLOW_PATH_PIN,LOW); //disable follow_path 
    ultrasonic_Servo.attach(SERVO_PIN);
    lcd.backlight();  
    //digitalWrite(RUN_MODE_LED,HIGH);
    autoPilot();   

    //digitalWrite(RUN_MODE_LED,LOW);
  } 
  else if(run_mode == FOLLOW_THE_PATH)
  {
    //Serial.println("Follow the path...");
    digitalWrite(ENABLE_FOLLOW_PATH_PIN,HIGH);
    ultrasonic_Servo.detach(); 
    lcd.noBacklight();  
    //digitalWrite(RUN_MODE_LED,LOW);
    followThePath();
  }
  else if(run_mode == MANUAL_CONTROL_MODE)
  {
    //Serial.println("Manual control...");        

    digitalWrite(ENABLE_FOLLOW_PATH_PIN,LOW); //disable follow_path 
    ultrasonic_Servo.detach(); 
    lcd.noBacklight();  
    //digitalWrite(RUN_MODE_LED,LOW);
    if (results.decode_type == 1) 
    {                
      if((results.value == 0xFFFFFFFF) && (previous_cmd != -1))
      {
        results.value = previous_cmd;
      }

      switch(results.value)
      {
      case car_move_forward_SIGNAL:
        {
          Serial.println("Move forward...");   
          previous_cmd = results.value;  
          car_move_forward(MOTOR_SLOW_SPEED, 500); 
          car_stop();
        }
        break;        
      case TURN_LEFT_SIGNAL:
        {
          Serial.println("Turn left...");    
          previous_cmd = results.value; 
          car_turn_left(MOTOR_FAST_SPEED, MOTOR_FAST_SPEED, TURN_20_DEGREE_TIME);
          car_stop();
        }
        break;        
      case TURN_RIGHT_SIGNAL:
        {
          Serial.println("Turn right...");    
          previous_cmd = results.value; 
          car_turn_right(MOTOR_FAST_SPEED, MOTOR_FAST_SPEED, TURN_20_DEGREE_TIME);
          car_stop();
        }
        break;       
      case MOVE_BACKWARD_SIGNAL:
        {
          Serial.println("Move backward...");    
          previous_cmd = results.value; 
          car_move_backward(MOTOR_SLOW_SPEED, 500);
          car_stop();
        }
        break;               
      default:
        previous_cmd = -1;
        break; 
      }
      delay(200);  
    }
    else
    {
      delay(500);
    }
  }
  else
  {
    ultrasonic_Servo.detach();   
    digitalWrite(ENABLE_FOLLOW_PATH_PIN,LOW); //disable follow_path 
    lcd.noBacklight();
    delay(500);      
  }
}

void powerDownCPU()
{
  // close all motors.
  motorLeft1.run(RELEASE);
  motorRight1.run(RELEASE);
  motorLeft2.run(RELEASE);
  motorRight2.run(RELEASE);  
  ultrasonic_Servo.detach();
  // close all other devices.
  digitalWrite(LED_PIN,LOW);
  lcd.noBacklight();
  digitalWrite(ENABLE_FOLLOW_PATH_PIN,LOW);
  
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);   // sleep mode is set here
  cli();
  sleep_enable();
  sei();
  sleep_cpu();
  sei();
}

void setup() 
{ 
  Serial.begin(9600);  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN,LOW);

  //for servo and ultrasonic detector.
  pinMode(ULTRASONIC_OUT, OUTPUT);
  pinMode(ULTRASONIC_IN, INPUT);
  ultrasonic_Servo.attach(SERVO_PIN);  // attaches the servo on pin 9 to the servo object 
  //move detector to forward position.
  ultrasonic_Servo.write(ultraData[0].angle);  //reset the servo position.
  delay(500);

  // for IR receiver.
  irrecv.enableIRIn();

  // run mode
  //pinMode(RUN_MODE_LED, OUTPUT);
  //digitalWrite(RUN_MODE_LED,LOW);

  pinMode(PATH_DETECTOR_1_PIN, INPUT);
  pinMode(PATH_DETECTOR_2_PIN, INPUT);
  pinMode(PATH_DETECTOR_3_PIN, INPUT);
  pinMode(PATH_DETECTOR_4_PIN, INPUT);

  pinMode(ENABLE_FOLLOW_PATH_PIN, OUTPUT);
  digitalWrite(ENABLE_FOLLOW_PATH_PIN,LOW);
  
  //set car run mode.
  run_mode = AUTO_PILOT_MODE;//CAR_STOP_MODE;
  
  // init LCD1602
  lcd.init();                      // initialize the lcd 
  lcd.noBacklight();  
  lcd.home();  
  lcd.print("Initialing car.");
  delay(1000);
  lcd.clear();
  lcd.setCursor(0,1);        
  lcd.print(runMode_str[run_mode]);
  
} 

void loop() 
{ 
  //test_turn();
  //test_led(1000);

  // check battery healthy.  
  static float history_voltage[3] = {0};
  static char index=0;
  int data = analogRead(MEASURE_VOLTAGE_PIN);
  float voltage = data*5.0f/1023.0f/0.362f;
  history_voltage[index++] = voltage;
  index = (index == 3)?0:index;
  
  /*
  lcd.setCursor(0,0);
  int va1 = (int)(voltage * 100);
  int va4 = va1%10;
  int va3 = (va1 - va4)/10%10;
  int va2 = (va1 - va4 - va3*10)/100;
  sprintf(lcd_buffer,"%2d.%d%dV; ",va2, va3, va4); //ocupy 8 chars.
  lcd.print(lcd_buffer);
  */

  if(voltage > 6.0f && index == 0) //only checking 3x18650 battery
  {     
    voltage    = 0;
    char count =0;
    for(char i=0; i<3; i++)
    {
      if(history_voltage[i])
      {
         voltage += history_voltage[i];
         count ++;
      }
    }
    voltage /= count;

  lcd.setCursor(0,0);
  int va1 = (int)(voltage * 100);
  int va4 = va1%10;
  int va3 = (va1 - va4)/10%10;
  int va2 = (va1 - va4 - va3*10)/100;
  sprintf(lcd_buffer,"%2d.%d%dV; ",va2, va3, va4); //ocupy 8 chars.
  lcd.print(lcd_buffer);
  
    //power down if < 9v.
    if(voltage <= 9.0f )
    {
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("POWER DOWN Due");
      lcd.setCursor(0,1);
      lcd.print("To Low Battery.");
      lcd.noBacklight();
      powerDownCPU();  
      return;
    }
    
    if(voltage < 12.0f)
    {
      MOTOR_FAST_SPEED = 190;
    }
  }

  if(Serial.available()>0)// will for openwrt router controller.
  {
    Serial.print("[arduino] ");
    char val=Serial.read(); //
    Serial.print("received: "); 
    Serial.println(val); //
  }
  //autoPilot();
  //followThePath();
  run_the_car();  


} 


