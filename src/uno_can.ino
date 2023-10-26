// SEM2022 Folder

#include <circular_buffer.h>
#include <FlexCAN_T4.h>
#include <imxrt_flexcan.h>
#include <kinetis_flexcan.h>

#include <TimerOne.h>

#include <Bounce2.h>
// MODE CONTROL MACROS


#define POT_CONTROL
#define SERIAL_CONTROL

#define PEDAL_POT

#define DRIVING_LEFT true
#define DRIVING_RIGHT true

#define WAITFOR_INIT


// MOTOR MACROS

#define LEFT_MOTOR 0xF0
#define RIGHT_MOTOR 0x0F


// CAN MESSAGE IDS

#define SHUTDOWN                0x00
#define FATAL_ERROR             0x01
#define HEALTH_CHECK            0x02
#define CONTROLLER_OC           0x03
#define OC_RESET                0x04
#define INIT_COMPLETE_NOTIFY    0x05
#define CAN_PING                0x06
#define SHUTDOWN_ACK            0x07

#define INIT_MOTOR_PARAMS       0x10
#define ACCELERATE              0x11
#define BRAKE_APPLY             0x12
#define SET_MOTOR_DIRECTION     0x13
#define BRAKE_END               0x14

#define GET_MOTOR_SPEED         0x20
#define GET_PHASE_CURRENT       0x21


#define SEND_MOTOR_SPEED     0x30

#define BRAKE         11
#define ACCEL         40
#define DIRECTION

FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2;

CAN_message_t msg2;
CAN_message_t rx_msg2;

byte left_speed_mph = 0;
byte right_speed_mph = 0;

int8_t char1, char2;

int count = 0;

int8_t instruction = 0;

Bounce debouncer = Bounce();

void send_pot_speed(void);
void brake(void);
void read_instruction(void);

/** SETUP **/ 

void setup(void) {

  pinMode(BRAKE, INPUT);
  
  can2.begin();
  can2.setBaudRate(50000);

  Serial7.begin(9600);
  delay(3000);


debouncer.attach(BRAKE, INPUT_PULLDOWN);
debouncer.interval(25);

Serial.print("POT VAL: "); Serial.println(msg2.buf[0], HEX);
Serial.println("WAITING FOR MOTORS!");
#ifdef WAITFOR_INIT
  // Wait for both motor initializations to complete
  volatile bool left_motor_ready = false;
  volatile bool right_motor_ready = false;
  while(!(left_motor_ready && right_motor_ready)){
    if (can2.read(rx_msg2)){
      if (rx_msg2.id = INIT_COMPLETE_NOTIFY){
        if (rx_msg2.buf[0] == LEFT_MOTOR && !left_motor_ready){
          Serial.println("LEFT MOTOR RESPONDED");
          left_motor_ready = true;
          
        }
        else if (rx_msg2.buf[0] == RIGHT_MOTOR && !right_motor_ready){
          Serial.println("RIGHT MOTOR RESPONDED");
          right_motor_ready = true;
        }
          
      }  
    }
  }
#endif
Serial.println("MOTORS INITIALIZATION COMPLETE!");
msg2.len = 1;
msg2.id = INIT_MOTOR_PARAMS;
msg2.buf[0] = 0xFF;
can2.write(msg2);

msg2.len = 1;
msg2.id = SET_MOTOR_DIRECTION;
msg2.buf[0] = 0xFF;
can2.write(msg2);

Serial.println("MOTORS ENABLED");




  //attachInterrupt(BRAKE, brake, CHANGE);
     
#ifdef POT_CONTROL
  Timer1.initialize(450000);
  Timer1.attachInterrupt(send_pot_speed);
  #endif
    
}




/** LOOP **/

void loop() {
 debouncer.update();
  if(debouncer.fell() | debouncer.rose()) {
    brake();
  }

  
#ifdef SERIAL_CONTROL
if (Serial.available()){
  noInterrupts();
      read_instruction();
      msg2.len = 1;
      msg2.id = instruction;
      can2.write(msg2);
    interrupts();
      
}
#endif

if (can2.read(rx_msg2)){
#ifdef SERIAL_CONTROL
    Serial.print("RECEIVED ID: "); Serial.println(rx_msg2.id, HEX);
    int i = 0;
    for (i = 0; i < rx_msg2.len; i++){
      Serial.print("RECEIVED CHAR "); Serial.print(i); Serial.print(": ");Serial.println(rx_msg2.buf[i], HEX);  
    }
#endif
    process_can_rx();
  
}
   


}

void process_can_rx() {
  switch(rx_msg2.id){
    case SEND_MOTOR_SPEED:
      unsigned int rpm = (int) (15000 / ((rx_msg2.buf[1] * 256) + rx_msg2.buf[2]));
      if (rx_msg2.buf[0] == LEFT_MOTOR) {
         left_speed_mph = (byte)((unsigned int)rpm / 15);
         Serial.print("LEFT MOTOR RPM: "); 
      } else if(rx_msg2.buf[0] == RIGHT_MOTOR) {
        right_speed_mph = (byte)((unsigned int)rpm / 15);
        Serial.print("RIGHT MOTOR RPM: ");
      } else {
        Serial.println("MOTOR ID NOT RECOGNIZED");
      }
      
      Serial.println(rpm);
      break;


   case SHUTDOWN_ACK:
    if (rx_msg2.buf[0] == LEFT_MOTOR)
      Serial.print("LEFT MOTOR SHUTDOWN ACKNOWLEDGED");
    else if (rx_msg2.buf[0] == RIGHT_MOTOR)
      Serial.print("RIGHT MOTOR SHUTDOWN ACKNOWLEDGED");
    else 
      Serial.print("SHUTDOWN NOT ACKOWLEDGED");
    break;
  }
}


void send_pot_speed(void) {

    static byte test_speed = 0;

    static bool wasBraking = false;

      left_speed_mph = 9;
      right_speed_mph = 9;
      
      byte speed_mph = (byte)((left_speed_mph + right_speed_mph)/2);

      //if (speed_mph == 10) Serial.println("NICE");
     
//      Serial7.write(test_speed);
//      Serial.print("To display: "); Serial.write(test_speed);Serial.println();

      test_speed++;
      if (test_speed > 99) {
        test_speed = 0;
      }
  
      static uint32_t last = 1;
      
      
      uint32_t tempp = analogRead(ACCEL);
      //Serial.println(tempp);
      if (tempp > 355) {
      tempp = (uint32_t)(tempp / 4.03 + 1);
#ifdef PEDAL_POT
      //tempp = (uint32_t)(tempp - 0x54) * 4;
      tempp = (uint32_t)(tempp - 0x59) * 7;
#endif
  
      if (tempp < 1)
        tempp = 1;
       if (tempp > 0xFE)
        tempp = 0xFE; 

        
      
      if (tempp >  last + 0x30){
        last = tempp;
        return;
      }
      last = tempp;
       msg2.len = 1;
       
      //Serial.println("SEND SPEED"); 
      
        if (wasBraking) {
          Serial.println("FOOT ON");
          msg2.buf[0] = 0;
          msg2.id = BRAKE_END;
          wasBraking = false;
        } else {
        msg2.id = ACCELERATE;
        msg2.buf[0] = tempp;
        Serial.print("POT VAL: "); Serial.println(msg2.buf[0], HEX);
        }
      } else {
        wasBraking = true;
        Serial.println("FOOT OFF");
        msg2.buf[0] = 0;
        msg2.id = BRAKE_APPLY;
      }

      can2.write(msg2);
      
//      msg2.len = 1;
//      msg2.id = GET_MOTOR_SPEED;
//      msg2.buf[0] = LEFT_MOTOR;
//      can2.write(msg2);
//      msg2.buf[0] = RIGHT_MOTOR;
      
}

void send_speed_display(void) {
  
}

void brake(void) {
 
   msg2.len = 1;
   msg2.buf[0] = 0;
  if (digitalRead(BRAKE) == LOW) {     // brake apply
    Timer1.detachInterrupt();
    msg2.id = BRAKE_APPLY;
    Serial.println("BRAKES APPLIED");
    
  } else {
    Timer1.attachInterrupt(send_pot_speed);
    msg2.id = BRAKE_END;
    Serial.println("BRAKES RELEASED");
  }
  can2.write(msg2);
  
}



void read_instruction(){
  int8_t temp1 = Serial.read();
  int8_t temp1_;
  if (temp1 >= 48 && temp1 <=57){
    temp1_ = temp1 - 48;
  } else if (temp1 >= 65 && temp1 <=70) {
    temp1_ = temp1 - 55;
  }
  int8_t temp2 = Serial.read();
  int8_t temp2_;
  if (temp2 >= 48 && temp2 <=57){
    temp2_ = temp2 - 48;
  } else if (temp2 >= 65 && temp2 <=70) {
    temp2_ = temp2 - 55;
  }

  instruction = temp1_ * 16 + temp2_;
  
  Serial.read();

  int8_t temp3 = Serial.read();
  int8_t temp3_;
  if (temp3 >= 48 && temp3 <=57){
    temp3_ = temp3 - 48;
  } else if (temp3 >= 65 && temp3 <=70) {
    temp3_ = temp3 - 55;
  }
  int8_t temp4 = Serial.read();
  int8_t temp4_;
  if (temp4 >= 48 && temp4 <=57){
    temp4_ = temp4 - 48;
  } else if (temp4 >= 65 && temp4 <=70) {
    temp4_ = temp4 - 55;
  }

  msg2.buf[0] = temp3_ * 16 + temp4_;

   Serial.print("MESSAGE ID: "); Serial.println(instruction, HEX);
    Serial.print(temp1_, HEX); Serial.println(temp2_, HEX);
   Serial.print("MESSAGE DATA "); Serial.println(msg2.buf[0], HEX);
   Serial.print(temp3_, HEX); Serial.println(temp4_, HEX);  
  
  
}
