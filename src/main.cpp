#include <Arduino.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <OLED.h>
#include <PID_v1.h>
#include <pins.h>
#include <SPI.h>

// Clearing confusion in dirction
#define FORWARD true;
#define BACKWARD false;

typedef struct Data {
  // Setpoint or currentpoint
  unsigned int point;
  // Brake
  bool brake;
  // Direction, true forward and false backward
  bool direction;
  // Error
  bool error;
} Data;

OLED OLED_screen;
volatile double speed;

/**
* Rules for messages below:
* 4MSB is for operands, 12 lower bits are for setpoints
* 4MSB are arranged as below: Brake, Direction, Error, Empty
*/
volatile unsigned int message_A_out;
volatile unsigned int message_A_in = 0;
volatile unsigned int message_B_out;
volatile unsigned int message_B_in = 0;

// Record of break state
volatile bool brk = false;

volatile bool direction = FORWARD;

volatile bool error = false;

volatile bool error_A;
volatile bool error_B;

// Pedal sensing
volatile int pedal_adc;
volatile double pedal_power;

// PID loops
// double pid_inA, pid_outA, pid_setA;
// double pid_inB, pid_outB, pid_setB;
// PID pidA(&pid_inA, &pid_outA, &pid_setA, 1, 1, 0, REVERSE);
// PID pidB(&pid_inB, &pid_outB, &pid_setB, 1, 1, 0, REVERSE);

// The timer to switch on and off
IntervalTimer clock_timer;

IntervalTimer SPI_timer;

// Record for blinking the lights
int light_state = HIGH;

// For testing usage
const bool testing = true;

void blink();
// void pidSetup();
void brake_isr();
void get_and_send();
Data create_data(unsigned int point, bool brake, bool direction, bool error);
Data decode_data(unsigned int message);
unsigned int encode_data(Data current_data);
void print_data(const char* message, Data data);

void setup() {
  // Set pinmode
  pinMode(CLK, OUTPUT);
  pinMode(CS_A, OUTPUT);
  pinMode(CS_B, OUTPUT);

  // Begin transimitting to serial
  Serial.begin(9600);
  Serial.println("Starting");

  // disable interrupts
  cli();
  // attachInterrupt(FAULT_IN, fault_catch, CHANGE);
  attachInterrupt(BRAKE_SNS, brake_isr, CHANGE);
  SPI_timer.begin(get_and_send, 250000);
  // enable interrups
  sei();

  // Start timer
  clock_timer.begin(blink, 500000);
  clock_timer.priority(200);

  // Start the SPI
  SPI.begin();
  SPI1.begin();

  // Start PID setup
  // pidSetup();

  // Set up OLED screen
  OLED_screen.init(&speed);
}

void loop() {
  Data from_A = decode_data(message_A_in);
  Data from_B = decode_data(message_B_in);
  error_A = from_A.error;
  error_B = from_B.error;
  error = error_A || error_B;
  if (testing)
  {
    //print_data("Motor A -------", from_A);
    //print_data("Motor B -------", from_B);
  }
  delay(100);
}

/**
 * Blink the headlights
 */
void blink()
{
  // Serial.println("Blinking");
  digitalWrite(CLK, light_state);
  if (light_state == HIGH)
  {
    light_state = LOW;
  } else
  {
    light_state = HIGH;
  }
  // Serial.println(light_state);
}

// void pidSetup() {
//   pidA.SetOutputLimits(0, 4096);
//   pidB.SetOutputLimits(0, 4096);
//   pidA.SetMode(AUTOMATIC);
//   pidB.SetMode(AUTOMATIC);
//   Serial.println("Setup done");
// }

/**
 * If brake change is detected, let it be false
 */
void brake_isr() {
  cli();
  if (brk)
  {
    brk = false;
    sei();
  }
  else
  {
    brk = true;
    message_A_out = 1 << 15;
    message_B_out = 1 << 15;
    digitalWrite(CS_A, LOW);
    digitalWrite(CS_B, LOW);
    message_A_in = SPI.transfer16(message_A_out);
    message_B_in = SPI1.transfer16(message_B_out);
    digitalWrite(CS_A, HIGH);
    digitalWrite(CS_B, HIGH);
    sei();
  }
}

void get_and_send()
{
  Serial.println("GET AND SEND");
        if (testing)
        {
          // pedal_power = 3000;
          // direction = BACKWARD;
          // Serial.println("Sending data --------------------------");
          pedal_adc = analogRead(PEDAL_SNS);
          pedal_power = map(pedal_adc, 550, 900, 0, 4095);
        }
        else
        {
          pedal_adc = analogRead(PEDAL_SNS);
          pedal_power = map(pedal_adc, 550, 900, 0, 4095);
        }
        cli();
        Data send_A = create_data(pedal_power, brk, direction, error);
        Data send_B = create_data(pedal_power, brk, direction, error);
        message_A_out = encode_data(send_A);
        message_B_out = encode_data(send_B);
        if (testing)
        {
          print_data("Sending A -------", send_A);
          print_data("Sending B -------", send_B);
        }
        digitalWrite(CS_A, LOW);
        digitalWrite(CS_B, LOW);
        message_A_in = SPI.transfer16(message_A_out);
        message_B_in = SPI1.transfer16(message_B_out);
        digitalWrite(CS_A, HIGH);
        digitalWrite(CS_B, HIGH);
        sei();
}

/**
 * Create a "Data" structure with information needed
 */
Data create_data(unsigned int point, bool brake, bool direction, bool error)
{
  Data current_data;
  current_data.point = point;
  current_data.brake = brake;
  current_data.direction = direction;
  current_data.error = error;

  return current_data;
}

/**
 * Take in a encoded int, decode it as "Data" structure
 */
Data decode_data(unsigned int message)
{
  unsigned int point = message & 0xFFF; // Take 12 LSB
  bool brake = message >> 15; // Take MSB
  bool direction = (message >> 14) & 1; // Take 2nd MSB
  bool error = (message >> 13) & 1; // Take 3rd MSB
  Data current_data = create_data(point, brake, direction, error);
  return current_data;
}

/**
 * Take in a "Data" structure, encode it as a int
 */
unsigned int encode_data(Data current_data)
{
  // Set the point
  unsigned int message = current_data.point & 0xFFF;

  // Set the brake
  if (current_data.brake)
  {
    message |= (1 << 15);
  }

  // Set the direction
  if (current_data.direction)
  {
    message |= (1 << 14);
  }

  // Set the error
  if (current_data.error)
  {
    message |= (1 << 13);
  }

  return message;
}

void print_data(const char* message, Data data)
{
  Serial.println(message);
  Serial.print("Set point: ");
  Serial.println(data.point);
  Serial.print("Brake: ");
  Serial.println(data.brake ? "true" : "false");

  // Testing from button
  // Serial.print("Brake from pin: ");
  // Serial.println(digitalRead(BRAKE_SNS));

  // Testing from potentiometer
  //Serial.print("Pedal from pin: ");
  //Serial.println(analogRead(PEDAL_SNS));

  Serial.print("Direction: ");
  Serial.println(data.direction ? "forward" : "backward");
  Serial.print("Error: ");
  Serial.println(data.error ? "true" : "false");
}
