#include <Arduino.h>
#include <SPI.h>
#include <U8g2lib.h>

#ifndef OLED_H
#define OLED_H

class OLED{
private:
  U8G2_SSD1322_NHD_256X64_1_4W_SW_SPI screen_object;
  int screen_height;
  int screen_width;
  volatile int cursor_x;
  volatile int cursor_y;
  volatile int string_width;
  volatile unsigned long time_millis;
  const double speed_constant = 0.06098735739; // (d*pi*60)/63300
  const int motor_gear_ratio = 4;
  const double wheel_gear_ratio = 0.8;
  volatile double* rpm_address;
  void set_cursors();

public:
  OLED();
  void init(volatile double* rpm);
  void display_breaking();
  void display_rpm();
  void display_speed();
  void display_rotate();
};

#endif
