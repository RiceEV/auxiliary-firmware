#include <Arduino.h>
#include <OLED.h>
#include <SPI.h>
#include <U8g2lib.h>

OLED::OLED()
    : screen_object(U8G2_R0, /* clock=*/ 14, /* data=*/ 3, /* cs=*/ 4, /* dc=*/ 5, /* reset=*/ 6)
{
  screen_height = 0;
  screen_width = 0;
  cursor_x = 0;
  cursor_y = 0;
  time_millis = 0;
}

void OLED::set_cursors()
{
  /*
  With the current string width, set the cursors. The screen object should be
  set to center position. This cursor setting ensures that the string displayed
  is at center of the screen
  */
  cursor_x = (screen_width - string_width) / 2;
  cursor_y = screen_height / 2;
}

void OLED::init(volatile double* rpm)
{
  /*
  Take in a pointer to rpm and initializa the screen
  */
  rpm_address = rpm;
  screen_object.begin();
  screen_object.setFont(u8g2_font_inr24_mf);
  // Get initial information of the screen
  screen_height = screen_object.getDisplayHeight();
  screen_width = screen_object.getDisplayWidth();
  // Display welcoming message
  string_width = screen_object.getStrWidth("Hello World!");
  screen_object.setFontPosCenter();
  OLED::set_cursors();
  screen_object.firstPage();
  do
  {
    screen_object.setCursor(cursor_x, cursor_y);
    screen_object.print("Hello World!");
  } while (screen_object.nextPage());
}

void OLED::display_breaking()
{
  /*
  Display the message "Breaking"
  */
  screen_object.setFont(u8g2_font_inr38_mf);
  string_width = screen_object.getStrWidth("Breaking");
  OLED::set_cursors();
  screen_object.firstPage();
  do
  {
    screen_object.setCursor(cursor_x, cursor_y);
    screen_object.print("Breaking");
  } while (screen_object.nextPage());
}

void OLED::display_rpm()
{
  /*
  Display the current motor rotation speed in terms of rotation per minute
  */
  screen_object.setFont(u8g2_font_inr38_mf);
  int now_rpm = *rpm_address;
  // Need to multiply by the gear ratio between motor and encoder
  now_rpm *= motor_gear_ratio;
  // Haven't thought of a better way to get string length. It is a placeholder.
  string_width = screen_object.getStrWidth("1000 rpm");
  OLED::set_cursors();
  screen_object.firstPage();
  do
  {
    screen_object.setCursor(cursor_x, cursor_y);
    screen_object.print(now_rpm);
    screen_object.print(" rpm");
  } while (screen_object.nextPage());
}

void OLED::display_speed()
{
  /*
  Display current speed of the car in terms of mile per hour
  */
  screen_object.setFont(u8g2_font_inr49_mf);
  int now_speed = *rpm_address;
  // Haven't thought of a better way to get string length. It is a placeholder.
  string_width = screen_object.getStrWidth("60 mph");
  OLED::set_cursors();
  screen_object.firstPage();
  do
  {
    screen_object.setCursor(cursor_x, cursor_y);
    screen_object.print(now_speed);
    screen_object.print(" mph");
  } while (screen_object.nextPage());
}

void OLED::display_rotate()
{
  /*
  Display rpm of motor and speed of car in turn with an interval of 5 seconds
  */
  time_millis = millis();
  if ((time_millis / 1000) % 10 < 5)
  {
    OLED::display_speed();
  } else
  {
    OLED::display_rpm();
  }
}
