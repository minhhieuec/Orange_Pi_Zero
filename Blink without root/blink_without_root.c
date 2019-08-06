/*
 * blink.c:
 *  Standard "blink" program in wiringPi. Blinks an LED connected
 *  to the first GPIO pin.
 *
 * Copyright (c) 2012-2013 Gordon Henderson. <projects@drogon.net>
 ***********************************************************************
 * This file is part of wiringPi:
 *  https://projects.drogon.net/raspberry-pi/wiringpi/
 *
 *    wiringPi is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU Lesser General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    wiringPi is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public License
 *    along with wiringPi.  If not, see <http://www.gnu.org/licenses/>.
 ***********************************************************************
 */

#include <stdio.h>
#include <wiringPi.h>
#include <stdlib.h>

// blink gpio 11 (pin 26 on orange pi zero)

#define LED 11

int main (void)
{
  wiringPiSetupSys();
  system("gpio export 11 out");

  for (;;)
  {
    system("gpio write 11 1");
    printf ("LED ON\n") ;
    delay (500) ;   // mS
    system("gpio write 11 0");
    printf ("LED OFF\n") ;
    delay (500) ;
  }
  return 0 ;
}
