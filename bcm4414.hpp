/*
 * Copyright (c) 2019 Branilson Luiz
 * BCM4414.cpp - Class file for the BCM4414xG0F4440yzz Bus Converter.
 * Version: 1.0.0
 * 
 * joao dot abreu dot filho (at) hotmail dot com
 * Github: https://github.com/joaoabreufilho/pmbus
 * 
 * This program is free software: you can redistribute it and/or modify it un-
 * der the terms of the version 3 GNU General Public License as published by
 * the Free Software Foundation.
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FIT-
 * NESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef BCM4414_HPP_
#define BCM4414_HPP_

#include "mbed.h"
#include "pmbusdev.hpp"




class bcm4414 : public pmbus_dev {
 public:
  bcm4414(uint8_t dev_page, I2C& i2c, uint8_t address, int frequency);

//  int get_MFR_VIN_MAX(unsigned char* vin_MaxData);
  int get_MFR_VIN_MAX(void);
  int get_MFR_VOUT_MIN(void);
  int get_MFR_VOUT_MAX(void);
  int get_MFR_IOUT_MAX(void);
  int get_MFR_POUT_MAX(void);

  int read_K_FACTOR(void);
  int read_BCM_ROUT(void);
  int set_ALL_THRESHOLDS(void);
  int set_disable_fault(uint16_t disableflags);

 private:



};


#endif  // BCM4414_HPP_
