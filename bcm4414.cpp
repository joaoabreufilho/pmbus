/*
 * Copyright (c) 2019 Joao Venancio Abreu Filho
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


#include "bcm4414.hpp"

/*
 * Virtual registers Redefinition.
 * Useful to support attributes which are not supported by standard PMBus
 * registers but exist as manufacturer specific registers on individual chips.
 * Must be mapped to real registers in device specific code.
 
 */

#define PMBUS_VIRT_BASE               0xA0
#define PMBUS_VIRT_READ_VIN_MIN       (PMBUS_VIRT_BASE + 0)
#define PMBUS_VIRT_READ_VIN_MAX       (PMBUS_VIRT_BASE + 6)
#define PMBUS_VIRT_READ_VOUT_MIN      (PMBUS_VIRT_BASE + 19)
#define PMBUS_VIRT_READ_VOUT_MAX      (PMBUS_VIRT_BASE + 20)
#define PMBUS_VIRT_READ_IOUT_MAX      (PMBUS_VIRT_BASE + 24)
#define PMBUS_VIRT_READ_POUT_MAX      (PMBUS_VIRT_BASE + 16)

#define PMBUS_READ_K_FACTOR           (0xD1)
#define PMBUS_READ_BCM_ROUT           (0xD4)
#define SET_ALL_THRESHOLDS            (0xD4)
#define DISABLE_FAULT                 (0xD4)

bcm4414::bcm4414(uint8_t dev_page, I2C& i2c, uint8_t address, int frequency): pmbus_dev(dev_page, i2c, address, frequency) {

}


int bcm4414::get_MFR_VIN_MAX(void) {
  return read_word_data(curr_page, PMBUS_VIRT_READ_VIN_MIN);
}

// int bcm4414::get_MFR_VIN_MAX(unsigned char* vin_MaxData) {
//  return read_block_data(curr_page, PMBUS_VIRT_READ_VOUT_MAX, 3, vin_MaxData);
// }

int bcm4414::get_MFR_VOUT_MIN(void) {
  return read_word_data(curr_page, PMBUS_VIRT_READ_VOUT_MIN);

}
int bcm4414::get_MFR_VOUT_MAX(void) {
  return read_word_data(curr_page, PMBUS_VIRT_READ_VOUT_MAX);
}

int bcm4414::get_MFR_IOUT_MAX(void) {
  return read_word_data(curr_page, PMBUS_VIRT_READ_IOUT_MAX);
}

int bcm4414::get_MFR_POUT_MAX(void) {
  return read_word_data(curr_page, PMBUS_VIRT_READ_POUT_MAX);

}

int bcm4414::read_K_FACTOR(void) {
  return read_word_data(curr_page, PMBUS_READ_K_FACTOR);
}

int bcm4414::read_BCM_ROUT(void) {
  return read_word_data(curr_page, PMBUS_READ_BCM_ROUT);
}

int bcm4414::set_ALL_THRESHOLDS(void) {
  // Finish This Function
}

int bcm4414::set_disable_fault(uint16_t disableflags) {
  return write_word_data(curr_page, DISABLE_FAULT, disableflags);
}
