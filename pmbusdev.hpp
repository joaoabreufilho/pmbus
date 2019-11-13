/*
 * Copyright (c) 2019 Joao Venancio Abreu Filho
 * PMBUSDEV.hpp - Class file for PMBus devices communication.
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

  This code was based on a couple of other repos:

 * 
 * https://github.com/markosilla/pmbus
 * Linux Driver adaptations tp pmbus devices and driver.
 * 
 * AND
 * 
 * https://github.com/Michael-Equi/PMBus
 * A python library for interfacing with devices that are based on the PMBus Specification
 */


#ifndef PMBUS_HPP_
#define PMBUS_HPP_

#include "mbed.h"
// SMBUS Derivations
#include "smbus.hpp"
#include "pmbuscore.hpp"


/* Regulator ops */
extern struct regulator_ops pmbus_regulator_ops;

struct pmbus_sensor {
  struct pmbus_sensor *next;
  char name[PMBUS_NAME_SIZE]; /* sysfs sensor name  (Protocol sensor name)*/
  uint8_t page; /* page number */
  uint16_t reg; /* register */
  enum pmbus_sensor_classes sensor_class;  /* sensor class */
  bool update;  /* runtime sensor update needed */
  int data; /* Sensor data Negative if there was a read error */
};

struct pmbus_data {
  uint32_t flags; /* from platform data */

  int exponent[PMBUS_PAGES];  /* linear mode: exponent for output voltages */

  const struct pmbus_driver_info *info;

  int max_attributes;
  int num_attributes;

  struct pmbus_sensor *sensors;
  bool valid;
  unsigned  long last_updated;  /* in jiffies */

  /*
    * A single status register covers multiple attributes,
    * so we keep them all together.
    */
  uint8_t status[PB_NUM_STATUS_REG];
  uint8_t status_register;

  uint8_t currpage;
};

class pmbus_dev : public smbus_dev {
 public:
  // PMBus Functions
  pmbus_dev(uint8_t dev_page, I2C& i2c, uint8_t address, int frequency);
  uint8_t curr_page = 0xFF; /* page number */

  int OperationOn(void);
  int clear_faults(void);
  int get_capability(void);
  int set_page(uint8_t page);

  int set_OT_FAULT_LIMIT(uint16_t otflimit);
  int set_OT_WARN_LIMIT(uint16_t otwlimit);
  int set_VIN_OV_FAULT_LIMIT(uint16_t hvflimit);
  int set_VIN_OV_WARN_LIMIT(uint16_t hvwlimit);
  int set_IIN_OC_FAULT_LIMIT(uint16_t hvcflimit);
  int set_IIN_OC_WARN_LIMIT(uint16_t hvcwlimit);
  int set_TON_DELAY(uint16_t startupdelay);

  int get_STATUS_BYTE(void);
  int get_STATUS_WORD(void);
  int get_STATUS_IOUT(void);
  int get_STATUS_INPUT(void);
  int get_STATUS_TEMPERATURE(void);
  int get_STATUS_CML(void);
  int get_STATUS_MFR_SPECIFIC(void);

  int READ_VIN(void);
  int READ_IIN(void);
  int READ_VOUT(void);
  int READ_IOUT(void);
  int READ_TEMPERATURE_1(void);
  int READ_POUT(void);

  int get_PMBUS_REVISION(unsigned char* rev_data);
  int get_MFR_ID(unsigned char* mfr_id);
  int get_MFR_MODEL(unsigned char* model_data);
  int get_MFR_LOCATION(unsigned char* local_data);
  int get_MFR_DATE(unsigned char* date_data);
  int get_MFR_SERIAL(unsigned char* serial_data);

  int get_MFR_VIN_MIN(void);
  int get_MFR_VIN_MAX(void);
  int get_MFR_VOUT_MIN(void);
  int get_MFR_VOUT_MAX(void);
  int get_MFR_IOUT_MAX(void);
  int get_MFR_POUT_MAX(void);

  int READ_BCM_ROUT(void);

  int SET_ALL_THRESHOLDS(void);
  int DISABLE_FAULT(void);

  int write_byte(int page, uint8_t value);
  int write_word_data(uint8_t page, uint8_t reg, uint16_t word);
  int read_word_data(uint8_t page, uint8_t reg);
  int read_byte_data(int page, uint8_t reg);
  int write_byte_data(int page, uint8_t reg, uint8_t value);
  int read_block_data(uint8_t page, uint8_t reg, uint8_t length, uint8_t *values);
  int write_block_data(uint8_t page, uint8_t reg, uint8_t length, uint8_t *values);



/*
// This would be a way to dynamically add pmbus sensors with a linked list
struct pmbus_sensor *pmbus_add_sensor(struct pmbus_data *data,
  const char *name, const char *type,
  int seq, int page, int reg,
  enum pmbus_sensor_classes sensor_class,
  bool update, bool readonly);
*/
 private:
  uint8_t page_dev;

  uint16_t reg = 0; /* register */
  struct pmbus_sensor sensor_data;
  int update_byte_data(int page, uint8_t reg, uint8_t mask, uint8_t value);



  void clear_fault_page(int page);


  // PMBUS wrapper funcions
  int _decodePMBus(char *message);
  int _encodePMBus(char *message);

};


#endif /* PMBUS_H */