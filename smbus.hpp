/*
 * Copyright (c) 2019 Joao Venancio Abreu Filho
 * SMBUS.hpp - Class file for SMBus Devices.
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
 * 
 */


#ifndef SMBUS_HPP_
#define SMBUS_HPP_

#include "mbed.h"

#define I2C_NAME_SIZE 20
#define I2C_MODULE_PREFIX "i2c:"

/*
 * Data for SMBus Messages
 */
#define I2C_SMBUS_BLOCK_MAX 32  /* As specified in SMBus standard */


// Possible Flags to I2C CLient
#define I2C_CLIENT_PEC  0x04  /* Use Packet Error Checking */
#define I2C_CLIENT_TEN  0x10  /* we have a ten bit chip address ** Must equal I2C_M_TEN below */
#define I2C_CLIENT_SLAVE  0x20  /* we are the slave */
#define I2C_CLIENT_HOST_NOTIFY  0x40  /* We want to use I2C host notify */
#define I2C_CLIENT_WAKE 0x80  /* for board_info; true iff can wake */
#define I2C_CLIENT_SCCB 0x9000  /* Use Omnivision SCCB protocol Must match I2C_M_STOP|IGNORE_NAK */


/*!< Linux Based I2C Message                                */
struct i2c_msg {
  uint16_t  addr; /* slave address */
  uint16_t  flags;
#define I2C_M_RD  0x0001  /*  read data, from slave to master */
  /* I2C_M_RD is guaranteed to be 0x0001! */
#define I2C_M_TEN 0x0010  /* this is a ten bit chip address */
#define I2C_M_DMA_SAFE  0x0200  /* the buffer of this message is DMA safe */
  /* makes only sense in kernelspace */
  /* userspace buffers are copied anyway */
#define I2C_M_RECV_LEN  0x0400  /* length will be first received byte */
#define I2C_M_NO_RD_ACK 0x0800  /* if I2C_FUNC_PROTOCOL_MANGLING */
#define I2C_M_IGNORE_NAK  0x1000  /* if I2C_FUNC_PROTOCOL_MANGLING */
#define I2C_M_REV_DIR_ADDR  0x2000  /* if I2C_FUNC_PROTOCOL_MANGLING */
#define I2C_M_NOSTART 0x4000  /* if I2C_FUNC_NOSTART */
#define I2C_M_STOP  0x8000  /* if I2C_FUNC_PROTOCOL_MANGLING */
  uint16_t len; /*  msg length  */
  uint8_t *buf; /*  pointer to msg data */
};

// SMBUS DEFINITIONS
union i2c_smbus_data {
  uint8_t byte;
  uint16_t word;
  uint8_t block[I2C_SMBUS_BLOCK_MAX + 2]; /* block[0] is used for length */
  /* and one more for user-space compatibility */
};



class smbus_dev {
 public:
  smbus_dev(I2C& i2c, uint8_t address, int frequency);
  int setConfig(uint16_t flags, char* name);
  int i2c_smbus_read_byte(void);
  int i2c_smbus_write_byte(uint8_t value);
  int i2c_smbus_read_byte_data(uint8_t command);
  int i2c_smbus_write_byte_data(uint8_t command, uint8_t value);
  int i2c_smbus_read_word_data(uint8_t command);
  int i2c_smbus_write_word_data(uint8_t command, uint16_t value);
  int i2c_smbus_read_block_data(uint8_t command, uint8_t *values);
  int i2c_smbus_write_block_data(uint8_t command, uint8_t length, const uint8_t *values);
  int i2c_smbus_read_i2c_block_data(uint8_t command, uint8_t length, uint8_t *values);
  int i2c_smbus_write_i2c_block_data(uint8_t command, uint8_t length, const uint8_t *values);
  //  struct i2c_client *i2c_setup_smbus_alert( struct i2c_smbus_alert_setup *setup)


 private:
  I2C& _i2c;
  const uint8_t i2c_addr;
  const int i2c_freq;
  uint16_t i2c_flags = 0;
  char i2c_name[I2C_NAME_SIZE] = "i2cdev";


  int i2c_smbus_xfer_emulated(char read_write, uint8_t command, int size, union i2c_smbus_data *data);
  int i2c_transfer(struct i2c_msg *msgs, int num, bool pmbusread);
  int i2c_smbus_check_pec(uint8_t cpec, struct i2c_msg *msg);
  uint8_t crc8(uint16_t data);
  inline uint8_t i2c_8bit_addr_from_msg(const struct i2c_msg *msg);
  uint8_t i2c_smbus_pec(uint8_t crc, uint8_t *p, size_t count);
  uint8_t i2c_smbus_msg_pec(uint8_t pec, struct i2c_msg *msg);

};

#endif /* SMBUS_H */