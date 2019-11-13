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



#include "pmbusdev.hpp"
#include "pmbuscore.hpp"
// Functions

pmbus_dev::pmbus_dev(uint8_t dev_page, I2C& i2c, uint8_t address, int frequency): smbus_dev(i2c, address, frequency)  {
  set_page(dev_page);
}

// Public Functions:

int pmbus_dev::OperationOn(void) {
  return write_byte_data(curr_page, PMBUS_OPERATION, 0x80);
}

int pmbus_dev::clear_faults(void) {
  return write_byte(curr_page, PMBUS_CLEAR_FAULTS);
}

int pmbus_dev::get_capability(void) {
  return read_byte_data(curr_page, PMBUS_CAPABILITY);
}

int pmbus_dev::set_OT_FAULT_LIMIT(uint16_t otflimit) {
  return write_word_data(curr_page, PMBUS_OT_FAULT_LIMIT, otflimit);
}

int pmbus_dev::set_OT_WARN_LIMIT(uint16_t otwlimit) {
  return write_word_data(curr_page, PMBUS_OT_WARN_LIMIT, otwlimit);
}

int pmbus_dev::set_VIN_OV_FAULT_LIMIT(uint16_t hvflimit) {
  return write_word_data(curr_page, PMBUS_VIN_OV_FAULT_LIMIT, hvflimit);
}

int pmbus_dev::set_VIN_OV_WARN_LIMIT(uint16_t hvwlimit) {
  return write_word_data(curr_page, PMBUS_VIN_OV_WARN_LIMIT, hvwlimit);
} 

int pmbus_dev::set_IIN_OC_FAULT_LIMIT(uint16_t hvcflimit) {
  return write_word_data(curr_page, PMBUS_IIN_OC_FAULT_LIMIT, hvcflimit);
}

int pmbus_dev::set_IIN_OC_WARN_LIMIT(uint16_t hvcwlimit) {
  return write_word_data(curr_page, PMBUS_IIN_OC_WARN_LIMIT, hvcwlimit);
}

int pmbus_dev::set_TON_DELAY(uint16_t startupdelay) {
  return write_word_data(curr_page, PMBUS_TON_DELAY, startupdelay);
}


int pmbus_dev::get_STATUS_BYTE(void) {
  return read_byte_data(curr_page, PMBUS_STATUS_BYTE);
}

int pmbus_dev::get_STATUS_WORD(void) {
  return read_word_data(curr_page, PMBUS_STATUS_WORD);
}

int pmbus_dev::get_STATUS_IOUT(void) {
  return read_byte_data(curr_page, PMBUS_STATUS_IOUT);
}

int pmbus_dev::get_STATUS_INPUT(void) {
  return read_byte_data(curr_page, PMBUS_STATUS_INPUT);
}

int pmbus_dev::get_STATUS_TEMPERATURE(void) {
  return read_byte_data(curr_page, PMBUS_STATUS_TEMPERATURE);
}

int pmbus_dev::get_STATUS_CML(void) {
  return read_byte_data(curr_page, PMBUS_STATUS_CML);
}

int pmbus_dev::get_STATUS_MFR_SPECIFIC(void) {
  return read_byte_data(curr_page, PMBUS_STATUS_MFR_SPECIFIC);
}

int pmbus_dev::READ_VIN(void) {
  return read_word_data(curr_page, PMBUS_READ_VIN);
}

int pmbus_dev::READ_IIN(void) {
  return read_word_data(curr_page, PMBUS_READ_IIN);
}
int pmbus_dev::READ_VOUT(void) {
  return read_word_data(curr_page, PMBUS_READ_VOUT);
}

int pmbus_dev::READ_IOUT(void) {
  return read_word_data(curr_page, PMBUS_READ_IOUT);
}

int pmbus_dev::READ_TEMPERATURE_1(void) {
  return read_word_data(curr_page, PMBUS_READ_TEMPERATURE_1);
}

int pmbus_dev::READ_POUT(void) {
  return read_word_data(curr_page, PMBUS_READ_POUT);
}

int pmbus_dev::get_MFR_ID(unsigned char* mfr_id) {
  return read_block_data(curr_page, PMBUS_MFR_ID, 3, mfr_id);
}

int pmbus_dev::get_MFR_MODEL(unsigned char* model_data) {
  return read_block_data(curr_page, PMBUS_MFR_MODEL, 19, model_data);
}

int pmbus_dev::get_PMBUS_REVISION(unsigned char* rev_data) {
  return read_block_data(curr_page, PMBUS_MFR_REVISION, 19, rev_data);
}

int pmbus_dev::get_MFR_LOCATION(unsigned char* local_data) {
  return read_block_data(curr_page, PMBUS_MFR_LOCATION, 3, local_data);
}


int pmbus_dev::get_MFR_DATE(unsigned char* date_data) {
  return read_block_data(curr_page, PMBUS_MFR_DATE, 5, date_data);
}

int pmbus_dev::get_MFR_SERIAL(unsigned char* serial_data) {
  return read_block_data(curr_page, PMBUS_MFR_SERIAL, 17, serial_data);
}


int pmbus_dev::get_MFR_VIN_MIN(void) {
  return read_word_data(curr_page, PMBUS_MFR_ID);
}


// Private Functions:

  int pmbus_dev::set_page(uint8_t page) {
    int rv = 0;
    int newpage;
    if (page != curr_page) {
      rv = i2c_smbus_write_byte_data(PMBUS_PAGE, page);
      newpage = i2c_smbus_read_byte_data(PMBUS_PAGE);
      if (newpage != page)
        rv = -EIO;
      else
        curr_page = page;
    }
    return rv;
  }

  int pmbus_dev::write_byte(int page, uint8_t value) {
    int rv;

    if (page >= 0) {
      rv = set_page(page);
      if (rv < 0)
        return rv;
    }
    return  i2c_smbus_write_byte(value);
  }

  int pmbus_dev::write_word_data(uint8_t page, uint8_t reg, uint16_t word) {
    int rv;

    rv = set_page(page);
    if (rv < 0)
      return rv;

    return i2c_smbus_write_word_data(reg, word);
  }

  int pmbus_dev::read_word_data(uint8_t page, uint8_t reg) {
    int rv;

    rv = set_page(page);
    if (rv < 0)
      return rv;

    return i2c_smbus_read_word_data(reg);
  }
 
  int pmbus_dev::read_block_data(uint8_t page, uint8_t reg, uint8_t length, uint8_t *values) {
    int rv;

    rv = set_page(page);
    if (rv < 0)
      return rv;

    return i2c_smbus_read_i2c_block_data(reg, length, values);
  }

  int pmbus_dev::write_block_data(uint8_t page, uint8_t , uint8_t length, uint8_t *values) {
    int rv;

    rv = set_page(page);
    if (rv < 0)
      return rv;

    return i2c_smbus_write_i2c_block_data(reg, length, values);
  }


  int pmbus_dev::read_byte_data(int page, uint8_t reg) {
    int rv;

    if (page >= 0) {
      rv = set_page(page);
      if (rv < 0)
        return rv;
    }

    return i2c_smbus_read_byte_data(reg);
  }

  int pmbus_dev::write_byte_data(int page, uint8_t reg, uint8_t value) {
    int rv;

    rv = set_page(page);
    if (rv < 0)
      return rv;

    return i2c_smbus_write_byte_data(reg, value);
  }

  int pmbus_dev::update_byte_data(int page, uint8_t reg, uint8_t mask, uint8_t value) {
    unsigned int tmp;
    int rv;

    rv = read_byte_data(page, reg);
    if (rv < 0)
      return rv;

    tmp = (rv & ~mask) | (value & mask);

    if (tmp != rv)
      rv = write_byte_data(page, reg, tmp);

    return rv;
  }

  void pmbus_dev::clear_fault_page(int page) {
    write_byte(page, PMBUS_CLEAR_FAULTS);
  }
/*
  void pmbus_dev::clear_faults() {
    int i;

    for (i = 0; i < data->info->pages; i++)
      clear_fault_page(i);
  }


  bool pmbus_dev::check_byte_register(int page, int reg){

  }

  bool pmbus_dev::check_word_register(int page, int reg){
    
  }

*/