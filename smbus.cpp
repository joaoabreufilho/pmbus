/*
 * Copyright (c) 2019 Joao Venancio Abreu Filho
 * SMBUS.cpp - Class file for SMBus Devices..
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


#include "smbus.hpp"


/* i2c_smbus_xfer read or write markers */
#define I2C_SMBUS_READ  1
#define I2C_SMBUS_WRITE 0

/* SMBus transaction types (size parameter in the above functions)
   Note: these no longer correspond to the (arbitrary) PIIX4 internal codes! */
#define I2C_SMBUS_QUICK 0
#define I2C_SMBUS_BYTE  1
#define I2C_SMBUS_BYTE_DATA 2
#define I2C_SMBUS_WORD_DATA 3
#define I2C_SMBUS_PROC_CALL 4
#define I2C_SMBUS_BLOCK_DATA  5
#define I2C_SMBUS_I2C_BLOCK_BROKEN  6
#define I2C_SMBUS_BLOCK_PROC_CALL 7 /* SMBus 2.0 */
#define I2C_SMBUS_I2C_BLOCK_DATA  8

smbus_dev::smbus_dev(I2C& i2c, uint8_t address, int frequency)
  : _i2c(i2c),
    i2c_addr(address),
    i2c_freq(frequency) {
      _i2c.frequency(i2c_freq);
}

int smbus_dev::setConfig(uint16_t flags, char* name) {
  i2c_flags = flags;
  if (name) {
    strcpy(i2c_name, name);
  }
}

/**
 * i2c_smbus_read_byte - SMBus "receive byte" protocol
 * @client: Handle to slave device
 *
 * This executes the SMBus "receive byte" protocol, returning negative errno
 * else the byte received from the device.
 */
int smbus_dev::i2c_smbus_read_byte(void) {
  union i2c_smbus_data data;
  int status;

  status = i2c_smbus_xfer_emulated(I2C_SMBUS_READ, 0, I2C_SMBUS_BYTE, &data);
  return (status < 0) ? status : data.byte;
}

/**
 * i2c_smbus_write_byte - SMBus "send byte" protocol
 * @client: Handle to slave device
 * @value: Byte to be sent
 *
 * This executes the SMBus "send byte" protocol, returning negative errno
 * else zero on success.
 */
int smbus_dev::i2c_smbus_write_byte(uint8_t value) {
  return i2c_smbus_xfer_emulated(I2C_SMBUS_WRITE, value, I2C_SMBUS_BYTE, NULL);
}

/**
 * i2c_smbus_read_byte_data - SMBus "read byte" protocol
 * @client: Handle to slave device
 * @command: Byte interpreted by slave
 *
 * This executes the SMBus "read byte" protocol, returning negative errno
 * else a data byte received from the device.
 */
int smbus_dev::i2c_smbus_read_byte_data(uint8_t command) {
  union i2c_smbus_data data;
  int status;

  status = i2c_smbus_xfer_emulated(I2C_SMBUS_READ, command, I2C_SMBUS_BYTE_DATA, &data);
  return (status < 0) ? status : data.byte;
}

/**
 * i2c_smbus_write_byte_data - SMBus "write byte" protocol
 * @client: Handle to slave device
 * @command: Byte interpreted by slave
 * @value: Byte being written
 *
 * This executes the SMBus "write byte" protocol, returning negative errno
 * else zero on success.
 */
int smbus_dev::i2c_smbus_write_byte_data(uint8_t command, uint8_t value) {
  union i2c_smbus_data data;
  data.byte = value;
  return i2c_smbus_xfer_emulated(I2C_SMBUS_WRITE, command, I2C_SMBUS_BYTE_DATA, &data);
}

/**
 * i2c_smbus_read_word_data - SMBus "read word" protocol
 * @client: Handle to slave device
 * @command: Byte interpreted by slave
 *
 * This executes the SMBus "read word" protocol, returning negative errno
 * else a 16-bit unsigned "word" received from the device.
 */
int smbus_dev::i2c_smbus_read_word_data(uint8_t command) {
  union i2c_smbus_data data;
  int status;

  status = i2c_smbus_xfer_emulated(I2C_SMBUS_READ, command, I2C_SMBUS_WORD_DATA, &data);
  return (status < 0) ? status : data.word;
}

/**
 * i2c_smbus_write_word_data - SMBus "write word" protocol
 * @client: Handle to slave device
 * @command: Byte interpreted by slave
 * @value: 16-bit "word" being written
 *
 * This executes the SMBus "write word" protocol, returning negative errno
 * else zero on success.
 */
int smbus_dev::i2c_smbus_write_word_data(uint8_t command, uint16_t value) {
  union i2c_smbus_data data;
  data.word = value;
  return i2c_smbus_xfer_emulated(I2C_SMBUS_WRITE, command, I2C_SMBUS_WORD_DATA, &data);
}

/**
 * i2c_smbus_read_block_data - SMBus "block read" protocol
 * @client: Handle to slave device
 * @command: Byte interpreted by slave
 * @values: Byte array into which data will be read; big enough to hold
 *	the data returned by the slave.  SMBus allows at most 32 bytes.
 *
 * This executes the SMBus "block read" protocol, returning negative errno
 * else the number of data bytes in the slave's response.
 *
 * Note that using this function requires that the client's adapter support
 * the I2C_FUNC_SMBUS_READ_BLOCK_DATA functionality.  Not all adapter drivers
 * support this; its emulation through I2C messaging relies on a specific
 * mechanism (I2C_M_RECV_LEN) which may not be implemented.
 */
int smbus_dev::i2c_smbus_read_block_data(uint8_t command, uint8_t *values) {
  union i2c_smbus_data data;
  int status;

  status = i2c_smbus_xfer_emulated(I2C_SMBUS_READ, command, I2C_SMBUS_BLOCK_DATA, &data);
  if (status)
    return status;

  memcpy(values, &data.block[1], data.block[0]);
  return data.block[0];
}

/**
 * i2c_smbus_write_block_data - SMBus "block write" protocol
 * @client: Handle to slave device
 * @command: Byte interpreted by slave
 * @length: Size of data block; SMBus allows at most 32 bytes
 * @values: Byte array which will be written.
 *
 * This executes the SMBus "block write" protocol, returning negative errno
 * else zero on success.
 */
int smbus_dev::i2c_smbus_write_block_data(uint8_t command, uint8_t length, const uint8_t *values) {
  union i2c_smbus_data data;

  if (length > I2C_SMBUS_BLOCK_MAX)
    length = I2C_SMBUS_BLOCK_MAX;
  data.block[0] = length;
  memcpy(&data.block[1], values, length);
  return i2c_smbus_xfer_emulated( I2C_SMBUS_WRITE, command, I2C_SMBUS_BLOCK_DATA, &data);
}


/* Returns the number of read bytes */
int smbus_dev::i2c_smbus_read_i2c_block_data(uint8_t command, uint8_t length, uint8_t *values) {
  union i2c_smbus_data data;
  int status;

  if (length > I2C_SMBUS_BLOCK_MAX)
    length = I2C_SMBUS_BLOCK_MAX;
  data.block[0] = length;
  status = i2c_smbus_xfer_emulated(I2C_SMBUS_READ, command, I2C_SMBUS_I2C_BLOCK_DATA, &data);
  if (status < 0)
    return status;

  memcpy(values, &data.block[1], data.block[0]);
  return data.block[0];
}
int smbus_dev::i2c_smbus_write_i2c_block_data(uint8_t command, uint8_t length, const uint8_t *values) {
  union i2c_smbus_data data;

  if (length > I2C_SMBUS_BLOCK_MAX)
    length = I2C_SMBUS_BLOCK_MAX;
  data.block[0] = length;
  memcpy(data.block + 1, values, length);
  return i2c_smbus_xfer_emulated(I2C_SMBUS_WRITE, command, I2C_SMBUS_I2C_BLOCK_DATA, &data);
}


int smbus_dev::i2c_transfer(struct i2c_msg *msgs, int num, bool pmbusread) {
  int j, res, num_messages = 0;
  int debug_length = 0;

  for (j = 0; j < num; j++) {
    if (msgs[j].flags & I2C_M_RD) {
      //  Add Read Function
      debug_length = msgs[j].len;
      res = _i2c.read(i2c_addr, reinterpret_cast<char*>(&msgs[j].buf[0]), msgs[j].len + 1);
      if (res) {
        return -1;
      } else {
        num_messages += 1;
      }
      //  trace_i2c_read(adap, &msgs[i], i);
    } else {
      if (pmbusread) {
        //  Reapeted Start Write Function
        res = _i2c.write(i2c_addr, reinterpret_cast<char*>(msgs[j].buf), msgs[j].len, true);
      } else {
        res = _i2c.write(i2c_addr, reinterpret_cast<char*>(msgs[j].buf), msgs[j].len);
      }

      if (res) {
        return -1;
      } else {
        num_messages += 1;
      }
      //  trace_i2c_write(adap, &msgs[i], i);
    }
  }

  return num_messages;
}

/* Return <0 on CRC error
   If there was a write before this read (most cases) we need to take the
   partial CRC from the write part into account.
   Note that this function does modify the message (we need to decrease the
   message length to hide the CRC byte from the caller). */
int smbus_dev::i2c_smbus_check_pec(uint8_t cpec, struct i2c_msg *msg)  {
  uint8_t rpec = msg->buf[--msg->len];
  cpec = i2c_smbus_msg_pec(cpec, msg);

  if (rpec != cpec) {
    // Bad PEC
    return -EBADMSG;
  }
  return 0;
}


/* The SMBus parts */

#define POLY    (0x1070U << 3)
uint8_t smbus_dev::crc8(uint16_t data)  {
  int i;

  for (i = 0; i < 8; i++) {
    if (data & 0x8000)
       data = data ^ POLY;
    data = data << 1;
  }
  return (uint8_t)(data >> 8);
}

inline uint8_t smbus_dev::i2c_8bit_addr_from_msg(const struct i2c_msg *msg) {
  return (msg->addr << 1) | (msg->flags & I2C_M_RD ? 1 : 0);
}

/* Incremental CRC8 over count bytes in the array pointed to by p */
uint8_t smbus_dev::i2c_smbus_pec(uint8_t crc, uint8_t *p, size_t count) {
  size_t i;

  for (i = 0; i < count; i++)
    crc = crc8((crc ^ p[i]) << 8);
  return crc;
}

/* Assume a 7-bit address, which is reasonable for SMBus */
uint8_t smbus_dev::i2c_smbus_msg_pec(uint8_t pec, struct i2c_msg *msg)  {
  /* The address will be sent first */
  uint8_t addr = i2c_8bit_addr_from_msg(msg);
  pec = i2c_smbus_pec(pec, &addr, 1);

  /* The data buffer follows */
  return i2c_smbus_pec(pec, msg->buf, msg->len);
}

// Based on Linux Smbus emulated command Function
/*
 * Simulate a SMBus command using the I2C protocol.
 * No checking of parameters is done!
 */
int smbus_dev::i2c_smbus_xfer_emulated(char read_write, uint8_t command, int size, union i2c_smbus_data *data)  {
	/*
	 * So we need to generate a series of msgs. In the case of writing, we
	 * need to use only one message; when reading, we need two. We
	 * initialize most things with sane defaults, to keep the code below
	 * somewhat simpler.
	 */
  unsigned char msgbuf0[I2C_SMBUS_BLOCK_MAX+3];
  unsigned char msgbuf1[I2C_SMBUS_BLOCK_MAX+2];
  int num = read_write == I2C_SMBUS_READ ? 2 : 1;
  int i;
  uint8_t partial_pec = 0;
  int status;

// This is a Linux Format Based I2C_MSG
  struct  i2c_msg msg[2] = {
    {
      .addr = i2c_addr,
      .flags = i2c_flags,
      .len = 1,
      .buf = msgbuf0,
    },  {
      .addr = i2c_addr,
      .flags = i2c_flags | I2C_M_RD,
      .len = 0,
      .buf = msgbuf1,
    },
  };

  msgbuf0[0] = command;
  switch (size) {
    case I2C_SMBUS_QUICK:
      msg[0].len = 0;
      /* Special case: The read/write field is used as data */
      msg[0].flags = i2c_flags | (read_write == I2C_SMBUS_READ ?
            I2C_M_RD : 0);
      num = 1;
      break;
    case I2C_SMBUS_BYTE:
      if (read_write == I2C_SMBUS_READ) {
        /* Special case: only a read! */
        msg[0].flags = I2C_M_RD | i2c_flags;
        num = 1;
      }
      break;
    case I2C_SMBUS_BYTE_DATA:
      if (read_write == I2C_SMBUS_READ) {
        msg[1].len = 1;
      } else  {
        msg[0].len = 2;
        msgbuf0[1] = data->byte;
      }
      break;
    case I2C_SMBUS_WORD_DATA:
      if (read_write == I2C_SMBUS_READ) {
        msg[1].len = 2;
      } else {
        msg[0].len = 3;
        msgbuf0[1] = data->word & 0xff;
        msgbuf0[2] = data->word >> 8;
      }
      break;
    case I2C_SMBUS_PROC_CALL:
      num = 2; /* Special case */
      read_write = I2C_SMBUS_READ;
      msg[0].len = 3;
      msg[1].len = 2;
      msgbuf0[1] = data->word & 0xff;
      msgbuf0[2] = data->word >> 8;
      break;
    case I2C_SMBUS_BLOCK_DATA:
      if (read_write == I2C_SMBUS_READ) {
        msg[1].flags |= I2C_M_RECV_LEN;
        msg[1].len = 1; /* block length will be added by the underlying bus driver */
        // This function allocates memory to the buffer, kept it as it is
        // i2c_smbus_try_get_dmabuf(&msg[1], 0);
        msg[1].buf[0] = 0;
      } else {
        msg[0].len = data->block[0] + 2;
        if (msg[0].len > I2C_SMBUS_BLOCK_MAX + 2) {
          return -EINVAL;
        }
        msg[0].buf[0] =  command;
        //  i2c_smbus_try_get_dmabuf(&msg[0], command);
        for (i = 1; i < msg[0].len; i++)
          msg[0].buf[i] = data->block[i - 1];
      }
      break;
    case I2C_SMBUS_BLOCK_PROC_CALL:
      num = 2; /* Another special case */
      read_write = I2C_SMBUS_READ;
      if (data->block[0] > I2C_SMBUS_BLOCK_MAX) {
        return -EINVAL;
      }

      msg[0].len = data->block[0] + 2;
      msg[0].buf[0] =  command;
      // i2c_smbus_try_get_dmabuf(&msg[0], command);
      for (i = 1; i < msg[0].len; i++)
        msg[0].buf[i] = data->block[i - 1];

      msg[1].flags |= I2C_M_RECV_LEN;
      msg[1].len = 1; /* block length will be added by the underlying bus driver */
      msg[1].buf[0] =  0;
      //  i2c_smbus_try_get_dmabuf(&msg[1], 0);
      break;
    case I2C_SMBUS_I2C_BLOCK_DATA:
      if (data->block[0] > I2C_SMBUS_BLOCK_MAX) {
        return -EINVAL;
      }

      if (read_write == I2C_SMBUS_READ) {
        msg[1].len = data->block[0];
        msg[1].buf[0] =  0;
        //  i2c_smbus_try_get_dmabuf(&msg[1], 0);
      } else {
        msg[0].len = data->block[0] + 1;
        msg[0].buf[0] =  command;
        //  i2c_smbus_try_get_dmabuf(&msg[0], command);
        for (i = 1; i <= data->block[0]; i++)
          msg[0].buf[i] = data->block[i];
      }
      break;
    default:
      //  dev_err(&adapter->dev, "Unsupported transaction %d\n", size);
      return -EOPNOTSUPP;
  }

  i = ((i2c_flags & I2C_CLIENT_PEC) && size != I2C_SMBUS_QUICK
              && size != I2C_SMBUS_I2C_BLOCK_DATA);
  if (i) {
    /* Compute PEC if first message is a write */
    if (!(msg[0].flags & I2C_M_RD)) {
      if (num == 1) {
         /* Write only */
        //  Add PEC Routine
        msg->buf[msg->len] = i2c_smbus_msg_pec(0, &msg[0]);
        msg->len++;
        //  i2c_smbus_add_pec(&msg[0]);
      } else {
      /* Write followed by read */
      partial_pec = i2c_smbus_msg_pec(0, &msg[0]);
      }
    }
    /* Ask for PEC if last message is a read */
    if (msg[num-1].flags & I2C_M_RD)
      msg[num-1].len++;
  }
  /*
   *  This function is supposed to return negative errno, else the number of messages executed.
   */
  status = i2c_transfer(msg, num, read_write == I2C_SMBUS_READ);


  if (status < 0) {
    goto cleanup;
  }

  if (status != num) {
    status = -EIO;
    goto cleanup;
  }
  status = 0;

  /* Check PEC if last message is a read */
  if (i && (msg[num-1].flags & I2C_M_RD)) {
    status = i2c_smbus_check_pec(partial_pec, &msg[num-1]);
    if (status < 0)
      goto cleanup;
  }

  if (read_write == I2C_SMBUS_READ) {
    switch (size) {
    case I2C_SMBUS_BYTE:
      data->byte = msgbuf0[0];
      break;
    case I2C_SMBUS_BYTE_DATA:
      data->byte = msgbuf1[0];
      break;
    case I2C_SMBUS_WORD_DATA:
    case I2C_SMBUS_PROC_CALL:
      data->word = msgbuf1[0] | (msgbuf1[1] << 8);
      break;
    case I2C_SMBUS_I2C_BLOCK_DATA:
      for (i = 0; i < data->block[0]; i++)
        data->block[i + 1] = msg[1].buf[i];
      break;
    case I2C_SMBUS_BLOCK_DATA:
    case I2C_SMBUS_BLOCK_PROC_CALL:
      for (i = 0; i < msg[1].buf[0] + 1; i++)
        data->block[i] = msg[1].buf[i];
      break;
    }
  }
cleanup:
  //  if (msg[0].flags & I2C_M_DMA_SAFE)
  //    kfree(msg[0].buf);
  //  if (msg[1].flags & I2C_M_DMA_SAFE)
  //    kfree(msg[1].buf);
  return status;
}