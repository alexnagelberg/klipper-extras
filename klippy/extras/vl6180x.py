# VL6180X Distance Sensor Code
#
# Copyright (C) 2022 Alexander Nagelberg <alex@alexnagelberg.com>
#
# This file may be distributed under the terms of the GNU GPLv2 license.
from . import bus
import codecs
import logging
import time
import mcu

class vl6180x:
  __VL6180X_IDENTIFICATION_MODEL_ID               = 0x0000
  __VL6180X_IDENTIFICATION_MODEL_REV_MAJOR        = 0x0001
  __VL6180X_IDENTIFICATION_MODEL_REV_MINOR        = 0x0002
  __VL6180X_IDENTIFICATION_MODULE_REV_MAJOR       = 0x0003
  __VL6180X_IDENTIFICATION_MODULE_REV_MINOR       = 0x0004
  __VL6180X_IDENTIFICATION_DATE                   = 0x0006    # 16bit value
  __VL6180X_IDENTIFICATION_TIME                   = 0x0008    # 16bit value

  __VL6180X_SYSTEM_MODE_GPIO0                     = 0x0010
  __VL6180X_SYSTEM_MODE_GPIO1                     = 0x0011
  __VL6180X_SYSTEM_HISTORY_CTRL                   = 0x0012
  __VL6180X_SYSTEM_INTERRUPT_CONFIG_GPIO          = 0x0014
  __VL6180X_SYSTEM_INTERRUPT_CLEAR                = 0x0015
  __VL6180X_SYSTEM_FRESH_OUT_OF_RESET             = 0x0016
  __VL6180X_SYSTEM_GROUPED_PARAMETER_HOLD         = 0x0017

  __VL6180X_SYSRANGE_START                        = 0x0018
  __VL6180X_SYSRANGE_THRESH_HIGH                  = 0x0019
  __VL6180X_SYSRANGE_THRESH_LOW                   = 0x001A
  __VL6180X_SYSRANGE_INTERMEASUREMENT_PERIOD      = 0x001B
  __VL6180X_SYSRANGE_MAX_CONVERGENCE_TIME         = 0x001C
  __VL6180X_SYSRANGE_CROSSTALK_COMPENSATION_RATE  = 0x001E
  __VL6180X_SYSRANGE_CROSSTALK_VALID_HEIGHT       = 0x0021
  __VL6180X_SYSRANGE_EARLY_CONVERGENCE_ESTIMATE   = 0x0022
  __VL6180X_SYSRANGE_PART_TO_PART_RANGE_OFFSET    = 0x0024
  __VL6180X_SYSRANGE_RANGE_IGNORE_VALID_HEIGHT    = 0x0025
  __VL6180X_SYSRANGE_RANGE_IGNORE_THRESHOLD       = 0x0026
  __VL6180X_SYSRANGE_MAX_AMBIENT_LEVEL_MULT       = 0x002C
  __VL6180X_SYSRANGE_RANGE_CHECK_ENABLES          = 0x002D
  __VL6180X_SYSRANGE_VHV_RECALIBRATE              = 0x002E
  __VL6180X_SYSRANGE_VHV_REPEAT_RATE              = 0x0031

  __VL6180X_SYSALS_START                          = 0x0038
  __VL6180X_SYSALS_THRESH_HIGH                    = 0x003A
  __VL6180X_SYSALS_THRESH_LOW                     = 0x003C
  __VL6180X_SYSALS_INTERMEASUREMENT_PERIOD        = 0x003E
  __VL6180X_SYSALS_ANALOGUE_GAIN                  = 0x003F
  __VL6180X_SYSALS_INTEGRATION_PERIOD             = 0x0040

  __VL6180X_RESULT_RANGE_STATUS                   = 0x004D
  __VL6180X_RESULT_ALS_STATUS                     = 0x004E
  __VL6180X_RESULT_INTERRUPT_STATUS_GPIO          = 0x004F
  __VL6180X_RESULT_ALS_VAL                        = 0x0050
  __VL6180X_RESULT_HISTORY_BUFFER                 = 0x0052
  __VL6180X_RESULT_RANGE_VAL                      = 0x0062
  __VL6180X_RESULT_RANGE_RAW                      = 0x0064
  __VL6180X_RESULT_RANGE_RETURN_RATE              = 0x0066
  __VL6180X_RESULT_RANGE_REFERENCE_RATE           = 0x0068
  __VL6180X_RESULT_RANGE_RETURN_SIGNAL_COUNT      = 0x006C
  __VL6180X_RESULT_RANGE_REFERENCE_SIGNAL_COUNT   = 0x0070
  __VL6180X_RESULT_RANGE_RETURN_AMB_COUNT         = 0x0074
  __VL6180X_RESULT_RANGE_REFERENCE_AMB_COUNT      = 0x0078
  __VL6180X_RESULT_RANGE_RETURN_CONV_TIME         = 0x007C
  __VL6180X_RESULT_RANGE_REFERENCE_CONV_TIME      = 0x0080

  __VL6180X_READOUT_AVERAGING_SAMPLE_PERIOD       = 0x010A
  __VL6180X_FIRMWARE_BOOTUP                       = 0x0119
  __VL6180X_FIRMWARE_RESULT_SCALER                = 0x0120
  __VL6180X_I2C_SLAVE_DEVICE_ADDRESS              = 0x0212
  __VL6180X_INTERLEAVED_MODE_ENABLE               = 0x02A3

  __VL6180X_ERROR_NONE = 0
  __VL6180X_ERROR_SYSERR_1 = 1
  __VL6180X_ERROR_SYSERR_5 = 5
  __VL6180X_ERROR_ECEFAIL = 6
  __VL6180X_ERROR_NOCONVERGE = 7
  __VL6180X_ERROR_RANGEIGNORE = 8
  __VL6180X_ERROR_SNR = 11
  __VL6180X_ERROR_RAWUFLOW = 12
  __VL6180X_ERROR_RAWOFLOW = 13
  __VL6180X_ERROR_RANGEUFLOW = 14
  __VL6180X_ERROR_RANGEOFLOW = 15

  __VL6180X_ERROR_LOOKUP = {
    __VL6180X_ERROR_SYSERR_1: "System Error",
    __VL6180X_ERROR_SYSERR_5: "System Error",
    __VL6180X_ERROR_ECEFAIL: "ECE Failure",
    __VL6180X_ERROR_NOCONVERGE: "No Convergence",
    __VL6180X_ERROR_RANGEIGNORE: "Ignoring Range",
    __VL6180X_ERROR_SNR: "Signal/Noise Error",
    __VL6180X_ERROR_RAWUFLOW: "Raw Reading Underflow",
    __VL6180X_ERROR_RAWOFLOW: "Raw Reading Overflow",
    __VL6180X_ERROR_RANGEUFLOW: "Range Reading Underflow",
    __VL6180X_ERROR_RANGEOFLOW: "Range Reading Overflow"
  }

  __ALS_GAIN_1    = 0x06
  __ALS_GAIN_1_25 = 0x05
  __ALS_GAIN_1_67 = 0x04
  __ALS_GAIN_2_5  = 0x03
  __ALS_GAIN_5    = 0x02
  __ALS_GAIN_10   = 0x01
  __ALS_GAIN_20   = 0x00
  __ALS_GAIN_40   = 0x07

  ALS_GAIN_REG = {
    1:      __ALS_GAIN_1,
    1.25:   __ALS_GAIN_1_25,
    1.67:   __ALS_GAIN_1_67,
    2.5:    __ALS_GAIN_2_5,
    5:      __ALS_GAIN_5,
    10:     __ALS_GAIN_10,
    20:     __ALS_GAIN_20,
    40:     __ALS_GAIN_40
  }

  ALS_GAIN_ACTUAL = {    # Data sheet shows gain values as binary list
    1:      1.01,      # Nominal gain 1;    actual gain 1.01
    1.25:   1.28,      # Nominal gain 1.25; actual gain 1.28
    1.67:   1.72,      # Nominal gain 1.67; actual gain 1.72
    2.5:    2.60,      # Nominal gain 2.5;  actual gain 2.60
    5:      5.21,      # Nominal gain 5;    actual gain 5.21
    10:     10.32,     # Nominal gain 10;   actual gain 10.32
    20:     20.00,     # Nominal gain 20;   actual gain 20
    40:     40.00,     # Nominal gain 40;   actual gain 40
  }

  def __init__(self, config):
    printer = config.get_printer()

    # Get i2c channel from name
    # TODO: allow channel to be overriden by config i2c_multiplexer_index
    sensor_name = config.get_name().split(' ')[1]
    if len(sensor_name.split('filament_distance')) < 2:
      logging.exception("vl6180x must name sensor as filament_distanceX (where X is associated with filament block)")
      raise
    self._i2c_index = int(sensor_name.split('filament_distance')[1])

    # Initialize and select device on multiplexer to i2c_index (we'll select every time we communicate with vl6180x)
    i2c_mcu = mcu.get_printer_mcu(printer, config.get('i2c_mcu', 'mcu'))
    i2c_bus = config.get('i2c_bus', None)
    multiplexer_address = config.getint('multiplexer_address', default = 0x70)
    i2c_speed = config.getint('i2c_speed', 100000, minval=100000)
    self._multiplexer = bus.MCU_I2C(i2c_mcu, i2c_bus, multiplexer_address, speed = i2c_speed)

    # Initialize VL6180X with default settings
    self._i2c = bus.MCU_I2C_from_config(config, default_addr = 0x29)

    self.gcode = printer.lookup_object('gcode')
    sensor_name = config.get_name().split()[1]
    self.gcode.register_mux_command('GET_DISTANCE', 'SENSOR', sensor_name, self.cmd_GET_DISTANCE,
      desc=self.cmd_GET_DISTANCE_help)

    printer.register_event_handler("klippy:connect", self.handle_connect)

  def handle_connect(self):
    self._multiplexer.i2c_write([1 << self._i2c_index])
    time.sleep(0.1)

    # Init with required settings from application notes pg24
    # https://www.st.com/resource/en/application_note/an4545-vl6180x-basic-ranging-application-note-stmicroelectronics.pdf
    self.set_register(0x0207, 0x01)
    self.set_register(0x0208, 0x01)
    self.set_register(0x0096, 0x00)
    self.set_register(0x0097, 0xfd)
    self.set_register(0x00e3, 0x00)
    self.set_register(0x00e4, 0x04)
    self.set_register(0x00e5, 0x02)
    self.set_register(0x00e6, 0x01)
    self.set_register(0x00e7, 0x03)
    self.set_register(0x00f5, 0x02)
    self.set_register(0x00d9, 0x05)
    self.set_register(0x00db, 0xce)
    self.set_register(0x00dc, 0x03)
    self.set_register(0x00dd, 0xf8)
    self.set_register(0x009f, 0x00)
    self.set_register(0x00a3, 0x3c)
    self.set_register(0x00b7, 0x00)
    self.set_register(0x00bb, 0x3c)
    self.set_register(0x00b2, 0x09)
    self.set_register(0x00ca, 0x09)
    self.set_register(0x0198, 0x01)
    self.set_register(0x01b0, 0x17)
    self.set_register(0x01ad, 0x00)
    self.set_register(0x00ff, 0x05)
    self.set_register(0x0100, 0x05)
    self.set_register(0x0199, 0x05)
    self.set_register(0x01a6, 0x1b)
    self.set_register(0x01ac, 0x3e)
    self.set_register(0x01a7, 0x1f)
    self.set_register(0x0030, 0x00)

    # Recommended settings from application notes pg24
    # Set GPIO1 high when sample complete
    self.set_register(self.__VL6180X_SYSTEM_MODE_GPIO1, 0x10)
    # Set Avg sample period
    self.set_register(self.__VL6180X_READOUT_AVERAGING_SAMPLE_PERIOD, 0x30)
    # Set the ALS gain
    self.set_register(self.__VL6180X_SYSALS_ANALOGUE_GAIN, 0x46)
    # Set auto calibration period (Max = 255)/(OFF = 0)
    self.set_register(self.__VL6180X_SYSRANGE_VHV_REPEAT_RATE, 0xFF)
    # Set ALS integration time to 100ms
    self.set_register(self.__VL6180X_SYSALS_INTEGRATION_PERIOD, 0x63)
    # perform a single temperature calibration
    self.set_register(self.__VL6180X_SYSRANGE_VHV_RECALIBRATE, 0x01)

    ## Optional settings from application notes pg 25
    ## Set default ranging inter-measurement period to 100ms
    self.set_register(self.__VL6180X_SYSRANGE_INTERMEASUREMENT_PERIOD, 0x09)
    ## Set default ALS inter-measurement period to 100ms
    self.set_register(self.__VL6180X_SYSALS_INTERMEASUREMENT_PERIOD, 0x31)
    ## Configures interrupt on 'New Sample Ready threshold event' 
    self.set_register(self.__VL6180X_SYSTEM_INTERRUPT_CONFIG_GPIO, 0x24)

    ## Additional settings defaults from community
    #self.set_register(self.__VL6180X_SYSRANGE_MAX_CONVERGENCE_TIME, 0x32)
    #self.set_register(
    #    self.__VL6180X_SYSRANGE_RANGE_CHECK_ENABLES, 0x10 | 0x01)
    #self.set_register_16bit(
    #    self.__VL6180X_SYSRANGE_EARLY_CONVERGENCE_ESTIMATE, 0x7B)
    #self.set_register_16bit(self.__VL6180X_SYSALS_INTEGRATION_PERIOD, 0x64)
    #self.set_register(self.__VL6180X_SYSALS_ANALOGUE_GAIN, 0x40)
    #self.set_register(self.__VL6180X_FIRMWARE_RESULT_SCALER, 0x01)

    id = [self.get_register(self.__VL6180X_IDENTIFICATION_MODEL_ID + i) for i in range(0,8)]
    if self.get_register(self.__VL6180X_SYSTEM_FRESH_OUT_OF_RESET) & 0x01 != 0:
      self.set_register(self.__VL6180X_SYSTEM_FRESH_OUT_OF_RESET, 0x00)
    self.set_register(self.__VL6180X_SYSRANGE_START, 0x03)

  cmd_GET_DISTANCE_help = "Get distance of object from sensor"
  def cmd_GET_DISTANCE(self, gcmd):
    self.gcode.respond_info("Distance: %i" % self.get_distance())

  def get_distance(self):
    # select channel on i2c multiplexer
    self._multiplexer.i2c_write([1 << self._i2c_index])
    time.sleep(0.1)

    # check for errors reported by sensor
    status = self.get_register(self.__VL6180X_RESULT_RANGE_STATUS) >> 4
    if status != self.__VL6180X_ERROR_NONE:
      self.gcode.respond_info("Error getting range: %s" % self.__VL6180X_ERROR_LOOKUP[status])
      self.set_register(self.__VL6180X_SYSTEM_INTERRUPT_CLEAR, 0x07)
      self.set_register(self.__VL6180X_SYSRANGE_START, 0x03)
      return -1

    # wait until ready, poll status register
    #while self.get_register(self.__VL6180X_RESULT_INTERRUPT_STATUS_GPIO) & 0x04 != 0x04:
    #  time.sleep(0.05)
      # TODO: this needs to fail after a timeout

    distance = self.get_register(self.__VL6180X_RESULT_RANGE_VAL)
    self.set_register(self.__VL6180X_SYSTEM_INTERRUPT_CLEAR, 0x07) # clear status register

    time.sleep(0.1) # make sure there's enough time before next sample
    return distance
  
  def set_register(self, register, data):
    reg_high = (register >> 8) & 0xFF
    reg_low = register & 0xFF
    self._i2c.i2c_write([reg_high, reg_low, (data & 0xFF)])

  def set_register_16bit(self, register, data):
    reg_high = (register >> 8) & 0xFF
    reg_low = register & 0xFF
    data_high = (data >> 8) & 0xFF
    data_low = data & 0xFF
    self._i2c.i2c_write([reg_high, reg_low, data_high, data_low])

  def get_register(self, register):
    register_high = (register >> 8) & 0xFF
    register_low = register & 0xFF
    val = self._i2c.i2c_read([register_high, register_low], 1)
    return int(codecs.encode(val['response'], 'hex'), 16)

def load_config_prefix(config):
    return vl6180x(config)