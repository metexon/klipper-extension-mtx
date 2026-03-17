# Copyright (C) 2025-2026 Metexon
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import logging
import os
import json
import re
import zlib
import mcu
from . import bus

MTX_I2C_ADDR_7BIT = 0x54
MTX_I2C_BUS = "i2c1b"  # HW I2C
MTX_I2C_PINS = None  # for SW I2C set to ["PIN_SCL", "PIN_SDA"]

MTX_CMD_GET_REGISTER = 0x01
MTX_CMD_SET_REGISTER = 0x02
MTX_CMD_RESET_VALUES = 0x03
MTX_CMD_GET_REGISTER_BULK = 0x04
MTX_CMD_GET_DATA_INIT = 0x05
MTX_CMD_GET_DATA_STREAM = 0x06
MTX_CMD_SEND_DATA_INIT = 0x07
MTX_CMD_SEND_DATA_STREAM = 0x08
MTX_CMD_TEST_REPEAT_PARAM = 0xAA

MTX_STATUS_OK = 0x00
MTX_STATUS_BUSY = 0x05
MTX_STATUS_COMPLETE = 0x06

MTX_GET_STREAM_HARDWARE_TYPE = 0x00
MTX_GET_STREAM_HARDWARE_VERSION = 0x01
MTX_GET_STREAM_FIRMWARE_VERSION = 0x02
MTX_GET_STREAM_FIRMWARE_VARIANT = 0x03
MTX_GET_STREAM_DEVICE_STRING = 0x04

MTX_SEND_STREAM_LOG_STRING = 0x00
MTX_SEND_STREAM_FIRMWARE_UPDATE = 0x01

MTX_FIRMWARE_UPDATE_HEADER_INVALID = 0x20
MTX_FIRMWARE_UPDATE_WRITE_FAILED = 0x21
MTX_FIRMWARE_UPDATE_SIZE_MISMATCH = 0x22
MTX_FIRMWARE_UPDATE_VERIFY_FAILED = 0x23
MTX_FIRMWARE_UPDATE_APPLY_FAILED = 0x24
MTX_FIRMWARE_UPDATE_ABORTED = 0x25
MTX_FIRMWARE_UPDATE_FORMAT_UNCOMPRESSED = 0x00
MTX_FIRMWARE_UPDATE_FORMAT_TAMP = 0x01

MTX_REGISTER_MIN = 0x00                 # minimum valid register ID
MTX_REGISTER_ID = 0x00                  # read-only, MTX CAN ID id
MTX_REGISTER_FIRMWARE_VERSION = 0x01    # read-only, firmware version
MTX_REGISTER_LED_STATUS = 0x02          # read-only, current RGBLEDState as numeric enum value
MTX_REGISTER_RESERVED_03 = 0x03         # reserved for future use
MTX_REGISTER_TEMP1 = 0x04               # read-only, temperature 1 in 0.01°C
MTX_REGISTER_TEMP1_SETPOINT = 0x05      # read-write, temperature 1 setpoint in 0.01°C
MTX_REGISTER_TEMP1_PWM = 0x06           # read-only in PID mode; read-write in MANUAL mode, 0-255
MTX_REGISTER_TEMP2 = 0x07               # read-only, temperature 2 in 0.01°C
MTX_REGISTER_TEMP2_SETPOINT = 0x08      # read-write, temperature 2 setpoint in 0.01°C
MTX_REGISTER_TEMP2_PWM = 0x09           # read-only in PID mode; read-write in MANUAL mode, 0-255
MTX_REGISTER_TEMP_BODY = 0x0A           # read-only, body temperature in 0.01°C
MTX_REGISTER_TEMP_BODY_SETPOINT = 0x0B  # reserved for future use
MTX_REGISTER_TEMP_BODY_PWM = 0x0C       # reserved for future use
MTX_REGISTER_TEMP_ESP32 = 0x0D          # read-only, ESP32 temperature in 0.01°C
MTX_REGISTER_TEMP_MOTOR = 0x0E          # reserved for future use
MTX_REGISTER_FAN1_PWM = 0x0F            # read-write, Fan 1 PWM 0-255
MTX_REGISTER_FAN2_PWM = 0x10            # read-write, Fan 2 PWM 0-255
MTX_REGISTER_FAN1_RPM = 0x11            # read-only, Fan 1 RPM
MTX_REGISTER_FAN2_RPM = 0x12            # read-only, Fan 2 RPM
MTX_REGISTER_FAN3_RPM = 0x13            # read-only, Fan 3 RPM
MTX_REGISTER_TEMP1_MODE = 0x14          # read-write, Heater 1 mode (0=PID, 1=MANUAL)
MTX_REGISTER_TEMP2_MODE = 0x15          # read-write, Heater 2 mode (0=PID, 1=MANUAL)
MTX_REGISTER_MAX = 0x15                 # maximum valid register ID
MTX_REGISTER_BULK_COUNT = MTX_REGISTER_FAN3_RPM + 1

MTX_HARDWARE_ID = 0x22

MTX_DEBUG = True
MTX_DEBUG_I2C = False

# dictionary channel name to property name
MTX_CHANNEL_TO_TEMP_PROPERTY_GET = {
    't1': 'temperature1',
    't2': 'temperature2',
    'body': 'temperature_body',
    'esp32': 'temperature_esp32'
}

MTX_CHANNEL_TO_TEMP_PROPERTY_SET = {
    't1': 'setpoint_temperature1',
    't2': 'setpoint_temperature2',
}

MTX_CHANNEL_TO_PWM_PROPERTY_GET = {
    't1': 'pwm1',
    't2': 'pwm2',
}



def mtxlog(msg, *args):
    if MTX_DEBUG:
        arguments = args if args else ()
        logging.info("mtx: " + msg % arguments)
        print("mtx: " + msg % arguments)


class MetexonToolboard:
    """
    Main class for I2C connection to extruder
    Instatiated in a [metexon_toolboard <board_id>] section.
    This causes the load_config() function to be called.
    It will also register the MTXSensor class as sensor_type: mtx
    for use in [temperature_sensor] sections.
    The board_id is used to identify the board in macros and in [temperature_sensor] sections
    """

    def __init__(self, config, board_name):
        mtxlog("MetexonToolboard init called")
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()

        self.board_name = board_name
        mtxlog("MetexonToolboard board name: %s" % self.board_name)
        self.sample_interval = config.getfloat('sample_time', 1, minval=.5)
        self.sample_interval_streaming = config.getfloat('sample_time_streaming', .010, minval=.005)
        self.sample_interval_streaming_after_busy = config.getfloat('sample_time_streaming_after_busy', .002, minval=.001)
        self.streaming_burst = config.getint('streaming_burst', 16, minval=1, maxval=32)
        self.i2c_speed = config.getint('i2c_speed', 1000000, minval=100000, maxval=1000000)
        self.mcu_name = config.get('board_mcu')
        self.i2c = self._get_MCU_I2C(self.mcu_name)
        # place MetexonToolboard object in object store, can be retrieved as printer["<name>"] in macros
        self.printer.add_object(self.board_name, self)
        mtxlog("MetexonToolboard object created as %s in object store", self.board_name)

        # Data fields
        self.temperature1 = 0.
        self.temperature2 = 0.
        self.setpoint_temperature1 = 0.
        self.setpoint_temperature2 = 0.
        self.pwm1 = 0.
        self.pwm2 = 0.
        self.temperature_body = 0.
        self.temperature_esp32 = 0.
        self.fan_speed = 0.
        self.fan_rpm = 0.
        self.fan_body_speed = 0.
        self.fan_body_rpm = 0.
        self.init_sent = False
        self.is_ready = False

        self.i2c_get_bulk_error_count = 0
        self.max_i2c_get_bulk_error_count = config.getint('max_i2c_get_bulk_errors', 3, minval=1)
        self.firmware_update_filename = os.path.expanduser(
            config.get('firmware_update_file', "~/printer_data/mtx_esp32_firmware.bin"))
        self._did_first_connect_firmware_update_check = False
        self._console_ready = False
        self._startup_console_messages = []
        self._startup_console_message_delay = config.getfloat(
            'startup_console_message_delay', 1.0, minval=0.0)
        self._stream_sampler = MtxStreamSampler(self)

        self.mcu = self.i2c.get_mcu()

        # _sample_mtx will be called regularly, started in _handle_connect
        self.sample_timer = self.reactor.register_timer(self._sample_mtx)
        self.startup_console_timer = self.reactor.register_timer(
            self._emit_startup_console_messages)

        # Start/stop on connect/shutdown
        self.printer.register_event_handler("klippy:connect", self._handle_connect)
        self.printer.register_event_handler("klippy:ready", self._handle_ready)
        self.printer.register_event_handler("klippy:shutdown", self._handle_shutdown)
        # stop polling when client disconnects
        self.printer.register_event_handler("klippy:disconnect", self._handle_disconnect)

        self.printer.register_event_handler("mtx:set_property", self._handle_set_property)

        # Register G-Code command
        self.gcode = self.printer.lookup_object('gcode')
        self.gcode.register_command(
            'MTX_SEND_CMD', self.cmd_MTX_SEND_CMD,
            desc=self.cmd_MTX_SEND_CMD_help)
        self.gcode.register_command(
            'MTX_FIRMWARE_UPDATE', self.cmd_MTX_FIRMWARE_UPDATE,
            desc=self.cmd_MTX_FIRMWARE_UPDATE_help)

    def _get_MCU_I2C(self, mcu_name):
        try:
            i2c_mcu = mcu.get_printer_mcu(self.printer, mcu_name)
        except:
            raise Exception("metexon_toolboard: no mcu named '%s' found" % mcu_name)

        ppins = self.printer.lookup_object("pins")
        sw_pins = None
        bus_name = None

        if MTX_I2C_PINS is None:
            # use bus name
            if MTX_I2C_BUS is None:
                raise Exception("mtx: no I2C bus or pins defined")
            bus_name = MTX_I2C_BUS
            mtxlog("using bus %s" % bus_name)
        else:
            sw_pin_params = [ppins.lookup_pin(name, share_type=name)
                             for name in MTX_I2C_PINS]  # type: ignore
            sw_pins = tuple([pin_params['pin'] for pin_params in sw_pin_params])
            mtxlog("using pins %s" % str(sw_pins))
        mtxlog("using i2c speed %d", self.i2c_speed)
        return bus.MCU_I2C(i2c_mcu, bus_name, MTX_I2C_ADDR_7BIT, self.i2c_speed, sw_pins)



    def _make_measurement(self):
        try:
            registers_raw = self._get_registers_bulk(MTX_REGISTER_BULK_COUNT)

            # bulk convert bytes to register values (value = response[n] | (response[n+1] << 8))
            registers = [registers_raw[2 * i] | (registers_raw[2 * i + 1] << 8) for i in range(MTX_REGISTER_BULK_COUNT)]

            if not self.init_sent:
                if self._init_device(registers):
                    self.init_sent = True
                else:
                    logging.error("mtx: device initialization failed")
                    return False

            self.temperature1 = registers[MTX_REGISTER_TEMP1] / 100.0
            self.temperature2 = registers[MTX_REGISTER_TEMP2] / 100.0
            self.setpoint_temperature1 = registers[MTX_REGISTER_TEMP1_SETPOINT] / 100.0
            self.setpoint_temperature2 = registers[MTX_REGISTER_TEMP2_SETPOINT] / 100.0
            self.pwm1 = min(1.0, max(0.0, registers[MTX_REGISTER_TEMP1_PWM] / 255.0))
            self.pwm2 = min(1.0, max(0.0, registers[MTX_REGISTER_TEMP2_PWM] / 255.0))
            self.temperature_body = registers[MTX_REGISTER_TEMP_BODY] / 100.0
            self.temperature_esp32 = registers[MTX_REGISTER_TEMP_ESP32] / 100.0
            self.fan_body_speed = registers[MTX_REGISTER_FAN1_PWM] / 255.0
            self.fan_body_rpm = registers[MTX_REGISTER_FAN1_RPM]
            self.fan_speed = registers[MTX_REGISTER_FAN2_PWM] / 255.0
            fan2_rpm = registers[MTX_REGISTER_FAN2_RPM]
            fan3_rpm = registers[MTX_REGISTER_FAN3_RPM]
            self.fan_rpm = min(fan2_rpm, fan3_rpm)  # take the lower RPM as actual speed

            # simulate connection error
            # if self.fan_speed > 0.5:
            #    raise Exception("simulated connection error")

            self.is_ready = True
            self.i2c_get_bulk_error_count = 0

            mtxlog("T1 = %5.1f/%5.1f (pwm=%0.2f), "
                   "T2 = %5.1f/%5.1f (pwm=%0.2f), "
                   "Fan_body=%0.2f (rpm=%d), Fan=%0.2f (rpm=%d)",
                   self.temperature1, self.setpoint_temperature1, self.pwm1,
                   self.temperature2, self.setpoint_temperature2, self.pwm2,
                   self.fan_body_speed, self.fan_body_rpm, self.fan_speed, self.fan_rpm)
        except Exception as e:
            logging.exception("mtx: exception encountered" +
                              " reading data: %s" % str(e))
            return False
        return True


    def _sample_mtx(self, eventtime):
        if self.printer.is_shutdown():
            self.is_ready = False
        elif self._stream_sampler.is_streaming():
            return self._stream_sampler.sample_stream(eventtime)
        else:
            result = self._make_measurement()
            if not result:
                self.i2c_get_bulk_error_count += 1
                mtxlog("error reading MTX registers, count = %d" % self.i2c_get_bulk_error_count)

                if self.i2c_get_bulk_error_count >= self.max_i2c_get_bulk_error_count:
                    # report error, put klipper into error state
                    self.printer.invoke_shutdown("Error reading from MTX toolboard")
                    self.is_ready = False

        measured_time = self.reactor.monotonic()
        return measured_time + self.sample_interval


    def _handle_connect(self):
        self.init_sent = False
        self.i2c_get_bulk_error_count = 0
        self._stream_sampler.reset()
        # start sampling
        mtxlog("starting MTX sampling")
        self.reactor.update_timer(self.sample_timer, self.reactor.NOW)

    def _handle_ready(self):
        self._console_ready = True
        if self._startup_console_messages:
            self.reactor.update_timer(
                self.startup_console_timer,
                self.reactor.monotonic() + self._startup_console_message_delay)

    def _handle_shutdown(self):
        pass

    def _handle_disconnect(self):
        # mark not ready so sensors stop reporting
        self.is_ready = False

    def _queue_startup_console_message(self, msg):
        self._startup_console_messages.append(msg)
        if self._console_ready:
            self.reactor.update_timer(
                self.startup_console_timer,
                self.reactor.monotonic() + 0.05)

    def _emit_startup_console_messages(self, eventtime):
        if not self._console_ready or not self._startup_console_messages:
            return self.reactor.NEVER
        queued_messages = self._startup_console_messages
        self._startup_console_messages = []
        for msg in queued_messages:
            self.gcode.respond_info(msg)
        return self.reactor.NEVER


    def _init_device(self, register_vals):
        # Magic number check on first valid sample
        if len(register_vals) > MTX_REGISTER_FIRMWARE_VERSION:
            hardware_id = register_vals[MTX_REGISTER_ID]
            firmware_version = register_vals[MTX_REGISTER_FIRMWARE_VERSION]
            mtxlog("init, got id:0x%02X, firmware version: %d.%d" % (hardware_id, firmware_version >> 8, firmware_version & 0xFF))
            if hardware_id == MTX_HARDWARE_ID:
                self._queue_startup_console_message(
                    "Connected to Metexon toolboard, firmware version: %d.%d"
                    % (firmware_version >> 8, firmware_version & 0xFF))
                self._check_available_firmware_update_on_first_connect(firmware_version)
                return True
            else:
                logging.error("mtx: hardware ID mismatch, expected 0x%02X, got 0x%02X",
                              MTX_HARDWARE_ID, hardware_id)
        return False

    def _check_available_firmware_update_on_first_connect(self, device_firmware_version):
        if self._did_first_connect_firmware_update_check:
            return
        self._did_first_connect_firmware_update_check = True

        try:
            update_info = self._stream_sampler.get_available_firmware_update_info()
        except Exception as e:
            mtxlog("firmware update check skipped due to invalid local file: %s", str(e))
            return

        if update_info is None:
            return

        device_version_tuple = ((device_firmware_version >> 8) & 0xFF,
                                device_firmware_version & 0xFF,
                                0)
        available_version_tuple = update_info["firmware_version_tuple"]
        if available_version_tuple <= device_version_tuple:
            return

        self._queue_startup_console_message(
            "MTX board '%s': firmware update available (file=%s, firmware_version=%s). "
            "Run 'MTX_FIRMWARE_UPDATE' to start firmware update. Make sure no print is running."
            % (self.board_name,
               update_info["filename"],
               update_info["firmware_version"]))

    def _calculate_checksum(self, data):
        checksum = 0
        for byte in data:
            checksum ^= byte
        return checksum

    def _handle_set_property(self, board_name, property_name, value):
        # called for mtx:set_property events, used only for setpoints at the moment
        if board_name != self.board_name:
            return  # not for us
        mtxlog("set_property called for board %s: %s = %s", board_name, property_name, str(value))

        if property_name == 'setpoint_temperature1':
            return self.set_setpoint_temperature1(value)
        elif property_name == 'setpoint_temperature2':
            return self.set_setpoint_temperature2(value)
        else:
            logging.error("mtx: unknown property %s" % property_name)
            return False

    def set_setpoint_temperature1(self, degrees):
        if not self._set_register(MTX_REGISTER_TEMP1_SETPOINT, int(degrees * 100)):
            logging.error("mtx: failed to set setpoint temperature 1")
            return False
        # do not set self.setpoint_temperature1 here, it will be updated on the next measurement read
        return True

    def set_setpoint_temperature2(self, degrees):
        if not self._set_register(MTX_REGISTER_TEMP2_SETPOINT, int(degrees * 100)):
            logging.error("mtx: failed to set setpoint temperature 2")
            return False
        # do not set self.setpoint_temperature2 here, it will be updated on the next measurement read
        return True

    def set_fan1_pwm(self, speed):
        pwm = int(max(0, min(1.0, speed)) * 255)
        if not self._set_register(MTX_REGISTER_FAN1_PWM, pwm):
            logging.error("mtx: failed to set fan 1 speed")
            return False
        return True

    def set_fan2_pwm(self, speed):
        pwm = int(max(0, min(1.0, speed)) * 255)
        if not self._set_register(MTX_REGISTER_FAN2_PWM, pwm):
            logging.error("mtx: failed to set fan 2 speed")
            return False
        return True

    def _set_register(self, reg, value):
        mtxlog("setting register %d to %d" % (reg, value))
        try:
            self._send_i2c_command(MTX_CMD_SET_REGISTER, reg, value & 0xFFFF)
            return True
        except Exception as e:
            logging.exception("mtx: failed to set register: %s", str(e))
            return False

    def _reset_device(self):
        try:
            self._send_i2c_command(MTX_CMD_RESET_VALUES, 0, 0)
            # Wait 100ms after reset
            self.reactor.pause(self.reactor.monotonic() + .10)
        except Exception as e:
            logging.exception("mtx: reset failed: %s", str(e))

    cmd_MTX_SEND_CMD_help = "Send a command to the Metexon toolboard"

    def cmd_MTX_SEND_CMD(self, gcmd):
        cmd = gcmd.get_int('CMD')
        reg = gcmd.get_int('REG')
        data = gcmd.get_int('DATA', 0)
        self._send_i2c_command(cmd, reg, data)
        mtxlog(f"[mtx] Sent cmd:{cmd} reg:{reg} data:{data}")

    cmd_MTX_FIRMWARE_UPDATE_help = (
        "Start Metexon toolboard internal firmware update.")

    def cmd_MTX_FIRMWARE_UPDATE(self, gcmd):
        mode = gcmd.get_int('MODE', 0, minval=0, maxval=1)
        if not self.init_sent:
            gcmd.respond_info("MTX firmware update failed: toolboard not initialized yet")
            return
        if self._stream_sampler.is_streaming():
            gcmd.respond_info("MTX firmware update failed: another stream operation is running")
            return
        started = self._stream_sampler.start_firmware_update(mode=mode)
        if not started:
            gcmd.respond_info("MTX firmware update did not start")
            return
        # Run the stream state machine immediately instead of waiting for the next sample interval.
        self.reactor.update_timer(self.sample_timer, self.reactor.NOW)

    # properties acessible in macros via printer["MTX"]
    def get_status(self, eventtime):
        return {
            'temperature1': self.temperature1,
            'temperature2': self.temperature2,
            'setpoint_temperature1': self.setpoint_temperature1,
            'setpoint_temperature2': self.setpoint_temperature2,
            'pwm1': self.pwm1,
            'pwm2': self.pwm2,
            'temperature_body': self.temperature_body,
            'temperature_esp32': self.temperature_esp32,
            'fan_speed': self.fan_speed,
            'fan_rpm': self.fan_rpm,
            'fan_body_speed': self.fan_body_speed,
            'fan_body_rpm': self.fan_body_rpm,
        }


    def _send_i2c_command(self, cmd, param, data_uint16=0, init_check=True,
                          max_repeats=3):
        # msg format:
        # 1 byte: command
        # 1 byte: param
        # 2 bytes: data (little-endian)
        # 1 byte: checksum

        if init_check and not self.init_sent:
            return None

        try:
            # turn data into 2 bytes
            data_bytes = [data_uint16 & 0xFF, (data_uint16 >> 8) & 0xFF]
            # calculate checksum
            msg = [cmd, param] + data_bytes
            checksum = self._calculate_checksum(msg)
            msg.append(checksum)

            response = self._send_i2c_low_level(msg, 4, max_repeats=max_repeats)
            if MTX_DEBUG_I2C:
                mtxlog("cmd response raw: cmd=0x%02X param=0x%02X len=%d data=%s",
                       cmd, param, len(response), " ".join(f"{b:02X}" for b in response))

            # combine data bytes
            value = response[1] | (response[2] << 8)

            mtxlog("command=0x%02X, reg=0x%02X, data=0x%04X=%d -> response status=0x%02X data=0x%04X=%d" %
                   (cmd, param, data_uint16, data_uint16, response[0], value, value))

            return value

        except Exception as e:
            logging.exception("mtx: exception encountered" +
                              " reading data: %s" % str(e))
            raise e

    def _get_registers_bulk(self, reg_count) -> bytes:
        # msg format:
        # 1 byte: command
        # 1 byte: param
        # 2 bytes: data (little-endian)
        # 1 byte: checksum

        cmd = MTX_CMD_GET_REGISTER_BULK
        param = reg_count

        try:
            # calculate checksum
            msg = [cmd, param, 0, 0]
            checksum = self._calculate_checksum(msg)
            msg.append(checksum)

            response_size = 1 + reg_count * 2 + 1  # status byte + data + checksum
            response = self._send_i2c_low_level(msg, response_size)
            if MTX_DEBUG_I2C:
                mtxlog("bulk response raw: reg_count=%d len=%d data=%s",
                       reg_count, len(response), " ".join(f"{b:02X}" for b in response))

            return response[1:-1]  # return data bytes only, exclude status and checksum

        except Exception as e:
            logging.exception("mtx: exception encountered" +
                              " get registers bulk: %s" % str(e))
            raise e

    def _send_i2c_low_level(self, msg, rx_length, allowed_statuses=(MTX_STATUS_OK,),
                            max_repeats=3):
        last_exception = None
        for attempt in range(max_repeats):
            try:
                if MTX_DEBUG_I2C:
                    mtxlog("i2c tx len=%d rx_length=%d data=%s attempt=%d/%d",
                           len(msg), rx_length, " ".join(f"{b:02X}" for b in msg),
                           attempt + 1, max_repeats)
                i2c_response = self.i2c.i2c_read(msg, rx_length)

                # response format (when rx_length=4):
                # 1 byte: status
                # 2 bytes: data (little-endian)
                # 1 byte: checksum
                response = i2c_response['response']
                if MTX_DEBUG_I2C:
                    mtxlog("i2c rx len=%d data=%s", len(response),
                           " ".join(f"{b:02X}" for b in response))

                # check response length
                if len(response) != rx_length:
                    logging.error("mtx: invalid response length got=%d expected=%d tx=%s rx=%s",
                                  len(response), rx_length,
                                  " ".join(f"{b:02X}" for b in msg),
                                  " ".join(f"{b:02X}" for b in response))
                    raise Exception("mtx: i2c invalid response length")

                # check checksum
                expected_checksum = self._calculate_checksum(response[:-1])
                if expected_checksum != response[-1]:
                    logging.error("mtx: checksum error expected=0x%02X got=0x%02X tx=%s rx=%s",
                                  expected_checksum, response[-1],
                                  " ".join(f"{b:02X}" for b in msg),
                                  " ".join(f"{b:02X}" for b in response))
                    raise Exception("mtx: i2c checksum error")

                # check status
                if response[0] not in allowed_statuses:
                    logging.error("mtx: status error status=0x%02X allowed=%s tx=%s rx=%s",
                                  response[0],
                                  ",".join(f"0x{s:02X}" for s in allowed_statuses),
                                  " ".join(f"{b:02X}" for b in msg),
                                  " ".join(f"{b:02X}" for b in response))
                    raise Exception("mtx: i2c status error")

                return response
            except Exception as e:
                mtxlog("i2c error attempt=%d/%d: %s", attempt + 1, max_repeats, str(e))
                last_exception = e 
                if attempt >= max_repeats:
                    mtxlog("i2c error, no more retries, giving up")
                    raise
                # delay before retrying
                self.reactor.pause(self.reactor.monotonic() + 0.1)
        raise Exception("mtx: i2c error: " + str(last_exception))

    def _test(self):
        mtxlog("I2C test")

        # r = self.i2c.i2c_read([0x02,0x02,0x03,0x04], 4)
        # mtx6log("i2c response = %s" % r)

        for i in range(100):
            v = self._send_i2c_command(MTX_CMD_TEST_REPEAT_PARAM, i, i)
            mtxlog("i2c test %d -> %d" % (i, v))





class MtxStreamSampler:
    def __init__(self, toolboard):
        self.toolboard = toolboard
        self._stream_dataset_ids = [
            MTX_GET_STREAM_HARDWARE_VERSION,
            MTX_GET_STREAM_FIRMWARE_VERSION,
            MTX_GET_STREAM_FIRMWARE_VARIANT,
            MTX_GET_STREAM_DEVICE_STRING,
        ]
        self._default_log_bytes = b"Hello from Klipper\x00"
        self._firmware_update_filename = os.path.expanduser(
            getattr(toolboard, "firmware_update_filename", "~/printer_data/mtx_esp32_firmware.bin"))
        self._send_firmware_update_mode = 0  # (0=apply+reboot, 1=dry-run)
        self.reset()

    def reset(self):
        self._is_performing_data_streaming = False
        self._active_operation = None
        self._stream_dataset_index = 0
        self._stream_mode = 'idle'
        self._stream_results = {}
        self._reset_dataset_state()
        self._reset_send_state()

    def is_streaming(self):
        return self._is_performing_data_streaming

    def get_streamed_properties(self):
        return dict(self._stream_results)

    def start_get_all_streamed_properties(self):
        if self._is_performing_data_streaming:
            return False
        mtxlog("starting streamed property fetch")
        self._is_performing_data_streaming = True
        self._active_operation = 'get_streamed_properties'
        self._stream_dataset_index = 0
        self._stream_mode = 'get'
        self._stream_results = {}
        self._reset_dataset_state()
        self._reset_send_state()
        return True

    def start_send_log_message(self, message=None):
        if self._is_performing_data_streaming:
            return False
        if message is None:
            log_bytes = self._default_log_bytes
        else:
            log_text = str(message)
            if not log_text.endswith('\x00'):
                log_text += '\x00'
            log_bytes = log_text.encode('utf-8', errors='replace')
        return self._start_send_operation(
            'send_log_message',
            [self._build_log_job(log_bytes)])

    def start_firmware_update(self, mode=0):
        if self._is_performing_data_streaming:
            self._gcode_info("MTX firmware update failed: another stream operation is running")
            return False
        if mode not in (0, 1):
            self._report_firmware_update_error("invalid mode %s (expected 0 or 1)" % str(mode))
            return False
        self._send_firmware_update_mode = mode
        try:
            job = self._build_firmware_update_job()
        except Exception as e:
            self._report_firmware_update_error(str(e))
            return False

        self._gcode_info(
            "MTX firmware update starting: version=%s format=%d compressed=%dB uncompressed=%dB mode=%d"
            % (job['firmware_version'], job['firmware_format'],
               job['firmware_size_compressed'], job['firmware_size_uncompressed'], mode))
        return self._start_send_operation('firmware_update', [job])

    def _reset_dataset_state(self):
        self._stream_init_done_for_dataset = False
        self._stream_wait_after_init = False
        self._stream_payload_buffer = bytearray()

    def _reset_send_state(self):
        self._send_jobs = []
        self._send_job_index = 0
        self._send_stream_id = MTX_SEND_STREAM_LOG_STRING
        self._send_job_name = ""
        self._send_init_done = False
        self._send_chunk_index = 0
        self._send_part_number = 0
        self._send_poll_for_result = False
        self._send_firmware_size = 0
        self._send_firmware_bytes_acked = 0
        self._send_firmware_last_percent = -1
        self._send_firmware_busy_count = 0
        self._send_firmware_ok_count = 0
        self._send_firmware_started_at = None
        self._firmware_update_early_part_delay = 1
        self._send_chunks = []
        self._send_total_payload_bytes = 0

    def _split_chunks_48(self, data, pad_if_empty=True):
        payload_size = 48
        chunks = []
        for i in range(0, len(data), payload_size):
            chunk = data[i:i + payload_size]
            if len(chunk) < payload_size:
                chunk += b'\x00' * (payload_size - len(chunk))
            chunks.append(chunk)
        if not chunks and pad_if_empty:
            chunks.append(b'\x00' * payload_size)
        return chunks

    def _start_send_operation(self, operation_name, jobs):
        if self._is_performing_data_streaming:
            return False
        mtxlog("starting send stream operation: %s", operation_name)
        self._is_performing_data_streaming = True
        self._active_operation = operation_name
        self._stream_mode = 'send'
        self._reset_dataset_state()
        self._reset_send_state()
        self._send_jobs = list(jobs)
        if not self._load_send_job():
            self._stop_data_streaming()
            return False
        return True

    def _build_log_job(self, log_bytes):
        log_chunks = self._split_chunks_48(log_bytes)
        return {
            'id': MTX_SEND_STREAM_LOG_STRING,
            'name': 'log_string',
            'chunks': log_chunks,
            'payload_bytes': len(log_bytes),
        }

    def _get_firmware_update_metadata_filename(self):
        base, _ext = os.path.splitext(self._firmware_update_filename)
        return base + ".json"

    def _parse_firmware_version_tuple(self, version_value):
        version_text = str(version_value)
        number_parts = [int(part) for part in re.findall(r'\d+', version_text)]
        if not number_parts:
            raise Exception("firmware update metadata field 'firmware_version' must contain numeric version parts")
        while len(number_parts) < 3:
            number_parts.append(0)
        return tuple(number_parts[:3])

    def _load_firmware_update_metadata(self):
        metadata_filename = self._get_firmware_update_metadata_filename()
        if not os.path.isfile(metadata_filename):
            raise Exception("firmware update metadata file not found: %s" % metadata_filename)

        with open(metadata_filename, "r", encoding="utf-8") as f:
            meta = json.load(f)
        if not isinstance(meta, dict):
            raise Exception("firmware update metadata must be a JSON object")

        if "description" not in meta:
            raise Exception("firmware update metadata missing required field: description")
        if "format" not in meta:
            raise Exception("firmware update metadata missing required field: format")
        if "size_compressed" not in meta:
            raise Exception("firmware update metadata missing required field: size_compressed")
        if "size_uncompressed" not in meta:
            raise Exception("firmware update metadata missing required field: size_uncompressed")

        fw_version = None
        for key in ("firmware_version", "firmware version"):
            if key in meta:
                fw_version = meta[key]
                break
        if fw_version is None:
            raise Exception("firmware update metadata missing required field: firmware_version")

        description = str(meta["description"]).strip()
        if not description:
            raise Exception("firmware update metadata field 'description' must not be empty")
        firmware_version = str(fw_version).strip()
        if not firmware_version:
            raise Exception("firmware update metadata field 'firmware_version' must not be empty")

        try:
            fmt = int(meta["format"])
        except Exception:
            raise Exception("firmware update metadata field 'format' must be numeric")
        if fmt not in (MTX_FIRMWARE_UPDATE_FORMAT_UNCOMPRESSED, MTX_FIRMWARE_UPDATE_FORMAT_TAMP):
            raise Exception("firmware update metadata field 'format' must be 0 or 1")

        try:
            size_compressed = int(meta["size_compressed"])
        except Exception:
            raise Exception("firmware update metadata field 'size_compressed' must be numeric")
        try:
            size_uncompressed = int(meta["size_uncompressed"])
        except Exception:
            raise Exception("firmware update metadata field 'size_uncompressed' must be numeric")
        if size_compressed < 0 or size_compressed > 0xFFFFFFFF:
            raise Exception("firmware update metadata field 'size_compressed' must be in range 0..4294967295")
        if size_uncompressed < 0 or size_uncompressed > 0xFFFFFFFF:
            raise Exception("firmware update metadata field 'size_uncompressed' must be in range 0..4294967295")

        return {
            "description": description,
            "firmware_version": firmware_version,
            "format": fmt,
            "size_compressed": size_compressed,
            "size_uncompressed": size_uncompressed,
            "filename": metadata_filename,
        }

    def get_available_firmware_update_info(self):
        if not os.path.isfile(self._firmware_update_filename):
            return None

        fw_meta = self._load_firmware_update_metadata()
        fw_size = os.path.getsize(self._firmware_update_filename)
        if fw_meta["size_compressed"] != fw_size:
            raise Exception("metadata size_compressed (%d) does not match file size (%d)"
                            % (fw_meta["size_compressed"], fw_size))

        return {
            "filename": self._firmware_update_filename,
            "metadata_filename": fw_meta["filename"],
            "firmware_version": fw_meta["firmware_version"],
            "firmware_version_tuple": self._parse_firmware_version_tuple(
                fw_meta["firmware_version"]),
            "firmware_format": fw_meta["format"],
            "firmware_description": fw_meta["description"],
            "firmware_size_compressed": fw_meta["size_compressed"],
            "firmware_size_uncompressed": fw_meta["size_uncompressed"],
        }

    def _build_firmware_update_job(self):
        update_info = self.get_available_firmware_update_info()
        if update_info is None:
            raise Exception("firmware update file not found: %s" % self._firmware_update_filename)

        with open(update_info["filename"], "rb") as f:
            fw = f.read()

        fw_size = len(fw)
        fw_crc32 = zlib.crc32(fw) & 0xFFFFFFFF

        meta = bytearray(48)
        # Firmware update metadata-only first chunk format:
        # byte 0: mode (0=apply+reboot, 1=dry-run)
        # byte 1: format (0=uncompressed, 1=tamp: window 10, lookahead 4)
        # byte 2..5: expected uncompressed firmware size (LE uint32)
        # byte 6..9: crc32 of compressed firmware payload as sent (LE uint32)
        # byte 10..13: expected compressed firmware size (LE uint32)
        meta[0] = self._send_firmware_update_mode
        meta[1] = update_info["firmware_format"]
        meta[2:6] = update_info["firmware_size_uncompressed"].to_bytes(4, byteorder='little', signed=False)
        meta[6:10] = fw_crc32.to_bytes(4, byteorder='little', signed=False)
        meta[10:14] = update_info["firmware_size_compressed"].to_bytes(4, byteorder='little', signed=False)
        fw_chunks = self._split_chunks_48(fw, pad_if_empty=False)
        firmware_chunks = [bytes(meta)] + fw_chunks
        return {
            'id': MTX_SEND_STREAM_FIRMWARE_UPDATE,
            'name': 'firmware_update',
            'firmware_size': fw_size,
            'firmware_crc32': fw_crc32,
            'firmware_size_compressed': update_info["firmware_size_compressed"],
            'firmware_size_uncompressed': update_info["firmware_size_uncompressed"],
            'firmware_version': update_info["firmware_version"],
            'firmware_description': update_info["firmware_description"],
            'firmware_format': update_info["firmware_format"],
            'chunks': firmware_chunks,
            'payload_bytes': 48 + fw_size,
        }

    def _load_send_job(self):
        if self._send_job_index >= len(self._send_jobs):
            return False
        job = self._send_jobs[self._send_job_index]
        self._send_stream_id = job['id']
        self._send_job_name = job['name']
        self._send_firmware_size = job.get('firmware_size', 0)
        self._send_firmware_bytes_acked = 0
        self._send_firmware_last_percent = -1
        self._send_firmware_busy_count = 0
        self._send_firmware_ok_count = 0
        self._send_firmware_started_at = None
        self._firmware_update_early_part_delay = 1
        self._send_chunks = job['chunks']
        self._send_total_payload_bytes = job.get('payload_bytes', len(self._send_chunks) * 48)
        self._send_init_done = False
        self._send_chunk_index = 0
        self._send_part_number = 0
        self._send_poll_for_result = False
        return True

    def _gcode_info(self, msg):
        gcode = getattr(self.toolboard, 'gcode', None)
        if gcode is not None:
            gcode.respond_info(msg)

    def _report_firmware_update_error(self, msg):
        logging.error("mtx: firmware update error: %s", msg)
        self._gcode_info("MTX firmware update error: %s" % msg)

    def _log_firmware_update_progress(self):
        if self._send_stream_id != MTX_SEND_STREAM_FIRMWARE_UPDATE:
            return
        if self._send_firmware_size <= 0:
            return
        now = self.toolboard.reactor.monotonic()
        elapsed = 0.0
        if self._send_firmware_started_at is not None:
            elapsed = max(0.0, now - self._send_firmware_started_at)
        kb_per_sec = 0.0
        if elapsed > 0.0:
            kb_per_sec = (self._send_firmware_bytes_acked / 1000.0) / elapsed
        eta_text = "remaining:--min:--sec"
        if kb_per_sec > 0.0 and self._send_firmware_bytes_acked < self._send_firmware_size:
            remaining_bytes = self._send_firmware_size - self._send_firmware_bytes_acked
            eta_seconds = int((remaining_bytes / 1000.0) / kb_per_sec)
            eta_minutes = eta_seconds // 60
            eta_secs = eta_seconds % 60
            eta_text = "remaining=%02dmin:%02dsec" % (eta_minutes, eta_secs)
        elif self._send_firmware_bytes_acked >= self._send_firmware_size:
            eta_text = "remaining=00min:00sec"
        percent = int((self._send_firmware_bytes_acked * 100) / self._send_firmware_size)
        if percent > 100:
            percent = 100
        busy_percent = 0.0
        if self._send_firmware_ok_count > 0:
            busy_percent = (self._send_firmware_busy_count * 100.0
                            / self._send_firmware_ok_count)
        while self._send_firmware_last_percent < percent:
            self._send_firmware_last_percent += 1
            mtxlog("firmware update progress: %d%% (BUSY=%.1f%% %.1f kB/s %s)",
                   self._send_firmware_last_percent, busy_percent,
                   kb_per_sec, eta_text)
            self._gcode_info("MTX firmware update progress: %d%%" % self._send_firmware_last_percent)

    def _is_firmware_update_status_error(self, status):
        return status in (
            MTX_FIRMWARE_UPDATE_HEADER_INVALID,
            MTX_FIRMWARE_UPDATE_WRITE_FAILED,
            MTX_FIRMWARE_UPDATE_SIZE_MISMATCH,
            MTX_FIRMWARE_UPDATE_VERIFY_FAILED,
            MTX_FIRMWARE_UPDATE_APPLY_FAILED,
            MTX_FIRMWARE_UPDATE_ABORTED)

    def _stop_data_streaming(self):
        mtxlog("stopping data streaming")
        self._is_performing_data_streaming = False
        self._active_operation = None
        self._stream_mode = 'idle'
        self._reset_dataset_state()
        self._reset_send_state()

    def _decode_stream_payload(self, payload):
        # payload contains C strings (null-terminated) from device firmware.
        text = bytes(payload).split(b'\x00', 1)[0]
        return text.decode('utf-8', errors='replace')

    def sample_stream(self, eventtime):
        board = self.toolboard
        try:
            for _ in range(board.streaming_burst):
                if not self._is_performing_data_streaming:
                    return board.reactor.NOW
                if self._stream_mode == 'send':
                    delay = self._sample_send_stream_step()
                else:
                    delay = self._sample_get_stream_step()
                if delay is not None:
                    return board.reactor.monotonic() + delay
            # Burst limit reached, reschedule quickly to keep throughput high.
            return board.reactor.monotonic() + 0.001
        except Exception as e:
            logging.exception("mtx: stream sampling failed: %s", str(e))
            if self._active_operation == 'firmware_update':
                self._report_firmware_update_error("streaming exception: %s" % str(e))
            self._stop_data_streaming()
            return board.reactor.monotonic() + board.sample_interval_streaming

    def _sample_get_stream_step(self):
        board = self.toolboard
        if self._stream_dataset_index >= len(self._stream_dataset_ids):
            mtxlog("streamed property fetch complete")
            self._stop_data_streaming()
            return None

        dataset_id = self._stream_dataset_ids[self._stream_dataset_index]

        if not self._stream_init_done_for_dataset:
            board._send_i2c_command(MTX_CMD_GET_DATA_INIT, dataset_id, 0)
            self._stream_init_done_for_dataset = True
            self._stream_wait_after_init = True
            self._stream_payload_buffer = bytearray()
            return board.sample_interval_streaming

        if self._stream_wait_after_init:
            self._stream_wait_after_init = False
            return board.sample_interval_streaming

        msg = [MTX_CMD_GET_DATA_STREAM, dataset_id, 0, 0]
        msg.append(board._calculate_checksum(msg))
        response = board._send_i2c_low_level(
            msg, 51, allowed_statuses=(
                MTX_STATUS_OK, MTX_STATUS_BUSY, MTX_STATUS_COMPLETE))

        status = response[0]
        param = response[1]
        payload = response[2:-1]
        param_low6 = param & 0x3F

        if status == MTX_STATUS_BUSY:
            return board.sample_interval_streaming_after_busy

        if status == MTX_STATUS_OK:
            self._stream_payload_buffer.extend(payload)
            return None

        # status == MTX_STATUS_COMPLETE
        valid_len = param_low6
        if valid_len > len(payload):
            valid_len = len(payload)
        self._stream_payload_buffer.extend(payload[:valid_len])
        stream_text = self._decode_stream_payload(self._stream_payload_buffer)
        mtxlog("stream data[%d]: %s", dataset_id, stream_text)
        self._stream_results[dataset_id] = stream_text
        self._stream_dataset_index += 1
        self._reset_dataset_state()
        return None

    def _sample_send_stream_step(self):
        board = self.toolboard

        if not self._send_init_done:
            board._send_i2c_command(MTX_CMD_SEND_DATA_INIT, self._send_stream_id, 0)
            self._send_init_done = True
            if self._send_stream_id == MTX_SEND_STREAM_FIRMWARE_UPDATE:
                self._send_firmware_started_at = board.reactor.monotonic()
                return self._firmware_update_early_part_delay
            return None

        if self._send_poll_for_result:
            # After successfully sending final chunk, poll result without complete flag.
            payload = b'\x00' * 48
            is_last_chunk = False
            param = self._send_part_number & 0x3F
            sent_part_number = self._send_part_number
        else:
            payload = self._send_chunks[self._send_chunk_index]
            is_last_chunk = self._send_chunk_index == len(self._send_chunks) - 1
            if is_last_chunk:
                # New protocol: on final chunk bit6=1 and bits 0-5 contain valid payload bytes (0..48).
                sent_bytes_before = self._send_chunk_index * 48
                valid_len = self._send_total_payload_bytes - sent_bytes_before
                if valid_len < 0:
                    valid_len = 0
                if valid_len > 48:
                    valid_len = 48
                param = (valid_len & 0x3F) | 0x40
            else:
                # Non-final chunk: bits 0-5 contain part number.
                param = self._send_part_number & 0x3F
            sent_part_number = self._send_part_number

        msg = [MTX_CMD_SEND_DATA_STREAM, param]
        msg.extend(payload)
        msg.append(board._calculate_checksum(msg))
        is_firmware_update_early_part = (
            self._send_stream_id == MTX_SEND_STREAM_FIRMWARE_UPDATE
            and not self._send_poll_for_result
            and self._send_chunk_index < 5)
        max_repeats = 8 if is_firmware_update_early_part else 3
        response = board._send_i2c_low_level(
            msg, 4, allowed_statuses=(
                MTX_STATUS_OK, MTX_STATUS_BUSY, MTX_STATUS_COMPLETE,
                MTX_FIRMWARE_UPDATE_HEADER_INVALID,
                MTX_FIRMWARE_UPDATE_WRITE_FAILED,
                MTX_FIRMWARE_UPDATE_SIZE_MISMATCH,
                MTX_FIRMWARE_UPDATE_VERIFY_FAILED,
                MTX_FIRMWARE_UPDATE_APPLY_FAILED,
                MTX_FIRMWARE_UPDATE_ABORTED),
            max_repeats=max_repeats)
        status = response[0]

        if status == MTX_STATUS_BUSY:
            if self._send_stream_id == MTX_SEND_STREAM_FIRMWARE_UPDATE:
                self._send_firmware_busy_count += 1
            return board.sample_interval_streaming_after_busy

        if status == MTX_STATUS_COMPLETE:
            if self._send_stream_id == MTX_SEND_STREAM_FIRMWARE_UPDATE:
                self._send_firmware_bytes_acked = self._send_firmware_size
                self._log_firmware_update_progress()
            mtxlog("send stream[%d:%s] complete", self._send_stream_id, self._send_job_name)
            self._send_job_index += 1
            if not self._load_send_job():
                if self._active_operation == 'firmware_update':
                    success_msg = (
                        "MTX firmware update finished successfully. "
                        "Run FIRMWARE_RESTART to continue.")
                    self._gcode_info(success_msg)
                    # Stop stream state first so follow-up errors aren't reported as update failures.
                    self._stop_data_streaming()
                    board.printer.invoke_shutdown(success_msg)
                    return board.sample_interval_streaming
                self._stop_data_streaming()
            return None

        if self._is_firmware_update_status_error(status):
            logging.error("mtx: send stream[%d:%s] failed with firmware update status 0x%02X",
                          self._send_stream_id, self._send_job_name, status)
            if self._active_operation == 'firmware_update':
                self._report_firmware_update_error("device returned status 0x%02X" % status)
            self._stop_data_streaming()
            return board.sample_interval_streaming

        # status == OK
        if self._send_stream_id == MTX_SEND_STREAM_FIRMWARE_UPDATE:
            self._send_firmware_ok_count += 1
        if (self._send_stream_id == MTX_SEND_STREAM_FIRMWARE_UPDATE
                and not self._send_poll_for_result
                and sent_part_number >= 1):
            remaining = self._send_firmware_size - self._send_firmware_bytes_acked
            if remaining > 0:
                self._send_firmware_bytes_acked += min(48, remaining)
                self._log_firmware_update_progress()

        if is_last_chunk:
            # Final chunk accepted; next command is a poll for final result.
            self._send_poll_for_result = True
        else:
            if not self._send_poll_for_result:
                self._send_chunk_index += 1
                self._send_part_number = (self._send_part_number + 1) & 0x3F
        if is_firmware_update_early_part:
            # give more time for the device to process the initial firmware update chunks
            return self._firmware_update_early_part_delay
        return None


class DummySensor:
    def __init__(self, config):
        self.channel = config.get('mtx_sensor_channel', 't1')
        self.mtx_board_name = config.get('mtx_board', 'default')
        self.sample_time = config.getint('sample_time', 1, minval=.5)

        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.sample_timer = self.reactor.register_timer(self._sample_mtx_proxy)
        self.printer.register_event_handler("klippy:ready", self._handle_connect)

    def _handle_connect(self):
        self.reactor.update_timer(self.sample_timer, self.reactor.NOW)

    def _sample_mtx_proxy(self, eventtime):
        measured_time = self.reactor.monotonic()
        temp = 90.0
        self.temperature_callback(measured_time, temp)
        return measured_time + self.sample_time

    def setup_callback(self, temperature_callback):
        self.temperature_callback = temperature_callback

    def get_report_time_delta(self):
        return 0.3

    def setup_minmax(self, min_temp, max_temp):
        pass


class MTXSensor:
    """
    MTX temperature sensor class, used as sensor_type: mtx in [temperature_sensor] section
    This class acts as a proxy to the main MetexonToolboard class, which must be defined
    in a separate [metexon_toolboard] section.
    """

    def __init__(self, config):
        mtxlog("MTXSensor init called")
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        self.reactor = self.printer.get_reactor()
        self.channel = config.get('mtx_sensor_channel', 't1')
        self.mtx_board_name = config.get('mtx_board', 'default')
        self.report_time = config.getint('sample_time', 1, minval=.5)

        self.board_id = "metexon_toolboard"
        if self.mtx_board_name != "default":
            self.board_id = self.mtx_board_name

        self.pwm = 0.

        board = self.printer.lookup_object(self.board_id, None)
        if not board:
            if self.mtx_board_name == "default":
                raise Exception("No definition of [metexon_toolboard] section found.")
            else:
                raise Exception("mtx_board: %s defined, but no [metexon_toolboard %s] section found." %
                                (self.mtx_board_name, self.mtx_board_name))

        if self.channel not in MTX_CHANNEL_TO_TEMP_PROPERTY_GET:
            raise Exception("mtx: invalid channel %s, must be one of %s" %
                            (self.channel, list(MTX_CHANNEL_TO_TEMP_PROPERTY_GET.keys())))
        self._callbacks = []

        mtxlog("creating sensor proxy for MTX toolboard %s, channel %s", self.board_id, self.channel)

        # _sample_mtx_proxy will be called regularly
        self.sample_timer = self.reactor.register_timer(self._sample_mtx_proxy)
        # register the connect callback, handle_connect will start the sampling
        # use ready event (instead of connect) to ensure the main MTX sensor is initialized
        self.printer.register_event_handler("klippy:ready", self._handle_connect)

    def setup_minmax(self, min_temp, max_temp):
        # gets called from PrinterSensorGeneric in temperature_sensor.py
        self.min_temp = min_temp
        self.max_temp = max_temp

    def setup_callback(self, cb):
        self._callback = cb

    def get_report_time_delta(self):
        return self.report_time

    def _sample_mtx_proxy(self, eventtime):
        mtx = self.printer.lookup_object(self.board_id, None)
        if not mtx:
            logging.error("mtx: no MetexonBoard object found, cannot sample")
        measured_time = self.reactor.monotonic()

        if mtx and mtx.is_ready:
            # get data from main MTX object
            properties = mtx.get_status(eventtime)
            temperature = properties[MTX_CHANNEL_TO_TEMP_PROPERTY_GET[self.channel]]
            if temperature < self.min_temp or temperature > self.max_temp:
                self.printer.invoke_shutdown(
                    "MTX temperature %0.1f outside range of %0.1f:%.01f (channel %s)"
                    % (temperature, self.min_temp, self.max_temp, self.channel))

            pwm_property = MTX_CHANNEL_TO_PWM_PROPERTY_GET.get(self.channel, '')
            pwm = properties[pwm_property] if pwm_property in properties else 0.
            self.pwm = pwm

            # this reports the temperature to Klipper
            self._callback(measured_time, temperature)
        # else do not update yet
        return measured_time + self.report_time

    def _handle_connect(self):
        # if attached to extruder and channel has a setter, rewire set methods
        if self.channel in MTX_CHANNEL_TO_TEMP_PROPERTY_SET.keys():
            mtxlog("MTX sensor attached to extruder and has a setter method, rewiring extruder heater")
            self._rewire_extruder()
        # start sampling
        self.reactor.update_timer(self.sample_timer, self.reactor.NOW)

    def _rewire_extruder(self):
        mtxlog("rewiring extruder to use MTX sensor")
        pheaters = self.printer.lookup_object("heaters")
        if not pheaters:
            logging.error("mtx: no heaters object found, cannot rewire extruder")
            return
        # pheaters is a PrinterHeaters object, defined in heaters.py
        # pheaters.heaters is a dictionary of all heaters (string -> Heater), defined in heater.py
        heaters = pheaters.heaters
        mtxheater = None
        # go through all heaters
        for heater_name, heater in heaters.items():
            mtxlog("heater '%s': %s" % (heater_name, heater))
            mtxlog("sensor '%s'" % (heater.sensor if heater.sensor else "???"))
            # find the heater which uses this sensor
            if heater.sensor is self:
                mtxlog("found heater '%s', using this sensor" % heater_name)
                mtxheater = heater
        if not mtxheater:
            logging.error("mtx: no heater using this sensor, no rewiring done")
            return

        # replace the method heater.set_temp(self, degrees) with custom one
        # this is needed to set the setpoint temperature in the MTX device
        if mtxheater and hasattr(mtxheater, 'set_temp'):
            mtxlog("rewiring heater set_temp")
            # replace the set_temp method with our own
            mtxheater.original_set_temp = mtxheater.set_temp

            def replacement(orig_self, degrees):
                self.set_temp_rewired(orig_self, degrees)
                # call the original set_temp method
                orig_self.original_set_temp(degrees)
                mtxlog("orig set_temp called with %0.1f" % degrees)

            mtxheater.set_temp = replacement.__get__(mtxheater, mtxheater.__class__)

        if mtxheater and hasattr(mtxheater, "get_status"):
            mtxlog("rewiring heater get_status")
            mtxheater.original_get_status = mtxheater.get_status

            def get_status_replacement(orig_self, eventtime):
                status = orig_self.original_get_status(eventtime)
                # update status with MTX values
                status['power'] = self.pwm
                return status

            mtxheater.get_status = get_status_replacement.__get__(mtxheater, mtxheater.__class__)

    def set_temp_rewired(self, heater, degrees):
        mtxlog("set_temp_rewired called with %0.1f" % degrees)
        self.setpoint_temp = degrees
        # set the setpoint temperature in the MTX device
        self.printer.send_event("mtx:set_property", self.board_id,
                                MTX_CHANNEL_TO_TEMP_PROPERTY_SET[self.channel], degrees)


# when module is loaded and a param was set in printer.cfg
def load_config_prefix(config):
    board_name = config.get_name().split()[-1]
    return on_load(config, config.get('mtx_board', board_name))

# when module is loaded but no param was set in printer.cfg


def load_config(config):
    return on_load(config, "default")


def on_load(config, board_name):
    mtxlog("on_load() called for board %s" % board_name)
    # Register MTX6 as temperature sensor
    mtxlog("registering MTX as temperature sensor")
    pheaters = config.get_printer().load_object(config, "heaters")
    pheaters.add_sensor_factory("mtx", MTXSensor)
    # pheaters.add_sensor_factory("mtx", DummySensor)

    return MetexonToolboard(config, board_name)
