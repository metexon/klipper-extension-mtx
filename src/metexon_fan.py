# Copyright (C) 2025-2026 Metexon
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import logging
from . import fan


def mtxlog(msg, *args):
    arguments = args if args else ()
    logging.info("mtx: " + msg % arguments)
    print("mtx: " + msg % arguments)       


class ConfigWrapperWrapper:
    class sentinel:
        pass
    def __init__(self, config, option_dict, new_name=None):
        self._config = config
        self._new_name = new_name if new_name else config.get_name()
        self.option_dict = option_dict
    def __getattr__(self, name):
        return getattr(self._config, name)      
    def get_name(self):
        return self._new_name
    def get(self, option, default=sentinel, note_valid=True):
        if option in self.option_dict:
            return self.option_dict[option]
        if default == self.sentinel:
            return self._config.get(option, note_valid=note_valid)
        return self._config.get(option, default=default, note_valid=note_valid)
    
class PrinterFan:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.mtx_board_name = config.get('mtx_board', 'default')
        self.report_time = config.getint('sample_time', 1, minval=.5)

        self.board_id = "metexon_toolboard" 
        if self.mtx_board_name != "default": 
            self.board_id = self.mtx_board_name

        board = self.printer.lookup_object(self.board_id, None)
        if not board:
            if self.mtx_board_name == "default":
                raise Exception("No definition of [metexon_toolboard] section found.")
            else:
                raise Exception("mtx_board: %s defined, but no [metexon_toolboard %s] section found." %
                                (self.mtx_board_name, self.mtx_board_name))
        
        # add  pin: mtxCAN:PIN_UNUSED_7 to config to avoid error
        mcu_name = board.mcu_name
        config_modified = ConfigWrapperWrapper(config, {'pin': mcu_name + ':PIN_UNUSED_7'})
        
        # create standard fan object
        self.fan = fan.Fan(config_modified)
        # save self as "fan" in printer object, so it will be found by other code
        self.printer.add_object("fan", self)

        # Register commands
        gcode = config.get_printer().lookup_object('gcode')
        gcode.register_command("M106", self.cmd_M106)
        gcode.register_command("M107", self.cmd_M107)

    def get_status(self, eventtime):
        board = self.printer.lookup_object(self.board_id)
        return {
            'speed': board.fan_speed,
            'rpm': board.fan_rpm,
        }
    def cmd_M106(self, gcmd):
        # Set fan speed
        value = gcmd.get_float('S', 255., minval=0.)
        board = self.printer.lookup_object(self.board_id)
        speed = value / 255.0
        mtxlog("Setting fan speed to %.2f%%", speed * 100)
        board.set_fan2_pwm(speed)
        
    def cmd_M107(self, gcmd):
        # Turn fan off
        board = self.printer.lookup_object(self.board_id)
        board.set_fan2_pwm(0)

def load_config(config):
    mtxlog("Loading Metexon Fan module")
    return PrinterFan(config)
