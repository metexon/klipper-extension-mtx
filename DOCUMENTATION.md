
# Klipper Extension Reference

The extension is provided as a GitHub repository at https://github.com/metexon/klipper-extension-mtx. For instructions on how to install and use the extension, please refer to the README file in the repository.

## Configuration

### [metexon_toolboard]

The `[metexon_toolboard]` section defines a Metexon toolboard and registers the `mtx` temperature sensor type for use in temperature sensor configurations.

```
[metexon_toolboard <board_id>]
# The `<board_id>` is a unique identifier for the toolboard (e.g., `mtx6`). 
# This ID is referenced in temperature sensor and fan configurations, and 
# is used to access toolboard properties in macros.

board_mcu:
# The MCU name configured in the [mcu] section for the CAN toolboard.  
# Type: string  
# Required: Yes  
# Example: mtxCAN

firmware_update_file:
# Path to the firmware update binary file.  
# Type: string  
# Default: ~/printer_data/mtx_esp32_firmware.bin  
```


Example:

```
[metexon_toolboard mtx6]
board_mcu: mtxCAN
```



### Temperature Sensors with `sensor_type: mtx`

The extension registers the `mtx` sensor type for use in `[temperature_sensor]` and heater configurations.


The `mtx_sensor_channel` parameter specifies which temperature to read:

- `t1` - Heating zone 1 (tip temperature)
- `t2` - Heating zone 2 (barrel temperature)
- `body` - Extruder body temperature
- `esp32` - ESP32 MCU temperature
- `motor` - Stepper motor temperature

```
[temperature_sensor <name>]
sensor_type:
# Set to `mtx` for Metexon temperature sensors.

mtx_board:
# The toolboard ID defined in the `[metexon_toolboard]` section.
# Type: string
# Required: Yes 

mtx_sensor_channel:
# The temperature channel to read.
# Type: string
# Required: Yes
# Valid values: t1, t2, body, esp32, motor
```


Examples:


Extruder Body temperature sensor:

```
[temperature_sensor Extruder_Body]
sensor_type: mtx
mtx_board: mtx6
mtx_sensor_channel: body
```

Motor temperature sensor:

```
[temperature_sensor Extruder_Motor]
sensor_type: mtx
mtx_board: mtx6
mtx_sensor_channel: motor
```

---

### Extruder and Second Heating Zone Configuration

The Metexon MTX6 toolboard handles heating control internally via its firmware. This requires a specific configuration approach for the extruder and optional second heating zone.

#### Extruder Configuration

The extruder is configured as a standard Klipper `[extruder]` section, with the following important differences:

- Temperature sensor: Uses `sensor_type: mtx` with `mtx_sensor_channel: t1` (heating zone 1 / tip temperature)
- Heating control: The PID control loop is handled internally by the MTX6 firmware, not by Klipper
- Control parameter: Required by Klipper but has no effect (typically set to `watermark`)
- Heater pin: Must be set to an unused pin (e.g., `PIN_UNUSED_1`) since heating is handled by the MTX6 firmware
- Stepper pins: The `step_pin`, `dir_pin`, and `enable_pin` use pin aliases (e.g., `mtxCAN:PIN_STEP`) to match the actual connections on the toolboard. 

Example:

```
[extruder]
step_pin: mtxCAN:PIN_STEP
dir_pin: mtxCAN:PIN_DIR
enable_pin: !mtxCAN:PIN_EN
microsteps: 32
rotation_distance: 1.538
nozzle_diameter: 0.400
filament_diameter: 1.750
sensor_type: mtx                 # use mtx temperature sensor type
mtx_board: mtx6                  # toolboard ID
mtx_sensor_channel: t1           # t1 is heating zone 1 (tip)
heater_pin: mtxCAN:PIN_UNUSED_1  # just to make klipper happy
                                 # heating is handed over to MTX6 firmware
control: watermark               # required, but no effect, heating is done by MTX6 firmware
min_temp: 10
max_temp: 320
```

Notes:
* The MTX6 is specified for a maximum temperature of 300°C. The example sets `max_temp: 320` to allow temporary overshoot during heating.
* The pin aliases (PIN_STEP, PIN_DIR, PIN_EN, PIN_UNUSED_1) must be defined in the `[board_pins]` section for the toolboard MCU. See example configuration which includes a `[board_pins]` section for the toolboard.

#### Second Heating Zone Configuration

The second heating zone (barrel temperature) is configured using the standard Klipper `[heater_generic]` section:

- Temperature sensor: Uses `sensor_type: mtx` with `mtx_sensor_channel: t2` (heating zone 2 / barrel temperature)
- Heating control: Like the extruder, PID control is handled internally by the MTX6 firmware
- Control parameter: Required by Klipper but has no effect (typically set to `watermark`)
- Heater pin: Must be set to an unused pin (e.g., `PIN_UNUSED_2`

Example:

```
[heater_generic Extruder_T2]
sensor_type: mtx                 # use mtx temperature sensor type
mtx_board: mtx6                  # toolboard ID, as defined above
mtx_sensor_channel: t2           # t2 is heating zone 2 (barrel)
heater_pin: mtxCAN:PIN_UNUSED_2  # just to make klipper happy
                                 # heating is handed over to MTX6 firmware
control: watermark               # required, but no effect, heating is done by MTX6 firmware
min_temp: 10
max_temp: 320
```

#### Heater Verification

Both heaters should have corresponding `[verify_heater]` sections for safety:

```
[verify_heater extruder]
max_error: 120
check_gain_time: 50
hysteresis: 10
heating_gain: 2

[verify_heater Extruder_T2]
max_error: 120
check_gain_time: 50
hysteresis: 10
heating_gain: 2
```

---

### TMC5160 Driver Configuration

The Metexon MTX6 toolboard includes a TMC5160 stepper driver for the extruder motor. This driver is connected via software SPI and requires specific configuration.

Example:

```
[tmc5160 extruder]
spi_software_sclk_pin: mtxCAN:PIN_TMC_SCK
spi_software_mosi_pin: mtxCAN:PIN_TMC_MOSI
spi_software_miso_pin: mtxCAN:PIN_TMC_MISO
spi_speed: 400000
cs_pin: mtxCAN:PIN_TMC_CSN
sense_resistor: 0.05              # Ω – value on board
run_current: 1.8                  # RMS Amps
hold_current: 0.10
stealthchop_threshold: 0
interpolate: True
diag0_pin: ^!mtxCAN:PIN_TMC_DIAG0
```

```
[tmc5160 extruder]

spi_software_sclk_pin: 
spi_software_mosi_pin: 
spi_software_miso_pin:
# Software SPI pins for communication with the TMC5160. Use the pin aliases for them.
# Type: string

spi_speed:
# SPI bus speed in Hz. Set to 400000.
# Type: integer

cs_pin:
# Chip select pin for the TMC5160. Set to pin alias 'mtxCAN:PIN_TMC_CSN'.
# Type: string

sense_resistor:
# Sense resistor value in ohms. This is fixed on the MTX6 board at 0.05 Ω.
# Type: float

run_current:
# Motor run current in RMS amperes. Set to 1.8 A to avoid overheating the motor.
# Type: float

hold_current:
# Motor hold current in RMS amperes when the motor is not moving. Usually a holding torque is not needed for the extruder motor.
# Type: float

stealthchop_threshold:
# Speed threshold at which to switch from stealthChop to spreadCycle mode. Set to 0 to disable stealthChop.
# Type: float

interpolate:
# Enable interpolation (256 microstep interpolation).
# Type: boolean

diag0_pin:
# DIAG0 pin for stall detection or sensorless homing. Use pin alias 'mtxCAN:PIN_TMC_DIAG0'. The `^!` prefix enables pull-up and inverts the pin.
# Type: string
```  

Note: The pin aliases (PIN_TMC_SCK, PIN_TMC_MOSI, PIN_TMC_MISO, PIN_TMC_CSN, PIN_TMC_DIAG0) must be defined in the `[board_pins]` section for the toolboard MCU. See example configuration which includes a `[board_pins]` section.



### [metexon_fan]

The `[metexon_fan]` section configures the part cooling fan managed by the Metexon toolboard. This replaces the standard `[fan]` section.

```
[metexon_fan]
mtx_board:
# The toolboard ID defined in the `[metexon_toolboard]` section.
# Type: string
# Required: Yes
```

Example:

```
[metexon_fan]
mtx_board: mtx6
```


## Mainsail/Fluidd Integration

### Temperature Sensors

Temperature sensors configured with `sensor_type: mtx` appear in Mainsail/Fluidd as standard temperature sensors. They display:

- Current temperature
- Target temperature (for channels `t1` and `t2`)
- Power/PWM output (for channels `t1` and `t2`)

The sensor names are taken from the section name in `printer.cfg` (e.g., `Extruder_Body`, `Extruder_Motor`).

### Fan Control

The `[metexon_fan]` section replaces the standard fan control. In Mainsail/Fluidd:

- The fan appears in the fan control panel
- Fan speed can be adjusted via the slider (0-100%)
- Fan RPM is displayed if reported by the toolboard
- Standard M106/M107 commands work as expected


### Firmware Update Notifications

When a newer firmware version is detected in `~/printer_data/`, Klipper displays a console message at startup with instructions to run `MTX_FIRMWARE_UPDATE`. This message appears in the Mainsail/Fluidd console.

---

## G-Code Commands


### M106 / M107

Standard fan control commands are handled by the `[metexon_fan]` module:

```
M106 S<speed>
```

- `S` # Fan speed (0-255, where 255 is 100%)

```
M107
```

- Turns off the fan # equivalent to M106 S0

These commands work identically to standard Klipper fan control.

---


### MTX_FIRMWARE_UPDATE

Initiate a firmware update for the Metexon toolboard's internal control firmware.

```
MTX_FIRMWARE_UPDATE [MODE=<mode>]

MODE:
# Update mode.
# Type: integer
# Required: No
# Default: 0
# Valid values:
# 0 - Apply update and reboot (normal operation)
# 1 - Dry-run (test without applying)
```

Notes:

- This updates the internal control firmware on the toolboard, not the Klipper firmware.
- Firmware binary file must be present at the path specified by `firmware_update_file` (default: `~/printer_data/mtx_esp32_firmware.bin`). Corresponding metadata JSON file must be also present (e.g., `~/printer_data/mtx_esp32_firmware.json`). These files are typically put by the extension installer from the GitHub repository.
- Only do fimware updates when no print is running


---

## Macro Objects and Properties

To support writing macros that interact with the toolboard, the extension provides a toolboard object that can be accessed in macros.

The toolboard is registered as a printer object with the name specified in the `[metexon_toolboard]` section (e.g., `mtx6`). This object can be accessed in macros using:

```jinja2
{% set mtx = printer["mtx6"] %}
```

Available Properties:

- `temperature1` - Current temperature of heating zone 1 (tip) in °C
- `temperature2` - Current temperature of heating zone 2 (barrel) in °C
- `setpoint_temperature1` - Target temperature for heating zone 1 in °C
- `setpoint_temperature2` - Target temperature for heating zone 2 in °C
- `pwm1` - PWM output for heating zone 1 (0.0-1.0)
- `pwm2` - PWM output for heating zone 2 (0.0-1.0)
- `temperature_body` - Extruder body temperature in °C
- `temperature_esp32` - ESP32 MCU temperature in °C
- `temperature_motor` - Stepper motor temperature in °C
- `fan_speed` - Part cooling fan speed (0.0-1.0)
- `fan_rpm` - Part cooling fan RPM
- `fan_body_speed` - Body cooling fan speed (0.0-1.0)
- `fan_body_rpm` - Body cooling fan RPM

Example Macro:

```gcode
[gcode_macro QUERY_MTX6]
gcode:
    {% set mtx = printer["mtx6"] %}
    {action_respond_info(
        "Temperature 1: %.1f C\n"
        "Temperature 2: %.1f C\n"
        "Setpoint Temperature 1: %.1f C\n"
        "Setpoint Temperature 2: %.1f C\n"
        "PWM 1: %.2f\n"
        "PWM 2: %.2f\n"
        "Body Temperature: %.1f C\n"
        "Motor Temperature: %.1f C\n"
        "ESP32 Temperature: %.1f C\n"
        "Fan Speed: %.0f %%\n"
        "Fan RPM: %.0f RPM\n"
        "Body Fan Speed: %.0f %%\n"
        "Body Fan RPM: %.0f RPM"
         % (
            mtx.temperature1,
            mtx.temperature2,
            mtx.setpoint_temperature1,
            mtx.setpoint_temperature2,
            mtx.pwm1,
            mtx.pwm2,
            mtx.temperature_body,
            mtx.temperature_motor,
            mtx.temperature_esp32,
            mtx.fan_speed * 100,
            mtx.fan_rpm,
            mtx.fan_body_speed * 100,
            mtx.fan_body_rpm
            ))}
```



## Extension Ports

The Metexon MTX6 toolboard provides two expansion ports (JST XH 4-pin connectors) for connecting external devices such as sensors, buttons, or other peripherals.

### Port Pinout

Expansion Port 1:

- Pin 1: 5V
- Pin 2: GPIO_EXT1 (gpio26)
- Pin 3: GND
- Pin 4: GPIO_EXT2 (gpio27)

Expansion Port 2:

- Pin 1: 5V
- Pin 2: GPIO_EXT3 (gpio28)
- Pin 3: GND
- Pin 4: GPIO_EXT4 (gpio29)

### Pin Aliases

The extension port GPIO pins are defined as aliases in the `[board_pins]` section:

```
[board_pins]
mcu: mtxCAN
aliases:
    PIN_GPIO_EXT1=gpio26,
    PIN_GPIO_EXT2=gpio27,
    PIN_GPIO_EXT3=gpio28,
    PIN_GPIO_EXT4=gpio29,
```

### Usage Example

To use an extension port pin in your Klipper configuration, reference it using the MCU name and pin alias:

```
# Example: Using GPIO_EXT1 as a button
[button my_button]
pin: mtxCAN:PIN_GPIO_EXT1

# Example: Using GPIO_EXT3 as a probe pin
[probe]
pin: mtxCAN:PIN_GPIO_EXT3
```

### Notes

- All GPIO pins are directly connected to the RP2040 and provide 3.3V logic level
- Ensure that any connected devices are compatible with the RP2040 voltage levels. Please refer to the RP2040 datasheet for detailed specifications.

