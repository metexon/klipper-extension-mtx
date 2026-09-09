
Klipper extension for Metexon CAN toolboards.

# Installation

## One-line install

This downloads the installer, clones the repository to `~/klipper-extension-mtx`, links the extension files into `~/klipper/klippy/extras`, links the firmware files into `~/printer_data` when that folder exists, and restarts the `klipper` service:

```bash
wget -qO- https://raw.githubusercontent.com/metexon/klipper-extension-mtx/main/install.sh | bash
```

## Install from an existing checkout

If you already cloned this repository locally:

```bash
cd ~/klipper-extension-mtx
./install.sh
```

## What the installer does

The installer follows the same approach commonly used by Klipper extensions:

1. Verifies that Klipper is installed in `~/klipper`
2. Clones this repository to `~/klipper-extension-mtx` when the script is not run from an existing checkout
3. Symlinks the files from `src/` into `~/klipper/klippy/extras/`
4. Symlinks `firmware/mtx_esp32_firmware.bin` and `firmware/mtx_esp32_firmware.json` into `~/printer_data/` when that folder exists
5. Restarts the `klipper` systemd service


## Moonraker Update Manager

Add this to your `moonraker.conf` if you want Moonraker to manage updates for this extension:

```ini
[update_manager klipper-extension-mtx]
type: git_repo
channel: dev
path: ~/klipper-extension-mtx
origin: https://github.com/metexon/klipper-extension-mtx.git
primary_branch: main
managed_services: klipper
```

After editing `moonraker.conf`, restart Moonraker.

## Updating

If the repository already exists locally, update it and rerun the installer:

```bash
cd ~/klipper-extension-mtx
git pull
./install.sh
```

# Documentation

See [`DOCUMENTATION.md`](DOCUMENTATION.md) for the documentation.

See [`example_printer.cfg`](example_printer.cfg) for an example of how to use it in your Klipper configuration.

# Firmware Updates

The installed extension picks up the firmware files from `~/printer_data`.

If the firmware file on disk is newer, Klipper will show a notification in the console with instructions for updating the firmware on the Metexon CAN toolboard.

Note: This is the internal control firmware, not the Klipper firmware that also runs on the toolboard.


# Firmware Changes:

## Firmware 2.0

### New: readout of supply voltage

* Reported as macro property `supply_voltage` in volts
* Requires MTX firmware 2.0 or later; earlier firmware versions report `0.0`
* Example macro access:
```
{% set mtx = printer["mtx6"] %}
{action_respond_info("Supply Voltage: %.1f V" % mtx.supply_voltage)}
```

### New: Support for hardware revision 3.0

Temperature measurement routines were modified to make use of improved circuitry in hardware revision 3.0. Compatibility for older hardware revisions is maintained.

### Changed: Internal Firmware Architecture

Several internal firmware components were rewritten to improve stability and maintainability.

## Firmware 1.6

### New: readout of motor temperature sensor (on the outer surface of the stepper motor)

* Reported on mtx temperature sensor channel "motor"
* Example config:
```
[temperature_sensor Extruder_Motor]
sensor_type: mtx                 # use mtx temperature sensor type
mtx_board: mtx6                  # toolboard ID
mtx_sensor_channel: motor        # motor -> temperature at extruder motor
```

