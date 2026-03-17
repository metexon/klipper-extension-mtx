# klipper-extension-mtx

Klipper extension for Metexon CAN toolboards.

## Installation

### One-line install

This downloads the installer, clones the repository to `~/klipper-extension-mtx`, links the extension files into `~/klipper/klippy/extras`, and restarts the `klipper` service:

```bash
wget -qO- https://raw.githubusercontent.com/metexon/klipper-extension-mtx/main/install.sh | bash
```

### Install from an existing checkout

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
4. Restarts the `klipper` systemd service

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

## Klipper configuration

See [`example_printer.cfg`](example_printer.cfg) for an example of how to use it in your Klipper configuration.
