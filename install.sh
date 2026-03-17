#!/usr/bin/env bash

set -e

KLIPPER_PATH="${HOME}/klipper"
KLIPPER_SERVICE="klipper"
EXTENSION_REPO="https://github.com/metexon/klipper-extension-mtx.git"
EXTENSION_PATH="${HOME}/klipper-extension-mtx"
KLIPPER_EXTRAS_PATH="${KLIPPER_PATH}/klippy/extras"

MODULES=(
    "metexon_toolboard.py"
    "metexon_fan.py"
)


report_status() {
    echo
    echo "###### $1"
    echo
}


check_klipper() {
    if [ ! -d "${KLIPPER_PATH}" ]; then
        echo "Klipper path not found: ${KLIPPER_PATH}"
        echo "Update KLIPPER_PATH in install.sh or move Klipper to the default location."
        exit 1
    fi

    if [ ! -d "${KLIPPER_EXTRAS_PATH}" ]; then
        echo "Klipper extras path not found: ${KLIPPER_EXTRAS_PATH}"
        exit 1
    fi
}


check_dependencies() {
    if ! command -v git >/dev/null 2>&1; then
        echo "'git' is required but not installed."
        exit 1
    fi
}


resolve_extension_path() {
    local script_dir
    script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"

    if [ -f "${script_dir}/src/metexon_toolboard.py" ] && [ -f "${script_dir}/src/metexon_fan.py" ]; then
        ACTIVE_EXTENSION_PATH="${script_dir}"
        report_status "Using local checkout at ${ACTIVE_EXTENSION_PATH}"
        return
    fi

    if [ -e "${EXTENSION_PATH}" ] && [ ! -d "${EXTENSION_PATH}/.git" ]; then
        echo "Target path exists but is not a git checkout: ${EXTENSION_PATH}"
        exit 1
    fi

    if [ ! -d "${EXTENSION_PATH}/.git" ]; then
        report_status "Cloning repository to ${EXTENSION_PATH}"
        git clone "${EXTENSION_REPO}" "${EXTENSION_PATH}"
    else
        report_status "Using existing checkout at ${EXTENSION_PATH}"
    fi

    ACTIVE_EXTENSION_PATH="${EXTENSION_PATH}"
}


link_extension() {
    local module

    report_status "Linking extension files into ${KLIPPER_EXTRAS_PATH}"

    for module in "${MODULES[@]}"; do
        ln -sf "${ACTIVE_EXTENSION_PATH}/src/${module}" "${KLIPPER_EXTRAS_PATH}/${module}"
        echo "Linked ${module}"
    done
}


restart_klipper() {
    report_status "Restarting ${KLIPPER_SERVICE}"
    sudo systemctl restart "${KLIPPER_SERVICE}"
}


print_update_manager_instructions() {
    report_status "Moonraker Update Manager"
    cat <<EOF
Add this to moonraker.conf if you want Moonraker to manage updates for this extension:

[update_manager klipper-extension-mtx]
type: git_repo
channel: dev
path: ~/klipper-extension-mtx
origin: https://github.com/metexon/klipper-extension-mtx.git
primary_branch: main
managed_services: klipper
EOF
}


main() {
    check_klipper
    check_dependencies
    resolve_extension_path
    link_extension
    restart_klipper
    print_update_manager_instructions
}


main "$@"
