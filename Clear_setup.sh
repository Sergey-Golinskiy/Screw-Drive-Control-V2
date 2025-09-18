#!/bin/bash
set -e

# Function to print green bold section headers
header() {
  echo -e "\n\033[1;32m=== $1 ===\033[0m\n"
}

header "System update"
sudo apt update
sudo apt upgrade -y

header "Installing Python and tools"
sudo apt install -y python3 python3-pip python3-venv

header "Installing GPIO and Serial libraries"
sudo apt install -y python3-rpi-lgpio python3-serial

# Add user 'screwdrive' to 'gpio' group
if id "screwdrive" &>/dev/null; then
    sudo adduser screwdrive gpio
else
    echo -e "\033[1;33m⚠️ User 'screwdrive' does not exist. Create it manually or change the username in the script.\033[0m"
fi

# Grant current user access to serial devices (/dev/ttyACM0, etc.)
sudo usermod -aG dialout "$USER"

header "Installing PyQt5 and Qt libraries"
sudo apt install -y python3-pyqt5 python3-pyqt5.qtquick \
  libqt5gui5 libqt5widgets5 libqt5network5 libegl1-mesa libgles2-mesa

# (optional, for systems without X11)
sudo apt install -y libdrm2

header "Creating Python virtual environment and installing packages"
python3 -m venv ~/venv-ajx
source ~/venv-ajx/bin/activate
pip install --upgrade pip
pip install RPi.GPIO Flask requests pyserial
deactivate

header "Configuring raspi-config"

# Enable serial (login shell enabled + serial hardware enabled)
sudo raspi-config nonint do_serial 0

# Set boot mode: Console (B1)
sudo raspi-config nonint do_boot_behaviour B1

# Set boot mode: Console Autologin (B2)
sudo raspi-config nonint do_boot_behaviour B2

# Set timezone to Europe/Kyiv
sudo raspi-config nonint do_change_timezone "Europe/Kyiv"


header "Git clone Screw-Drive-Control-V2 repository"
if [ ! -d ~/Screw-Drive-Control-V2 ]; then
  git clone https://github.com/Sergey-Golinskiy/Screw-Drive-Control-V2.git ~/Screw-Drive-Control-V2
else
  echo -e "\033[1;33m⚠️ Directory ~/Screw-Drive-Control-V2 already exists. Skipping git clone.\033[0m"
fi  

header "Setting up SD.service"
if [ -f ~/Screw-Drive-Control-V2/System/service/SD.service ]; then
  sudo cp ~/Screw-Drive-Control-V2/System/service/SD.service /etc/systemd/system/SD.service
  sudo systemctl daemon-reload
  sudo systemctl enable SD.service
  sudo systemctl start SD.service
else
  echo -e "\033[1;33m⚠️ SD.service file not found. Skipping service setup.\033[0m"
fi 


header "Done ✅ Please reboot your Raspberry Pi to apply the changes"
echo -e "\033[1;32mYou can reboot now with:\n  sudo reboot\033[0m"