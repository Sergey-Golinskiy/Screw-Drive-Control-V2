#!/bin/bash
set -euo pipefail

# Function to print green bold section headers
header() {
  echo -e "\n\033[1;32m=== $1 ===\033[0m\n"
}

header "System update"
sudo apt update
sudo apt upgrade -y

header "Detecting arch..."
arch=$(dpkg --print-architecture)
foreign=$(dpkg --print-foreign-architectures || true)
echo "    arch=$arch foreign=$foreign"

header "Purging foreign armhf libs (if any)..."
sudo apt -y purge 'libqt5*:armhf' 'libegl*:armhf' 'libgles*:armhf' 'libgbm*:armhf' 'libdrm*:armhf' 2>/dev/null || true

header "Purging libqt5gui5-gles (if present)..."
sudo apt -y purge libqt5gui5-gles 2>/dev/null || true

header "Fixing deps..."
sudo apt -f -y install

header "Installing PyQt5 and graphics stack..."
sudo apt install -y python3-pyqt5 libqt5gui5 libegl1-mesa libgles2-mesa libgbm1 libdrm2

header "Installing project deps..."
sudo apt install -y python3-serial python3-requests python3-flask python3-rpi.gpio python3-yaml git python3-evdev

header "Adding user to groups gpio,dialout..."
sudo newgrp gpio
sudo adduser "$USER" gpio || true
sudo adduser "$USER" dialout || true


# Grant current user access to serial devices (/dev/ttyACM0, etc.)
#sudo usermod -aG dialout "$USER"

#header "Installing PyQt5 and Qt libraries"

#sudo apt install -y \
##  python3 python3-pip python3-pyqt5 python3-pyqt5.sip \
#  libqt5core5a libqt5dbus5 libqt5designer5 libqt5gui5 libqt5help5 \
#  libqt5network5 libqt5printsupport5 libqt5qml5 libqt5qmlmodels5 \
#  libqt5quick5 libqt5sql5 libqt5sql5-sqlite libqt5svg5 libqt5test5 \
#  libqt5waylandclient5 libqt5waylandcompositor5 libqt5widgets5 libqt5xml5 \
#  qt5-gtk-platformtheme \
#  libegl-mesa0 libegl1 libgles2 libgles2-mesa libgbm1 libdrm2 libwayland-egl1
  
# (optional, for systems without X11)
##sudo apt install -y libdrm2

##header "Creating Python virtual environment and installing packages"
#python3 -m venv ~/venv-ajx
#source ~/venv-ajx/bin/activate
#pip install --upgrade pip
#pip install RPi.GPIO Flask requests pyserial
#deactivate

header "Configuring raspi-config"

# Enable serial (login shell enabled + serial hardware enabled)
sudo raspi-config nonint do_serial 0

# Set boot mode: Console (B1)
#sudo raspi-config nonint do_boot_behaviour B1

sudo systemctl set-default multi-user.target

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

header "Loading logo setup"
sudo mkdir -p /opt/splash
sudo mkdir -p /etc/systemd/system/SD_touchdesk.service.d
sudo cp /home/screwdrive/Screw-Drive-Control-V2/splash.png /opt/splash/splash.png
sudo systemctl disable plymouth-start.service plymouth-quit.service plymouth-quit-wait.service --now || true
sudo cp /home/screwdrive/Screw-Drive-Control-V2/System/SD_touchdesk.service.d/override.conf /etc/systemd/system/SD_touchdesk.service.d/override.conf
sudo cp /home/screwdrive/Screw-Drive-Control-V2/System/clear-splash.sh /opt/splash/clear-splash.sh
sudo cp /home/screwdrive/Screw-Drive-Control-V2/System/show-splash.sh /opt/splash/show-splash.sh
sudo chmod +x /opt/splash/clear-splash.sh
sudo chmod +x /opt/splash/show-splash.sh

sudo apt purge -y plymouth plymouth-themes
sudo rm -rf /usr/share/plymouth || true
sudo sed -i '/^disable_splash=/d' /boot/firmware/config.txt
echo "disable_splash=1" | sudo tee -a /boot/firmware/config.txt

sudo sed -i 's/console=tty1/console=tty3/g' /boot/firmware/cmdline.txt

sudo sed -i 's/$/ quiet loglevel=3 vt.global_cursor_default=0/' /boot/firmware/cmdline.txt

sudo apt install -y fbi


header "Setting up splashscreen.service"
if [ -f ~/Screw-Drive-Control-V2/System/service/splashscreen.service ]; then
  sudo cp ~/Screw-Drive-Control-V2/System/service/splashscreen.service /etc/systemd/system/splashscreen.service
  sudo systemctl daemon-reload
  sudo systemctl enable splashscreen.service
  sudo systemctl start splashscreen.service
else
  echo -e "\033[1;33m⚠️ splashscreenD.service file not found. Skipping service setup.\033[0m"
fi 


header "Setting up SD.service"
sudo touch /tmp/selected_device.json
sudo touch /tmp/screw_events.jsonl
if [ -f ~/Screw-Drive-Control-V2/System/service/SD.service ]; then
  sudo cp ~/Screw-Drive-Control-V2/System/service/SD.service /etc/systemd/system/SD.service
  sudo systemctl daemon-reload
  sudo systemctl enable SD.service
  sudo systemctl start SD.service
else
  echo -e "\033[1;33m⚠️ SD.service file not found. Skipping service setup.\033[0m"
fi 

header "Setting up SD_touchdesk.service"

PROFILE=".bash_profile"

if ! grep -q 'QT_QPA_PLATFORM=eglfs' "$PROFILE" 2>/dev/null; then
  {
    echo ''
    echo '# TouchDesk PyQt eglfs KMS setup'
    echo 'export QT_QPA_PLATFORM=eglfs'
    echo 'export QT_QPA_EGLFS_INTEGRATION=eglfs_kms'
    echo "export QT_QPA_EGLFS_KMS_CONFIG=/home/screwdrive/Screw-Drive-Control-V2/kms.json"
  } >> "$PROFILE"
  echo "[OK] Variables added in $PROFILE"
else
  echo "[SKIP] Variables are already in $PROFILE"
fi

# --- 3. Применяем в текущей сессии ---
export QT_QPA_PLATFORM=eglfs
export QT_QPA_EGLFS_INTEGRATION=eglfs_kms
export QT_QPA_EGLFS_KMS_CONFIG="/home/screwdrive/Screw-Drive-Control-V2/kms.json"
echo "[OK] Variables applied in this session"

if [ -f ~/Screw-Drive-Control-V2/System/service/SD_touchdesk.service ]; then
  sudo cp ~/Screw-Drive-Control-V2/System/service/SD_touchdesk.service /etc/systemd/system/SD_touchdesk.service
  sudo systemctl daemon-reload
  sudo systemctl enable SD_touchdesk.service
  sudo systemctl start SD_touchdesk.service
else
  echo -e "\033[1;33m⚠️ SD_touchdesk.service file not found. Skipping service setup.\033[0m"
fi  





header "Done ✅ Please reboot your Raspberry Pi to apply the changes"
echo -e "\033[1;32mYou can reboot now with:\n  sudo reboot\033[0m"