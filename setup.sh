#!/bin/bash

# we can only run this script as root
if [[ $EUID -ne 0 ]]; then
   echo "This script must be run as root" 
   exit 1
fi

# install car dependencies
cd car/
sudo ./install_dependencies.sh
cd ../dashboard
sudo ./install_dependencies.sh
cd ..

# enable startup scripts
working_dir=`pwd`
echo "Working directory: ${working_dir}"
sed -i "s,{root_dir},${working_dir},g" setup_files/aicar.service
sed -i "s,/home/pi/Documents,${working_dir},g" car/startup.sh

sudo cp setup_files/aicar.service /etc/systemd/system/
sudo chmod 644 /etc/systemd/system/aicar.service
sudo systemctl enable /etc/systemd/system/aicar.service
sudo mkdir /etc/selfdriving-rc/
echo 1 | sudo tee /etc/selfdriving-rc/carnumber
sudo chown -R pi:pi /etc/selfdriving-rc/
sudo chown -R pi:pi "${working_dir}"

echo "Please restart for changes to take effect"
echo "Remember to change check and change file directories in car/startup.sh for the system to correctly boot on startup!"
