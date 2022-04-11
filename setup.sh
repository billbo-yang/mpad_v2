#!/bin/bash

# we can only run this script as root
if [[ $EUID -ne 0 ]]; then
   echo "This script must be run as root" 
   exit 1
fi

# install car dependencies
sudo ./car/install_dependencies.sh
sudo ./dashboard/install_dependencies.sh

# enable startup scripts
working_dir=`pwd`
sed -i -e 's/{root_dir}/${working_dir}/g' setup_files/aicar.service

sudo cp setup_files/aicar.service /etc/systemd/system/
sudo chmod 644 /etc/systemd/system/aicar.service
sudo systemctl enable /etc/systemd/system/aicar.service
sudo mkdir /etc/selfdriving-rc/
echo 1 | sudo tee /etc/selfdriving-rc/car_id.txt
sudo chown -R pi:pi /etc/selfdriving-rc/
sudo chown -R pi:pi '${working_dir}'

echo "Please restart for changes to take effect"