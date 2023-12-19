# Run this script to install the packages and dependencies needed on 
# a Raspberry Pi Zero

sudo apt-get install -y libblas-dev liblapack-dev gfortran
sudo apt install -y libsuitesparse-dev
export CPPFLAGS="-I/usr/include/suitesparse"

sudo apt-get install -y glpk-utils libglpk-dev glpk-doc

pip3 install numpy scipy autograd pyzmq msgpack scikit-learn pyserial
pip3 install astropy poliastro skyfield

pip3 install git+https://github.com/duncaneddy/brahe

echo "Done running additional_package_install_1.sh, reboot and run additional_package_install_2.sh"