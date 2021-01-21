# Lulupet FW

# SDK Installation

Refer : `https://docs.espressif.com/projects/esp-idf/en/v4.0/get-started/index.html`;

1. **Install Prerequisites**

```no-highlight
sudo apt-get install git wget libncurses-dev flex bison gperf python python-pip python-setuptools python-serial python-click python-cryptography python-future python-pyparsing python-pyelftools cmake ninja-build ccache libffi-dev libssl-dev
```

2. **Adding user to dialout on Linux**

```no-highlight
sudo usermod -a -G dialout $USER
```

4. **Install ESP-IDF SDK**

```no-highlight
mkdir -p ~/test/
cd ~/test/
git clone --recursive https://github.com/espressif/esp-who.git espidf4.0-esp-who
cd ~/test/espidf4.0-esp-who
cd esp-idf
./install.sh
```

# Build the Lulupet FW project code

1. **Import the SDK build environment**

```no-highlight
cd ~/test/espidf4.0-esp-who
. esp-idf/export.sh
```

2. **Checkout code**

```no-highlight
cd examples/single_chip/
git clone https://github.com/williamhsu17/lulupet_firmware.git
```

3. **Build the FW** 

```no-highlight
make clean
make
```

4. **Flash the FW**
Please check if ttyUSB0 exist or not before flash

```no-highlight
(Plug the USB to PC)
ls -al /dev/ttyUSB0
(Power on the board and Hold GPIO0 button first)
make flash
(see the message Connecting.... and release the GPIO0 button)
(if error, please try again.)
```

5. **Run the FW**

```no-highlight
(Power OFF the power)
make monitor
(Power On the board power)
(Then you will see the console message(
```
