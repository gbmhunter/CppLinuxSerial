# exit when any command fails
set -e

echo "Building/installing CppLinuxSerial..."
cd build/
cmake ..
sudo make install

echo "Building test application..."
cd ../test/arduino
g++ main.cpp -lCppLinuxSerial -o test

echo "Compiling Arduino firmware..."
arduino-cli compile --fqbn arduino:avr:uno ./Basic/

echo "Uploading Arduino firmware..."
arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:avr:uno ./Basic/

echo "Running test application..."
./test
