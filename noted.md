
git clone -b v2.4.3 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git //what is -b flag?
cd Micro-XRCE-DDS-Agent
mkdir build // I already have build folder /build, 
cd build //so I can enter the build folder I have
cmake .. //what is this? 
make //what is this? 
sudo make install // and this?
sudo ldconfig /usr/local/lib/ ?? and this

### my implementation ->check
cd /home/ws/src/
git submodule add -b v2.4.3 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib

