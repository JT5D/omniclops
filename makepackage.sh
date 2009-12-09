cd Debug
make clean
make
cd ../deb
mkdir usr
cd usr
mkdir bin
cd bin
rm *
cd ../../../Debug
cp omniclops ../deb/usr/bin
#sudo chmod -R 0755 ../deb/usr/bin
cd ..
sudo chmod -R 0755 .
dpkg -b deb omniclops.deb
alien -r omniclops.deb
