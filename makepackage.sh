rm -r temp
cd Debug
make clean
make
cd ..
mkdir temp
cd temp
mkdir DEBIAN
cp ../deb/DEBIAN/control DEBIAN
cp ../deb/DEBIAN/copyright DEBIAN
cp ../deb/DEBIAN/docs DEBIAN
mkdir usr
cd usr
mkdir bin
cd bin
rm *
cd ../../../Debug
cp omniclops ../temp/usr/bin
#sudo chmod -R 0755 ../temp/usr/bin
cd ..
sudo chmod -R 0755 .
dpkg -b temp omniclops.deb
alien -r omniclops.deb
