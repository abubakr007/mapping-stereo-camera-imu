mkdir /usr/local/lib/svs
cp svs/libsvs.so /usr/local/lib/svs 
cp svs/libsvscap.so /usr/local/lib/svs
cp svs/libdcap.so /usr/local/lib/svs

ln /usr/lib/libraw1394.so.11 /usr/lib/libraw1394.so.8

echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib/svs" >> /etc/bash.bashrc
