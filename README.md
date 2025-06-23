# FLIR-One-Pro-Lock-In-Thermography

Extending work done in this EEVBLOG threod to try to achieve lock in thermography.
http://www.eevblog.com/forum/thermal-imaging/question-about-flir-one-for-android/

Credit goes to tomas123, cynfab etc from EEVBLOG for deciphering the USB Protocol and writing a C driver.

# Mount a ramdisk first
```
sudo mkdir -p /mnt/RAMDisk
sudo mount -t tmpfs  -o size=200m none  /mnt/RAMDisk
sudo chmod 777 /mnt/RAMDisk
```
this creates and mounts a 200mb RAM Disk for the C file to write images to. 200mb is what I used because i was running acquisitions that were around 500 seconds long and resulted in around 190mb of data. you can use a smaller ramdisk if you want to run smaller acquisitins, but they do tend to be noisy if the temp delta isn't large enough.


# Running Lock in analysis
To run lock in analysis you can use the following command format
frequency : 0.5hz (power on for 1 second, power off for 1 second, cycle time is 2s i.e. 0.5hz)
voltage 4V, current 0.1A (sets the voltage and current on the korad KA3005P PSU (serial port is hardcoded in the c file to `/dev/tty/ACM0`
duration 500 (500 seconds of capture. and at 9.7hz you get around 4850 frames)
output folder for images to be copied to from the ramdisk

`./flir_lockin --freq 0.5 --voltage 4 --current 0.1 --duration 500 --output ./result`


# Testing if the C driver is running as expected
The C driver saves thermal.jpg and real.jpg to /mnt/RAMDisk
test.py has a little gui to view the images in real time. Make sure you forward your X11 to see the GUI
use `ssh -X <hostname>`

![image](https://github.com/user-attachments/assets/7c82abc6-4dbe-4290-86f6-842a539987e5)

# Saving uncompressed data
flir8k.c now saves uncompressed thermal data as PPM and PGM files. these are saved in a circular buffer to /mnt/RAMDisk
