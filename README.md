# FLIR-One-Pro-Lock-In-Thermography

Extending work done in this EEVBLOG threod to try to achieve lock in thermography.
http://www.eevblog.com/forum/thermal-imaging/question-about-flir-one-for-android/

Credit goes to tomas123, cynfab etc from EEVBLOG for deciphering the USB Protocol and writing a C driver.


# Testing if the C driver is running as expected
The C driver saves thermal.jpg and real.jpg to /mnt/RAMDisk
test.py has a little gui to view the images in real time. Make sure you forward your X11 to see the GUI
use `ssh -X <hostname>`

![image](https://github.com/user-attachments/assets/7c82abc6-4dbe-4290-86f6-842a539987e5)

# Saving uncompressed data
flir8k.c now saves uncompressed thermal data as PPM and PGM files. these are saved in a circular buffer to /mnt/RAMDisk
