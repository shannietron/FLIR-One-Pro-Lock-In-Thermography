# FLIR One Pro Lock In Thermography
[FLIR One Pro](https://www.flir.com/products/flir-one-pro/?vertical=condition+monitoring&segment=solutions)

This app builds on the Linux driver developed by the EEVBlog community to acquire images from the FLIR One Pro, as discussed in [this thread](https://www.eevblog.com/forum/thermal-imaging/question-about-flir-one-for-android/). I have extended the driver to interface with a benchtop power supply unit to close the loop and create a lock-in thermography system for failure analysis of PCBs.

Credit to tomas123, cynfab, and others on EEVBlog for reverse-engineering the USB protocol and creating the original C driver.

Also thanks to [dmytro's lock in repo with the T2S+ for the inspiration](https://dmytroengineering.com/content/projects/t2s-plus-thermal-camera-hacking)


# Acquire data from FLIR One Pro

## Mount a ramdisk first
We write raw images captured by flir one to a ramdisk as .pgm files. We also write a bunch of metadata to the header of these files which is used during analysis.

```
sudo mkdir -p /mnt/RAMDisk
sudo mount -t tmpfs  -o size=200m none  /mnt/RAMDisk
sudo chmod 777 /mnt/RAMDisk
```
this creates and mounts a 200mb RAM Disk for the C file to write images to. 200mb is what I used because i was running acquisitions that were around 500 seconds long and resulted in around 190mb of data. you can use a smaller ramdisk if you want to run smaller acquisitins, but they do tend to be noisy if the temp delta isn't large enough.


## Run a lock in acquisition

To run lock in acquisition you can use the following command format.\
```
./flir_lockin --freq 0.5 --voltage 4 --current 0.1 --duration 500 --output ./result
```

**frequency** : 0.5hz (power on for 1 second, power off for 1 second, cycle time is 2s i.e. 0.5hz)\
**voltage** 4V, current 0.1A (sets the voltage and current on the korad KA3005P PSU (serial port is hardcoded in the c file to `/dev/tty/ACM0`\
**duration** 500 (500 seconds of capture. and at 9.7hz you get around 4850 frames)\
**output** folder for images to be copied to from the ramdisk\


# Running Analysis


# Saving uncompressed data
flir8k.c now saves uncompressed thermal data as PPM and PGM files. these are saved in a circular buffer to /mnt/RAMDisk
