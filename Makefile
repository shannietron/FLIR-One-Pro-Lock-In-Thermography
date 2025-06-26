CC = gcc -I/usr/include/libusb-1.0
GXX = g++
CXXFLAGS      = -pipe -O2 -Wall -W -D_REENTRANT -lusb-1.0 -lm
INCPATH = -I. -I/usr/include/libusb-1.0

all: Palettes.o flir_lockin.o flir_save_images.o flir_lockin flir_save_images


Palettes.o: Palettes.cpp Palettes.h
	${CXX} -c ${CXXFLAGS} ${INCPATH} -o Palettes.o Palettes.cpp

flir_lockin.o: flir_lockin.c

flir_save_images.o: flir_save_images.c



flir_save_images: flir_save_images.o
	${CC} -o flir_save_images Palettes.o flir_save_images.o -lusb-1.0 -ljpeg -lm -Wall


flir_lockin: flir_lockin.o
	${CC} -o flir_lockin Palettes.o flir_lockin.o -lusb-1.0 -ljpeg -lm -Wall


clean:
	rm -f  Palettes.o flir_lockin.o flir_save_images.o flir_lockin flir_save_images
