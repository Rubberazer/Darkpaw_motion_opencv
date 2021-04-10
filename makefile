IDIR1 =/usr/include/
IDIR2 =/usr/local/include/
IDIR3 =/usr/local/include/opencv4/

CC=g++
CFILES =mongoose.c 4_darkpaw.cpp
TARGET =darkpaw
LDIR =//usr/local/lib/

LIBS=-lpigpio -lpthread -lrt -lopencv_videoio -lopencv_core -lopencv_imgproc -lopencv_ml -lopencv_video -lopencv_features2d -lopencv_stitching -lopencv_aruco -lopencv_bgsegm -lopencv_bioinspired -lopencv_calib3d -lopencv_ccalib -lopencv_datasets -lopencv_dnn -lopencv_dpm -lopencv_face -lopencv_flann -lopencv_freetype -lopencv_fuzzy -lopencv_gapi -lopencv_hfs -lopencv_highgui -lopencv_imgcodecs -lopencv_img_hash -lopencv_imgproc -lopencv_line_descriptor -lopencv_ml -lopencv_objdetect -lopencv_tracking
CINCLUDE=-I$(IDIR1) -I$(IDIR2) -I$(IDIR3) -L$(LDIR)
CFLAGS=-Wall -g
all:
	$(CC) $(CFLAGS) -o $(TARGET) $(CFILES) $(CINCLUDE) $(LIBS)

clean:
	rm -rf *.o
