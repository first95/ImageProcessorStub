#
#  Linux Make file for the Point Cloud Generator application meant to 
#  run on the Jetson TX1
#
# 2016-08-12 MPU Created for Rough Bottom.
# 2016-12-21 JDW Modified for Optical Guide
#

# =============================================================================
# ARCHITECTURE SPECIFIC
# =============================================================================

CPU = x64
OS = LINUX
PVAPI_BIN = ../bin/64
INSTALL_DIR = ../bin/64


CXX := g++  # This is the main compiler
CU := /usr/local/cuda/bin/nvcc # CUDA compiler
SRCDIR := src
BUILDDIR := build
BUILD_SUBDIRS := $(BUILDDIR)/ptCloudGenAlgs $(BUILDDIR)/spiSensors
BINDIR = bin
MAINEXEC := $(BINDIR)/pcg
CAL_EXEC := $(BINDIR)/calibrate_magnetometer

#OPT := -O3
OPT := -O0

SRCEXT  := cpp
SOURCES := $(shell find $(SRCDIR) -type f -name *.$(SRCEXT))
MAINS   := build/main.o build/calibrateMagMain.o build/quanergyTestMain.o
OBJECTS := $(patsubst $(SRCDIR)/%,$(BUILDDIR)/%,$(SOURCES:.$(SRCEXT)=.o))
OBJECTS := $(filter-out $(MAINS), $(OBJECTS))
LIB     := -L/usr/lib/aarch64-linux/ -L/usr/lib/ -pthread  -lrt -lflycapture  -lflycapture-c -l:libopencv_core.so.3.4 -lopencv_cudastereo  -lopencv_imgcodecs -lopencv_cudawarping -lopencv_cudafilters -l:libopencv_cudafeatures2d.so.3.4 -lopencv_cudaimgproc -l:libopencv_highgui.so.3.4 -l:libopencv_calib3d.so.3.4 -l:libopencv_imgproc.so.3.4 -l:libopencv_features2d.so.3.4
INC     := -Iinclude -I../../Shared
DFLAGS  := -D_$(CPU) -D_$(OS) -D NO_OMNIVISION -D NO_PVAPI
TRDINC  := -Ithird_party/include -I/usr/include/opencv -I/usr/include
CXXFLAGS := -g -Wall -std=c++11 -fno-strict-aliasing -fexceptions -Wno-parentheses -Wno-unknown-pragmas $(OPT) $(DFLAGS)

CUFLAGS := -m64 #-gencode arch=compute_20,code=sm_20 -gencode arch=compute_30,code=sm_30 -gencode arch=compute_32,code=sm_32 -gencode arch=compute_35,code=sm_35 -gencode arch=compute_37,code=sm_37 -gencode arch=compute_50,code=sm_50 -gencode arch=compute_52,code=sm_52 -gencode arch=compute_53,code=sm_53 -gencode arch=compute_53,code=compute_53 

COMMIT=`git log -n 1 --format=oneline | grep -oE '[0-9a-f]{40}'`

.PHONY: all
all: $(CAL_EXEC) $(MAINEXEC)

$(MAINEXEC): $(OBJECTS) build/main.o
	@mkdir -p $(BINDIR)
	echo "Linking main executable..."
	$(CXX) $(CXXFLAGS) $^ -o $(MAINEXEC) $(LIB)

$(CAL_EXEC): build/logger.o build/spiSensors/MPU9250.o build/spiSensors/spiDevice.o build/calibrateMagMain.o
	@mkdir -p $(BINDIR)
	echo "Linking calibration executable..."
	$(CXX) $(CXXFLAGS) $^ -o $(CAL_EXEC) $(LIB)

$(BUILDDIR)/%.o: $(SRCDIR)/%.$(SRCEXT)
	@mkdir -p $(BUILD_SUBDIRS)
	$(CXX) $(CXXFLAGS) $(INC) $(TRDINC) -c -o $@ $<

version:
	printf "const char * BUILD_VERSION = \"%s\"\n" $(COMMIT) > $(SRCDIR)/pcgVersion.c

.PHONY: clean
clean:
	@echo " Cleaning...";
	$(RM) -r $(BUILDDIR)/* $(MAINEXEC) $(CAL_EXEC)
	$(RM) -r $(BINDIR)/*

