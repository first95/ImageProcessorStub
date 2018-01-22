/*
	imageProcessing.h
	
	Declarations for PCG image processing.
	
	2016-12-23  JDW  Created.
*/

#ifndef __PCG_PROC_H__
#define __PCG_PROC_H__

#include <stdio.h>
#include <iostream>
#include <sys/time.h>
#include <chrono> // C++11
#include <list>

#include <opencv2/opencv.hpp>

#include <message_formats.h>
#include "json.hpp"
#include "logger.h"
#include "imageAcquisition.h"
#include "stereoCal.h"
#include "benchmarker.h"
// All stereo processing algorithms supported must be listed here
#include "ptCloudGenAlgs/dummyAlg.h"
#include "ptCloudGenAlgs/gpuFastWithBinnedKps.h"
using namespace cv;
using json = nlohmann::json;
using namespace std;


class ImageProcessing {
private:
	list<const Benchmarker *> * bms;
	Logger * logger;
	StereoCal cal_data;
	bool enableGpu;
	Benchmarker bmImgTotal;
	
	StereoPtCloudGenAlg * alg = NULL;
	
public:
	ImageProcessing(list<const Benchmarker *> * _bms) : 
		bms(_bms),
		bmImgTotal("Total image processing")
	{
		bms->push_back(&bmImgTotal);
	}
	
	~ImageProcessing() {
		if(alg != NULL) {
			delete alg;
		}
	}
	void init(json options, Logger * lgr);
	void processImages(ImageDataSet imgData);
	bool areCvWindowsOpen();
	
	// It's more efficient for this class to generate the message.
	// This class will be responsible for delet[]ion; the caller should not delete[].
	PointCloudDataMessage * getPointCloud();
	bool isPointCloudAvailable();
};

#endif // __PCG_PROC_H__
