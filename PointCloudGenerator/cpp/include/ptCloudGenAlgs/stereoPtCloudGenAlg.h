/*
	stereoPtCloudGenAlg.h
	
	Parent class for all stereo point cloud generation algorithms
	
	2017-2-28  JDW  Created.
*/

#ifndef __PCG_ALGS_H__
#define __PCG_ALGS_H__

#include <stdio.h>
#include <iostream>
#include <sys/time.h>
#include <list>
#include <chrono> // C++11
#include <algorithm> // C++11, for sort()

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <message_formats.h>
#include "logger.h"
#include "imageAcquisition.h"
#include "stereoCal.h"
#include "benchmarker.h"
using namespace cv;
using namespace cuda;
using json = nlohmann::json;
using namespace std;

// Parent class of all point cloud generation algorithms
class StereoPtCloudGenAlg {
protected:
	list<const Benchmarker *> * bms;
	Logger * logger;
	bool showImages;
	double stereoDistThreshold;
	StereoCal cal_data;
	
	bool cvWindowsAreOpen = false;
	
	// The results of the most recent processing.
	// This class is responsible for delet[]ion prior to each write.
	PointCloudDataMessage * msg = NULL;
	bool pointCloudValid = false;
	
	Benchmarker bmShowingImages;
	Benchmarker bmClearingPtCloud;
	
	void clearPointCloud();
	
	// Commonly-useful processing steps
	// void 
public:
	virtual void init(json options, Logger * lgr, StereoCal calData);
	
	// Processes a set of raw input images and stores a point
	// cloud as member data.  Following succesful processing,
	// isPointCloudAvailable() will return true and getPointCloud()
	// will return the resultant point cloud.
	virtual void processImages(ImageDataSet imgData);
	
	// Implementors are permitted to open feedback windows.
	// This permits the caller to perform the required cvWait.
	bool areCvWindowsOpen() { return cvWindowsAreOpen; }
	
	// It's more efficient for this class to generate the message.
	// This class will be responsible for delet[]ion; the caller should not delete[].
	PointCloudDataMessage * getPointCloud() { return pointCloudValid? msg : NULL;  }
	bool isPointCloudAvailable() { return pointCloudValid; }
	// Direct this class to delete internal resources,
	// but not de-initialize.  After this function is called,
	// the object should remain ready to have processImages() called.
	// Used when switching between algorithms on the fly.
	// A subset of destructor functionality.
	virtual void selfClean() { clearPointCloud(); }

	StereoPtCloudGenAlg(list<const Benchmarker *> * _bms) : 
		bms(_bms),
		bmShowingImages  ("Showing images"      ),
		bmClearingPtCloud("Clearing point cloud")
	{
		bms->push_back(&bmShowingImages  );
		bms->push_back(&bmClearingPtCloud);
	}
	virtual ~StereoPtCloudGenAlg() { selfClean(); }
};
#endif // __PCG_ALGS_H__
