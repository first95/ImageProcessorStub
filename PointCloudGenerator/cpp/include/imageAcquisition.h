/*
	imageAcquisition.h
	
	Declarations for image acquisition
	
	2016-12-23  JDW  Created.
*/

#ifndef __PCG_IMAQ_H__
#define __PCG_IMAQ_H__

#include <FlyCapture2.h>
#include <stdio.h>
#include <iostream>
#include <list>
#include <utility>
#include <thread>
// #include <sys/time.h>
#include <chrono> // C++11
#include <sys/stat.h>
#include <semaphore.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "logger.h"
#include "benchmarker.h"

using namespace FlyCapture2;

// Used like a struct, holds one frame of input data from camera-like sensors.
class ImageDataSet {
public:
	chrono::time_point<chrono::system_clock> acquisitionTime;
	cv::Mat imgVisibleL, imgVisibleR;
	bool imgVisibleLValid, imgVisibleRValid; // true for each succesfully acquired image
	cv::Mat imgInfrared;
	bool imgInfraredValid;

	unsigned long getTimeAcquiredS();
	unsigned long getTimeAcquiredMs(); // milliseconds within the second
	
	unsigned long getDurationSinceAcquiredMs(); // milliseconds between acquisition and now
};

struct CameraCallbackItems {
	cv::Mat image;   // Set by the callback
	sem_t * pSem;    // Used by the callback
	Logger * logger; // Used by the callback
	char side[6];    // "left " or "right".  Used by callback.
};

class ImageAcquisition {
private:
	static const string REC_EXT;
	BusManager mgr;
	Camera camL, camR;
	CameraInfo camInfoL, camInfoR;
	bool running;
	bool camLConnected, camRConnected;
	list<const Benchmarker *> * bms;
	Benchmarker bmCamControl;
	Benchmarker bmFlipImageCpu;
	Benchmarker bmSaveImages  ;
	Logger * logger;
	
	// True for success
	bool initPtGreyCam (unsigned int sn, list<pair<string, double>> cam_props, 
		Camera &cam, CameraInfo &camInfo);
	void startPtGreyCam(Camera &cam, CameraCallbackItems* cbItems);
	void stopPtGreyCam (Camera &cam);
	static cv::Mat cvMatFromFlyCap2Image(FlyCapture2::Image *img);
	void saveImages(ImageDataSet data);
	PropertyType propTypeFromString(string p);
	string propTypeToString(PropertyType  p);
	
	// Point Grey camera serial numbers
	unsigned int leftVisibleCamSn;
	unsigned int rightVisibleCamSn;
	
	// used for automatic control of camera properties
	bool autoGainControlEnabled = false;
	list<PropertyType> autoControlledProps;

	// Record and playback features
	chrono::time_point<chrono::system_clock> readTimeFromTabSepInput(istream &input);
	bool playedFirstFrame = false;
	bool recordEnabled = false;
	bool playbackEnabled = false;
	string folderName;
	string timestampPattern;
	string filenamePattern;
	ofstream recIndexFile;
	ifstream playIndexFile;
	chrono::time_point<chrono::system_clock> lastFrameTime;
	bool playbackComplete = false;
	chrono::system_clock::duration interframeDelay;
	bool flipImgL, flipImgR; // To flip or not
	int imgFlipCodeL, imgFlipCodeR; // Argument to cv::flip: 0 flips around x axis, 1 around y, -1 around both
	string autoGainControlValues = "";
	chrono::time_point<chrono::system_clock> lastAcquisitionTime;
	
	// Used in image acquisition callback
	sem_t imageWaitL, imageWaitR;
	struct timespec imageWaitTimeout = {1, 0};
	CameraCallbackItems cbItemsL = {cv::Mat(), NULL, NULL, "left "};
	CameraCallbackItems cbItemsR = {cv::Mat(), NULL, NULL, "right"};

public:
	ImageAcquisition(list<const Benchmarker *> * _bms) :
		mgr(),
		bms(_bms),
		bmCamControl  ("API interactions with cameras"),
		bmFlipImageCpu("Flipping images"),
		bmSaveImages  ("Saving images"),
		recIndexFile()
	{
		bms->push_back(&bmCamControl  );
		bms->push_back(&bmFlipImageCpu);
		bms->push_back(&bmSaveImages  );
	}

	void init(json options, Logger * lgr);
	
	// Control operation
	void start();
	void stop();
	inline bool isRunning() { return running; }
	
	// Query state
	bool isPlaybackEnabled() { return playbackEnabled; }
	bool isPlaybackComplete() { return playbackComplete; }
	chrono::system_clock::duration getFrameDelay() { return interframeDelay; }
	
	ImageDataSet acquireImages();

	static void flyCapImgEvent(Image * pImage, const void * pCallbackData);

	virtual void beginAcquisition();
};


#endif // __PCG_IMAQ_H__
