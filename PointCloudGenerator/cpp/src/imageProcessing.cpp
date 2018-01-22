/*
	imageProcessing.h
	
	Declarations for PCG image processing
	
	2016-12-23  JDW  Created.
*/

#include <imageProcessing.h>
using namespace std;
using namespace std::chrono;
void ImageProcessing::init(json options, Logger * lgr) {
	logger = lgr;
	
	stringstream ss;
	ss << "OpenCV version is: " << CV_MAJOR_VERSION << '.' << CV_MINOR_VERSION;
	logger->logDebug(ss.str());
	
	// Load configuration options
	string cur_key = "";
	string cal_fn = "";
	string alg_name = "";
	try {
		cur_key = "stereoCalFile"; cal_fn    = options[cur_key];
		cur_key = "algorithm";     alg_name  = options[cur_key];
		cur_key = "enableGpu";     enableGpu = options[cur_key];
	} catch (domain_error e) {
		cerr << "JSON field missing or corrupted.  Please see example file in config directory."
			 << endl << "While reading key \"" << cur_key << "\" in processing section: "
			 << e.what() << endl;
		throw(e);
	}
	
	json stereo_cal;
	try {
		ifstream cal_file(cal_fn);
		cal_file >> stereo_cal;
	} catch(exception e) {
		cerr << "Could not parse JSON calibration file at "
			 << cal_fn << ".  There was an error opening or parsing:"
			 << endl << e.what() << endl << "Exiting.";
		throw(e);
	}
	stereo_cal["enableGpu"] = enableGpu;

	// Load calibration
	cal_data.init(stereo_cal);

	// Initialize GPU
	int numGpus = 0;
	try {
		numGpus = cuda::getCudaEnabledDeviceCount();
	} catch (exception e) {
		logger->logError(e.what());
	}
	if(numGpus > 0) {
		stringstream ss;
		ss << "Have " << numGpus << " GPUs.";
		logger->logDebug(ss.str());
	} else {
		logger->logError("No GPU found!");
	}
	
	// Initialize underlying algorithm
	if(alg_name == "GpuFastWithBinnedKps") {
		alg = new GpuFastWithBinnedKps(bms);
	} else {
		// Default to doing nothing
		alg = new DummyAlg(bms);
	}
	alg->init(options, logger, cal_data);
}

////////////////////////////////////////////////////////////
// Passthrough functions to underlyling algorithm
////////////////////////////////////////////////////////////

void ImageProcessing::processImages(ImageDataSet imgData) {
	if(alg != NULL) {
		bmImgTotal.start();
		alg->processImages(imgData);
		bmImgTotal.end();
	}
}

PointCloudDataMessage * ImageProcessing::getPointCloud() {
	if(alg != NULL) {
		return alg->getPointCloud();
	} else {
		return NULL;
	}
}

bool ImageProcessing::isPointCloudAvailable() {
	if(alg != NULL) {
		return alg->isPointCloudAvailable();
	} else {
		return false;
	}
}

bool ImageProcessing::areCvWindowsOpen() {
	if(alg != NULL) {
		return alg->areCvWindowsOpen();
	} else {
		return false;
	}
}
