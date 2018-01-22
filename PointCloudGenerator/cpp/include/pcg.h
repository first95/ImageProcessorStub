/*
	pcg.h
	
	Declarations for the Point Cloud Generator application.
	
	2016-12-23  JDW  Created.
*/

#ifndef __PCG_H__
#define __PCG_H__

#include <unistd.h> // for sync()
#include <list>

#include "imageAcquisition.h"
#include "imageProcessing.h"
#include "messaging.h"
#include "logger.h"
#include "attitudeTracker.h"
#include "lidarReader.h"
#include "dummyPointCloud.h"

using namespace std;

typedef struct affine3d_tag {
	double t[3][4];
} affine3d;

class PcgMain {
private:
	static const char * DEFAULT_CONFIG_FILENAME;

	// Submodule objects
	list<const Benchmarker *> allBms;
	Logger logger;
	Messaging messaging;
	ImageAcquisition img_acquisition;
	ImageProcessing img_processing;
	AttitudeTracker attitude_tracker;
	LidarReader lidar;
	Benchmarker bmOneFrame;
	Benchmarker bmImageAcq;
	Benchmarker bmPcXform;
	Benchmarker bmSaveFiles;
	Benchmarker bmSyncFs;

	// Items controlled by the configuration file
	bool pcgEnabled; 
	bool useLidar; 
	bool useStereoCams; 
	bool correctLidarPointCloud;
	bool correctStereoPointCloud;
	bool outputPcRecEnabled;
	affine3d lidarTransform ;
	affine3d stereoTransform;
	string recDataPath;
	string recOutputPcPath;
	string recOutputPcTsPattern;
	
	// Private methods
	void handleNewMessages();
	void affineTransformPointCloud(PointCloudDataMessage * pc, affine3d transform);
	void init(char * config_fn);
	void enable();
	void disable();
	affine3d makeXform(double pitchDownRad, double downwardOffsetCm = 0, double forwardOffsetCm = 0, double rightwardOffsetCm = 0);
	void saveOutputPcToFile(PointCloudMetadataMessage *md, PointCloudDataMessage * pc);
	void summarizeBenchmarksToLog();
	
public:
	int main(char * config_fn);
	PcgMain() : 
		allBms(),
		img_acquisition(&allBms),
		img_processing (&allBms),
		lidar          (&allBms),
		bmOneFrame ("Frame total"),
		bmImageAcq ("Image acquisition"),
		bmPcXform  ("Point cloud rotation"),
		bmSaveFiles("Saving point cloud to disk"),
		bmSyncFs   ("Synchronizing filesystem")
	{
		allBms.push_back(&bmOneFrame );
		allBms.push_back(&bmImageAcq );
		allBms.push_back(&bmPcXform  );
		allBms.push_back(&bmSaveFiles);
		allBms.push_back(&bmSyncFs   );
	}
};

#endif // __PCG_H__
