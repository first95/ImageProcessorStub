/*

Main entry point for Optical Guide's (6298) Point Cloud Generator (PCG) application.
Originally developed on Rough Bottom (8136) by MPU/DRC/JCW, modified for use here.

2016-12-21  JDW  Created from Rough Bottom code

*/
#include <pcg.h>
#include <stdio.h>
#include <string>
#include <iostream>
#include <fstream>
#include <sys/time.h>
#include <sys/stat.h>
#include <unistd.h>

using json = nlohmann::json;
using namespace std;

// Consider an option to override this with an argument
const char * PcgMain::DEFAULT_CONFIG_FILENAME = "../../config/pcgConfig.json";

void PcgMain::handleNewMessages() {
	// Check for messages - will block when PCG is disabled.
	Msg * rxd_msg = messaging.checkForMessage();
	if(rxd_msg != NULL) {
		switch(rxd_msg->getMsgId()) {
			case MsgId::EN_POINT_CLOUD_GEN:
				enable();
			break;
			case MsgId::DIS_POINT_CLOUD_GEN:
				disable();
			break;
			case MsgId::SHUTDOWN_PCG:
				logger.logInfo("Shutting down.");
				// Make sure this user has passwordless sudo privs
				system("sudo shutdown -hP now");
			break;
			default:
				stringstream ss;
				ss << "Unrecognized MID 0x" << hex << setw(4) << rxd_msg->getMsgId();
				logger.logWarning(ss.str());
			break;
		}
		
		delete[] rxd_msg;
		rxd_msg = NULL;
	}
}


void PcgMain::enable() {
	if(!pcgEnabled) {
		if(!(useLidar || useStereoCams)) {
			logger.logWarning("No point cloud sources configured.  Cannot enable.");
		} else {
			logger.logInfo("Enabling point cloud generation.");
			if(useStereoCams) { img_acquisition.start(); }
			if(useLidar)      { lidar          .start(); }
			pcgEnabled = true;
			messaging.setBlockingListen(false);
		}
	} else {
		logger.logInfo("Tried to enable when already enabled.");
	}
}
void PcgMain::disable() {
	if(pcgEnabled) {
		logger.logInfo("Disabling point cloud generation.");
		pcgEnabled = false;
			if(useStereoCams) { img_acquisition.stop(); }
			if(useLidar)      { lidar          .stop(); }
		messaging.setBlockingListen(true);
	} else {
		logger.logInfo("Tried to disable when already disabled.");
	}
}

void PcgMain::init(char * config_fn) {
	// Open and parse file.
	// Throw an exception on failure.
	json options;
	{
		if(config_fn == NULL)
		{ config_fn = (char*)DEFAULT_CONFIG_FILENAME; }
		ifstream config_file(config_fn);
		config_file >> options;
	}

	// Parse main-level options and pick out sections
	string cur_key = "";
	json logging_config;
	json acquisition_config;
	json processing_config;
	json attitude_config;
	json messaging_config;
	json lidar_config;
	json platform_offset;
	json output_rec_config;
	double platform_pitch_down_rad = 0.0;
	double platform_downward_offset_cm = 0.0;
	bool enable_gpu = true;
	try {
		cur_key = "startEnabled";               pcgEnabled         = options[cur_key];
		cur_key = "useLidar";                   useLidar           = options[cur_key];
		cur_key = "useStereoCams";              useStereoCams      = options[cur_key];
		cur_key = "recDataPath";                recDataPath        = options[cur_key];
		cur_key = "enableGpu";                  enable_gpu         = options[cur_key];

		cur_key = "platformOffsetFromVehicle";  platform_offset    = options[cur_key];
		cur_key = "pitchDownAngleRad";          platform_pitch_down_rad     = platform_offset[cur_key];
		cur_key = "downwardOffsetCm";           platform_downward_offset_cm = platform_offset[cur_key];
		cur_key = "correctLidarPointCloud";     correctLidarPointCloud      = platform_offset[cur_key];
		cur_key = "correctStereoPointCloud";    correctStereoPointCloud     = platform_offset[cur_key];

		cur_key = "outputRecording";            output_rec_config  = options[cur_key];
		cur_key = "recDir";                     recOutputPcPath      = output_rec_config[cur_key];
		cur_key = "recordProcessedPointClouds"; outputPcRecEnabled   = output_rec_config[cur_key];
		cur_key = "timestampPattern";           recOutputPcTsPattern = output_rec_config[cur_key];

		cur_key = "logging";          logging_config     = options[cur_key];
		cur_key = "imageAcquisition"; acquisition_config = options[cur_key];
		cur_key = "imageProcessing";  processing_config  = options[cur_key];
		cur_key = "attitudeTracker";  attitude_config    = options[cur_key];
		cur_key = "messaging";        messaging_config   = options[cur_key];
		cur_key = "lidar";            lidar_config       = options[cur_key];
	} catch (domain_error e) {
		cerr << "JSON field missing or corrupted.  Please see example file in config directory."
			 << endl << "While reading key \"" << cur_key << "\": " << e.what() << endl;
		throw e;
	}
	
	// Create a directory for this recording, and a subdir for output
	stringstream path;
	time_t now_s = system_clock::to_time_t(system_clock::now()); 
	path << put_time(localtime(&now_s), recDataPath.c_str());
	recDataPath = path.str();
	if(mkdir(recDataPath.c_str(), 0777) < 0) {
		cerr << "Couldn't create recording dir.  Error #" << errno << endl;
	}
	recOutputPcPath = recDataPath + "/" + recOutputPcPath;
	if(mkdir(recOutputPcPath.c_str(), 0777) < 0) {
		cerr << "Couldn't create processed point cloud dir.  Error #" << errno << endl;
	}
	// Submodules will handle their own subdir creation
	logging_config    ["path"] = recDataPath;
	acquisition_config["path"] = recDataPath;
	attitude_config   ["path"] = recDataPath;
	lidar_config      ["path"] = recDataPath;
	
	// Use of the GPU should be disable-able in all submodules that use it
	processing_config["enableGpu"] = enable_gpu;
	
	// Compute transformation matrices
	lidarTransform = stereoTransform = makeXform(platform_pitch_down_rad, platform_downward_offset_cm);
	
	// Pass subsections to component modules
	logger.init(logging_config);
	messaging.init(messaging_config, &logger);
	messaging.setBlockingListen(!pcgEnabled);
	img_acquisition .init(acquisition_config, &logger);
	img_processing  .init(processing_config,  &logger);
	attitude_tracker.init(attitude_config,    &logger);
	lidar           .init(lidar_config,       &logger);
	
	
	// Copy the config file into the recording directory so we know what was used.
	{
		string save_path = recDataPath + "/active_config.json";
		ifstream config_file(config_fn, ios::binary);
		ofstream saved_config(save_path, ios::binary);
		saved_config << config_file.rdbuf();
	}
}

int PcgMain::main(char * config_fn)
{
	// Load configuration file and initialize
	try {
		init(config_fn);
	} catch(exception e) {
		cerr << "The PCG application requires a valid JSON file at "
			 << config_fn << ".  There was an error opening or parsing:"
			 << endl << e.what() << endl << "Exiting.";
		return 1;
	}
	
	// From this point forward, the logger has been initialized.
	// Thus, the use of cout and cerr are discouraged.  Please use the logging
	// functions instead.
	logger.logInfo("---------------------------------------------------------");
	logger.logInfo("Initialized.");
	if(pcgEnabled) { 
		logger.logInfo("Starting enabled.");
		// Not actually enabled yet
		pcgEnabled = false;
		enable();
	}
	
	
	uint16_t pc_seq = 0;
	if(pcgEnabled && useStereoCams) {
		img_acquisition.beginAcquisition();
	}
	while(true) {
		bmOneFrame.start();
		handleNewMessages();
		
		// Store attitude
		attitude_tracker.estimateAttitude();
		
		// Process images
		if(pcgEnabled) {
			ImageDataSet image_data;
			if(useStereoCams) {
				image_data = img_acquisition.acquireImages();
				// logger.logDebug(img_acquisition.isPlaybackEnabled() ? "Playback is enabled." : "Playback is disabled.");
				// logger.logDebug(img_acquisition.isPlaybackComplete()? "Playback is complete." : "Playback is incomplete.");
				if(img_acquisition.isPlaybackEnabled() && img_acquisition.isPlaybackComplete()) {
					logger.logInfo("Reached end of playback.");
					disable();
				} else {
					img_processing.processImages(image_data);
				}
			} 
			if(useLidar) {
				lidar.handleIncomingData();

				// The LIDAR currently scans a full circle faster than
				// the rest of the code runs.  So we usually have a valid point cloud.
				if(lidar.isNewPcAvail()) {
					chrono::system_clock::duration acq_timestamp = lidar.getLastPcAcquisitionTime().time_since_epoch();
					PointCloudDataMessage * cloud = lidar.popPointCloud();
					
					// Use false data for testing
					// DummyPointCloud dpc;
					// cloud = dpc.getMsg();
					
					if(cloud != NULL && cloud->getNumPointsThisMsg() > 0) {
						if(correctLidarPointCloud) {
							affineTransformPointCloud(cloud, lidarTransform);
						}
						
						PointCloudMetadataMessage metadata;
						cloud->  setPointCloudSeqNum        (pc_seq);
						metadata.setPointCloudSeqNum        (pc_seq);
						metadata.setNumPktsThisPointCloud   (1);
						metadata.setNumPointsThisPointCloud (cloud->getNumPointsThisMsg());
						metadata.setOpticalDataCaptureTimeS (chrono::duration_cast<chrono::seconds>     (acq_timestamp).count());
						metadata.setOpticalDataCaptureTimeMs(chrono::duration_cast<chrono::milliseconds>(acq_timestamp).count() % 1000);
						metadata.setTimeSpentProcessingMs   (0); // TODO
						metadata.setPointCloudSource        (PointCloudSource::LIDAR_DOWNSAMPLED);
						
											stringstream lidarSs;
						lidarSs << "Sending " << cloud->getNumPointsThisMsg() << " LIDAR points.";
						logger.logDebug(lidarSs.str());

						messaging.sendMessage(&metadata);
						messaging.sendMessage(cloud);
						if(outputPcRecEnabled) {
							saveOutputPcToFile(&metadata, cloud);
						}
						pc_seq++;
					} else {
						logger.logDebug("No LIDAR point cloud to send this iteration.");
					}
				} else {
					logger.logDebug("No fresh LIDAR point cloud to send this iteration.");
				}
			}
			
			// Get the next one started
			img_acquisition.beginAcquisition();
			
			// Send out point cloud
			if(useStereoCams) {
				if(img_processing.isPointCloudAvailable()) {
					PointCloudDataMessage * cloud = img_processing.getPointCloud();

					// Use false data for testing
					// DummyPointCloud dpc;
					// cloud = dpc.getMsg();
					
					if(correctStereoPointCloud) {
						affineTransformPointCloud(cloud, stereoTransform);
					}

					// chrono::system_clock::duration now = chrono::system_clock::now().time_since_epoch();
					// chrono::duration_cast<chrono::milliseconds>(acq_timestamp).count() % 1000;
					PointCloudMetadataMessage metadata;
					cloud->  setPointCloudSeqNum        (pc_seq);
					metadata.setPointCloudSeqNum        (pc_seq);
					metadata.setNumPktsThisPointCloud   (1);
					metadata.setNumPointsThisPointCloud (cloud->getNumPointsThisMsg());
					metadata.setOpticalDataCaptureTimeS (image_data.getTimeAcquiredS() );
					metadata.setOpticalDataCaptureTimeMs(image_data.getTimeAcquiredMs());
					metadata.setTimeSpentProcessingMs   (image_data.getDurationSinceAcquiredMs());
					metadata.setPointCloudSource        (PointCloudSource::VIS_LIGHT_STEREO);
					
					messaging.sendMessage(&metadata);
					messaging.sendMessage(cloud);
					pc_seq++;
				} else {
					logger.logDebug("No stereo point cloud to send this iteration.");
				}
			} 
			
			summarizeBenchmarksToLog();
			
			// Sometimes we wish to sleep after processing, due to a max frame rate
			// or due to a preferred playback rate.
			// TODO: reduce sleep time based on how long the processing took.
			// TODO: add an optional frame rate cap
			chrono::system_clock::duration sleep_time = chrono::milliseconds(0);
			if(img_acquisition.isPlaybackEnabled()) {
				sleep_time = img_acquisition.getFrameDelay();
			} else {
				sleep_time = std::chrono::milliseconds(0);;
			}
			stringstream ss;
			ss << "Sleeping for " << chrono::duration_cast<chrono::milliseconds>(sleep_time).count()
				<< "ms between frames.";
			logger.logInfo(ss.str());
			if(img_processing.areCvWindowsOpen()) {
				// We need a GUI-friendly form of sleep for the debug windows to update
				int ms_to_sleep = chrono::duration_cast<chrono::milliseconds>(sleep_time).count();
				if(ms_to_sleep <= 0) { ms_to_sleep = 1; };
				cv::waitKey(ms_to_sleep);
			} else {
				// Standard C++11 sleep
				this_thread::sleep_for(sleep_time);
			}
		} else {
			// To avoid spinlock, it's important that messaging.setBlockingListen(true);
			// was called before this point.
		}
		// OS call to flush all file buffers
		bmSyncFs.start();
		sync();
		bmSyncFs.end();
		
		bmOneFrame.end();
	}
	
	return 0;
}

// Apply the given transformation matrix, on the CPU.
// This may be slower than doing this on the GPU.
// We don't use opencv matrix operations here, to avoid
// the overhead of copying into and out of the message structure.
// Sometime it might be worth testing that, to see if it's
// faster.
// 
// Matrix must be 3x4.  Transform will be applied as:
// x_new = t[0][0] * x + t[0][1] * y + t[0][2] * z + t[0][3]
// y_new = t[1][0] * x + t[1][1] * y + t[1][2] * z + t[1][3]
// z_new = t[2][0] * x + t[2][1] * y + t[2][2] * z + t[2][3]
void PcgMain::affineTransformPointCloud(PointCloudDataMessage * pc, affine3d transform) {
	bmPcXform.start();
	uint16_t num_points = pc->getNumPointsThisMsg();
	CloudPoint* point   = pc->getPointCloud();
	for(uint16_t i = 0; i < num_points; ++i) {
		// Since we sequentially modify the x,y,z components, we need to
		// store them beforehand.  Or as least the y,z parts.
		// It's very likely that this copy is a slow point.
		CloudPoint old_point;
		memcpy(&old_point, &(point[i]), sizeof(CloudPoint));
		
		// Perform multiplication as doubles, then cast down to float
		point[i].setPointX((float)(transform.t[0][0] * old_point.getPointX()
		                         + transform.t[0][1] * old_point.getPointY()
		                         + transform.t[0][2] * old_point.getPointZ()
		                         + transform.t[0][3]));
		point[i].setPointY((float)(transform.t[1][0] * old_point.getPointX()
		                         + transform.t[1][1] * old_point.getPointY()
		                         + transform.t[1][2] * old_point.getPointZ()
		                         + transform.t[1][3]));
		point[i].setPointZ((float)(transform.t[2][0] * old_point.getPointX()
		                         + transform.t[2][1] * old_point.getPointY()
		                         + transform.t[2][2] * old_point.getPointZ()
		                         + transform.t[2][3]));
	}
	bmPcXform.end(num_points);
}

affine3d PcgMain::makeXform(double pitchDownRad, double downwardOffsetCm, double forwardOffsetCm, double rightwardOffsetCm) {
	return {{{ cos(-pitchDownRad), 0, sin(-pitchDownRad), forwardOffsetCm   * 0.01},
	         {0,                  1, 0,                   rightwardOffsetCm * 0.01},
	         {-sin(-pitchDownRad), 0, cos(-pitchDownRad), downwardOffsetCm  * 0.01}}};
}

void PcgMain::saveOutputPcToFile(PointCloudMetadataMessage *md, PointCloudDataMessage * pc) {
	bmSaveFiles.resume();
	time_t time_acq_s = md->opticalDataCaptureTimeS;
	ofstream outfile;
	
	// Create file
	stringstream path;
	path << recOutputPcPath;
	path << put_time(localtime(&time_acq_s), recOutputPcTsPattern.c_str());
	path << "_" << setw(3) << setfill('0') << md->opticalDataCaptureTimeMs << ".csv";
	logger.logDebug("Saving PC to " + path.str());
	outfile.open(path.str(), ofstream::out);
	// Read in MATLAB with csvread(filename, 1).
	// It's MATLAB's default drag action, and also permits us to leave this header line.
	outfile << "\"X (m)\",\"Y (m)\",\"Z (m)\"" << endl;

	// Save point cloud
	int num_points = pc->getNumPointsThisMsg();
	CloudPoint* point = pc->getPointCloud();
	for(int i = 0; i < num_points; ++i) {
		outfile << point[i].getPointX() << ','
		        << point[i].getPointY() << ','
		        << point[i].getPointZ() << endl;
	}

	outfile.close();
	logger.logDebug("Saving complete.");
	bmSaveFiles.pause(1);
}

// Iterate through allBms and write a summary of benchmarkers
// to the logger object
void PcgMain::summarizeBenchmarksToLog() {
	stringstream ss;
	for(auto bm = allBms.begin(); bm != allBms.end(); ++bm) {
		ss << endl << (*bm)->getName() << ": "  << (*bm)->getAvgMs() << "ms avg";
		if((*bm)->getAvgItemsProcessed() > 0) {
			ss << ", " << (*bm)->getAvgItemsProcessed() << " avg items";
		}
		ss << " (" << (*bm)->getIterations() << " iterations).";
	}
	logger.logDebug(ss.str());
}


// Entry point
int main(int argc, char ** argv)
{
	PcgMain pcg;
	
	// We currently support exactly one argument, which is optional:
	// the path to the configuration file.
	char * config_fn = NULL;
	if(argc > 1) {
		config_fn = argv[1];
	}
	return pcg.main(config_fn);
}

