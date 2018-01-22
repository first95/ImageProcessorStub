/*
	gpuCpuFastWithBinnedKps.h
	
	This is a mix of CpuFastWithBinnedKps and GpuFastWithBinnedKps.  FAST and ORB
	seem to run faster on the CPU, but image preprocessing runs faster on the
	GPU.  This takes the faster side of each operation.
	Uses GPU.
	
	2018-1-5  JDW  Created.
*/
#ifndef __PCG_GPUCPUFASTWITHBINNEDKPS_H__
#define __PCG_GPUCPUFASTWITHBINNEDKPS_H__

#include "stereoPtCloudGenAlg.h"
#include "gpuPreUndistortAlg.h"

class GpuCpuFastWithBinnedKps : public GpuPreUndistortAlg {
private:
	static const unsigned int MAX_NUM_BINS = 128-1;
	
	// Helper function - performs per-image processing.
	// Input: image, algorithm parameters, member data
	// Output: keypoints & descriptors with common indices, bin boundary indicies
	void processImage(GpuMat image, vector<KeyPoint> &kp, Mat &desc, unsigned int (&bin_bound_idx)[MAX_NUM_BINS + 1]);

	// Helper function - the core of this particular algorithm.
	// Works on member data, specifically imgLRectGpu, imgRRectGpu
	void computeStereoPtCloudsGpu();
	
protected:
	// Member data
	Benchmarker bmBlurImage       ;
	Benchmarker bmFindingKps      ;
	Benchmarker bmSortingKps      ;
	Benchmarker bmComputingDesc   ;
	Benchmarker bmBinningKps      ;
	Benchmarker bmMatchingKps     ;
	Benchmarker bmFilteringKpsImp ;
	Benchmarker bmFilteringKpsDisp;
	Benchmarker bmComputingDepths ;

public:
	GpuCpuFastWithBinnedKps(list<const Benchmarker *> * _bms) : 
		GpuPreUndistortAlg(_bms),
		bmBlurImage       ("Blurring image"),
		bmFindingKps      ("Finding keypoints"),
		bmSortingKps      ("Sorting keypoints"),
		bmComputingDesc   ("Computing descriptors"),
		bmBinningKps      ("Binning keypoints"),
		bmMatchingKps     ("Matching keypoints"),
		bmFilteringKpsImp ("Filtering keypoints (improvement)"),
		bmFilteringKpsDisp("Filtering keypoints (disparity)"),
		bmComputingDepths ("Computing depths")
	{
		bms->push_back(&bmBlurImage       );
		bms->push_back(&bmFindingKps      );
		bms->push_back(&bmSortingKps      );
		bms->push_back(&bmComputingDesc   );
		bms->push_back(&bmBinningKps      );
		bms->push_back(&bmMatchingKps     );
		bms->push_back(&bmFilteringKpsImp );
		bms->push_back(&bmFilteringKpsDisp);
		bms->push_back(&bmComputingDepths );
	}
	virtual void init(json options, Logger * lgr, StereoCal calData);
	virtual void processImages(ImageDataSet imgData);
private:

	// Algorithm parameters
	int blurKernelSize;
	int fastThreshold;
	unsigned int numBins;
	double minImprovementFactor;
	unsigned int minDisparityPx;
	
	// OpenCV docs say this is faster than not: https://docs.opencv.org/2.4/modules/gpu/doc/image_filtering.html#gpu-filterengine-gpu
	cv::Ptr<cuda::FilterEngine_GPU> gaussianFilter;
};

#endif // __PCG_GPUCPUFASTWITHBINNEDKPS_H__
