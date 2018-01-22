/*
	stereoCal.h
	
	Class for storing stereo calibration data
	
	2016-12-23  JDW  Created.
	2017-03-02  JDW  Moved into its own file.
*/
#ifndef __PCG_STERCAL_H__
#define __PCG_STERCAL_H__

#include "json.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/core/cuda.hpp>
using namespace cv;
using json = nlohmann::json;
using namespace std;

class StereoCal {
private:
	static const int CAM_MAT_ROWS=3, CAM_MAT_COLS=3;
	static const int DIST_COEFFS=5;
	static const int R_MAT_ROWS=3, R_MAT_COLS=3;
	static const int P_MAT_ROWS=3, P_MAT_COLS=4;
	static const int T_MAT_ROWS=3, T_MAT_COLS=1;
	static const int Q_MAT_ROWS=4, Q_MAT_COLS=1;
	
	// Loaded from file
	unsigned int imageWidth  = 0;
	unsigned int imageHeight = 0;
	double focalLen  ; // In "pixel units"
	double baselineCm;
	double triangulationConst;
	
	// Loaded from file - computed with:
	// https://code.crearecomputing.com/LaserMetrology/LaserMetrologyCommon/blob/develop/PythonCommon/laser_metrology_toolbox/laser_metrology_toolbox/calibration/examples/stereo_example2.py
	// It would be cleaner to use arrays of matrices, but the default constructors
	// don't work as desired here, so we make them individual items.
	Mat_<double> leftCamMatrix ;
	Mat_<double> rightCamMatrix;
	Mat_<double> leftDistCoeffs, rightDistCoeffs;
	Mat_<double> R;
	Mat_<double> T;
	
	// Computed from calibration file once just after loading
	Mat_<double> R1; // Left  output rectification transform
	Mat_<double> R2; // Right output rectification transform
	Mat_<double> P1; // Left  output projection matrix
	Mat_<double> P2; // Right output projection matrix
	Mat_<double> Q ; // Disparity-to-depth mapping matrix
	        Mat cpuUndistortMapsLeft[2];
	        Mat cpuUndistortMapsRight[2];
	cuda::GpuMat gpuUndistortMapsLeft[2];
	cuda::GpuMat gpuUndistortMapsRight[2];
	
	
	// Copy from JSON matrix in the format [[1,2,3],[4,5,6],[7,8,9]]
	// into an OpenCV matrix.  Takes dimensions from the OpenCV matrix
	// and will throw an exception if the JSON matrix lacks sufficient elements.
	void loadInto(Mat_<double> &cvMat, json jsonMat);

public:
	StereoCal() :
		leftCamMatrix(CAM_MAT_ROWS, CAM_MAT_COLS),
		rightCamMatrix(CAM_MAT_ROWS, CAM_MAT_COLS),
		leftDistCoeffs(1, DIST_COEFFS), rightDistCoeffs(1, DIST_COEFFS),
		R (R_MAT_ROWS, R_MAT_COLS),
		T (T_MAT_ROWS, T_MAT_COLS),
		R1(R_MAT_ROWS, R_MAT_COLS),
		R2(R_MAT_ROWS, R_MAT_COLS),
		P1(P_MAT_ROWS, P_MAT_COLS),
		P2(P_MAT_ROWS, P_MAT_COLS),
		Q (Q_MAT_ROWS, Q_MAT_COLS)
	{ ; }
	const cuda::GpuMat* getGpuUndistortMapsLeft ()    { return (const cuda::GpuMat*)gpuUndistortMapsLeft ; }
	const cuda::GpuMat* getGpuUndistortMapsRight()    { return (const cuda::GpuMat*)gpuUndistortMapsRight; }
	const          Mat* getCpuUndistortMapsLeft ()    { return (const          Mat*)cpuUndistortMapsLeft ; }
	const          Mat* getCpuUndistortMapsRight()    { return (const          Mat*)cpuUndistortMapsRight; }
	               Mat  getCpuProjectionMatrixLeft () { return (               Mat )P1; }
	               Mat  getCpuProjectionMatrixRight() { return (               Mat )P2; }
	const double getTriangulationConst() { return triangulationConst; }
	void init(json options);
};

#endif // __PCG_STERCAL_H__
