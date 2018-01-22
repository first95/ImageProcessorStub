/*
	dummyPointCloud.h
	
	False data for use in testing
	
	2017-07-25  JDW  Created.
*/

#ifndef __DUMMYPOINTCLOUD_H__
#define __DUMMYPOINTCLOUD_H__

#include <message_formats.h>



class DummyPointCloud {
private:
	// Keep these two declarations sequential here so they are laid out
	// sequentially in memory
	PointCloudDataMessage msg;
	CloudPoint points[25];
	
public:
	DummyPointCloud() 
	{ 
		msg.setNumPointsThisMsg(sizeof(points) / sizeof(CloudPoint));
		
		// Form a simple V in the plane of the platform.
		// The following assumes the "points" array is of odd-numbered length.
		points[0].setPoint(0, 0, 0);
		const double slope = 0.5;
		for(int i = 0; i < (msg.getNumPointsThisMsg() / 2); ++i) {
			double depth = -0.01 * (i + 1);
			points[1 + i * 2    ].setPoint(depth,  depth * slope, 0);
			points[1 + i * 2 + 1].setPoint(depth, -depth * slope, 0);
		}
	}
	PointCloudDataMessage * getMsg() { return &msg; };
};

#endif // __DUMMYPOINTCLOUD_H__
