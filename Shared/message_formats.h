/*

Classes to pack and parse messages in the Optical Guide system.


*/

#ifndef __MESSAGE_FORMATS_H__
#define __MESSAGE_FORMATS_H__

#include <netinet/in.h>

typedef enum MsgIdTag {
	INVALID_MSG          = 0x0000,
	POINT_CLOUD_METADATA = 0x0100,
	POINT_CLOUD_DATA     = 0x0101,
	EN_POINT_CLOUD_GEN   = 0x0200,
	DIS_POINT_CLOUD_GEN  = 0x0201,
	SHUTDOWN_PCG         = 0x0202,
} MsgId;

// NOTE: if you add virtual functions to this class, a hidden member void *__vptr will be added
// to the beginning of each class.  This is fine but you'll need to account for it when casting
// incoming buffers.
#pragma pack(push, 1)
class Msg {
private:
	uint16_t msgId;
	uint16_t lenBytes;
protected:
	void setLenB(uint16_t value) { lenBytes = value; }
public:
	Msg(MsgId id, uint16_t len) :
		msgId(id), lenBytes(len)
	{ ; }
	
	MsgId    getMsgId()    const { return (MsgId)msgId; }
	uint16_t getMsgIdRaw() const { return msgId;        }
	uint16_t getLenB ()    const { return lenBytes;     }
};

typedef enum PointCloudSourceTag {
	INVALID           = 0x00,
	TEST_DATA         = 0x01,
	VIS_LIGHT_STEREO  = 0x02,
	LIDAR_DOWNSAMPLED = 0x03,
} PointCloudSource;

class PointCloudMetadataMessage : public Msg {
public:
	uint16_t pointCloudSeqNum;
	uint16_t numPktsThisPointCloud;
	uint16_t numPointsThisPointCloud;
	uint32_t opticalDataCaptureTimeS; // seconds since epoch
	uint16_t opticalDataCaptureTimeMs; // milliseconds since the above
	uint16_t timeSpentProcessingMs;
	uint16_t pointCloudSource_spare;
	
public:
	PointCloudMetadataMessage() :
		Msg(MsgId::POINT_CLOUD_METADATA, sizeof(PointCloudMetadataMessage)),
		pointCloudSeqNum(0),
		numPktsThisPointCloud(0),
		numPointsThisPointCloud(0),
		opticalDataCaptureTimeS(0), 
		opticalDataCaptureTimeMs(0),
		timeSpentProcessingMs(0),
		pointCloudSource_spare(0)
	{;}

	// Raw getters and setters, which merely account for byte ordering
	uint16_t getPointCloudSeqNum         () const { return pointCloudSeqNum        ; }
	uint16_t getNumPktsThisPointCloud    () const { return numPktsThisPointCloud   ; }
	uint16_t getNumPointsThisPointCloud  () const { return numPointsThisPointCloud ; }
	uint32_t getOpticalDataCaptureTimeS  () const { return opticalDataCaptureTimeS ; }
	uint16_t getOpticalDataCaptureTimeMs () const { return opticalDataCaptureTimeMs; }
	uint16_t getTimeSpentProcessingMs    () const { return timeSpentProcessingMs   ; }
	PointCloudSource getPointCloudSource () const { return (PointCloudSource)(pointCloudSource_spare & 0x00FF); }
	
	void     setPointCloudSeqNum         (uint16_t value) { pointCloudSeqNum         = value;}
	void     setNumPktsThisPointCloud    (uint16_t value) { numPktsThisPointCloud    = value;}
	void     setNumPointsThisPointCloud  (uint16_t value) { numPointsThisPointCloud  = value;}
	void     setOpticalDataCaptureTimeS  (uint32_t value) { opticalDataCaptureTimeS  = value;}
	void     setOpticalDataCaptureTimeMs (uint16_t value) { opticalDataCaptureTimeMs = value;}
	void     setTimeSpentProcessingMs    (uint16_t value) { timeSpentProcessingMs    = value;}
	void     setPointCloudSource         (PointCloudSource value) { pointCloudSource_spare = value & 0x00FF; }
};

// Be sure to compile with at least a minimal level of optimization;
// some of these methods will introduce needless overhead otherwise.
class CloudPoint {
private:
	float pointX;
	float pointY;
	float pointZ;
	
public:
	// Coordinates are in meters.
	// Positive X is out the nose of the vehicle,
	// positive Y is out the right wing,
	// positive Z is downward.
	float getPointX() const { return pointX; }
	float getPointY() const { return pointY; }
	float getPointZ() const { return pointZ; }
	
	void setPointX(float value) { pointX = value; }
	void setPointY(float value) { pointY = value; }
	void setPointZ(float value) { pointZ = value; }
	
	void setPoint(float x, float y, float z) {
		setPointX(x);
		setPointY(y);
		setPointZ(z);
	}
	
	CloudPoint() :
		CloudPoint(0, 0, 0)
	{ ; }
	
	CloudPoint(float x, float y, float z) :
		pointX(x), pointY(y), pointZ(z)
	{ ; }
};

class PointCloudDataMessage : public Msg {
private:
	uint16_t pointCloudSeqNum;
	uint16_t pktNumThisPointCloud;
	uint16_t numPointsThisMsg;
	uint16_t reserved0;
	// Following this point in the buffer are a number of CloudPoint objects
	
public:
	PointCloudDataMessage() :
		Msg(MsgId::POINT_CLOUD_DATA, sizeof(PointCloudDataMessage)),
		pointCloudSeqNum(0),
		pktNumThisPointCloud(0),
		numPointsThisMsg(0),
		reserved0(0)
	{;}

	// Raw getters and setters, which merely account for byte ordering
	uint16_t getPointCloudSeqNum     () const { return  pointCloudSeqNum    ; }
	uint16_t getPktNumThisPointCloud () const { return  pktNumThisPointCloud; }
	uint16_t getNumPointsThisMsg     () const { return  numPointsThisMsg    ; }
	
	void     setPointCloudSeqNum     (uint16_t value) { pointCloudSeqNum     = value;}
	void     setNumPktsThisPointCloud(uint16_t value) { pktNumThisPointCloud = value;}
	void     setNumPointsThisMsg     (uint16_t value) {
		numPointsThisMsg = value;
		setLenB(sizeof(PointCloudDataMessage) + value * sizeof(CloudPoint));
	}
	
	// Get a pointer to the start of the point cloud
	CloudPoint* getPointCloud() { return (CloudPoint*)((char*)this + sizeof(PointCloudDataMessage)); }
};

class EnablePointCloudGenerationMsg : public Msg {
	// No other fields
public:
	EnablePointCloudGenerationMsg() :
		Msg(MsgId::EN_POINT_CLOUD_GEN, sizeof(EnablePointCloudGenerationMsg))
	{;}
};

class DisablePointCloudGenerationMsg : public Msg {
	// No other fields
public:
	DisablePointCloudGenerationMsg() :
		Msg(MsgId::DIS_POINT_CLOUD_GEN, sizeof(DisablePointCloudGenerationMsg))
	{;}
};

class ShutdownPointCloudGenerator : public Msg {
	// No other fields
public:
	ShutdownPointCloudGenerator() :
		Msg(MsgId::SHUTDOWN_PCG, sizeof(ShutdownPointCloudGenerator))
	{;}
};

#pragma pack(pop)
#endif // __MESSAGE_FORMATS_H__

