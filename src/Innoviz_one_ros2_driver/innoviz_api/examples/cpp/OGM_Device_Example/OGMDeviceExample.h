///////////////////////////////////////////////////////////
//  OGMCMDeviceExample.h
//  Implementation of the Class OGMCMDeviceExample
//  Created on:      02-Aug-2022 9:57:27 AM
//  Original author: sarah.foox
///////////////////////////////////////////////////////////

#ifndef _OGM_CM_DEVICE_EXAMPLE_H
#define _OGM_CM_DEVICE_EXAMPLE_H

#include "interface/DeviceApi.h"
#include <string>

enum OGMRequiredBuffers
{
	OGM_LIDAR_DETECTIONS,
	OGM_OBJECT_LIST,
	OGM_GEOMETRIES_LIST
};

class OGMCMDeviceExample
{

public:
	explicit OGMCMDeviceExample(std::string config_path);
	~OGMCMDeviceExample();
	void ReadFrames();
	void RecordUDPStream();

private:
	static constexpr uint32_t max_num_of_buffers = 100;
	static constexpr uint32_t max_detections = 192000;
	static constexpr uint32_t print_detections_step = 6000;

	std::unique_ptr<invz::IDevice> m_device;
	std::vector<invz::FrameDataUserBuffer> m_userBuffers;

	void frameCallback(uint32_t* id);
	void initUserBuffers(size_t buffersCount, std::vector<invz::FrameDataUserBuffer>& userBuffers, std::vector<invz::FrameDataAttributes>& attributes);

};
#endif // _OGM_CM_DEVICE_EXAMPLE_H

