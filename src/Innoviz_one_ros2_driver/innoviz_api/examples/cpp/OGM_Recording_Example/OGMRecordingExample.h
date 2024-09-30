///////////////////////////////////////////////////////////
//  OGMCMRecordingExample.h
//  Implementation of the Class OGMCMRecordingExample
//  Created on:      02-Aug-2022 10:09:48 AM
//  Original author: sarah.foox
///////////////////////////////////////////////////////////

#ifndef _CM_OGM_FILE_DEMO_H
#define _CM_OGM_FILE_DEMO_H

#include "interface/FileReaderApi.h"
#include <string>

enum OGMRequiredBuffers
{
	OGM_LIDAR_DETECTIONS,
	OGM_OBJECT_LIST,
	OGM_GEOMETRIES_LIST
};

class OGMCMRecordingExample
{

public:
	explicit OGMCMRecordingExample(std::string om_file_path);
	virtual ~OGMCMRecordingExample();

	virtual void ReadFrames();

private:
	static constexpr uint32_t max_num_of_buffers = 100;
	static constexpr uint32_t max_detections = 192000;
	static constexpr uint32_t print_detections_step = 6000;

	std::unique_ptr<invz::IReader> m_reader = nullptr;
	std::vector<invz::FrameDataUserBuffer> m_userBuffers;

	void initUserBuffers(size_t buffersCount, std::vector<invz::FrameDataUserBuffer>& userBuffers, std::vector<invz::FrameDataAttributes>& attributes);

};
#endif // _CM_OGM_FILE_DEMO_H

