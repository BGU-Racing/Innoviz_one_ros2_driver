///////////////////////////////////////////////////////////
//  OGMCMRecordingExample.cpp
//  Implementation of the Class OGMCMRecordingExample
//  Created on:      02-Aug-2022 10:09:48 AM
//  Original author: sarah.foox
///////////////////////////////////////////////////////////

#include "OGMRecordingExample.h"
#include <iostream>


OGMCMRecordingExample::OGMCMRecordingExample(std::string recording_path)
{

	m_reader.reset(invz::FileReaderInit(recording_path));

}


OGMCMRecordingExample::~OGMCMRecordingExample()
{
	if (m_reader)
		FileReaderClose(m_reader.release());
}

void OGMCMRecordingExample::initUserBuffers(size_t buffersCount, std::vector<invz::FrameDataUserBuffer>& userBuffers, std::vector<invz::FrameDataAttributes>& attributes)
{
	// Initialize User Buffers
	for (size_t i = 0; i < buffersCount; i++)
	{
		if (attributes[i].known_type == invz::GRAB_TYPE_OGM_LIDAR_DETECTIONS || attributes[i].known_type == invz::GRAB_TYPE_OGM_OBJECT_LIST || attributes[i].known_type == invz::GRAB_TYPE_OGM_GEOMETRIES_LIST)
		{
			invz::FrameDataUserBuffer&& userBuffer(attributes[i]);
			// Swap the byte ordering to 'little-endianess' 
			userBuffer.handle_endianess = 1;
			userBuffers.push_back(userBuffer);
		}
	}
}


void OGMCMRecordingExample::ReadFrames()
{
	std::vector<invz::FrameDataAttributes> attributes;
	attributes.resize(max_num_of_buffers);

	// buffersCount holds the actual number of existing buffers - returned by GetFrameDataAttributes
	size_t buffersCount = max_num_of_buffers;
	m_reader->GetFrameDataAttributes(attributes.data(), buffersCount);

	// buffers to read the required data - 
	initUserBuffers(buffersCount, m_userBuffers, attributes);

	// Initialize to default values - returned by GrabFrame
	uint32_t frameNumber = UINT32_MAX;
	uint64_t timeStamp = 0;

	// Read Frame required data into userBuffers initialized above 
	size_t frames;
	m_reader->GetNumOfFrames(frames);

	std::cout << std::endl;
	std::cout << "Reading " << frames << " frames ... " << std::endl;
	for (uint32_t frame_index = 0; frame_index < frames; ++frame_index)
	{
		auto result = m_reader->GrabFrame(m_userBuffers.data(), (uint32_t)m_userBuffers.size(), frameNumber, timeStamp, frame_index);

		if (result.error_code != invz::ERROR_CODE_OK)
		{
			std::cout << "Grabe frame returned with error: " << result.error_message << std::endl;
		}

		if (m_userBuffers[OGM_LIDAR_DETECTIONS].status == invz::USER_BUFFER_FULL)
		{
			std::cout << std::endl;
			std::cout << "OGM lidar detections buffer received with total size: " << m_userBuffers[OGM_LIDAR_DETECTIONS].dataAttrs.length << " and frame number: " << frameNumber << std::endl;
			auto ogm_detections = reinterpret_cast<invz::OGM_LidarDetectionInterface*>(m_userBuffers[OGM_LIDAR_DETECTIONS].dataBuffer);
			std::cout << std::endl;
			std::cout << "\tFrame cycle counter: " << ogm_detections->header.cycle_counter << ". Frame timestamp: " << ogm_detections->header.timestamp_measurement << std::endl;

			// iterate points array and print every k point
			for (size_t i = 0; i < max_detections; i += print_detections_step) {

				// get and print point position
				const auto& point_pos = ogm_detections->detections.position[i];
				std::cout << std::endl;
				std::cout <<
					"\t\tpoint index: " << i << std::endl;
				std::cout <<
					"\t\t\tposition: " << "radial distance -  " << point_pos.radial_distance
					<< " ,azimuth - " << point_pos.azimuth
					<< " ,elevation - " << point_pos.elevation << std::endl;
				std::cout <<
					"\t\t\treflectivity: " << ogm_detections->detections.reflectivity[i] << std::endl;
				std::cout <<
					"\t\t\texistence probability: " << static_cast<int>(ogm_detections->detections.existence_probability[i]) << std::endl;
			}
		}

		if (m_userBuffers[OGM_OBJECT_LIST].status == invz::USER_BUFFER_FULL)
		{
			std::cout << std::endl;
			std::cout << "OGM objects buffer received with total size: " << m_userBuffers[OGM_OBJECT_LIST].dataAttrs.length << " and frame number: " << frameNumber << std::endl;
			auto objects = reinterpret_cast<invz::OGM_ObjectList*>(m_userBuffers[OGM_OBJECT_LIST].dataBuffer);
			std::cout << std::endl;
			uint64_t timestamp = (objects->header.timeStamp.high * static_cast<uint64_t>(1e+9)) + (objects->header.timeStamp.low);
			std::cout << "\tFrame cycle counter: " << objects->header.frameID << ". Frame timestamp : " << timestamp << std::endl;
			std::cout << std::endl;
		}

		if (m_userBuffers[OGM_GEOMETRIES_LIST].status == invz::USER_BUFFER_FULL)
		{
			std::cout << std::endl;
			std::cout << "OGM geometries buffer received with total size: " << m_userBuffers[OGM_GEOMETRIES_LIST].dataAttrs.length << " and frame number: " << frameNumber << std::endl;
			auto geometries = reinterpret_cast<invz::OGM_GeometryList*>(m_userBuffers[OGM_GEOMETRIES_LIST].dataBuffer);
			std::cout << std::endl;
			uint64_t timestamp = (geometries->header.timeStamp.high * static_cast<uint64_t>(1e+9)) + (geometries->header.timeStamp.low);
			std::cout << "\tFrame cycle counter: " << geometries->header.frameID << ". Frame timestamp : " << timestamp << std::endl;
			std::cout << std::endl;
		}
	}
	std::cout << std::endl;
	std::cout << "Finished reading frames." << std::endl;
}

int main(void)
{
	//set absolute recording file path
	std::string recording_file_path = "";
	std::unique_ptr<OGMCMRecordingExample> m_CMRecordingExample = std::make_unique<OGMCMRecordingExample>(recording_file_path);

	m_CMRecordingExample->ReadFrames();

	return 0;
}