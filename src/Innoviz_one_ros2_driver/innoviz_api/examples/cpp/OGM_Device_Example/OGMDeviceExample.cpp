///////////////////////////////////////////////////////////
//  OGMCMDeviceExample.cpp
//  Implementation of the Class OGMCMDeviceExample
//  Created on:      02-Aug-2022 9:57:27 AM
//  Original author: sarah.foox
///////////////////////////////////////////////////////////

#include "OGMDeviceExample.h"
#include <iostream>
#include <thread>


OGMCMDeviceExample::OGMCMDeviceExample(std::string config_path)
{
	m_device.reset(invz::DeviceInit(config_path));
}


OGMCMDeviceExample::~OGMCMDeviceExample()
{
	if (m_device)
		invz::DeviceClose(m_device.release());
}


void OGMCMDeviceExample::initUserBuffers(size_t buffersCount, std::vector<invz::FrameDataUserBuffer>& userBuffers, std::vector<invz::FrameDataAttributes>& attributes)
{
	// Initialize User Buffers
	for (size_t i = 0; i < buffersCount; i++)
	{
		if (attributes[i].known_type == invz::GRAB_TYPE_OGM_LIDAR_DETECTIONS || attributes[i].known_type == invz::GRAB_TYPE_OGM_OBJECT_LIST || attributes[i].known_type == invz::GRAB_TYPE_OGM_GEOMETRIES_LIST)
		{
			invz::FrameDataUserBuffer&& userBuffer(attributes[i]);
			userBuffers.push_back(userBuffer);
			// Swap the byte ordering to 'little-endianess' 
			userBuffers.back().handle_endianess = 1;
			m_device->ActivateBuffer(attributes[i], true);
		}
	}
}


void OGMCMDeviceExample::frameCallback(uint32_t* id)
{
	uint32_t frameNumber;
	uint64_t timeStamp;

	// Grabbing Frame from device inteface
	auto result = m_device->GrabFrame(m_userBuffers.data(), (uint32_t)m_userBuffers.size(), frameNumber, timeStamp, *id);

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


void OGMCMDeviceExample::ReadFrames()
{
	std::vector<invz::FrameDataAttributes> attributes;
	attributes.resize(max_num_of_buffers);

	// buffersCount holds the actual number of existing buffers - returned by GetFrameDataAttributes
	size_t buffersCount = max_num_of_buffers;

	// buffers to read the required data - 
	m_device->GetFrameDataAttributes(attributes.data(), buffersCount, false);
	initUserBuffers(buffersCount, m_userBuffers, attributes);

	invz::Result status;
	auto callback = std::bind(&OGMCMDeviceExample::frameCallback, this, std::placeholders::_1);
	status = m_device->RegisterFrameCallback(callback);
	std::cout << "Start Grabbing ... Press any key + Enter to stop" << std::endl;

	char user;
	std::cin >> user;
	m_device->UnregisterFrameCallback(); 
}

void OGMCMDeviceExample::RecordUDPStream()
{
	invz::Result status;
	auto recording_path = "./MyRecording";

	std::cout << "Start recording..." << std::endl;
	m_device->StartRecording(recording_path);
	
	std::this_thread::sleep_for(std::chrono::seconds(5));
	
	m_device->StopRecording();

	std::cout << "Finished recording." << std::endl;
}

int main(void)
{
	std::cout << "Start of examples.." << std::endl;
	std::string config_files_path = "../../lidar_configuration_files";
	std::string device_config_file = "iPEM.json";
	std::unique_ptr<OGMCMDeviceExample> m_OGMDevice = std::make_unique<OGMCMDeviceExample>(config_files_path + "/" + device_config_file);

	m_OGMDevice->RecordUDPStream();
	m_OGMDevice->ReadFrames();

	std::cout << "End of examples.." << std::endl;
	return 0;
}