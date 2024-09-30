// Copyright 2021 Innoviz Technologies
//
// Licensed under the Innoviz Open Dataset License Agreement (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://github.com/InnovizTechnologies/InnovizAPI/blob/main/LICENSE.md
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

///////////////////////////////////////////////////////////
//  CommonUtils.cpp
//  Implementation of class CommonUtils
//  Created on:      07-Oct-2020 1:13:25 AM
//  Original author: julia.sher
///////////////////////////////////////////////////////////

#include "CommonUtils.h"
#include <iostream>
#include <fstream>
#include <chrono> // For high-resolution clock
#include <iomanip> // For time formatting


CommonUtils::CommonUtils()
{
	
}



CommonUtils::~CommonUtils() 
{

}



bool CommonUtils::objectExists(const invz::ObjectDetection& object)
{
	bool exists = false;
	for (int i = invz::CLASS_TYPE_PASSENGER_CAR; i < invz::CLASS_TYPE_NUM_OF_CLASS_TYPES; ++i)
	{
		if (object.probability_of_classtype.percentage[i] > 0)
		{
			exists = true;
		}
	}
	return exists;
}


invz::ClassesType CommonUtils::objectClass(invz::ClassProbability probability_of_classtype)
{
	invz::ClassesType object_class = invz::ClassesType::CLASS_TYPE_UNKNOWN_CLASS_TYPE;

	for (int i = invz::CLASS_TYPE_PASSENGER_CAR; i < invz::CLASS_TYPE_NUM_OF_CLASS_TYPES; ++i)
	{
		if (probability_of_classtype.percentage[i] > 0)
		{
			object_class = static_cast<invz::ClassesType>(i);
			break;
		}
	}
	return object_class;
}


std::string CommonUtils::getObjectClassStr(invz::ClassProbability probability_of_classtype)
{
	invz::ClassesType res_class = objectClass(probability_of_classtype);
	std::string res_class_str;

	switch (res_class)
	{
	case invz::ClassesType::CLASS_TYPE_PASSENGER_CAR:
		res_class_str = "Passenger car";
		break;
	case invz::ClassesType::CLASS_TYPE_TRUCKS:
		res_class_str = "Truck";
		break;
	case invz::ClassesType::CLASS_TYPE_MOTORCYCLE:
		res_class_str = "Motorcycle";
		break;
	case invz::ClassesType::CLASS_TYPE_PEDESTRIAN:
		res_class_str = "Pedestrian";
		break;
	case invz::ClassesType::CLASS_TYPE_BICYCLE:
		res_class_str = "Bibycle";
		break;
	case invz::ClassesType::CLASS_TYPE_UNKNOWN_CLASS_TYPE:
	default:
		res_class_str = "Unknown";
		break;
	}
	return res_class_str;
}


void CommonUtils::HandleObjectsData(invz::FrameDataUserBuffer& ref_buffer)
{
	auto detections = reinterpret_cast<invz::ObjectDetection*>(ref_buffer.dataBuffer);
	std::cout << std::endl;
	std::cout << "Reading object detections ..." << std::endl;
	for (uint32_t i = 0; i < ref_buffer.dataAttrs.length; ++i)
	{
		invz::ObjectDetection object = detections[i];
		if (objectExists(object))
		{
			std::cout << "\tObject " << i << " data:" << std::endl;
			std::cout << "\t\tid: " << object.unique_id << std::endl;
			std::cout << "\t\tclassification: " << getObjectClassStr(object.probability_of_classtype) << std::endl;
			std::cout << "\t\texistance probability: " << object.existance_probability << std::endl;
			std::cout << "\t\tdimensions: " << "width " << object.dim_and_occlusion.Dim.width << ", length " << object.dim_and_occlusion.Dim.length
				<< ", height " << object.dim_and_occlusion.Dim.height << std::endl;
			std::cout << "\t\tposition: " << "x " << object.position.x << ", y " << object.position.y << ", z " << object.position.z << std::endl;
		}
	}
}


// void CommonUtils::HandleReflection0Data(invz::FrameDataUserBuffer& ref_buffer)
// {
// 	auto measurements = reinterpret_cast<invz::INVZ2MeasurementXYZType*>(ref_buffer.dataBuffer);
// 	for (uint32_t i = 0; i < ref_buffer.dataAttrs.length; ++i)
// 	{
// 		//blooming pixels can be detected by comparin gto invz::PixelValidity::PIXEL_BLOOMING
// 		if (measurements[i].validity == invz::PixelValidity::PIXEL_VALIDITY_VALID)
// 		{
// 			//print pixel data
// 			int distance = measurements[i].distance;
// 			int reflectivity = measurements[i].reflectivity;

// 			std::cout << "\tPixel " << i << " measurement data:" << std::endl;
// 			std::cout << "\t\t" << "distance: " << distance << std::endl;
// 			std::cout << "\t\t" << "reflectivity: " << reflectivity << std::endl;
// 			//break;
// 		}
// 	}
// }

#include "CommonUtils.h"
#include <iostream>
#include <fstream>
#include <chrono>
#include <iomanip> // For time formatting
#include <sstream> // For string stream
#include <sys/stat.h> // For mkdir function

void CommonUtils::HandleReflection0Data(std::string directoryName, int frameNumber, invz::FrameDataUserBuffer& ref_buffer)
{
    auto measurements = reinterpret_cast<invz::INVZ2MeasurementXYZType*>(ref_buffer.dataBuffer);
    // Start timing
    auto start_time = std::chrono::high_resolution_clock::now();

    // Generate a timestamp-based directory for the recording
    auto now = std::chrono::system_clock::now();
    auto now_time_t = std::chrono::system_clock::to_time_t(now);
    auto now_us = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()) % 1000000;
    std::tm* localTime = std::localtime(&now_time_t);

    // Create a formatted string for the timestamp
    std::stringstream timestampStream;
    timestampStream << std::put_time(localTime, "%Y-%m-%d_%H:%M:%S") << '.' << std::setfill('0') << std::setw(6) << now_us.count();

    // Create and open the CSV file in the new directory
    std::string file_name = directoryName + "/Frame_" + std::to_string(frameNumber) + ".csv";
    std::ofstream csvFile(file_name);
    
    // Check if file is opened successfully
    if (!csvFile.is_open()) {
        std::cerr << "Failed to open file for writing: " << file_name << std::endl;
        return;
    }

    // Write CSV header
    csvFile << "Timestamp,Reflectivity,X,Y,Z\n";

    // Write the timestamp and data
    for (uint32_t i = 0; i < ref_buffer.dataAttrs.length; ++i)
    {
        if (measurements[i].validity == invz::PixelValidity::PIXEL_VALIDITY_VALID)
        {
            int distance = measurements[i].distance;
            int reflectivity = measurements[i].reflectivity;

            if (distance < 3000) { // Filter out pixels with distance < 2000
                csvFile << timestampStream.str() << ',' // Use timestamp as part of the data
                        << reflectivity << ',' 
                        << measurements[i].x << ',' 
                        << measurements[i].y << ',' 
                        << measurements[i].z << '\n';
            }
        }
    }

    // End timing
    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end_time - start_time;

    // Close the CSV file
    csvFile.close();

    // Print time taken to create the CSV file
    std::cout << "CSV creation time: " << elapsed.count() << " seconds" << std::endl;
}
