// OMDeviceExample.cpp

#include "interface/DeviceApi.h"
#include <string>
#include "../common/CommonUtils.h"
#include "../common/ExampleTypes.h"
#include <chrono>
#include <thread>
#include <iostream>
#include <sstream>
#include <fstream>
#include <iomanip>
#include <ctime>

void WriteFrameDataToCSV(const invz::FrameDataUserBuffer& buffer, int frame_count) {
    std::ofstream file;
    std::ostringstream file_name;
    file_name << "Frame_" << frame_count << ".csv";
    file.open(file_name.str());

    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    file << "Frame Count: " << frame_count << "\n";
    file << "Timestamp: " << std::put_time(&tm, "%Y-%m-%d %H:%M:%S") << "\n";
    file << "Reflectivity, x, y, z\n";

    for (const auto& point : buffer.data) {
        if (point.distance < 2000) {
            file << point.reflectivity << ", "
                 << point.x << ", "
                 << point.y << ", "
                 << point.z << "\n";
        }
    }

    file.close();
}

OMDeviceExample::OMDeviceExample(std::string config_path) {
    m_device.reset(invz::DeviceInit(config_path));
}

OMDeviceExample::~OMDeviceExample() {
    if (m_device)
        invz::DeviceClose(m_device.release());
}

void OMDeviceExample::initUserBuffers(size_t buffersCount, std::vector<invz::FrameDataUserBuffer>& userBuffers, std::vector<invz::FrameDataAttributes>& attributes) {
    for (size_t i = 0; i < buffersCount; i++) {
        if (attributes[i].known_type == invz::GRAB_TYPE_MEASURMENTS_REFLECTION0) {
            invz::FrameDataUserBuffer&& userBuffer(attributes[i]);
            userBuffers.push_back(userBuffer);
            m_device->ActivateBuffer(attributes[i], true);
        }
    }
}

void OMDeviceExample::frameCallback(uint32_t* id) {
    uint32_t frameNumber;
    uint64_t timeStamp;

    auto result = m_device->GrabFrame(m_userBuffers.data(), (uint32_t)m_userBuffers.size(), frameNumber, timeStamp, *id);

    if (result.error_code == invz::ERROR_CODE_OK) {
        if (m_userBuffers[static_cast<size_t>(EOMDataType::Ref0)].status == invz::USER_BUFFER_FULL) {
            std::cout << "Frame number: " << frameNumber << std::endl;
            WriteFrameDataToCSV(m_userBuffers[static_cast<size_t>(EOMDataType::Ref0)], frameNumber);
        }
    }
}

void OMDeviceExample::ReadFrames() {
    std::vector<invz::FrameDataAttributes> attributes;
    attributes.resize(DemoConfig::max_num_of_buffers);
    size_t buffersCount = DemoConfig::max_num_of_buffers;

    invz::Result result;
    result.error_code = invz::ERROR_CODE_GENERAL;

    std::cout << "Connecting to device. Waiting for data attributes ..." << std::endl;

    int count = 0;
    while (result.error_code != invz::ERROR_CODE_OK) {
        if (count >= device_timeout_count) {
            std::cout << "Failed to connect!" << std::endl;
            return;
        }
        result = m_device->GetFrameDataAttributes(attributes.data(), buffersCount, true);
        ++count;
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    initUserBuffers(buffersCount, m_userBuffers, attributes);

    auto callback = std::bind(&OMDeviceExample::frameCallback, this, std::placeholders::_1);
    m_device->RegisterFrameCallback(callback);

    std::cout << "Start Grabbing ... Press any key + Enter to stop" << std::endl;
    char user;
    std::cin >> user;

    m_device->UnregisterFrameCallback();
}

int main() {
    std::string config_files_path = "../../lidar_configuration_files";
    std::string device_config_file = "om_config.json";
    std::unique_ptr<OMDeviceExample> m_OMDeviceExample = std::make_unique<OMDeviceExample>(config_files_path + "/" + device_config_file);

    bool stop = false;
    while (!stop) {
        std::cout << "\n Please select the required demo: " << std::endl;
        std::cout << "\t1 - Read frames from OM" << std::endl;
        std::cout << "\tq - Quit" << std::endl;

        char user;
        std::cin >> user;
        switch (user) {
            case '1':
                m_OMDeviceExample->ReadFrames();
                break;
            case 'q':
                stop = true;
                break;
            default:
                std::cout << "Unsupported! Try again" << std::endl;
                break;
        }
    }
    return 0;
}
