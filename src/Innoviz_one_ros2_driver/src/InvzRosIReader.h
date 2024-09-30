///////////////////////////////////////////////////////////
//  InvzRosIReader.h
//  Implementation of the Class InvzRosIReader
//  Created on:      06-Mar-2022 12:19:45 PM
//  Original author: tal.levy
///////////////////////////////////////////////////////////

#if !defined(EA_924D7742_7FB5_472f_A9A3_43EAB404158F__INCLUDED_)
#define EA_924D7742_7FB5_472f_A9A3_43EAB404158F__INCLUDED_

// std
#include <memory>
#include <string>
#include <vector>
#include <set>
#include <map>
#include <functional>
#include <boost/optional.hpp>

// ROS
// #include "ros/ros.h"
#include "rclcpp/rclcpp.hpp"
//#include "pcl_ros/point_cloud.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_types.h"
// Innoviz
#include "../innoviz_api/include/interface/ReaderInterface.h"

// project
#include <tuple>

// pcl timestamp option
enum PclTimestampOption { sampled, received, published };

/**
 * Wraps an IReader for the specific use of Innoviz Ros
 */
class InvzRosIReader
{
public:
	// class ctor and dtor
	InvzRosIReader(std::unique_ptr<invz::IReader, std::function<void(invz::IReader*)>> iReader, unsigned int getAttributesTimeoutMilli, bool activateBuffers,rclcpp::Node& node,rclcpp::Logger& logger);
	virtual ~InvzRosIReader() = default;

	// grab frame using innoviz api into m_buffers
	bool GrabFrame(uint32_t& frameNumber, uint64_t& timestamp, uint32_t frameIndex);

	// published m_buffers to relevant publishers (only those intended for publishing)
	void PublishFrame(bool checkBufferFull);

protected:
	// current node handle
	rclcpp::Node& m_rosNode;
	rclcpp::Logger& m_logger;
	// buffers to which data is read
	std::vector<invz::FrameDataUserBuffer> m_buffers;

	// map that holds buffer ptrs (for convinience) and publishers
    //TODO: rclcpp::Publisher missing type - rclcpp::Publisher<messageT>
	std::map<invz::GrabType, std::pair<invz::FrameDataUserBuffer*, std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>>>> m_grabTypeBufferPublisherMap;


	// the timestamp received in the last GrabFrame call
	uint64_t m_lastGrabFrameTimestamp = 0;
	
	// false positive alarm
	float m_fpa = 0.f;

	// configures what to put in published pcl message timestamp
	PclTimestampOption m_pclTimestampOption = PclTimestampOption::published;

	// published frame id
	std::string m_frameId;

	// innoviz api handle (either device or recording)
	std::unique_ptr<invz::IReader, std::function<void(invz::IReader*)>> m_iReader;

private:
	// publish a single buffer according to its type
	void PublishBuffer(const invz::FrameDataUserBuffer& buffer, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>& publisher, uint64_t timestamp);
	
	// get timestamp for pcl message header according to m_pclTimestampOption
	uint64_t getTimestamp();
    template<typename PointType> void PublishPc(const PointType* pointArray, size_t pointArraySize, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>& publisher, float fpa, const std::string& frameId, uint64_t timestamp);
	// constant - get buffers necessary for this node
	static std::set<invz::GrabType> getRequestedGrabTypes();

};

#endif // !defined(EA_924D7742_7FB5_472f_A9A3_43EAB404158F__INCLUDED_)
