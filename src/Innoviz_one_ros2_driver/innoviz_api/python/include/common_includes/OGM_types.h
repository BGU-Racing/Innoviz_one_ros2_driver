#ifndef __OGM_TYPES_H__
#define __OGM_TYPES_H__

/**
* @file OGM_types.h
*/

#define K_NUM_REF_TARGETS (1)
#define K_NUM_SEGMENTS (1)
#define NUMBER_OF_PIXELS 192000
#define MAX_NUM_DETECTION_RANGES 1U
#define MAX_NUM_CLASSES 3
#define NUM_LASER_SENSOR 1
#define MAX_NUM_OBJECTS 100U
#define MAX_NUM_GEOMETRY_CONTOURS 1000
#define MAX_NUM_GEOMETRY_VERTICES 5000
#define MAX_GEOMETRY_HISTORY 16



namespace invz
{
#pragma pack(push,1)

	inline void endianess_switch(void* data, size_t len) {
		char* ptr = (char*)data;

		for (size_t i = 0; i < len / 2; i++) {
			// https://www.geeksforgeeks.org/swap-two-numbers-without-using-temporary-variable/
			// swap using xor without 3rd variable
			ptr[i] = ptr[i] ^ ptr[len - 1 - i];        // x = x ^ y, x now becomes 15 (1111) 
			ptr[len - 1 - i] = ptr[i] ^ ptr[len - 1 - i];  // y = x ^ y, // y becomes 10 (1010) 
			ptr[i] = ptr[i] ^ ptr[len - 1 - i]; 	   // x = x ^ y, // x becomes 5 (0101) 
		}

	}
	struct InterfaceVersionId
	{
		uint8_t major;
		uint8_t minor;
		uint8_t patch;

		/* functions */
		void handle_endianess();
	};

	/*
	* @description handle fields endianes, converts all fields to little endian.
	*/
	inline void InterfaceVersionId::handle_endianess() {
		endianess_switch(&major, sizeof(InterfaceVersionId::major));
		endianess_switch(&minor, sizeof(InterfaceVersionId::minor));
		endianess_switch(&patch, sizeof(InterfaceVersionId::patch));
	}


	enum DataQualifier : uint8_t
	{
		k_normal,
		k_not_avaliable,
		k_reduce_in_coverage,
		k_reduce_in_performance,
		k_reduce_in_coverage_and_performance,
		k_testMode,
		k_invalid
	};


	enum VehicleCoordinateSystem : uint8_t
	{
		k_rear_axle,
		k_road_level,
		k_front_axle
	};

	
	struct SensorOriginPoint
	{
		float x;
		float y;
		float z;

		/* functions */
		void handle_endianess();
	};

	/*
	* @description handle fields endianes, converts all fields to little endian.
	*/
	inline void SensorOriginPoint::handle_endianess() {
		endianess_switch(&x, sizeof(SensorOriginPoint::x));
		endianess_switch(&y, sizeof(SensorOriginPoint::y));
		endianess_switch(&z, sizeof(SensorOriginPoint::z));
	}

	
	struct SensorOrientation {
		float data[9];

		/* functions */
		void handle_endianess();
	};

	/*
	* @description handle fields endianes, converts all fields to little endian.
	*/
	inline void SensorOrientation::handle_endianess() {
		for (int i = 0; i < 9; i++)
			endianess_switch(&data[i], sizeof(*SensorOrientation::data));
	}


	struct SensorPose
	{
		SensorOriginPoint origin; // 12B
		SensorOrientation orientation; // 32B

		/* functions */
		void handle_endianess();
	};

	/*
	* @description handle fields endianes, converts all fields to little endian.
	*/
	inline void SensorPose::handle_endianess() {
		origin.handle_endianess();
		orientation.handle_endianess();
	}

	
	enum AsilLevel : uint8_t
	{
		k_QM,
		k_ASIL_A,
		k_ASIL_B,
		k_ASIL_C,
		k_ASIL_D
	};

	
	struct SegmentAzimuth
	{
		float begin; // rad -60
		float end; // rad 60

		/* functions */
		void handle_endianess();
	};

	/*
	* @description handle fields endianes, converts all fields to little endian.
	*/
	inline void SegmentAzimuth::handle_endianess() {
		endianess_switch(&begin, sizeof(SegmentAzimuth::begin));
		endianess_switch(&end, sizeof(SegmentAzimuth::end));
	}

	
	struct SegmentElevation
	{
		float begin; // rad -7.5
		float end; // rad +7.5

		/* functions */
		void handle_endianess();
	};

	/*
	* @description handle fields endianes, converts all fields to little endian.
	*/
	inline void SegmentElevation::handle_endianess() {
		endianess_switch(&begin, sizeof(SegmentElevation::begin));
		endianess_switch(&end, sizeof(SegmentElevation::end));
	}

	
	enum BlockageStatus : uint8_t
	{
		k_FullBlockage,
		k_PartialBlockageHighImpact,
		k_PartialBlockageMediumImpact,
		k_PartialBlockageLowImpact,
		Defect,
		None
	};

	
	struct DetectionRangeRadialDistance
	{
		float begin;//1.2
		float end;//250

		/* functions */
		void handle_endianess();
	};

	/*
	* @description handle fields endianes, converts all fields to little endian.
	*/
	inline void DetectionRangeRadialDistance::handle_endianess() {
		endianess_switch(&begin, sizeof(DetectionRangeRadialDistance::begin));
		endianess_switch(&end, sizeof(DetectionRangeRadialDistance::end));
	}


	struct ReferenceTargetRecognitionCapability
	{
		float reference_reflectivity;
		DetectionRangeRadialDistance range;

		/* functions */
		void handle_endianess();
	};

	/*
	* @description handle fields endianes, converts all fields to little endian.
	*/
	inline void ReferenceTargetRecognitionCapability::handle_endianess() {
		endianess_switch(&reference_reflectivity, sizeof(ReferenceTargetRecognitionCapability::reference_reflectivity));
		range.handle_endianess();
	}


	struct FieldOfViewSegment
	{
		SegmentAzimuth azimuth;
		SegmentElevation elevation;
		BlockageStatus blockage;
		ReferenceTargetRecognitionCapability target_recognition_capability;

		/* functions */
		void handle_endianess();
	};

	/*
	* @description handle fields endianes, converts all fields to little endian.
	*/
	inline void FieldOfViewSegment::handle_endianess() {
		azimuth.handle_endianess();
		elevation.handle_endianess();
		endianess_switch(&blockage, sizeof(FieldOfViewSegment::blockage));
		target_recognition_capability.handle_endianess();
	}


	struct FieldOfViewSegments
	{
		FieldOfViewSegment segment[K_NUM_SEGMENTS];

		/* functions */
		void handle_endianess();
	};

	/*
	* @description handle fields endianes, converts all fields to little endian.
	*/
	inline void FieldOfViewSegments::handle_endianess() {
		for (int i = 0; i < K_NUM_SEGMENTS; i++)
			segment[i].handle_endianess();
	}


	struct LidarSensorDetectionsHeader
	{
		InterfaceVersionId version_id;
		uint8_t num_sensors;
		uint8_t sensor_id;
		uint64_t timestamp_measurement; // OM timestamp
		uint32_t cycle_counter; // frame_id
		DataQualifier qualifier; // k_normal || k_not_availiable  from the EDQ
		VehicleCoordinateSystem vehicle_coordinate_system; // k_road_level
		SensorPose sensor_pose;
		AsilLevel asil;
		FieldOfViewSegments sections[K_NUM_REF_TARGETS];

		/* functions */
		void handle_endianess();
	};

	/*
	* @description handle fields endianes, converts all fields to little endian.
	*/
	inline void LidarSensorDetectionsHeader::handle_endianess() {
		version_id.handle_endianess();
		endianess_switch(&num_sensors, sizeof(LidarSensorDetectionsHeader::num_sensors));
		endianess_switch(&sensor_id, sizeof(LidarSensorDetectionsHeader::sensor_id));
		endianess_switch(&timestamp_measurement, sizeof(LidarSensorDetectionsHeader::timestamp_measurement));
		endianess_switch(&cycle_counter, sizeof(LidarSensorDetectionsHeader::cycle_counter));
		endianess_switch(&qualifier, sizeof(LidarSensorDetectionsHeader::qualifier));
		endianess_switch(&vehicle_coordinate_system, sizeof(LidarSensorDetectionsHeader::vehicle_coordinate_system));
		sensor_pose.handle_endianess();
		endianess_switch(&asil, sizeof(LidarSensorDetectionsHeader::asil));
		for (int i = 0; i < K_NUM_SEGMENTS; i++)
			sections[i].handle_endianess();
	}


	struct Position
	{
		float radial_distance;
		float azimuth;
		float elevation;

		/* functions */
		void handle_endianess();
	};

	/*
	* @description handle fields endianes, converts all fields to little endian.
	*/
	inline void Position::handle_endianess() {
		endianess_switch(&radial_distance, sizeof(Position::radial_distance));
		endianess_switch(&azimuth, sizeof(Position::azimuth));
		endianess_switch(&elevation, sizeof(Position::elevation));
	}


	struct LidarDetectionEntity
	{
		uint8_t existence_probability[NUMBER_OF_PIXELS];
		float reflectivity[NUMBER_OF_PIXELS];
		Position position[NUMBER_OF_PIXELS];

		/* functions */
		void handle_endianess();
	};

	/*
	* @description handle fields endianes, converts all fields to little endian.
	*/
	inline void LidarDetectionEntity::handle_endianess() {
		for (int i = 0; i < NUMBER_OF_PIXELS; i++)
			endianess_switch(&existence_probability[i], sizeof(*LidarDetectionEntity::existence_probability));
		for (int i = 0; i < NUMBER_OF_PIXELS; i++)
			endianess_switch(&reflectivity[i], sizeof(*LidarDetectionEntity::reflectivity));
		for (int i = 0; i < NUMBER_OF_PIXELS; i++)
			position[i].handle_endianess();
	}


	enum ScanDataStartPosition : uint8_t
	{
		k_top_left,
		k_top_right,
		k_bottom_left,
		k_bottom_right,
	};


	enum ScanDataDirection : uint8_t
	{
		k_column_first,
		k_row_first
	};


	struct SensorSpecs
	{
		uint8_t number_of_returns;
		uint8_t ordered; /* TRUE */
		ScanDataStartPosition scan_data_start_position;
		ScanDataDirection scan_data_direction;

		/* functions */
		void handle_endianess();
	};

	/*
	* @description handle fields endianes, converts all fields to little endian.
	*/
	inline void SensorSpecs::handle_endianess() {
		endianess_switch(&number_of_returns, sizeof(SensorSpecs::number_of_returns));
		endianess_switch(&ordered, sizeof(SensorSpecs::ordered));
		endianess_switch(&scan_data_start_position, sizeof(SensorSpecs::scan_data_start_position));
		endianess_switch(&scan_data_direction, sizeof(SensorSpecs::scan_data_direction));
	}


	struct OGM_LidarDetectionInterface
	{
		uint8_t num_fov_segments;
		uint8_t num_ref_targets;
		LidarSensorDetectionsHeader header;
		LidarDetectionEntity detections;
		SensorSpecs sensor_specs;

		/* functions */
		void handle_endianess();
	};

	/*
	* @description handle fields endianes, converts all fields to little endian.
	*/
	inline void OGM_LidarDetectionInterface::handle_endianess() {
		endianess_switch(&num_fov_segments, sizeof(OGM_LidarDetectionInterface::num_fov_segments));
		endianess_switch(&num_ref_targets, sizeof(OGM_LidarDetectionInterface::num_ref_targets));
		header.handle_endianess();
		detections.handle_endianess();
		sensor_specs.handle_endianess();
	}


	struct VersionInfo
	{
		uint8_t major;
		uint8_t minor;
		uint8_t patch;
		uint8_t reserved2;

		/* functions */
		void handle_endianess();
	};

	/*
	* @description handle fields endianes, converts all fields to little endian.
	*/
	inline void VersionInfo::handle_endianess() {

		endianess_switch(&major, sizeof(VersionInfo::major));
		endianess_switch(&minor, sizeof(VersionInfo::minor));
		endianess_switch(&patch, sizeof(VersionInfo::patch));
		endianess_switch(&reserved2, sizeof(VersionInfo::reserved2));
	}


	struct TimeStamp
	{
		uint32_t high;  //high part of the time stamp corresponds to the seconds part of the time
		uint32_t low;   //low part of the time stamp corresponds to the nanoseconds part of the time

		/* functions */
		void handle_endianess();
	};

	/*
	* @description handle fields endianess, converts all fields to little endian.
	*/
	inline void TimeStamp::handle_endianess() {
		endianess_switch(&high, sizeof(TimeStamp::high));
		endianess_switch(&low, sizeof(TimeStamp::low));
	}


	struct Header
	{
		VersionInfo version;
		uint8_t sensorID;
		AsilLevel asil;
		TimeStamp timeStamp;
		uint32_t  frameID;

		/* functions */
		void handle_endianess();
	};

	/*
	* @description handle fields endianes, converts all fields to little endian.
	*/
	inline void Header::handle_endianess() {
		timeStamp.handle_endianess();
		endianess_switch(&frameID, sizeof(Header::frameID));
	}


	struct SensorPoseObjects
	{
		float sensorOriginX;      
		float sensorOriginXStdDev;
		float sensorOriginY;      
		float sensorOriginYStdDev;
		float sensorOriginZ;      
		float sensorOriginZStdDev;
		float sensorYaw;          
		float sensorYawStdDev;    
		float sensorPitch;        
		float sensorPitchStdDev;  
		float sensorRoll;         
		float sensorRollStdDev; 

		/* functions */
		void handle_endianess();
	};

	/*
	* @description handle fields endianess, converts all fields to little endian.
	*/
	inline void SensorPoseObjects::handle_endianess() {
		endianess_switch(&sensorOriginX, sizeof(SensorPoseObjects::sensorOriginX));
		endianess_switch(&sensorOriginXStdDev, sizeof(SensorPoseObjects::sensorOriginXStdDev));
		endianess_switch(&sensorOriginY, sizeof(SensorPoseObjects::sensorOriginY));
		endianess_switch(&sensorOriginYStdDev, sizeof(SensorPoseObjects::sensorOriginYStdDev));
		endianess_switch(&sensorOriginZ, sizeof(SensorPoseObjects::sensorOriginZ));
		endianess_switch(&sensorOriginZStdDev, sizeof(SensorPoseObjects::sensorOriginZStdDev));
		endianess_switch(&sensorYaw, sizeof(SensorPoseObjects::sensorYaw));
		endianess_switch(&sensorYawStdDev, sizeof(SensorPoseObjects::sensorYawStdDev));
		endianess_switch(&sensorPitch, sizeof(SensorPoseObjects::sensorPitch));
		endianess_switch(&sensorPitchStdDev, sizeof(SensorPoseObjects::sensorPitchStdDev));
		endianess_switch(&sensorRoll, sizeof(SensorPoseObjects::sensorRoll));
		endianess_switch(&sensorRollStdDev, sizeof(SensorPoseObjects::sensorRollStdDev));
	}


	enum OGMCalibrationStatus : uint8_t
	{
		CALIB_UNKNOWN = 0,
		CALIB_NO_INITIAL_CALIBRATION = 1,
		CALIB_NO_CLAMP15_CONFIRMATION = 2,
		CALIB_OK_WITHIN_RANGE = 3,
		CALIB_MISALLIGNMENT_VALIDATION = 4,
		CALIB_NOT_OK_OUT_OF_RANGE = 5
	};


	enum SensorStatusFlag : uint8_t
	{
		SENSOR_STATUS_INIT = 0,
		SENSOR_STATUS_NOT_SET = 1,
		SENSOR_STATUS_SET = 2,
		__LAST_SENSOR_STATUS__
	};


	enum Classes : uint8_t
	{
		CLASS_UNKNOWN = 0,
		CLASS_OVERRIDE = 1,
		CLASS_UNDERRIDE = 2,
		CLASS_FREESPACE = 3,
		CLASS_DYNAMICOBJECT = 10,
		CLASS_SMALLOBJECT = 11,
		CLASS_BIGOBJECT = 12,
		CLASS_ANIMAL = 13,
		CLASS_ANIMALSMALL = 14,
		CLASS_CLUSTER = 15,
		CLASS_TRAILER = 16,
		CLASS_AMBULANCE = 17,
		CLASS_FIREFIGHTING = 18,
		CLASS_POLICE = 19,
		CLASS_AGRICULTURAL = 20,
		CLASS_TWOWHEELER = 30,
		CLASS_BICYCLE = 31,
		CLASS_MOTORBIKE = 32,
		CLASS_FOURPLUSWHEELER = 40,
		CLASS_PASSENGERCAR = 41,
		CLASS_VAN = 42,
		CLASS_TRUCK = 43,
		CLASS_BUS = 44,
		CLASS_VULNERABLEROADUSER = 50,
		CLASS_PEDESTRIAN = 51,
		CLASS_PEDESTRIANGROUP = 52,
		CLASS_STATICOBJECT = 100,
		CLASS_ROADBOUNDARY = 110,
		CLASS_GUARDRAIL = 111,
		CLASS_CURB = 112,
		CLASS_POLE = 113,
		CLASS_FIELDOFVIEWBOUNDARY = 120,

		CLASS_ROADMARKING = 150,
		CLASS_ROADMARKING_START = 151,
		CLASS_ROADMARKING_END = 152,

		CLASS_INIT = 255
	};


	struct DetectionRange
	{
		float radiusStart;
		float radiusEnd;
		float azimuthStart;
		float azimuthEnd;
		float elevationStart;
		float elevationEnd;
		float confidence;
		Classes objectType;

		/* functions */
		void handle_endianess();
	};

	/*
	* @description handle fields endianess, converts all fields to little endian.
	*/
	inline void DetectionRange::handle_endianess() {
		endianess_switch(&radiusStart, sizeof(DetectionRange::radiusStart));
		endianess_switch(&radiusEnd, sizeof(DetectionRange::radiusEnd));
		endianess_switch(&azimuthStart, sizeof(DetectionRange::azimuthStart));
		endianess_switch(&azimuthEnd, sizeof(DetectionRange::azimuthEnd));
		endianess_switch(&elevationStart, sizeof(DetectionRange::elevationStart));
		endianess_switch(&elevationEnd, sizeof(DetectionRange::elevationEnd));
		endianess_switch(&confidence, sizeof(DetectionRange::confidence));
		endianess_switch(&objectType, sizeof(DetectionRange::objectType));
	}


	struct DetectionRangeList
	{
		uint8_t numDetectionRanges;
		DetectionRange detectionRangeList[MAX_NUM_DETECTION_RANGES];

		/* functions */
		void handle_endianess();
	};

	/*
	* @description handle fields endianess, converts all fields to little endian.
	*/
	inline void DetectionRangeList::handle_endianess() {
		for (int i = 0; i < sizeof(detectionRangeList) / sizeof(*detectionRangeList); i++)
			detectionRangeList[i].handle_endianess();
	}


	struct SensorStatus
	{
		SensorPoseObjects sensorOrigin;
		OGMCalibrationStatus calibration;
		SensorStatusFlag unavailable;
		SensorStatusFlag failure;
		DetectionRangeList detectionRanges;

		/* functions */
		void handle_endianess();
	};

	/*
	* @description handle fields endianess, converts all fields to little endian.
	*/
	inline void SensorStatus::handle_endianess() {
		sensorOrigin.handle_endianess();
		detectionRanges.handle_endianess();
	}


	struct Classification
	{
		uint8_t classType;
		float probability;

		/* functions */
		void handle_endianess();
	};

	/*
	* @description handle fields endianess, converts all fields to little endian.
	*/
	inline void Classification::handle_endianess() {
		endianess_switch(&classType, sizeof(Classification::classType));
		endianess_switch(&probability, sizeof(Classification::probability));
	}


	enum Ambiguity : uint8_t
	{
		AMBIGUITY_UNDEFINED = 0,
		AMBIGUITY_NONAMBIGUOUS = 1,
		AMBIGUITY_AMBIGUOUS_180 = 2,
		AMBIGUITY_AMBIGUOUS_90 = 3,
		__LAST_REFPOINT_AMBIGUITY__
	};


	enum MeasStatus : uint8_t
	{
		MS_UNDEFINED = 0,
		MS_PREDICTION = 1,
		MS_MEASUREMENT = 2,
		__LAST_MEAS_STATUS_
	};


	struct SensorContribution
	{
		TimeStamp timeOfLastMeasuredBySensor[NUM_LASER_SENSOR];
		TimeStamp timeOfFirstMeasuredBySensor[NUM_LASER_SENSOR];
		uint16_t  measurementCount[NUM_LASER_SENSOR];

		/* functions */
		void handle_endianess();
	};

	/*
	* @description handle fields endianess, converts all fields to little endian.
	*/
	inline void SensorContribution::handle_endianess() {
		for (int i = 0; i < NUM_LASER_SENSOR; i++)
			timeOfLastMeasuredBySensor[i].handle_endianess();
		for (int i = 0; i < NUM_LASER_SENSOR; i++)
			timeOfFirstMeasuredBySensor[i].handle_endianess();
		for (int i = 0; i < NUM_LASER_SENSOR; i++)
			endianess_switch(&measurementCount[i], sizeof(*SensorContribution::measurementCount));
	}


	enum DimensionMeasuredStatus : uint8_t
	{
		DMS_UNKNOWN = 0,
		DMS_FULLY_MEASURED = 1,
		DMS_PARTIALLY_MEASURED = 2,
		__LAST_DIMENSION_MEASURED_STATUS__

	};


	struct Edge
	{
		DimensionMeasuredStatus  startPointMeasuredStatus;
		DimensionMeasuredStatus  endPointMeasuredStatus;
		float measurement;
		float stdDev;

		/* functions */
		void handle_endianess();
	};

	/*
	* @description handle fields endianess, converts all fields to little endian.
	*/
	inline void Edge::handle_endianess() {
		endianess_switch(&measurement, sizeof(Edge::measurement));
		endianess_switch(&stdDev, sizeof(Edge::stdDev));

	}


	enum LongitudinalSide : uint8_t
	{
		LOS_UNKNOWN = 0,
		LOS_LEFT = 1,
		LOS_RIGHT = 2,
		LOS_RADIUS = 3,
		__LAST_LONGITUDINAL_SIDE__
	};

	
	enum LateralSide : uint8_t
	{
		LAS_UNKNOWN = 0,
		LAS_FRONT = 1,
		LAS_REAR = 2,
		LAS_RADIUS = 3,
		__LAST_LATERAL_SIDE__
	};


	struct Object
	{
		uint32_t objectID;
		uint32_t timeOffset;
		uint8_t historyFlag;
		uint8_t existenceProbability;
		Classification classification[MAX_NUM_CLASSES];
		Ambiguity ambiguity;
		MeasStatus measStatus;
		SensorContribution sensorContributions;
		float positionX;
		float positionXStdDev;
		float positionY;
		float positionYStdDev;
		float positionZ;
		float positionZStdDev;
		float heightAboveGround;
		float heightAboveGroundStdDev;
		float yawAngle;
		float yawAngleStdDev;
		float velocityX;
		float velocityXStdDev;
		float velocityY;
		float velocityYStdDev;
		float velocityZ;
		float velocityZStdDev;
		Edge length;
		Edge width;
		Edge height;
		LongitudinalSide lengthSide;
		LateralSide widthSide;
		float correlationPositionXY;
		float correlationPositionYZ;
		float correlationPositionXZ;
		float correlationPositionXYawAngle;
		float correlationPositionYYawAngle;
		float correlationVelocityXY;
		float refPointRatioLength;
		float refPointRatioWidth;
		float refPointRatioHeight;

		/* functions */
		void handle_endianess();
	};

	/*
	* @description handle fields endianess, converts all fields to little endian.
	*/
	inline void Object::handle_endianess() {
		endianess_switch(&objectID, sizeof(Object::objectID));
		endianess_switch(&timeOffset, sizeof(Object::timeOffset));
		for (int i = 0; i < MAX_NUM_CLASSES; i++)
			classification[i].handle_endianess();
		sensorContributions.handle_endianess();
		endianess_switch(&positionX, sizeof(Object::positionX));
		endianess_switch(&positionXStdDev, sizeof(Object::positionXStdDev));
		endianess_switch(&positionY, sizeof(Object::positionY));
		endianess_switch(&positionYStdDev, sizeof(Object::positionYStdDev));
		endianess_switch(&positionZ, sizeof(Object::positionZ));
		endianess_switch(&positionZStdDev, sizeof(Object::positionZStdDev));
		endianess_switch(&heightAboveGround, sizeof(Object::heightAboveGround));
		endianess_switch(&heightAboveGroundStdDev, sizeof(Object::heightAboveGroundStdDev));
		endianess_switch(&yawAngle, sizeof(Object::yawAngle));
		endianess_switch(&yawAngleStdDev, sizeof(Object::yawAngleStdDev));
		endianess_switch(&velocityX, sizeof(Object::velocityX));
		endianess_switch(&velocityXStdDev, sizeof(Object::velocityXStdDev));
		endianess_switch(&velocityY, sizeof(Object::velocityY));
		endianess_switch(&velocityYStdDev, sizeof(Object::velocityYStdDev));
		endianess_switch(&velocityZ, sizeof(Object::velocityZ));
		endianess_switch(&velocityZStdDev, sizeof(Object::velocityZStdDev));
		length.handle_endianess();
		width.handle_endianess();
		height.handle_endianess();
		endianess_switch(&correlationPositionXY, sizeof(Object::correlationPositionXY));
		endianess_switch(&correlationPositionYZ, sizeof(Object::correlationPositionYZ));
		endianess_switch(&correlationPositionXZ, sizeof(Object::correlationPositionXZ));
		endianess_switch(&correlationPositionXYawAngle, sizeof(Object::correlationPositionXYawAngle));
		endianess_switch(&correlationPositionYYawAngle, sizeof(Object::correlationPositionYYawAngle));
		endianess_switch(&correlationVelocityXY, sizeof(Object::correlationVelocityXY));
		endianess_switch(&refPointRatioLength, sizeof(Object::refPointRatioLength));
		endianess_switch(&refPointRatioWidth, sizeof(Object::refPointRatioWidth));
		endianess_switch(&refPointRatioHeight, sizeof(Object::refPointRatioHeight));
	}

	
	struct object_list_OGM
	{
		uint32_t numberOfObj;
		Object objectList[MAX_NUM_OBJECTS];
		TimeStamp linkTimestamp;

		/* functions */
		void handle_endianess();
	};

	/*
	* @description handle fields endianess, converts all fields to little endian.
	*/
	inline void object_list_OGM::handle_endianess() {
		endianess_switch(&numberOfObj, sizeof(object_list_OGM::numberOfObj));
		for (int i = 0; i < MAX_NUM_OBJECTS; i++)
			objectList[i].handle_endianess();
		linkTimestamp.handle_endianess();
	}

	
	struct OGM_ObjectList
	{
		Header header;
		SensorStatus sensorStatus;
		object_list_OGM tracked_objects;

		/* functions */
		void handle_endianess();
	};

	/*
	* @description handle fields endianess, converts all fields to little endian.
	*/
	inline void OGM_ObjectList::handle_endianess() {
		header.handle_endianess();
		sensorStatus.handle_endianess();
		tracked_objects.handle_endianess();
	}


	enum ContourType : uint8_t
	{
		CONTOUR_TYPE_POINTS = 0,
		CONTOUR_TYPE_POLYLINE = 1,
		CONTOUR_TYPE_POLYGON = 2, 
	};


	enum GFlags : uint8_t
	{
		GF_INVALID = 0,
		GF_VALID = 1,
	};


	enum LineType : uint8_t
	{
		LINE_TYPE_UNKNOWN = 0,
		LINE_TYPE_SOLID = 1,
		LINE_TYPE_DASHED = 2,
		LINE_TYPE_DOTTED = 3,

		__LAST_LINE_TYPE__
	};

	
	struct GeometryContour
	{
		uint32_t  geometryID;
		uint32_t  history[MAX_GEOMETRY_HISTORY];
		ContourType type;
		GFlags flags;
		LineType lineType;
		uint16_t  startVertexIndex;
		uint16_t  numberOfVertices;
		Classes classification; //!< [1] The most probable class type.

		/* functions */
		void handle_endianess();
	};

	/*
	* @description handle fields endianess, converts all fields to little endian.
	*/
	inline void GeometryContour::handle_endianess() {
		endianess_switch(&geometryID, sizeof(GeometryContour::geometryID));
		for (int i = 0; i < MAX_GEOMETRY_HISTORY; i++)
			endianess_switch(&history[i], sizeof(*GeometryContour::history));
		endianess_switch(&type, sizeof(GeometryContour::type));
		endianess_switch(&flags, sizeof(GeometryContour::flags));
		endianess_switch(&lineType, sizeof(GeometryContour::lineType));
		endianess_switch(&startVertexIndex, sizeof(GeometryContour::startVertexIndex));
		endianess_switch(&numberOfVertices, sizeof(GeometryContour::numberOfVertices));
		endianess_switch(&classification, sizeof(GeometryContour::classification));
	}

	
	struct GeometryVertex
	{
		int16_t positionX;                        
		int16_t positionXStdDev;                  
		int16_t positionY;                        
		int16_t positionYStdDev;                  
		int16_t positionZ;                        
		int16_t positionZStdDev;  

		int16_t heightAboveGround;                
		int16_t heightAboveGroundStdDev; 

		int16_t height;                           
		int16_t heightStdDev;   

		int8_t width;
		int8_t widthStdDev;  

		float correlationXY;                      
		float correlationXZ;                      
		float correlationYZ;        

		uint8_t existenceProbability;             
		uint8_t freespaceProbability;             
		Classes classification;                   
		uint32_t objectID;

		/* functions */
		void handle_endianess();
	};

	/*
	* @description handle fields endianess, converts all fields to little endian.
	*/
	inline void GeometryVertex::handle_endianess() {
		endianess_switch(&positionX, sizeof(GeometryVertex::positionX));
		endianess_switch(&positionXStdDev, sizeof(GeometryVertex::positionXStdDev));
		endianess_switch(&positionY, sizeof(GeometryVertex::positionY));
		endianess_switch(&positionYStdDev, sizeof(GeometryVertex::positionYStdDev));
		endianess_switch(&positionZ, sizeof(GeometryVertex::positionZ));
		endianess_switch(&positionZStdDev, sizeof(GeometryVertex::positionZStdDev));
		endianess_switch(&heightAboveGround, sizeof(GeometryVertex::heightAboveGround));
		endianess_switch(&heightAboveGroundStdDev, sizeof(GeometryVertex::heightAboveGroundStdDev));
		endianess_switch(&height, sizeof(GeometryVertex::height));
		endianess_switch(&heightStdDev, sizeof(GeometryVertex::heightStdDev));
		endianess_switch(&width, sizeof(GeometryVertex::width));
		endianess_switch(&widthStdDev, sizeof(GeometryVertex::widthStdDev));
		endianess_switch(&correlationXY, sizeof(GeometryVertex::correlationXY));
		endianess_switch(&correlationXZ, sizeof(GeometryVertex::correlationXZ));
		endianess_switch(&correlationYZ, sizeof(GeometryVertex::correlationYZ));
		endianess_switch(&existenceProbability, sizeof(GeometryVertex::existenceProbability));
		endianess_switch(&freespaceProbability, sizeof(GeometryVertex::freespaceProbability));
		endianess_switch(&classification, sizeof(GeometryVertex::classification));
		endianess_switch(&objectID, sizeof(GeometryVertex::objectID));
	}


	struct GeometryList
	{
		uint16_t numberOfContours;
		GeometryContour contourList[MAX_NUM_GEOMETRY_CONTOURS];
		GeometryVertex vertexList[MAX_NUM_GEOMETRY_VERTICES];

		/* functions */
		void handle_endianess();
	};

	/*
	* @description handle fields endianess, converts all fields to little endian.
	*/
	inline void GeometryList::handle_endianess() {
		endianess_switch(&numberOfContours, sizeof(GeometryList::numberOfContours));
		for (int i = 0; i < MAX_NUM_GEOMETRY_CONTOURS; i++)
			contourList[i].handle_endianess();
		for (int i = 0; i < MAX_NUM_GEOMETRY_VERTICES; i++)
			vertexList[i].handle_endianess();
	}

	
	struct OGM_GeometryList
	{
		Header header;
		SensorStatus sensorStatus;     
		GeometryList geometryList;

		/* functions */
		void handle_endianess();
	};

	/*
	* @description handle fields endianess, converts all fields to little endian.
	*/
	inline void OGM_GeometryList::handle_endianess() {

		header.handle_endianess();
		sensorStatus.handle_endianess();
		geometryList.handle_endianess();
	}


#pragma pack(pop)
}
#endif /* __OGM_TYPES_H__ */