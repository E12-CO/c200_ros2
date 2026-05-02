// ROS2 driver for FocusRay (Free Optics) C200 series LiDAR
// By TinLethax at Robot Club KMITL (RB26)

#include <chrono>
#include <cmath>
#include <string>
#include <iostream>
#include <thread>
#include <stdexcept>

#include <stdio.h>
#include <stdint.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/ioctl.h> 
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>

// ROS2 library
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#define SERIAL_TIME_MIL	10 // around 80Hz to ping pong fifo the scan data
#define FSM_TIME_MIL	200 // around 5Hz

#define FOCUSRAY_TCP_PORT		2111

#define FOCUSRAY_OPCODE_READ	0
#define FOCUSRAY_OPCODE_WRITE	1
#define FOCUSRAY_OPCODE_METHOD	2

#define FOCUSRAY_OPCODE_TX		0x00
#define FOCUSRAY_OPCODE_RX		0x10

#define FOCUSRAY_OPCODE(X,Y)	(uint8_t)(X | Y)

// Opcode and Command to match the scan return packet
#define FOCUSRAY_SCAN_OPCODE	FOCUSRAY_OPCODE(FOCUSRAY_OPCODE_METHOD, FOCUSRAY_OPCODE_RX)
#define FOCUSRAY_SCAN_COMMAND	0x32

#define RX_BUFFER_PARAM_OFFSET	8

#define DEG_TO_RADS_COSNT		0.0174533f
#define ANGLE_OFFSET			0.785398f	// 45 degree

class c200_if : public rclcpp::Node{
	
	public:
	
	// LiDAR publisher
	rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr 	pubLaserScan;
	// LiDAR message buffer
	sensor_msgs::msg::LaserScan			msgLaser;
	
	// Used in wall timer callback
	rclcpp::TimerBase::SharedPtr	timerFsmLoop;
	
	// Laser IP address
	std::string strLaserIp;
	// Laser socket file descriptor
	int i32LaserSocketFd;
	
	// Laser frame id
	std::string strLaserFrameId;
	
	// Scan topic name
	std::string strScanTopic;
	
	// Tx buffer
	uint8_t u8TcpTxBuffer[50];
	// Rx buffer
	uint8_t u8TcpRxBuffer[545];
	int i32RxByteCount;
	
	// Laser properties
	uint32_t u32TotalScanNumPoints;
	
	uint8_t u8ScanNumber;
	uint16_t u16PacketScanNumPoints;
	uint16_t u16ScanNumPtsAcumu;
	uint16_t u16ScanBufferOffset;
	
	// Laser angle offset (in radiant system)
	float f32MinAngle;
	float f32MaxAngle;
	
	float f32LaserScanRate;
	float f32LaserScanResolution;
	
	// FSM
	uint8_t u8LaserFSM;

	
	c200_if() : Node("c200_node"){
		RCLCPP_INFO(
			this->get_logger(), 
			"Robot Club KMITL : Starting C200 LiDAR node..."
			);
		
		declare_parameter("laser_ip_addr", "192.168.0.13");
		get_parameter("laser_ip_addr", strLaserIp);
		
		declare_parameter("laser_frame_id", "laser_frame");
		get_parameter("laser_frame_id", strLaserFrameId);
		
		declare_parameter("laser_topic", "/scan");
		get_parameter("laser_topic", strScanTopic);
		
		declare_parameter("scan_rate", 25.0f);
		get_parameter("scan_rate", f32LaserScanRate);
		
		declare_parameter("scan_resolution", 0.2222f);
		get_parameter("scan_resolution", f32LaserScanResolution);

		declare_parameter("min_angle", -0.785398f);
		get_parameter("min_angle", f32MinAngle);
		
		declare_parameter("max_angle", 4.45059f);
		get_parameter("max_angle", f32MaxAngle);
		
		// Initialize TCP Socket 
		i32LaserSocketFd = socket(
			AF_INET, 
			SOCK_STREAM | SOCK_NONBLOCK,
			0
		);
		
		if(i32LaserSocketFd < 0){
			RCLCPP_ERROR(
				this->get_logger(),
				"Error %d, can't create socket to connect to lidar",
				errno
			);
			std::raise(SIGTERM);
			return;
		}
		
		struct sockaddr_in laserServer;
		laserServer.sin_family 	= AF_INET;
		laserServer.sin_port 	= htons(FOCUSRAY_TCP_PORT);
		laserServer.sin_addr.s_addr =
			inet_addr(strLaserIp.c_str());
		
		connect(
			i32LaserSocketFd, 
			(struct sockaddr *)&laserServer, 
			sizeof(laserServer)
			);
		
		// Setup basics data in the laser message
		msgLaser.header.frame_id 	= strLaserFrameId;
		
		// Laser Publisher 
		pubLaserScan =
			create_publisher<sensor_msgs::msg::LaserScan>(
				strScanTopic,
				rclcpp::QoS(rclcpp::SensorDataQoS())
			);

		// TCP listener thread
		std::thread tRunner(
			&c200_if::c200_tcpRunner, 
			this
		);
		tRunner.detach();
			
		// C200 FSM loop timer
		timerFsmLoop =
			this->create_wall_timer(
				std::chrono::milliseconds(FSM_TIME_MIL),
				std::bind(
					&c200_if::c200_fsm,
					this
				)
			);
	}
	
	int c200_getRxBytes(){
		// ioctl(
			// i32LaserSocketFd,
			// FIONREAD,
			// &i32RxByteCount
			// );
		
		RCLCPP_DEBUG(
			this->get_logger(), 
			"tcp received : %d bytes", i32RxByteCount
			);
		
		return i32RxByteCount;
	}
	
	bool c200_checkRxEqual(int expected_rx){
		return (c200_getRxBytes() >= expected_rx) ? true : false;
	}
	
	void c200_protocolWrite(
		uint8_t u8OpCode,
		uint8_t u8CmdNumber,
		uint8_t *paramList,
		uint8_t u8ParamLength
		){
		int ret;
		uint16_t u16PacketLength = 0;
		uint8_t u8ChecksumCal = 0;
		
		// Make sure to handle null pointer appropriately
		if(paramList == NULL){
			u8ParamLength = 0;
		}
		
		// Populate header
		*(uint32_t *)&u8TcpTxBuffer[0] = 0x02020202;
		
		// Calculate the length
		u16PacketLength = 4 + 2 + 1 + 1 + 1 + u8ParamLength;
		// Copy the packet lengh in the Big-endian type.
		u8TcpTxBuffer[4] = (u16PacketLength << 8);
		u8TcpTxBuffer[5] = (uint8_t)(u16PacketLength);
			
		// Opcode
		u8TcpTxBuffer[6] = u8OpCode;
		// Command 
		u8TcpTxBuffer[7] = u8CmdNumber;
		
		// Skip parameter check if the lenth is zero
		if(u8ParamLength < 1)
			goto calChksum;
		
		// Copy parameter to buffer (if any)
		for(uint8_t i=0; i < u8ParamLength; i ++){
			u8TcpTxBuffer[8+i] = 
				*(paramList + i);
		}
		
		// Calculate packet checksum
	calChksum:
		for(uint8_t i=0; i < (u16PacketLength - 1); i++){
			u8ChecksumCal += u8TcpTxBuffer[i];
		}
		
		u8TcpTxBuffer[u16PacketLength - 1] = u8ChecksumCal;
		
		// Send it over TCP
		if(
			ret = write(i32LaserSocketFd, u8TcpTxBuffer, u16PacketLength), 
			(ret < 0)
		){
			RCLCPP_ERROR(
				this->get_logger(),
				"Error sending TCP packet with code %d",
				ret
			);
		}
	}
	
	void c200_getDeviceName(){
		c200_protocolWrite(
			FOCUSRAY_OPCODE(FOCUSRAY_OPCODE_READ, FOCUSRAY_OPCODE_TX),
			0x0E,
			NULL, 0
		);
	}
	
	void c200_sensorLogin(){
		uint8_t u8LoginPasswd[5] = {0x03, 0xF4, 0x72, 0x47, 0x44};
		
		c200_protocolWrite(
			FOCUSRAY_OPCODE(FOCUSRAY_OPCODE_METHOD, FOCUSRAY_OPCODE_TX),
			0x01,
			u8LoginPasswd, 5
		);
	}
	
	void c200_stopScan(){
		uint8_t param = 0;
		
		param = 0x00;
		
		c200_protocolWrite(
			FOCUSRAY_OPCODE(FOCUSRAY_OPCODE_METHOD, FOCUSRAY_OPCODE_TX),
			0x31,
			&param, 1
		);
	}
	
	void c200_setTrailing(){
		

	}
	
	void c200_startScan(){
		uint8_t param = 0;
		
		param = 0x01;
		
		c200_protocolWrite(
			FOCUSRAY_OPCODE(FOCUSRAY_OPCODE_METHOD, FOCUSRAY_OPCODE_TX),
			0x31,
			&param, 1
		);
	}
	
	void c200_setScanRateAndResolution(
		float f32ScanRate,
		float f32AngleResolution
		){
		uint8_t u8ScanRateAngle[4] = {0};
		
		*(uint16_t *)&u8ScanRateAngle[0] = htons(
			(uint16_t)(f32ScanRate * 100)
		);
		*(uint16_t *)&u8ScanRateAngle[2] = htons(
			(uint16_t)(f32AngleResolution * 10000)
		);
		
		c200_protocolWrite(
			FOCUSRAY_OPCODE(FOCUSRAY_OPCODE_WRITE, FOCUSRAY_OPCODE_TX),
			0x19,
			u8ScanRateAngle, 4
		);
	}
	
	void c200_getScanRateAndResolution(){
		c200_protocolWrite(
			FOCUSRAY_OPCODE(FOCUSRAY_OPCODE_READ, FOCUSRAY_OPCODE_TX),
			0x1A,
			NULL, 0
		);
	}
	
	void c200_setScanBeginEndAngle(
		float f32BeginAngle, 
		float f32EndAngle)
	{
		uint8_t u8ScanAngle[9] = {0};
		
		u8ScanAngle[0] = 0;// Default Area is 0 
		*(uint32_t *)&u8ScanAngle[1] = htons(
			(int)(f32BeginAngle * 10000)
			);
			
		*(uint32_t *)&u8ScanAngle[5] = htons(
			(int)(f32EndAngle * 10000)
			);	
			
		c200_protocolWrite(
			FOCUSRAY_OPCODE(FOCUSRAY_OPCODE_WRITE, FOCUSRAY_OPCODE_TX),
			0x1B,
			u8ScanAngle, 9
		);
	}
	
	void c200_getScanBeginEndAngle(){
		c200_protocolWrite(
			FOCUSRAY_OPCODE(FOCUSRAY_OPCODE_READ, FOCUSRAY_OPCODE_TX),
			0x1C,
			NULL, 0
		);
	}
	
	// TCP receiver
	void c200_tcpRunner(){
		uint16_t u16PacketLength;
		int ret;
		
		RCLCPP_INFO(
			this->get_logger(),
			"Starting TCP listener thread"
		);
		
		// Sync with header
		while(1){
		lWait:		
			ret = read(i32LaserSocketFd, u8TcpRxBuffer, 4);
			if(ret < 0){
				// Non-EAGAIN error will throw an error
				if(errno != EAGAIN){
					RCLCPP_ERROR(
						this->get_logger(),
						"Error reading header with code %d",
						errno
					);
				}
				goto lWait;
			}
			
			if(
				(*(uint32_t *)&u8TcpRxBuffer[0] != 0x02020202) &&
				(ret != 4)
			){
				RCLCPP_DEBUG(
					this->get_logger(),
					"Header mismatched, skipping..."
				);
				*(uint32_t *)&u8TcpRxBuffer[0] = 0x00000000;
				goto lWait;
			}
			
			if(read(i32LaserSocketFd, &u8TcpRxBuffer[4], 2) < 0){
				RCLCPP_ERROR(
					this->get_logger(),
					"Error reading packet length!"
				);
				goto lWait;
			}
			u16PacketLength = (
					(u8TcpRxBuffer[4] << 8) |
					(u8TcpRxBuffer[5])
				);
				
			i32RxByteCount = u16PacketLength;	
				
			if(u16PacketLength < 9){
				RCLCPP_DEBUG(
					this->get_logger(),
					"packet length is less than minimum, skipping..."
				);
				goto lWait;
			}	
				
			RCLCPP_DEBUG(
				this->get_logger(),
				"Received %d bytes", u16PacketLength
			);	
			
			// Calculate Opcode, command and parameter length 
			u16PacketLength -= 6;
			// Read the remaining packet
			if(read(i32LaserSocketFd, &u8TcpRxBuffer[6], u16PacketLength) < 0){
				RCLCPP_ERROR(
					this->get_logger(),
					"Error reading the rest parameter data"
				);	
				goto lWait;
			}
				
			// Check Op code and Command of the return scan data and process it 
			if(
				(u8TcpRxBuffer[6] == FOCUSRAY_SCAN_OPCODE) &&
				(u8TcpRxBuffer[7] == FOCUSRAY_SCAN_COMMAND)
			){
				RCLCPP_DEBUG(
					this->get_logger(),
					"Received scan packet!"
				);
				c200_publisher();
			}
		}
	}
	
	bool c200_checkIgnoreAngle(uint16_t scan_index){
		float current_angle;
		
		current_angle = (scan_index * msgLaser.angle_increment) - 2.356194f;
		
		if(
		(current_angle < f32MinAngle) || 
		(current_angle > f32MaxAngle)
		)
			return true;
		
		return false;
	}
	
	void c200_publisher(){
		// Parse scan 
		u8ScanNumber = u8TcpRxBuffer[RX_BUFFER_PARAM_OFFSET + 3];

		RCLCPP_DEBUG(
			this->get_logger(),
			"Scan number : %d",
			u8ScanNumber
		);

		// Add timestamp at the first message of the scan
		if(u8ScanNumber == 0){
			msgLaser.header.stamp = this->get_clock()->now();
		}

		u16PacketScanNumPoints = 
			(u8TcpRxBuffer[RX_BUFFER_PARAM_OFFSET + 12] << 8) | 
			u8TcpRxBuffer[RX_BUFFER_PARAM_OFFSET + 13];
			
		RCLCPP_DEBUG(
			this->get_logger(),
			"Scan count : %d",
			u16PacketScanNumPoints
		);
			
		u16ScanNumPtsAcumu += u16PacketScanNumPoints;
		
		RCLCPP_DEBUG(
			this->get_logger(),
			"Scan buffer offset : %d",
			u16ScanBufferOffset
		);
		
		// Copy all points from TCP buffer to the ROS message buffer
		for(uint16_t i = 0; i < u16PacketScanNumPoints; i++){
			msgLaser.ranges[i + u16ScanBufferOffset] = 
				(
				(u8TcpRxBuffer[RX_BUFFER_PARAM_OFFSET + 14 + (i << 2)] << 8) | 
				(u8TcpRxBuffer[RX_BUFFER_PARAM_OFFSET + 15 + (i << 2)])
				) * 0.001f;// Convert from mm to meter

			msgLaser.intensities[i + u16ScanBufferOffset] =  
				(
				(u8TcpRxBuffer[RX_BUFFER_PARAM_OFFSET + 16 + (i << 2)] << 8) | 
				(u8TcpRxBuffer[RX_BUFFER_PARAM_OFFSET + 17 + (i << 2)])
				) * 0.00001f;
		}

		// Publish on the last packet
		if(u16ScanNumPtsAcumu >= u32TotalScanNumPoints){
			RCLCPP_DEBUG(
				this->get_logger(),
				"Publishing scan..."
			);
			u16ScanNumPtsAcumu = 0;
			pubLaserScan->publish(msgLaser);
		}

		u16ScanBufferOffset = u16ScanNumPtsAcumu;

	}
	
	void c200_fsm(){
		
		switch(u8LaserFSM){
			case 0:// Check sensor type
			{
				RCLCPP_INFO(
					this->get_logger(), 
					"Checking sensor type..."
					);
					
				c200_getDeviceName();
				u8LaserFSM = 1;
			}
			break;
			
			case 1:// Check sensor type response, stop scan
			{
				if(!c200_checkRxEqual(13))
					return;
				
				if(
					(u8TcpRxBuffer[8] == 'C') &&
					(u8TcpRxBuffer[9] == '2') &&
					(u8TcpRxBuffer[10] == '0') &&
					(u8TcpRxBuffer[11] == '0')
				){
					RCLCPP_INFO(
						this->get_logger(), 
						"Found the C200 series sensor!"
						);
					
					RCLCPP_INFO(
						this->get_logger(),
						"Logging in to the sensor..."
						);
					
					c200_sensorLogin();

					u8LaserFSM = 2;
					
				}else{
					RCLCPP_ERROR(
						this->get_logger(),
						"Error invalid sensor!"
					);
					close(i32LaserSocketFd);
					std::raise(SIGTERM);
				}
				
			}
			break;
			
			case 2:// Login response, stop scan
			{
				if(!c200_checkRxEqual(10))
					return;
				
				if(u8TcpRxBuffer[8] == 1){
					RCLCPP_INFO(
						this->get_logger(),
						"Login success!"
					);
										
					c200_stopScan();
					u8LaserFSM = 3;
					
				}else{
					RCLCPP_ERROR(
						this->get_logger(),
						"Error can't login to the sensor!"
					);
					close(i32LaserSocketFd);
					std::raise(SIGTERM);
				}
			}
			break;
			
			case 3:// Stop scan response, set speed and resolution
			{
				if(!c200_checkRxEqual(10))// return of Scan status
					return;
		
				if(u8TcpRxBuffer[8] == 0){
					RCLCPP_INFO(
						this->get_logger(),
						"Scan stopped, setting up the parameters"
					);
					c200_setScanRateAndResolution(
						f32LaserScanRate,
						f32LaserScanResolution
					);

					u8LaserFSM = 4;
					
				}else{
					RCLCPP_ERROR(
						this->get_logger(),
						"Error, can't stop scan!"
					);
					
					close(i32LaserSocketFd);
					std::raise(SIGTERM);
				}
				
			}
			break;
			
			case 4:// speed and resolution param response, send command to get Begin-End scan angle
			{
				if(!c200_checkRxEqual(11))// return of Speed & Angular resolution
					return;

				msgLaser.scan_time = 100.0f / (
					(u8TcpRxBuffer[8] << 8) | 
					u8TcpRxBuffer[9]);

				msgLaser.angle_increment = (
					(u8TcpRxBuffer[10] << 8) | 
					u8TcpRxBuffer[11]) / 10000.0f;
				
				RCLCPP_INFO(
					this->get_logger(),
					"Scan angular resolution : %.4f degree",
					msgLaser.angle_increment
				);

				RCLCPP_INFO(
					this->get_logger(),
					"Scan time : %.3fms",
					msgLaser.scan_time
				);
				
				c200_setScanBeginEndAngle(
					f32MinAngle,
					f32MaxAngle
				);
				
				u8LaserFSM = 5;
			}
			break;
			
			case 5:// start and end angle resposen, send command to start scan
			{
				if(!c200_checkRxEqual(18))
					return;

				msgLaser.angle_min 			= (int32_t)(
					(u8TcpRxBuffer[9] << 24)	|
					(u8TcpRxBuffer[10] << 16)	|
					(u8TcpRxBuffer[11] << 8)	|
					(u8TcpRxBuffer[12])
					) / 10000.0f;
				
				msgLaser.angle_max			= (int32_t)(
					(u8TcpRxBuffer[13] << 24)	|
					(u8TcpRxBuffer[14] << 16)	|
					(u8TcpRxBuffer[15] << 8)	|
					(u8TcpRxBuffer[16])
					) / 10000.0f;
				RCLCPP_INFO(
					this->get_logger(),
					"Scan min angle : %.2f degree",
					msgLaser.angle_min
				);

				RCLCPP_INFO(
					this->get_logger(),
					"Scan max angle : %.2f degree",
					msgLaser.angle_max
				);
				
				u32TotalScanNumPoints = (msgLaser.angle_max - msgLaser.angle_min) / msgLaser.angle_increment;
				u32TotalScanNumPoints += 1;
				
				// Convert degree to radiant
				msgLaser.angle_min = msgLaser.angle_min * DEG_TO_RADS_COSNT;
				msgLaser.angle_max = msgLaser.angle_max * DEG_TO_RADS_COSNT;
				msgLaser.angle_increment = msgLaser.angle_increment * DEG_TO_RADS_COSNT;
				
				RCLCPP_INFO(
					this->get_logger(),
					"Scan points : %d",
					u32TotalScanNumPoints
				);
				
				// Scan time, convert Hz to period time
				msgLaser.time_increment		= u32TotalScanNumPoints / msgLaser.scan_time;
				
				msgLaser.range_min			= 0.0f;
				msgLaser.range_max			= 25.0f;
				msgLaser.ranges.resize(u32TotalScanNumPoints);
				msgLaser.intensities.resize(u32TotalScanNumPoints);

				RCLCPP_INFO(
					this->get_logger(), 
					"Starting Scan..."
					);		
				
				c200_startScan();
				u8LaserFSM = 6;
			}
			break;
			
			case 6:// Idle, scan is done by the TCP timer loop
			{
				
				
			}
			break;
		}
	}
};

int main(int argc, char **argv){
	rclcpp::init(argc, argv);
	auto focusray_if {std::make_shared<c200_if>()};
	rclcpp::spin(focusray_if);
	
	rclcpp::shutdown();
}
