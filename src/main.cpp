#include <iostream>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>

#include "CopyMe3DConfig.h"

/*
 * Detect and open Kinect
 * Repeatedly read de
*/

int main( int argc, const char * argv[] ) {
	using namespace libfreenect2;

	// Enumerate Kinect devices
	Freenect2 freenect2;
	if( freenect2.enumerateDevices( ) != 0 ) {
		std::string deviceSerial = freenect2.getDefaultDeviceSerialNumber( );
		PacketPipeline * pipeline = new CpuPacketPipeline( );
		Freenect2Device * dev = freenect2.openDevice( deviceSerial, pipeline );

		int types = Frame::Color | Frame::Ir | Frame::Depth;
  		SyncMultiFrameListener listener(types);

  		FrameMap frames;
  		dev->setColorFrameListener(&listener);
  		dev->setIrAndDepthFrameListener(&listener);

		// Start the device
		if( dev -> start( ) ) {
			// Wait for frames
			if ( !listener.waitForNewFrame( frames, 10*1000 ) )  {// 10 sconds
				Frame * rgb = frames[ Frame::Color];
				Frame * ir = frames[ Frame::Ir];
				Frame * depth = frames[ Frame::Depth];

				listener.release( frames );
			}

			// No frames in time
			else {
				std::cout << "timeout!" << std::endl;
     		}
			dev->stop();
		}

		// Device didn;t start
		else {
			std::cout << "Couldn't start device" << std::endl;
		}
		// Close device
		dev -> close( );
	}

	// Else no devices found
	else {
		std::cout << "No devices found" << std::endl;
	}

}
