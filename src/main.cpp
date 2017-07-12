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

		// Close device
		dev -> close( );
	}

	// Else no devices found
	else {
		std::cout << "No devices found" << std::endl;
	}

}
