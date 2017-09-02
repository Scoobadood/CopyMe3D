#include <iostream>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>

#include "CopyMe3DConfig.h"

#include "libsdf/TSDFVolume.hpp"

bool process( const libfreenect2::Frame&  rgb, const libfreenect2::Frame& depth , TSDFVolume & volume ) {

    // Insert processing code
        
    return true;
}

/*
 * Detect and open Kinect
 * Repeatedly read de
*/

int main( int argc, const char * argv[] ) {
    using namespace libfreenect2;
    
    // Enumerate Kinect devices
    Freenect2 freenect2;
    if( freenect2.enumerateDevices( ) == 0 ) {
        std::cout << "No devices found" << std::endl;
        exit( -1 );
    }
    
    std::string deviceSerial = freenect2.getDefaultDeviceSerialNumber( );
    PacketPipeline * pipeline = new CpuPacketPipeline( );
    Freenect2Device * dev = freenect2.openDevice( deviceSerial, pipeline );

    int types = Frame::Color | Frame::Depth;
    SyncMultiFrameListener listener(types);

    FrameMap frames;
    dev->setColorFrameListener(&listener);
    dev->setIrAndDepthFrameListener(&listener);

    // Set up for registration
    Registration* registration = new Registration( dev->getIrCameraParams(), dev->getColorCameraParams() );
    Frame undistortedDepth( 512, 424, 4 );
    Frame registeredColour( 512, 424, 4 );


    // Start the device
    if( !(dev -> start( ) ) ) {
        std::cout << "Couldn't start device" << std::endl;
    }

    // Grab fames and process
    TSDFVolume volume{256, 256, 256, 1500.0f, 1500.0f, 1500.0f};

    // Loop until done or timed out
    bool done = false;
    while( !done ) {
        // Wait for frames
        if ( listener.waitForNewFrame( frames, 10*1000 ) )  {// 10 sconds
            Frame * rgb = frames[ Frame::Color];
            Frame * depth = frames[ Frame::Depth];

            // Register frames
            registration->apply(rgb, depth, &undistortedDepth, &registeredColour);

            // Process images
            done = process( registeredColour, undistortedDepth, volume );

            listener.release( frames );
        }
        // No frames in time
        else {
            std::cout << "timeout!" << std::endl;
                done = true;
        }
    }
    dev->stop();

    // Close device
    dev -> close( );
}
