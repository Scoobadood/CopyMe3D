#include <iostream>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>

#include "CopyMe3DConfig.h"

#include "libsdf/TSDFVolume.hpp"
#include "libsdf/PngWrapper.hpp"
#include "RenderUtilities.hpp"   // For scene as png



/**
 * Convert a freenect depth buffer (float) to a uint16 depth buffer
 */
uint16_t* freenect2_depth_frame_to_uint16( const libfreenect2::Frame& depth ) {
    // Should be 512x424 but lets not assume
    size_t num_values = depth.width * depth.height;

    float * depthData = (float *) depth.data;

    uint16_t * new_buffer= new uint16_t[num_values];
    for( int i=0; i<num_values; i++ ) {
        // Values are already mm so just convert to uint16
        float f = depthData[i];
        uint16_t u = (uint16_t) f;
        new_buffer[i] = u;
    }
    return new_buffer;
}


/**
 * Render the TSDF to PNG files for scene and normal
 */
void render(const TSDFVolume * volume) {
        Eigen::Matrix< float, 3, Eigen::Dynamic> vertices;
        Eigen::Matrix< float, 3, Eigen::Dynamic> normals;

        std::cout << "Raycasting" << std::endl;

        Camera * camera = Camera::default_depth_camera( );
        volume->raycast( 512, 424, *camera, vertices, normals );

        std::cout << "Rendering to image " << std::endl;
        Eigen::Vector3f light_source { 0,0,-1000 };

        PngWrapper *p = scene_as_png( 512, 424, vertices, normals, *camera, light_source );

        std::cout << "Saving PNG" << std::endl;
        p->save_to("/home/dave/Desktop/scene.png");
        delete p;

        std::cout << "Rendering normals to image " << std::endl;
        p = normals_as_png( 512, 424, normals );
        std::cout << "Saving PNG" << std::endl;
        p->save_to("/home/dave/Desktop/normals.png");
        delete p;
    }


/** 
 * Return the minimum and maximum floats found in the frame
 */
void minMaxFrame( const libfreenect2::Frame& frame, float& min, float& max ) {
    min = frame.data[0];
    max = frame.data[0];
    size_t data_size = frame.width * frame.height;
    for( int i=0; i<data_size; i++ ) {
        float value;
        if( frame.format == 1 )  {
            if( frame.bytes_per_pixel == 2 ) {
                value = ((int16_t *) frame.data)[i];
            } else if( frame.bytes_per_pixel == 4 ) {
                value = ((int32_t *) frame.data)[i];
            } else {
                value = ((int8_t *) frame.data)[i];
            }

        } else if ( frame.format == 2) {
            value = ((float *)frame.data)[i];
        } else {
            value = frame.data[i];
        }

        if( min >  value ) min = value;
        if( max < value ) max = value;
    }
}


/**
 * Dump frame data in human readable form
 */
void dumpFrameData( const std::string& frameName, const libfreenect2::Frame& frame ) {
    std::cout << "Frame ["<<frameName<<"] is " << frame.width<< "x"<< frame.height;
    std::cout << ", Bytes per pixel : " << frame.bytes_per_pixel;
    std::cout << ", Format is ";
    switch( frame.format) {
      case 0: std::cout << "Invalid";break;
      case 1: std::cout << "Raw";break;
      case 2: std::cout << "Float";break;
      case 4: std::cout << "BGRX";break;
      case 5: std::cout << "RGBX";break;
      case 6: std::cout << "Grey";break;
      default: std::cout << "UNKNOWN";break;
    }
    float min, max;
    minMaxFrame( frame, min, max );
    std::cout << ", ( min : " << min << ", max : " << max << ")" << std::endl;
}

void save_frame_as_png( uint32_t width, uint32_t height, const uint16_t * frameData, const std::string& file_name ) {
    // Render the buffer to a PNG for test purposes
    PngWrapper * wrapper = new PngWrapper( width, height, (const uint8_t *)frameData, PngWrapper::GREYSCALE_16);
    wrapper->save_to( file_name );
    delete wrapper;
}

bool process( const Camera& camera, const libfreenect2::Frame&  rgb, const libfreenect2::Frame& depth , TSDFVolume * volume ) {
    static int frames=10;

    dumpFrameData( "Undistorted Depth", depth );


    // Insert processing code
    uint16_t * depth_buffer = freenect2_depth_frame_to_uint16( depth );


    // Min Max depth_buffer
    uint16_t min, max;
    min = max = depth_buffer[0];
    for( int i=0; i<depth.width * depth.height; i++ ) {
        if( min > depth_buffer[i]) min = depth_buffer[i];
        if( max < depth_buffer[i]) max = depth_buffer[i];
    }
    std::cout << "After conversion min: " << min << " , max : " << max << std::endl;


    char file_name[300];
    sprintf( file_name, "/home/dave/Desktop/depth_%d.png", 10-frames );
    save_frame_as_png( depth.width, depth.height, depth_buffer, file_name );

    volume->integrate( depth_buffer, (uint32_t) depth.width, (uint32_t) depth.height, camera );
        
    delete [] depth_buffer;

    frames--;
    return ( frames == 0 );
}


/**
 * Dump the IR Camera parameters to console
 */
void dumpIRParams( const libfreenect2::Freenect2Device::IrCameraParams& irParams) {
    std::cout << "IR Camera Parameters\n--------------------" << std::endl;
    std::cout << "fx   : " << irParams.fx << std::endl;
    std::cout << "fy   : " << irParams.fy << std::endl;
    std::cout << "cx   : " << irParams.cx << std::endl;
    std::cout << "cy   : " << irParams.cy << std::endl;
    std::cout << "k1   : " << irParams.k1 << std::endl;
    std::cout << "k2   : " << irParams.k2 << std::endl;
    std::cout << "k3   : " << irParams.k3 << std::endl;
    std::cout << "p1   : " << irParams.p1 << std::endl;
    std::cout << "p2   : " << irParams.p2 << std::endl;
}


/**
 * Dump the ColourIR Camera parameters to console
 */
void dumpColourParams( const libfreenect2::Freenect2Device::ColorCameraParams& colorParams) {
    std::cout << "Colour Camera Parameters\n------------------------" << std::endl;
    std::cout << "fx   : " << colorParams.fx << std::endl;
    std::cout << "fy   : " << colorParams.fy << std::endl;
    std::cout << "cx   : " << colorParams.cx << std::endl;
    std::cout << "cy   : " << colorParams.cy << std::endl;
}



/**
 * Find and start the default device
 */
bool  findAndStartDevice( libfreenect2::Freenect2& freenect2, libfreenect2::Freenect2Device* &device, libfreenect2::PacketPipeline* &pipeline, libfreenect2::SyncMultiFrameListener& listener) {
    using namespace libfreenect2;
    
    // Enumerate Kinect devices
    if( freenect2.enumerateDevices( ) == 0 ) {
        std::cout << "No devices found" << std::endl;
        return( false);
    }
    
    std::string deviceSerial = freenect2.getDefaultDeviceSerialNumber( );
    pipeline = new CpuPacketPipeline( );
    device = freenect2.openDevice( deviceSerial, pipeline );

    device->setColorFrameListener(&listener);
    device->setIrAndDepthFrameListener(&listener);

   

    // Start the device
    if( !(device -> start( ) ) ) {
        std::cout << "Couldn't start device" << std::endl;
        delete pipeline;
        pipeline = NULL;
        delete device;
        device = NULL;
        return false;
    }


    return true;
}

libfreenect2::Registration * setupRegistration( libfreenect2::Freenect2Device* device ) {
    using namespace libfreenect2;

    Freenect2Device::IrCameraParams irParams = device->getIrCameraParams( );
    Freenect2Device::ColorCameraParams colorParams = device->getColorCameraParams() ;
    return new Registration( irParams, colorParams );
}



TSDFVolume * buildSDF( libfreenect2::SyncMultiFrameListener& listener, libfreenect2::Registration * registration ) {
using namespace libfreenect2;

    TSDFVolume * volume = new TSDFVolume( 256, 256, 256, 1500.0f, 1500.0f, 1500.0f);

    volume->offset( -750, -750, 0);

    Camera * camera = Camera::default_depth_camera( );

    // Loop until done or timed out
    bool done = false;
    FrameMap frames;
    // Registered image storage
    Frame undistortedDepth( 512, 424, 4 );
    undistortedDepth.format = Frame::Float;
    Frame registeredColour( 512, 424, 4 );
    registeredColour.format = Frame::BGRX;


    while( !done ) {
        // Wait for frames
        if ( listener.waitForNewFrame( frames, 10*1000 ) )  {// 10 sconds
            Frame * rgb = frames[ Frame::Color];
            Frame * depth = frames[ Frame::Depth];

            // Register frames
            registration->apply(rgb, depth, &undistortedDepth, &registeredColour);

            // Process images
            done = process( *camera, registeredColour, undistortedDepth, volume );

            listener.release( frames );
        }
        // No frames in time
        else {
            std::cout << "timeout!" << std::endl;
                done = true;
        }
    }

    return volume;
}


int main( int argc, const char * argv[] ) {
    using namespace libfreenect2;

    Freenect2 freenect2;
    
    PacketPipeline * pipeline;
    Freenect2Device * dev;
    int types = Frame::Color | Frame::Depth;
    SyncMultiFrameListener listener(types);


    if ( !findAndStartDevice( freenect2, dev, pipeline, listener) ) {
        return -1;
    }


    // Set up for registration: MUST BE CALLED AFTER start
    Registration* registration = setupRegistration( dev );


    // Grab frames and process
    TSDFVolume *volume = buildSDF( listener, registration);

    // Finish with device
    dev->stop();
    dev -> close( );


    render( volume );    

    volume->save_to_file("/home/dave/tsdf.dat");
    delete volume;
}


