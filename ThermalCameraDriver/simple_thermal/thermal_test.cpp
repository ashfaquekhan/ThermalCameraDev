#include <iostream>
#include <cstring>
#include <unistd.h>
#include <signal.h>
#include <iomanip>

// Include SDK headers
extern "C" {
    #include "libiruvc.h"
    #include "libirtemp.h"
}

#include <opencv2/opencv.hpp>

static bool g_running = true;

void signal_handler(int signum) {
    g_running = false;
}

// ==================================================
// MAIN DIAGNOSTIC PROGRAM
// ==================================================

int main() {
    signal(SIGINT, signal_handler);
    
    std::cout << "🌡️ THERMAL CAMERA TEMPERATURE TEST" << std::endl;
    std::cout << "==================================" << std::endl;
    std::cout << "Finding correct temperature conversion..." << std::endl;
    std::cout << std::endl;
    
    // Initialize
    int result = uvc_camera_init();
    if (result < 0) {
        std::cerr << "❌ Camera init failed: " << result << std::endl;
        return -1;
    }
    std::cout << "✅ Camera initialized" << std::endl;
    
    // Find devices
    DevCfg_t devices[64];
    memset(devices, 0, sizeof(devices));
    
    result = uvc_camera_list(devices);
    if (result < 0) {
        std::cerr << "❌ Device scan failed" << std::endl;
        uvc_camera_release();
        return -1;
    }
    
    // Find thermal camera
    int cam_idx = -1;
    for (int i = 0; i < 10 && devices[i].vid != 0; i++) {
        printf("Device %d: VID=0x%04X, PID=0x%04X\n", i, devices[i].vid, devices[i].pid);
        if (devices[i].vid == 0x0BDA && (devices[i].pid == 0x5840 || devices[i].pid == 0x5830)) {
            cam_idx = i;
            std::cout << "✅ Found thermal camera at index " << i << std::endl;
            break;
        }
    }
    
    if (cam_idx < 0) {
        std::cerr << "❌ No thermal camera found" << std::endl;
        std::cerr << "Make sure camera is connected and you're running with sudo" << std::endl;
        uvc_camera_release();
        return -1;
    }
    
    // Get camera info
    CameraStreamInfo_t streams[32];
    memset(streams, 0, sizeof(streams));
    
    result = uvc_camera_info_get(devices[cam_idx], streams);
    if (result < 0) {
        std::cerr << "❌ Failed to get camera info" << std::endl;
        uvc_camera_release();
        return -1;
    }
    
    // Setup parameters
    CameraParam_t param;
    memset(&param, 0, sizeof(param));
    
    param.dev_cfg = devices[cam_idx];
    param.format = streams[0].format;
    param.width = streams[0].width;
    param.height = streams[0].height;
    param.frame_size = param.width * param.height * 2;
    param.fps = streams[0].fps[0];
    param.timeout_ms_delay = 100;
    
    std::cout << "📐 Resolution: " << param.width << "x" << param.height 
              << " @ " << param.fps << " fps" << std::endl;
    
    // Determine thermal size
    int thermal_width = param.width;
    int thermal_height = param.height;
    bool is_p2 = false;
    
    if (param.width == 256 && param.height == 384) {
        thermal_height = 192;  // P2 series
        is_p2 = true;
        std::cout << "📷 P2 series detected (256x384 combined mode)" << std::endl;
        std::cout << "   Thermal resolution: 256x192" << std::endl;
    }
    
    // Open camera
    result = uvc_camera_open(devices[cam_idx]);
    if (result < 0) {
        std::cerr << "❌ Failed to open camera" << std::endl;
        uvc_camera_release();
        return -1;
    }
    std::cout << "✅ Camera opened" << std::endl;
    
    // Create frame buffer
    void* frame_buffer = uvc_frame_buf_create(param);
    if (!frame_buffer) {
        std::cerr << "❌ Failed to create frame buffer" << std::endl;
        uvc_camera_close();
        uvc_camera_release();
        return -1;
    }
    
    // Start streaming
    result = uvc_camera_stream_start(param, nullptr);
    if (result < 0) {
        std::cerr << "❌ Failed to start streaming" << std::endl;
        uvc_frame_buf_release(frame_buffer);
        uvc_camera_close();
        uvc_camera_release();
        return -1;
    }
    std::cout << "✅ Streaming started" << std::endl;
    
    // Wait for camera to stabilize
    std::cout << "\n⏳ Waiting for camera to stabilize..." << std::endl;
    sleep(2);
    
    // Allocate buffers
    uint16_t* temp_data = new uint16_t[thermal_width * thermal_height];
    uint8_t* rgb_data = new uint8_t[thermal_width * thermal_height * 3];
    
    // Create window
    cv::namedWindow("Thermal View", cv::WINDOW_AUTOSIZE);
    
    std::cout << "\n🎮 Controls:" << std::endl;
    std::cout << "  ESC - Exit" << std::endl;
    std::cout << "  Space - Pause analysis" << std::endl;
    std::cout << "\n📊 Analyzing temperature data...\n" << std::endl;
    
    // Main loop
    int frame_count = 0;
    bool paused = false;
    
    while (g_running) {
        // Get frame
        result = uvc_frame_get(frame_buffer);
        if (result != 0) {
            usleep(10000);
            continue;
        }
        
        frame_count++;
        
        // Extract thermal data
        uint8_t* raw = (uint8_t*)frame_buffer;
        uint16_t* thermal_src = nullptr;
        
        if (is_p2) {
            // P2 series: thermal data after RGB data
            thermal_src = (uint16_t*)(raw + (256 * 192 * 2));
        } else {
            // Regular series: direct thermal data
            thermal_src = (uint16_t*)raw;
        }
        
        memcpy(temp_data, thermal_src, thermal_width * thermal_height * sizeof(uint16_t));
        
        // Analyze temperature data every 25 frames
        if (!paused && frame_count % 25 == 0) {
            std::cout << "\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━" << std::endl;
            std::cout << "📊 Frame " << frame_count << " Analysis:" << std::endl;
            
            // Get center pixel
            int cx = thermal_width / 2;
            int cy = thermal_height / 2;
            uint16_t center_val = temp_data[cy * thermal_width + cx];
            
            std::cout << "\n🎯 Center pixel (" << cx << "," << cy << "):" << std::endl;
            std::cout << "   Raw value: " << center_val 
                      << " (0x" << std::hex << center_val << std::dec << ")" << std::endl;
            
            // Test different conversion methods
            std::cout << "\n🔬 Temperature conversions:" << std::endl;
            
            // Method 1: Direct /16 (for module pre-converted data)
            float t1 = (center_val / 16.0) - 273.15;
            std::cout << "   1) Value/16 - 273.15 = " 
                      << std::fixed << std::setprecision(1) << t1 << "°C";
            if (t1 >= 10 && t1 <= 40) std::cout << " ✅ REASONABLE!";
            std::cout << std::endl;
            
            // Method 2: Direct /64 (standard SDK formula)
            float t2 = (center_val / 64.0) - 273.15;
            std::cout << "   2) Value/64 - 273.15 = " << t2 << "°C";
            if (t2 >= 10 && t2 <= 40) std::cout << " ✅ REASONABLE!";
            std::cout << std::endl;
            
            // Method 3: Upper 14 bits then /64
            uint16_t upper14 = (center_val >> 2) & 0x3FFF;
            float t3 = (upper14 / 64.0) - 273.15;
            std::cout << "   3) Upper14bits/64 - 273.15 = " << t3 << "°C";
            if (t3 >= 10 && t3 <= 40) std::cout << " ✅ REASONABLE!";
            std::cout << std::endl;
            
            // Method 4: Byte swap then /64
            uint16_t swapped = ((center_val & 0xFF) << 8) | ((center_val & 0xFF00) >> 8);
            float t4 = (swapped / 64.0) - 273.15;
            std::cout << "   4) ByteSwap/64 - 273.15 = " << t4 << "°C";
            if (t4 >= 10 && t4 <= 40) std::cout << " ✅ REASONABLE!";
            std::cout << std::endl;
            
            // Test module's built-in function
            std::cout << "\n🔧 Module function test (tpd_get_point_temp_info):" << std::endl;
            IruvcPoint_t pt = {(uint16_t)cx, (uint16_t)cy};
            uint16_t module_temp = 0;
            iruvc_error_t res = tpd_get_point_temp_info(pt, &module_temp);
            
            if (res == IRUVC_SUCCESS) {
                float t5 = (module_temp / 16.0) - 273.15;
                std::cout << "   Raw value: " << module_temp << std::endl;
                std::cout << "   Temperature: " << t5 << "°C";
                if (t5 >= 10 && t5 <= 40) std::cout << " ✅ WORKING!";
                std::cout << std::endl;
            } else {
                std::cout << "   ❌ Failed (error " << res << ")" << std::endl;
                std::cout << "   Module functions may need initialization" << std::endl;
            }
            
            // Test frame min/max
            std::cout << "\n📈 Frame statistics (tpd_get_max_min_temp_info):" << std::endl;
            MaxMinTempInfo_t stats;
            res = tpd_get_max_min_temp_info(&stats);
            
            if (res == IRUVC_SUCCESS) {
                // Note: Need to check actual struct members
                // The error messages suggest these member names exist:
                // min_temp_point (not min_temp_pos)
                // max_temp_point (not max_temp_pos)
                std::cout << "   ✅ Function succeeded" << std::endl;
                std::cout << "   (Check struct members in SDK header)" << std::endl;
            } else {
                std::cout << "   ❌ Failed (error " << res << ")" << std::endl;
            }
            
            // Raw data statistics
            std::cout << "\n📊 Raw data statistics:" << std::endl;
            uint16_t min_val = 65535, max_val = 0;
            uint64_t sum = 0;
            
            for (int i = 0; i < thermal_width * thermal_height; i++) {
                if (temp_data[i] < min_val) min_val = temp_data[i];
                if (temp_data[i] > max_val) max_val = temp_data[i];
                sum += temp_data[i];
            }
            
            uint16_t avg_val = sum / (thermal_width * thermal_height);
            
            std::cout << "   Min: " << min_val 
                      << " (0x" << std::hex << min_val << std::dec << ")" << std::endl;
            std::cout << "   Max: " << max_val 
                      << " (0x" << std::hex << max_val << std::dec << ")" << std::endl;
            std::cout << "   Avg: " << avg_val 
                      << " (0x" << std::hex << avg_val << std::dec << ")" << std::endl;
        }
        
        // Create visualization
        // Find min/max for scaling
        uint16_t vis_min = 65535, vis_max = 0;
        for (int i = 0; i < thermal_width * thermal_height; i++) {
            if (temp_data[i] < vis_min) vis_min = temp_data[i];
            if (temp_data[i] > vis_max) vis_max = temp_data[i];
        }
        
        // Scale to 8-bit for display
        for (int i = 0; i < thermal_width * thermal_height; i++) {
            uint8_t scaled = 0;
            if (vis_max > vis_min) {
                scaled = (uint8_t)(((temp_data[i] - vis_min) * 255) / (vis_max - vis_min));
            }
            
            // Thermal colormap (iron)
            uint8_t r, g, b;
            if (scaled < 64) {
                r = scaled * 4; g = 0; b = 0;
            } else if (scaled < 128) {
                r = 255; g = (scaled - 64) * 4; b = 0;
            } else if (scaled < 192) {
                r = 255; g = 255; b = (scaled - 128) * 4;
            } else {
                r = 255; g = 255; b = 255;
            }
            
            rgb_data[i * 3] = b;      // BGR for OpenCV
            rgb_data[i * 3 + 1] = g;
            rgb_data[i * 3 + 2] = r;
        }
        
        // Display
        cv::Mat img(thermal_height, thermal_width, CV_8UC3, rgb_data);
        cv::Mat display;
        cv::resize(img, display, cv::Size(thermal_width * 2, thermal_height * 2), 0, 0, cv::INTER_NEAREST);
        
        // Add text overlay
        cv::putText(display, "Frame: " + std::to_string(frame_count), 
                   cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
        cv::putText(display, paused ? "PAUSED" : "ANALYZING", 
                   cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.7, 
                   paused ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 255, 0), 2);
        cv::putText(display, "Press ESC to exit, SPACE to pause", 
                   cv::Point(10, 90), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
        
        // Draw center crosshair
        int center_x = display.cols / 2;
        int center_y = display.rows / 2;
        cv::drawMarker(display, cv::Point(center_x, center_y), 
                      cv::Scalar(0, 255, 255), cv::MARKER_CROSS, 20, 2);
        
        cv::imshow("Thermal View", display);
        
        // Handle keyboard
        int key = cv::waitKey(1) & 0xFF;
        if (key == 27) break;  // ESC
        if (key == ' ') {
            paused = !paused;
            if (paused) {
                std::cout << "\n⏸️  PAUSED - Press SPACE to resume" << std::endl;
            } else {
                std::cout << "\n▶️  RESUMED" << std::endl;
            }
        }
    }
    
    // Print summary before exiting
    std::cout << "\n\n🏁 SUMMARY" << std::endl;
    std::cout << "==========" << std::endl;
    std::cout << "Total frames processed: " << frame_count << std::endl;
    std::cout << "\n💡 NEXT STEPS:" << std::endl;
    std::cout << "1. Look at the temperature conversions above" << std::endl;
    std::cout << "2. Find which method gives reasonable room temperature (20-30°C)" << std::endl;
    std::cout << "3. Use that conversion formula in your application" << std::endl;
    std::cout << "4. If module functions work, prefer those over manual conversion" << std::endl;
    
    // Cleanup
    std::cout << "\n🧹 Cleaning up..." << std::endl;
    
    delete[] temp_data;
    delete[] rgb_data;
    
    uvc_camera_stream_close(KEEP_CAM_SIDE_PREVIEW);
    uvc_frame_buf_release(frame_buffer);
    uvc_camera_close();
    uvc_camera_release();
    cv::destroyAllWindows();
    
    std::cout << "✅ Done!" << std::endl;
    
    return 0;
}
