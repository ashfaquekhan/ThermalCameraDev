#include <iostream>
#include <cstring>
#include <unistd.h>
#include <signal.h>
#include <iomanip>
#include <sstream>
#include <chrono>
#include <fstream>
#include <vector>
#include <algorithm>
#include <numeric>
#include <cmath>

#if defined(linux) || defined(unix)
    #include <semaphore.h>
#endif

#ifndef DLLEXPORT
    #if defined(_WIN32)
        #define DLLEXPORT __declspec(dllexport)
    #else
        #define DLLEXPORT
    #endif
#endif

extern "C" {
    #include "all_config.h"
    #include "libiruvc.h"
    #include "libirparse.h"
    #include "libirprocess.h"
    #include "libirtemp.h"
    #include "thermal_cam_cmd.h"
}

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

// ==================================================
// CONFIGURATION
// ==================================================

struct ThermalConfig {
    bool show_thermal_mode = true;
    bool show_crosshair = true;
    bool show_stats = true;
    int display_scale = 2;
    float temp_offset_celsius = 0.0f;
    bool is_temperature_mode = false;
};

struct FrameStats {
    float min_temp_c = -999.0f;
    float max_temp_c = -999.0f;
    float avg_temp_c = -999.0f;
    bool valid = false;
};

// Global state
static ThermalConfig g_config;
static bool g_running = true;
static cv::Point g_selected_point(128, 96);
static TempDataRes_t g_temp_res = {256, 192};

// Global variables required by SDK
uint8_t is_streaming = 0;
int stream_time = 1000;
int fps = 25;

// ==================================================
// TEMPERATURE FUNCTIONS (FROM YOUR WORKING CODE)
// ==================================================

float get_point_temperature_safe(uint16_t* temp_data, int x, int y, int width, int height) {
    if (!temp_data || x < 0 || x >= width || y < 0 || y >= height) {
        return -999.0f;
    }
    
    // Method 1: Try SDK point temperature function
    IruvcPoint_t point = {(uint16_t)x, (uint16_t)y};
    uint16_t temp_raw = 0;
    
    if (tpd_get_point_temp_info(point, &temp_raw) == 0) {
        double temp_kelvin = (double)temp_raw / 16.0;
        double temp_celsius = temp_kelvin - 273.15;
        return (float)(temp_celsius + g_config.temp_offset_celsius);
    }
    
    // Method 2: Direct pixel conversion as fallback
    uint16_t raw_pixel = temp_data[y * width + x];
    float temp_celsius = ((float)raw_pixel / 64.0f) - 273.15f;
    return temp_celsius + g_config.temp_offset_celsius;
}

FrameStats calculate_frame_stats_safe(uint16_t* temp_data, int width, int height) {
    FrameStats stats;
    
    if (!temp_data || width <= 0 || height <= 0) {
        return stats;
    }
    
    // Try SDK max/min function first
    MaxMinTempInfo_t max_min_info;
    if (tpd_get_max_min_temp_info(&max_min_info) == 0) {
        double min_kelvin = (double)max_min_info.min_temp / 16.0;
        double max_kelvin = (double)max_min_info.max_temp / 16.0;
        
        stats.min_temp_c = (min_kelvin - 273.15) + g_config.temp_offset_celsius;
        stats.max_temp_c = (max_kelvin - 273.15) + g_config.temp_offset_celsius;
        stats.avg_temp_c = (stats.min_temp_c + stats.max_temp_c) / 2.0f;
        stats.valid = true;
    } else {
        // Manual calculation as fallback
        std::vector<float> temps;
        temps.reserve(100);
        
        for (int y = 4; y < height; y += 8) {
            for (int x = 4; x < width; x += 8) {
                float temp = get_point_temperature_safe(temp_data, x, y, width, height);
                if (temp > -900) {
                    temps.push_back(temp);
                }
            }
        }
        
        if (!temps.empty()) {
            auto minmax = std::minmax_element(temps.begin(), temps.end());
            stats.min_temp_c = *minmax.first;
            stats.max_temp_c = *minmax.second;
            stats.avg_temp_c = std::accumulate(temps.begin(), temps.end(), 0.0f) / temps.size();
            stats.valid = true;
        }
    }
    
    return stats;
}

// ==================================================
// CAMERA CONFIGURATION
// ==================================================

bool configure_camera_with_diagnostics() {
    std::cout << "🔧 Configuring P2 camera parameters..." << std::endl;
    
    usleep(500000); // 500ms wait
    
    // Try to set emissivity (0.95 = 15564 in 1/16384 scale)
    uint16_t emissivity_value = (uint16_t)(0.95 * (1 << 14));
    int result = set_prop_tpd_params(TPD_PROP_EMS, emissivity_value);
    if (result == 0) {
        std::cout << "✅ Emissivity set: " << emissivity_value << std::endl;
    } else {
        std::cout << "⚠️  Failed to set emissivity: " << result << std::endl;
    }
    
    // Try to set ambient temperature (27°C in 1/16 K units)
    uint16_t ambient_temp = (uint16_t)((27.0 + 273.15) * 16);
    result = set_prop_tpd_params(TPD_PROP_TA, ambient_temp);
    if (result == 0) {
        std::cout << "✅ Ambient temperature set: " << ambient_temp << std::endl;
    } else {
        std::cout << "⚠️  Failed to set ambient temperature: " << result << std::endl;
    }
    
    // Try to set gain mode
    result = set_prop_tpd_params(TPD_PROP_GAIN_SEL, 1); // High gain
    if (result == 0) {
        std::cout << "✅ Gain mode set to HIGH" << std::endl;
    } else {
        std::cout << "⚠️  Failed to set gain mode: " << result << std::endl;
    }
    
    return true;
}

bool switch_to_temperature_mode_safe() {
    std::cout << "🌡️ Attempting to switch to Y16 temperature mode..." << std::endl;
    std::cout << "   Waiting 3 seconds for camera stabilization..." << std::endl;
    sleep(3);
    
    int result = y16_preview_start(PREVIEW_PATH0, Y16_MODE_TEMPERATURE);
    if (result == 0) {
        std::cout << "✅ Successfully switched to Y16 temperature mode" << std::endl;
        g_config.is_temperature_mode = true;
        usleep(1000000); // 1 second additional wait
        return true;
    } else {
        std::cout << "❌ Failed to switch to Y16 temperature mode. Error code: " << result << std::endl;
        std::cout << "   Continuing in YUY2 mode - temperature data may not be accurate" << std::endl;
        return false;
    }
}

// ==================================================
// SIGNAL HANDLER
// ==================================================

void signal_handler(int signum) {
    std::cout << "\n🛑 Signal received, shutting down gracefully..." << std::endl;
    g_running = false;
    is_streaming = 0;
}

// ==================================================
// MOUSE CALLBACK
// ==================================================

void mouse_callback(int event, int x, int y, int flags, void* userdata) {
    if (event == cv::EVENT_LBUTTONDOWN) {
        int thermal_x = x / g_config.display_scale;
        int thermal_y = y / g_config.display_scale;
        
        if (thermal_x >= 0 && thermal_x < g_temp_res.width && 
            thermal_y >= 0 && thermal_y < g_temp_res.height) {
            g_selected_point = cv::Point(thermal_x, thermal_y);
            std::cout << "🎯 Selected: (" << thermal_x << "," << thermal_y << ")" << std::endl;
        }
    }
}

// ==================================================
// MAIN PROGRAM
// ==================================================

int main(int argc, char* argv[]) {
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    std::cout << "🌡️ P2 Thermal Camera Application (Working Lean)" << std::endl;
    std::cout << "===============================================" << std::endl;
    std::cout << "Based on working diagnostic code" << std::endl;
    
    // Initialize command system
    if (vdcmd_init() == 0) {
        std::cout << "✅ Command system initialized" << std::endl;
    } else {
        std::cout << "⚠️  Command system initialization failed" << std::endl;
    }
    
    // Initialize UVC camera
    std::cout << "📷 Initializing UVC camera..." << std::endl;
    int result = uvc_camera_init();
    if (result < 0) {
        std::cerr << "❌ UVC camera initialization failed: " << result << std::endl;
        return -1;
    }
    
    // Scan for devices
    DevCfg_t devices[64];
    memset(devices, 0, sizeof(devices));
    
    result = uvc_camera_list(devices);
    if (result < 0) {
        std::cerr << "❌ Device scan failed: " << result << std::endl;
        uvc_camera_release();
        return -1;
    }
    
    std::cout << "🔍 Scanning for thermal cameras..." << std::endl;
    
    // Find P2 thermal camera
    int thermal_index = -1;
    for (int i = 0; i < 10 && devices[i].vid != 0; i++) {
        std::cout << "Device " << i << ": VID=0x" << std::hex << devices[i].vid 
                  << ", PID=0x" << devices[i].pid << std::dec << std::endl;
        
        if (devices[i].vid == 0x0BDA && devices[i].pid == 0x5840) {
            thermal_index = i;
            std::cout << "✅ Found P2 thermal camera at index " << i << std::endl;
            break;
        }
    }
    
    if (thermal_index < 0) {
        std::cerr << "❌ No P2 thermal camera found (VID=0x0BDA, PID=0x5840)" << std::endl;
        uvc_camera_release();
        return -1;
    }
    
    // Get camera stream information
    CameraStreamInfo_t streams[32];
    memset(streams, 0, sizeof(streams));
    
    result = uvc_camera_info_get(devices[thermal_index], streams);
    if (result < 0) {
        std::cerr << "❌ Failed to get camera info: " << result << std::endl;
        uvc_camera_release();
        return -1;
    }
    
    // Setup camera parameters
    CameraParam_t camera_param;
    memset(&camera_param, 0, sizeof(camera_param));
    
    camera_param.dev_cfg = devices[thermal_index];
    camera_param.format = streams[0].format;
    camera_param.width = streams[0].width;
    camera_param.height = streams[0].height;
    camera_param.frame_size = camera_param.width * camera_param.height * 2;
    camera_param.fps = streams[0].fps[0];
    camera_param.timeout_ms_delay = 1000;
    
    fps = camera_param.fps;
    
    std::cout << "📐 Camera configuration:" << std::endl;
    std::cout << "   Resolution: " << camera_param.width << "x" << camera_param.height << std::endl;
    std::cout << "   FPS: " << camera_param.fps << std::endl;
    std::cout << "   Format: " << camera_param.format << std::endl;
    
    // Set thermal data dimensions
    int thermal_width = camera_param.width;
    int thermal_height = camera_param.height;
    g_temp_res.width = thermal_width;
    g_temp_res.height = thermal_height;
    
    // Open camera
    std::cout << "📷 Opening camera..." << std::endl;
    result = uvc_camera_open(devices[thermal_index]);
    if (result < 0) {
        std::cerr << "❌ Failed to open camera: " << result << std::endl;
        uvc_camera_release();
        return -1;
    }
    
    // Create frame buffer
    std::cout << "📦 Creating frame buffer..." << std::endl;
    void* frame_buffer = uvc_frame_buf_create(camera_param);
    if (!frame_buffer) {
        std::cerr << "❌ Failed to create frame buffer" << std::endl;
        uvc_camera_close();
        uvc_camera_release();
        return -1;
    }
    
    // Start streaming
    std::cout << "🎬 Starting camera stream..." << std::endl;
    result = uvc_camera_stream_start(camera_param, nullptr);
    if (result < 0) {
        std::cerr << "❌ Failed to start streaming: " << result << std::endl;
        uvc_frame_buf_release(frame_buffer);
        uvc_camera_close();
        uvc_camera_release();
        return -1;
    }
    
    is_streaming = 1;
    std::cout << "✅ Camera streaming started successfully" << std::endl;
    
    // Configure camera parameters
    configure_camera_with_diagnostics();
    
    // Try to switch to temperature mode
    if (!switch_to_temperature_mode_safe()) {
        std::cout << "⚠️  Continuing in RGB mode - temperature readings may be inaccurate" << std::endl;
    }
    
    // Allocate processing buffers
    uint16_t* temp_data = new uint16_t[thermal_width * thermal_height];
    uint8_t* rgb_display = new uint8_t[thermal_width * thermal_height * 3];
    
    std::cout << "🖥️  Creating OpenCV display window..." << std::endl;
    
    try {
        cv::namedWindow("P2 Thermal Camera - Working Lean", cv::WINDOW_AUTOSIZE);
        cv::setMouseCallback("P2 Thermal Camera - Working Lean", mouse_callback, nullptr);
        std::cout << "✅ OpenCV window created successfully" << std::endl;
    } catch (const cv::Exception& e) {
        std::cerr << "❌ Failed to create OpenCV window: " << e.what() << std::endl;
        std::cout << "Continuing without display..." << std::endl;
    }
    
    // Frame processing variables
    int frame_count = 0;
    int successful_frames = 0;
    auto start_time = std::chrono::steady_clock::now();
    auto last_fps_time = start_time;
    int fps_counter = 0;
    float current_fps = 0;
    FrameStats current_stats;
    
    std::cout << "🚀 Starting main processing loop..." << std::endl;
    std::cout << "\n📋 CONTROLS: 'q' to quit, 't' to toggle temperature mode" << std::endl;
    std::cout << "Click on the image to select measurement points\n" << std::endl;
    
    // Main processing loop
    while (g_running && is_streaming) {
        // Get frame from camera
        result = uvc_frame_get(frame_buffer);
        if (result != 0) {
            if (frame_count % 100 == 0) {
                std::cout << "⚠️  Frame " << frame_count << " failed (error " << result << ")" << std::endl;
            }
            usleep(10000);
            frame_count++;
            continue;
        }
        
        frame_count++;
        successful_frames++;
        fps_counter++;
        
        // Extract data from frame
        uint8_t* raw_data = (uint8_t*)frame_buffer;
        
        if (g_config.is_temperature_mode) {
            // In Y16 mode, data is already temperature data
            uint16_t* thermal_source = (uint16_t*)raw_data;
            memcpy(temp_data, thermal_source, thermal_width * thermal_height * sizeof(uint16_t));
        } else {
            // In YUY2 mode, convert grayscale data to pseudo-temperature
            for (int i = 0; i < thermal_width * thermal_height; i++) {
                uint8_t y_value = raw_data[i * 2];
                temp_data[i] = (uint16_t)(y_value * 64 + 17000);
            }
        }
        
        // Create visualization
        try {
            // Simple grayscale conversion for reliability
            for (int i = 0; i < thermal_width * thermal_height; i++) {
                uint8_t gray;
                if (g_config.is_temperature_mode) {
                    // Convert temperature data to grayscale
                    float temp_c = ((float)temp_data[i] / 64.0f) - 273.15f;
                    // Map temperature range roughly 15-40°C to 0-255
                    int mapped = (int)((temp_c - 15.0f) * 255.0f / 25.0f);
                    gray = (uint8_t)std::max(0, std::min(255, mapped));
                } else {
                    // Use Y component directly
                    gray = raw_data[i * 2];
                }
                
                rgb_display[i * 3 + 0] = gray; // B
                rgb_display[i * 3 + 1] = gray; // G  
                rgb_display[i * 3 + 2] = gray; // R
            }
            
            // Create display image
            cv::Mat thermal_img(thermal_height, thermal_width, CV_8UC3, rgb_display);
            cv::Mat display_img;
            cv::resize(thermal_img, display_img, 
                      cv::Size(thermal_width * g_config.display_scale, 
                              thermal_height * g_config.display_scale), 
                      0, 0, cv::INTER_NEAREST);
            
            // Draw crosshair
            if (g_config.show_crosshair) {
                int display_x = g_selected_point.x * g_config.display_scale;
                int display_y = g_selected_point.y * g_config.display_scale;
                cv::drawMarker(display_img, cv::Point(display_x, display_y), 
                              cv::Scalar(0, 255, 255), cv::MARKER_CROSS, 20, 2);
            }
            
            // Calculate statistics periodically
            if (frame_count % 30 == 0 && g_config.is_temperature_mode) {
                current_stats = calculate_frame_stats_safe(temp_data, thermal_width, thermal_height);
            }
            
            // Add overlay text
            std::vector<std::string> overlay_text;
            
            std::string mode_str = g_config.is_temperature_mode ? "Temperature (Y16)" : "RGB (YUY2)";
            overlay_text.push_back("Mode: " + mode_str);
            overlay_text.push_back("Frames: " + std::to_string(successful_frames) + "/" + std::to_string(frame_count) + 
                                  " | FPS: " + std::to_string((int)current_fps));
            
            // Selected point temperature
            if (g_config.is_temperature_mode) {
                float point_temp = get_point_temperature_safe(temp_data, 
                    g_selected_point.x, g_selected_point.y, thermal_width, thermal_height);
                
                if (point_temp > -900) {
                    std::stringstream ss;
                    ss << "Point (" << g_selected_point.x << "," << g_selected_point.y << "): "
                       << std::fixed << std::setprecision(1) << point_temp << "°C";
                    overlay_text.push_back(ss.str());
                }
                
                // Frame statistics
                if (g_config.show_stats && current_stats.valid) {
                    std::stringstream ss;
                    ss << "Min: " << std::fixed << std::setprecision(1) << current_stats.min_temp_c << "°C";
                    overlay_text.push_back(ss.str());
                    
                    ss.str("");
                    ss << "Max: " << std::fixed << std::setprecision(1) << current_stats.max_temp_c << "°C";
                    overlay_text.push_back(ss.str());
                }
            }
            
            // Draw overlay text
            int y_offset = 25;
            for (const auto& text : overlay_text) {
                cv::putText(display_img, text, cv::Point(10, y_offset), 
                           cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
                y_offset += 25;
            }
            
            // Display image
            cv::imshow("P2 Thermal Camera - Working Lean", display_img);
        } catch (const cv::Exception& e) {
            std::cout << "⚠️  Display error: " << e.what() << std::endl;
        }
        
        // Update FPS counter
        auto current_time = std::chrono::steady_clock::now();
        auto fps_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_fps_time);
        if (fps_elapsed.count() >= 1000) {
            current_fps = fps_counter * 1000.0f / fps_elapsed.count();
            fps_counter = 0;
            last_fps_time = current_time;
            
            // Console status update
            auto total_elapsed = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time);
            std::cout << "\r📊 " << successful_frames << "/" << frame_count << " frames | " 
                     << total_elapsed.count() << "s | "
                     << std::fixed << std::setprecision(1) << current_fps << " fps | "
                     << (g_config.is_temperature_mode ? "Y16" : "YUY2") << " mode"
                     << "        " << std::flush;
        }
        
        // Handle keyboard input
        int key = cv::waitKey(1) & 0xFF;
        
        switch (key) {
            case 'q':
            case 27:  // ESC
                std::cout << "\n👋 User requested exit" << std::endl;
                g_running = false;
                is_streaming = 0;
                break;
                
            case 't':
                if (g_config.is_temperature_mode) {
                    y16_preview_stop(PREVIEW_PATH0);
                    g_config.is_temperature_mode = false;
                    std::cout << "\n📷 Switched to RGB mode" << std::endl;
                } else {
                    if (switch_to_temperature_mode_safe()) {
                        std::cout << "\n🌡️ Successfully switched to temperature mode" << std::endl;
                    }
                }
                break;
                
            case 'c':
                g_config.show_crosshair = !g_config.show_crosshair;
                break;
                
            case 's':
                g_config.show_stats = !g_config.show_stats;
                break;
                
            case 'h':
                std::cout << "\n📋 CONTROLS:" << std::endl;
                std::cout << "  'q' or ESC - Quit" << std::endl;
                std::cout << "  't' - Toggle temperature/RGB mode" << std::endl;
                std::cout << "  'c' - Toggle crosshair" << std::endl;
                std::cout << "  's' - Toggle statistics" << std::endl;
                std::cout << "  'h' - Show this help" << std::endl;
                break;
        }
    }
    
    // Cleanup
    std::cout << "\n\n🧹 Starting cleanup..." << std::endl;
    
    auto total_time = std::chrono::steady_clock::now() - start_time;
    auto total_seconds = std::chrono::duration_cast<std::chrono::seconds>(total_time).count();
    
    is_streaming = 0;
    
    // Stop Y16 mode if active
    if (g_config.is_temperature_mode) {
        y16_preview_stop(PREVIEW_PATH0);
    }
    
    // Cleanup in reverse order
    delete[] temp_data;
    delete[] rgb_display;
    
    try {
        cv::destroyAllWindows();
    } catch (...) {
        std::cout << "Error destroying OpenCV windows" << std::endl;
    }
    
    uvc_camera_stream_close(KEEP_CAM_SIDE_PREVIEW);
    uvc_frame_buf_release(frame_buffer);
    uvc_camera_close();
    usleep(500000); // 500ms
    uvc_camera_release();
    
    // Final statistics
    std::cout << "\n📊 SESSION SUMMARY" << std::endl;
    std::cout << "Total frames attempted: " << frame_count << std::endl;
    std::cout << "Successful frames: " << successful_frames << std::endl;
    std::cout << "Success rate: " << std::fixed << std::setprecision(1) 
              << (frame_count > 0 ? (successful_frames * 100.0f / frame_count) : 0) << "%" << std::endl;
    std::cout << "Total time: " << total_seconds << " seconds" << std::endl;
    if (total_seconds > 0) {
        std::cout << "Average FPS: " << std::fixed << std::setprecision(1) 
                  << (successful_frames / (float)total_seconds) << std::endl;
    }
    
    std::cout << "\n✅ Application ended successfully" << std::endl;
    
    return 0;
}