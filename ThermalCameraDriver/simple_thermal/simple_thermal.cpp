#include <iostream>
#include <cstring>
#include <unistd.h>
#include <signal.h>
#include <iomanip>
#include <sstream>
#include <chrono>
#include <vector>
#include <algorithm>
#include <numeric>
#include <cmath>
#include <fstream>

// Define DLLEXPORT for Linux
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

// Configuration
struct ThermalConfig {
    bool show_crosshair = true;
    bool show_stats = true;
    int display_scale = 2;
    float temp_offset_celsius = 0.0f;
    bool is_temperature_mode = false;
    bool use_pseudocolor = true;
    int color_mode = 3;  // IRPROC_COLOR_MODE_3 (default rainbow)
};

struct FrameStats {
    float min_temp_c = -999.0f;
    float max_temp_c = -999.0f;
    float avg_temp_c = -999.0f;
    float center_temp_c = -999.0f;
    bool valid = false;
};

// Color mode names for display
const char* color_mode_names[] = {
    "None",
    "White Hot",        // MODE_1
    "Reserved",         // MODE_2
    "Rainbow",          // MODE_3
    "Color 4",          // MODE_4
    "Color 5",          // MODE_5
    "Iron/Fire",        // MODE_6
    "Color 7",          // MODE_7
    "Color 8",          // MODE_8
    "Color 9",          // MODE_9
    "Color 10",         // MODE_10
    "Black Hot"         // MODE_11
};

// Global state
static ThermalConfig g_config;
static bool g_running = true;
static cv::Point g_selected_point(128, 96);
static TempDataRes_t g_temp_res = {256, 192};

// Required by SDK
uint8_t is_streaming = 0;
int stream_time = 1000;
int fps = 25;

// Signal handler
void signal_handler(int signum) {
    std::cout << "\nShutting down..." << std::endl;
    g_running = false;
    is_streaming = 0;
}

// Mouse callback
void mouse_callback(int event, int x, int y, int flags, void* userdata) {
    if (event == cv::EVENT_LBUTTONDOWN) {
        int thermal_x = x / g_config.display_scale;
        int thermal_y = y / g_config.display_scale;
        
        if (thermal_x >= 0 && thermal_x < g_temp_res.width && 
            thermal_y >= 0 && thermal_y < g_temp_res.height) {
            g_selected_point = cv::Point(thermal_x, thermal_y);
            std::cout << "Selected point: (" << thermal_x << "," << thermal_y << ")" << std::endl;
        }
    }
}

// Get temperature at specific point
float get_point_temperature(uint16_t* temp_data, int x, int y, int width, int height) {
    if (!temp_data || x < 0 || x >= width || y < 0 || y >= height) {
        return -999.0f;
    }
    
    // Try SDK function first
    IruvcPoint_t point = {(uint16_t)x, (uint16_t)y};
    uint16_t temp_raw = 0;
    
    if (tpd_get_point_temp_info(point, &temp_raw) == 0) {
        float temp_kelvin = temp_raw / 16.0f;
        float temp_celsius = temp_kelvin - 273.15f;
        return temp_celsius + g_config.temp_offset_celsius;
    }
    
    // Fallback: direct pixel conversion
    uint16_t raw_pixel = temp_data[y * width + x];
    float temp_celsius = (raw_pixel / 64.0f) - 273.15f;
    return temp_celsius + g_config.temp_offset_celsius;
}

// Calculate frame statistics
FrameStats calculate_frame_stats(uint16_t* temp_data, int width, int height) {
    FrameStats stats;
    
    if (!temp_data || width <= 0 || height <= 0) {
        return stats;
    }
    
    // Try SDK function
    MaxMinTempInfo_t max_min_info;
    if (tpd_get_max_min_temp_info(&max_min_info) == 0) {
        float min_kelvin = max_min_info.min_temp / 16.0f;
        float max_kelvin = max_min_info.max_temp / 16.0f;
        
        stats.min_temp_c = (min_kelvin - 273.15f) + g_config.temp_offset_celsius;
        stats.max_temp_c = (max_kelvin - 273.15f) + g_config.temp_offset_celsius;
        stats.avg_temp_c = (stats.min_temp_c + stats.max_temp_c) / 2.0f;
        stats.center_temp_c = get_point_temperature(temp_data, width/2, height/2, width, height);
        stats.valid = true;
    } else {
        // Manual calculation
        std::vector<float> temps;
        temps.reserve(width * height / 64);
        
        for (int y = 0; y < height; y += 8) {
            for (int x = 0; x < width; x += 8) {
                float temp = get_point_temperature(temp_data, x, y, width, height);
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
            stats.center_temp_c = get_point_temperature(temp_data, width/2, height/2, width, height);
            stats.valid = true;
        }
    }
    
    return stats;
}

// Check calibration files
bool check_calibration_files() {
    bool has_tau_l = (access("tau_L.bin", F_OK) == 0);
    bool has_tau_h = (access("tau_H.bin", F_OK) == 0);
    
    if (has_tau_l && has_tau_h) {
        std::cout << "Found calibration files (tau_L.bin, tau_H.bin)" << std::endl;
        return true;
    } else {
        std::cout << "Warning: Calibration files not found" << std::endl;
        if (!has_tau_l) std::cout << "  Missing: tau_L.bin" << std::endl;
        if (!has_tau_h) std::cout << "  Missing: tau_H.bin" << std::endl;
        std::cout << "Temperature readings may be inaccurate" << std::endl;
        return false;
    }
}

// Configure camera parameters
bool configure_camera() {
    std::cout << "Configuring camera parameters..." << std::endl;
    
    // Wait longer for camera stabilization (SDK needs time)
    std::cout << "Waiting for camera initialization..." << std::endl;
    sleep(2);
    
    // Try to set gain mode
    int result = set_prop_tpd_params(TPD_PROP_GAIN_SEL, 1);
    if (result != 0) {
        std::cout << "Note: Gain mode not set (using default)" << std::endl;
    } else {
        std::cout << "Gain mode configured" << std::endl;
    }
    
    usleep(100000);
    
    // Set emissivity
    uint16_t emissivity = (uint16_t)(0.95 * 16384);
    result = set_prop_tpd_params(TPD_PROP_EMS, emissivity);
    if (result != 0) {
        std::cout << "Note: Emissivity not set (using default 0.95)" << std::endl;
    } else {
        std::cout << "Emissivity set to 0.95" << std::endl;
    }
    
    // Set ambient temperature
    uint16_t ambient = (uint16_t)((25.0 + 273.15) * 16);
    result = set_prop_tpd_params(TPD_PROP_TA, ambient);
    if (result != 0) {
        std::cout << "Note: Ambient temp not set (using default)" << std::endl;
    } else {
        std::cout << "Ambient temperature set to 25°C" << std::endl;
    }
    
    return true;
}

// Switch to Y16 temperature mode
bool switch_to_temperature_mode() {
    std::cout << "Switching to Y16 temperature mode..." << std::endl;
    
    // Wait for camera stabilization
    sleep(2);
    
    int result = y16_preview_start(PREVIEW_PATH0, Y16_MODE_TEMPERATURE);
    if (result == 0) {
        std::cout << "Successfully switched to Y16 mode" << std::endl;
        g_config.is_temperature_mode = true;
        usleep(500000);
        return true;
    } else {
        std::cout << "Failed to switch to Y16 mode (error " << result << ")" << std::endl;
        std::cout << "Continuing in YUY2 mode" << std::endl;
        return false;
    }
}

// Apply pseudocolor to thermal data
void apply_pseudocolor(uint16_t* temp_data, int width, int height, 
                      int color_mode, uint8_t* rgb_output) {
    int pixel_count = width * height;
    
    // Allocate YUV422 buffer for SDK pseudocolor function
    uint8_t* yuv_buffer = new uint8_t[pixel_count * 2];
    
    // Apply SDK pseudocolor mapping (Y14 to YUV422)
    y14_map_to_yuyv_pseudocolor(temp_data, pixel_count, 
                                 (irproc_color_mode_t)color_mode, yuv_buffer);
    
    // Convert YUV422 to RGB
    for (int i = 0; i < pixel_count; i++) {
        // YUV422 is packed as: Y0 U Y1 V (4 bytes for 2 pixels)
        int pair_idx = i / 2;
        int is_second = i % 2;
        
        uint8_t y = yuv_buffer[i * 2];
        uint8_t u = yuv_buffer[pair_idx * 4 + 1];
        uint8_t v = yuv_buffer[pair_idx * 4 + 3];
        
        // YUV to RGB conversion (ITU-R BT.601)
        int c = y - 16;
        int d = u - 128;
        int e = v - 128;
        
        int r = (298 * c + 409 * e + 128) >> 8;
        int g = (298 * c - 100 * d - 208 * e + 128) >> 8;
        int b = (298 * c + 516 * d + 128) >> 8;
        
        // Clamp to valid range
        rgb_output[i * 3]     = std::max(0, std::min(255, b));  // B
        rgb_output[i * 3 + 1] = std::max(0, std::min(255, g));  // G
        rgb_output[i * 3 + 2] = std::max(0, std::min(255, r));  // R
    }
    
    delete[] yuv_buffer;
}

int main(int argc, char* argv[]) {
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    std::cout << "P2 Thermal Camera with Pseudocolor Support" << std::endl;
    std::cout << "===========================================" << std::endl;
    
    // Check calibration files
    check_calibration_files();
    
    // Initialize command system
    if (vdcmd_init() == 0) {
        std::cout << "Command system initialized" << std::endl;
    }
    
    // Initialize UVC camera
    int result = uvc_camera_init();
    if (result < 0) {
        std::cerr << "Failed to initialize camera: " << result << std::endl;
        return -1;
    }
    
    // Scan for devices
    DevCfg_t devices[64];
    memset(devices, 0, sizeof(devices));
    
    result = uvc_camera_list(devices);
    if (result < 0) {
        std::cerr << "Device scan failed: " << result << std::endl;
        uvc_camera_release();
        return -1;
    }
    
    // Find P2 thermal camera
    int thermal_index = -1;
    for (int i = 0; i < 10 && devices[i].vid != 0; i++) {
        std::cout << "Device " << i << ": VID=0x" << std::hex 
                  << devices[i].vid << ", PID=0x" << devices[i].pid 
                  << std::dec << std::endl;
        
        if (devices[i].vid == 0x0BDA && devices[i].pid == 0x5840) {
            thermal_index = i;
            std::cout << "Found P2 thermal camera at index " << i << std::endl;
            break;
        }
    }
    
    if (thermal_index < 0) {
        std::cerr << "No P2 thermal camera found" << std::endl;
        uvc_camera_release();
        return -1;
    }
    
    // Get camera stream info
    CameraStreamInfo_t streams[32];
    memset(streams, 0, sizeof(streams));
    
    result = uvc_camera_info_get(devices[thermal_index], streams);
    if (result < 0) {
        std::cerr << "Failed to get camera info: " << result << std::endl;
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
    
    std::cout << "Resolution: " << camera_param.width << "x" 
              << camera_param.height << " @ " << camera_param.fps << " fps" << std::endl;
    
    // Set thermal dimensions
    int thermal_width = camera_param.width;
    int thermal_height = camera_param.height;
    g_temp_res.width = thermal_width;
    g_temp_res.height = thermal_height;
    
    // Open camera
    result = uvc_camera_open(devices[thermal_index]);
    if (result < 0) {
        std::cerr << "Failed to open camera: " << result << std::endl;
        uvc_camera_release();
        return -1;
    }
    
    // Create frame buffer
    void* frame_buffer = uvc_frame_buf_create(camera_param);
    if (!frame_buffer) {
        std::cerr << "Failed to create frame buffer" << std::endl;
        uvc_camera_close();
        uvc_camera_release();
        return -1;
    }
    
    // Start streaming
    result = uvc_camera_stream_start(camera_param, nullptr);
    if (result < 0) {
        std::cerr << "Failed to start streaming: " << result << std::endl;
        uvc_frame_buf_release(frame_buffer);
        uvc_camera_close();
        uvc_camera_release();
        return -1;
    }
    
    is_streaming = 1;
    std::cout << "Camera streaming started" << std::endl;
    
    // Configure camera
    configure_camera();
    
    // Try to switch to temperature mode
    switch_to_temperature_mode();
    
    // Allocate buffers
    uint16_t* temp_data = new uint16_t[thermal_width * thermal_height];
    uint8_t* rgb_display = new uint8_t[thermal_width * thermal_height * 3];
    
    // Create window
    cv::namedWindow("Thermal Camera", cv::WINDOW_AUTOSIZE);
    cv::setMouseCallback("Thermal Camera", mouse_callback, nullptr);
    
    // Frame processing variables
    int frame_count = 0;
    auto start_time = std::chrono::steady_clock::now();
    auto last_fps_time = start_time;
    int fps_counter = 0;
    float current_fps = 0;
    FrameStats current_stats;
    
    std::cout << "\n=== CONTROLS ===" << std::endl;
    std::cout << "  q/ESC     - Quit" << std::endl;
    std::cout << "  t         - Toggle temperature mode (YUY2/Y16)" << std::endl;
    std::cout << "  p         - Toggle pseudocolor ON/OFF" << std::endl;
    std::cout << "  [ ]       - Previous/Next color palette" << std::endl;
    std::cout << "  1-9       - Quick select color mode" << std::endl;
    std::cout << "  c         - Toggle crosshair" << std::endl;
    std::cout << "  s         - Toggle statistics" << std::endl;
    std::cout << "  Mouse     - Click to select measurement point" << std::endl;
    std::cout << "\nCurrent: " << color_mode_names[g_config.color_mode] << std::endl;
    std::cout << std::endl;
    
    // Main loop
    while (g_running && is_streaming) {
        // Get frame
        result = uvc_frame_get(frame_buffer);
        if (result != 0) {
            usleep(10000);
            continue;
        }
        
        frame_count++;
        fps_counter++;
        
        // Extract data
        uint8_t* raw_data = (uint8_t*)frame_buffer;
        
        if (g_config.is_temperature_mode) {
            // Y16 mode - temperature data
            uint16_t* thermal_source = (uint16_t*)raw_data;
            memcpy(temp_data, thermal_source, thermal_width * thermal_height * sizeof(uint16_t));
            
            // Apply visualization
            if (g_config.use_pseudocolor) {
                // SDK pseudocolor mapping
                apply_pseudocolor(temp_data, thermal_width, thermal_height,
                                g_config.color_mode, rgb_display);
            } else {
                // Grayscale visualization
                for (int i = 0; i < thermal_width * thermal_height; i++) {
                    float temp_c = (temp_data[i] / 64.0f) - 273.15f;
                    int mapped = (int)((temp_c - 15.0f) * 255.0f / 25.0f);
                    uint8_t gray = (uint8_t)std::max(0, std::min(255, mapped));
                    
                    rgb_display[i * 3] = gray;
                    rgb_display[i * 3 + 1] = gray;
                    rgb_display[i * 3 + 2] = gray;
                }
            }
        } else {
            // YUY2 mode - convert grayscale
            for (int i = 0; i < thermal_width * thermal_height; i++) {
                uint8_t y_value = raw_data[i * 2];
                temp_data[i] = (uint16_t)(y_value * 64 + 17000);
                
                rgb_display[i * 3] = y_value;
                rgb_display[i * 3 + 1] = y_value;
                rgb_display[i * 3 + 2] = y_value;
            }
        }
        
        // Create display
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
        
        // Calculate stats periodically
        if (frame_count % 30 == 0 && g_config.is_temperature_mode) {
            current_stats = calculate_frame_stats(temp_data, thermal_width, thermal_height);
        }
        
        // Add text overlay
        std::vector<std::string> overlay_text;
        
        overlay_text.push_back("Mode: " + std::string(g_config.is_temperature_mode ? "Y16" : "YUY2"));
        overlay_text.push_back("FPS: " + std::to_string((int)current_fps));
        
        if (g_config.is_temperature_mode) {
            // Color mode info
            if (g_config.use_pseudocolor) {
                overlay_text.push_back("Color: " + std::string(color_mode_names[g_config.color_mode]));
            } else {
                overlay_text.push_back("Color: Grayscale");
            }
            
            // Point temperature
            float point_temp = get_point_temperature(temp_data,
                g_selected_point.x, g_selected_point.y, thermal_width, thermal_height);
            
            if (point_temp > -900) {
                std::stringstream ss;
                ss << "Point: " << std::fixed << std::setprecision(1) << point_temp << "C";
                overlay_text.push_back(ss.str());
            }
            
            // Statistics
            if (g_config.show_stats && current_stats.valid) {
                std::stringstream ss;
                ss << "Min: " << std::fixed << std::setprecision(1) << current_stats.min_temp_c << "C";
                overlay_text.push_back(ss.str());
                
                ss.str("");
                ss << "Max: " << std::fixed << std::setprecision(1) << current_stats.max_temp_c << "C";
                overlay_text.push_back(ss.str());
            }
        }
        
        // Draw text
        int y_offset = 25;
        for (const auto& text : overlay_text) {
            cv::putText(display_img, text, cv::Point(10, y_offset),
                       cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
            y_offset += 20;
        }
        
        // Show image
        cv::imshow("Thermal Camera", display_img);
        
        // Update FPS
        auto current_time = std::chrono::steady_clock::now();
        auto fps_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            current_time - last_fps_time);
        
        if (fps_elapsed.count() >= 1000) {
            current_fps = fps_counter * 1000.0f / fps_elapsed.count();
            fps_counter = 0;
            last_fps_time = current_time;
            
            auto total_elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                current_time - start_time);
            
            std::cout << "\rFrames: " << frame_count 
                     << " | Time: " << total_elapsed.count() << "s"
                     << " | FPS: " << std::fixed << std::setprecision(1) << current_fps
                     << " | Mode: " << (g_config.is_temperature_mode ? "Y16" : "YUY2")
                     << " | Color: " << (g_config.use_pseudocolor ? color_mode_names[g_config.color_mode] : "Grayscale")
                     << "        " << std::flush;
        }
        
        // Handle keyboard
        int key = cv::waitKey(1) & 0xFF;
        
        switch (key) {
            case 'q':
            case 27:  // ESC
                g_running = false;
                is_streaming = 0;
                break;
                
            case 't':
                if (g_config.is_temperature_mode) {
                    y16_preview_stop(PREVIEW_PATH0);
                    g_config.is_temperature_mode = false;
                    std::cout << "\nSwitched to YUY2 mode" << std::endl;
                } else {
                    if (switch_to_temperature_mode()) {
                        std::cout << "\nSwitched to Y16 temperature mode" << std::endl;
                    }
                }
                break;
                
            case 'p':
                g_config.use_pseudocolor = !g_config.use_pseudocolor;
                std::cout << "\nPseudocolor: " 
                         << (g_config.use_pseudocolor ? "ON" : "OFF") << std::endl;
                if (g_config.use_pseudocolor) {
                    std::cout << "Current palette: " 
                             << color_mode_names[g_config.color_mode] << std::endl;
                }
                break;
                
            case '[':  // Previous palette
                if (g_config.color_mode > 1) {
                    g_config.color_mode--;
                    if (g_config.color_mode == 2) g_config.color_mode = 1;  // Skip reserved
                } else {
                    g_config.color_mode = 11;
                }
                std::cout << "\nColor mode: " << color_mode_names[g_config.color_mode] << std::endl;
                break;
                
            case ']':  // Next palette
                if (g_config.color_mode < 11) {
                    g_config.color_mode++;
                    if (g_config.color_mode == 2) g_config.color_mode = 3;  // Skip reserved
                } else {
                    g_config.color_mode = 1;
                }
                std::cout << "\nColor mode: " << color_mode_names[g_config.color_mode] << std::endl;
                break;
                
            case '1':
                g_config.color_mode = 1;  // White Hot
                g_config.use_pseudocolor = true;
                std::cout << "\nColor: " << color_mode_names[1] << std::endl;
                break;
                
            case '3':
                g_config.color_mode = 3;  // Rainbow
                g_config.use_pseudocolor = true;
                std::cout << "\nColor: " << color_mode_names[3] << std::endl;
                break;
                
            case '6':
                g_config.color_mode = 6;  // Iron/Fire
                g_config.use_pseudocolor = true;
                std::cout << "\nColor: " << color_mode_names[6] << std::endl;
                break;
                
            case '9':
                g_config.color_mode = 9;  // Color 9
                g_config.use_pseudocolor = true;
                std::cout << "\nColor: " << color_mode_names[9] << std::endl;
                break;
                
            case '0':
                g_config.color_mode = 11;  // Black Hot
                g_config.use_pseudocolor = true;
                std::cout << "\nColor: " << color_mode_names[11] << std::endl;
                break;
                
            case 'c':
                g_config.show_crosshair = !g_config.show_crosshair;
                break;
                
            case 's':
                g_config.show_stats = !g_config.show_stats;
                break;
        }
    }
    
    // Cleanup
    std::cout << "\n\nCleaning up..." << std::endl;
    
    is_streaming = 0;
    
    if (g_config.is_temperature_mode) {
        y16_preview_stop(PREVIEW_PATH0);
    }
    
    delete[] temp_data;
    delete[] rgb_display;
    
    cv::destroyAllWindows();
    
    uvc_camera_stream_close(KEEP_CAM_SIDE_PREVIEW);
    uvc_frame_buf_release(frame_buffer);
    uvc_camera_close();
    usleep(500000);
    uvc_camera_release();
    
    // Stats
    auto total_time = std::chrono::steady_clock::now() - start_time;
    auto total_seconds = std::chrono::duration_cast<std::chrono::seconds>(total_time).count();
    
    std::cout << "\nSession summary:" << std::endl;
    std::cout << "  Total frames: " << frame_count << std::endl;
    std::cout << "  Total time: " << total_seconds << " seconds" << std::endl;
    if (total_seconds > 0) {
        std::cout << "  Average FPS: " << (frame_count / (float)total_seconds) << std::endl;
    }
    
    std::cout << "Application ended" << std::endl;
    
    return 0;
}