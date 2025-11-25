#include <iostream>
#include <string>
#include <cstring>
#include <gazebo/gazebo_client.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include "map_creator_core.h"

void printUsage(const char* program_name) {
    std::cout << "Usage: " << program_name << " [OPTIONS]\n"
              << "\nOptions:\n"
              << "  -c, --corners <coords>    Lower right and upper left corners\n"
              << "                            Format: (x1,y1,z1)(x2,y2,z2)\n"
              << "                            Default: (-10.0,-10.0,0.05)(10.0,10.0,10.0)\n"
              << "  -r, --resolution <value>  Map resolution (lower = higher resolution)\n"
              << "                            Default: 0.01\n"
              << "  -d, --multiplier <value>  Collision distance multiplier\n"
              << "                            Default: 0.55\n"
              << "  -t, --threshold <value>   Pixel threshold for 2D map (0-255)\n"
              << "                            Default: 255\n"
              << "  -f, --filename <path>     Output file base name\n"
              << "                            Default: map\n"
              << "  --skip-vertical-scan      Skip full scan (faster, 2D focused)\n"
              << "  -h, --help                Show this help message\n"
              << "\nExample:\n"
              << "  " << program_name << " -c '(-4.8,-4.5,0.03)(4.8,4.5,8.0)' -r 0.01 -f mymap\n"
              << std::endl;
}

bool parseCoordinates(const std::string& coords, 
                      map_creator::Point3D& p1, 
                      map_creator::Point3D& p2) {
    // Parse format: (x1,y1,z1)(x2,y2,z2)
    size_t pos1 = coords.find('(');
    size_t pos2 = coords.find(')');
    size_t pos3 = coords.find('(', pos2);
    size_t pos4 = coords.find(')', pos3);
    
    if (pos1 == std::string::npos || pos2 == std::string::npos ||
        pos3 == std::string::npos || pos4 == std::string::npos) {
        return false;
    }
    
    std::string coord1 = coords.substr(pos1 + 1, pos2 - pos1 - 1);
    std::string coord2 = coords.substr(pos3 + 1, pos4 - pos3 - 1);
    
    // Parse first coordinate
    size_t comma1 = coord1.find(',');
    size_t comma2 = coord1.find(',', comma1 + 1);
    if (comma1 == std::string::npos || comma2 == std::string::npos) return false;
    
    p1.x = std::stod(coord1.substr(0, comma1));
    p1.y = std::stod(coord1.substr(comma1 + 1, comma2 - comma1 - 1));
    p1.z = std::stod(coord1.substr(comma2 + 1));
    
    // Parse second coordinate
    comma1 = coord2.find(',');
    comma2 = coord2.find(',', comma1 + 1);
    if (comma1 == std::string::npos || comma2 == std::string::npos) return false;
    
    p2.x = std::stod(coord2.substr(0, comma1));
    p2.y = std::stod(coord2.substr(comma1 + 1, comma2 - comma1 - 1));
    p2.z = std::stod(coord2.substr(comma2 + 1));
    
    return true;
}

int main(int argc, char** argv) {
    map_creator::MapConfig config;
    
    // Default coordinates
    config.lower_right = map_creator::Point3D(-10.0, -10.0, 0.05);
    config.upper_left = map_creator::Point3D(10.0, 10.0, 10.0);
    
    // Parse command line arguments
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        
        if (arg == "-h" || arg == "--help") {
            printUsage(argv[0]);
            return 0;
        }
        else if ((arg == "-c" || arg == "--corners") && i + 1 < argc) {
            if (!parseCoordinates(argv[++i], config.lower_right, config.upper_left)) {
                std::cerr << "Error: Invalid coordinate format" << std::endl;
                printUsage(argv[0]);
                return 1;
            }
        }
        else if ((arg == "-r" || arg == "--resolution") && i + 1 < argc) {
            config.resolution = std::stod(argv[++i]);
        }
        else if ((arg == "-d" || arg == "--multiplier") && i + 1 < argc) {
            config.range_multiplier = std::stod(argv[++i]);
        }
        else if ((arg == "-t" || arg == "--threshold") && i + 1 < argc) {
            config.threshold_2d = std::stoi(argv[++i]);
        }
        else if ((arg == "-f" || arg == "--filename") && i + 1 < argc) {
            config.output_filename = argv[++i];
        }
        else if (arg == "--skip-vertical-scan") {
            config.skip_vertical_scan = true;
        }
        else {
            std::cerr << "Unknown argument: " << arg << std::endl;
            printUsage(argv[0]);
            return 1;
        }
    }
    
    // Print configuration
    std::cout << "\nMap Creator Configuration:" << std::endl;
    std::cout << "  Lower Right: (" << config.lower_right.x << ", " 
              << config.lower_right.y << ", " << config.lower_right.z << ")" << std::endl;
    std::cout << "  Upper Left:  (" << config.upper_left.x << ", " 
              << config.upper_left.y << ", " << config.upper_left.z << ")" << std::endl;
    std::cout << "  Resolution: " << config.resolution << std::endl;
    std::cout << "  Range Multiplier: " << config.range_multiplier << std::endl;
    std::cout << "  Threshold: " << config.threshold_2d << std::endl;
    std::cout << "  Skip Vertical Scan: " << (config.skip_vertical_scan ? "Yes" : "No") << std::endl;
    std::cout << "  Output Filename: " << config.output_filename << std::endl;
    std::cout << std::endl;
    
    std::cout << "Note: This CLI tool requires Gazebo to be running with the map creator plugin loaded." << std::endl;
    std::cout << "Start Gazebo with: gazebo -s libgazebo_map_creator_plugin.so world_file.world" << std::endl;
    std::cout << "\nFor standalone usage without running Gazebo, use the Python GUI application." << std::endl;
    
    return 0;
}
