#include <fstream>
#include <iostream>
#include <cmath>
#include <thread>
#include <chrono>
#include "gazebo/gazebo.hh"
#include "gazebo/common/common.hh"
#include "gazebo/physics/physics.hh"
#include "map_creator_core.h"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

namespace gazebo {

/**
 * Auto-generating map creator plugin
 * Reads configuration from a JSON file and automatically generates map on world load
 */
class GazeboMapCreatorAutoPlugin : public SystemPlugin {
private:
    physics::WorldPtr world_;
    event::ConnectionPtr world_created_event_;
    event::ConnectionPtr update_connection_;
    map_creator::MapCreatorCore map_creator_;
    bool map_generated_;
    int update_count_;
    std::string config_file_path_;

public:
    GazeboMapCreatorAutoPlugin() : map_generated_(false), update_count_(0) {
        // Look for config file in current directory or environment variable
        const char* config_env = std::getenv("GAZEBO_MAP_CONFIG");
        if (config_env) {
            config_file_path_ = config_env;
        } else {
            config_file_path_ = "./map_config.json";
        }
    }

    void Load(int /*argc*/, char ** /*argv*/) override {
        std::cout << "========================================" << std::endl;
        std::cout << "Gazebo Map Creator Auto Plugin Loaded" << std::endl;
        std::cout << "========================================" << std::endl;
        std::cout << "Config file: " << config_file_path_ << std::endl;
        
        // Get a callback when a world is created
        this->world_created_event_ = event::Events::ConnectWorldCreated(
            std::bind(&GazeboMapCreatorAutoPlugin::OnWorldCreated, this, std::placeholders::_1));
    }

    void OnWorldCreated(const std::string& _world_name) {
        world_created_event_.reset();
        world_ = physics::get_world(_world_name);
        
        if (!world_) {
            std::cerr << "Failed to get world: " << _world_name << std::endl;
            return;
        }
        
        std::cout << "World created: " << _world_name << std::endl;
        
        // Initialize map creator
        if (!map_creator_.initialize(world_)) {
            std::cerr << "Failed to initialize map creator: " 
                      << map_creator_.getLastError() << std::endl;
            return;
        }
        
        // Connect to world update event to generate map after world is fully loaded
        update_connection_ = event::Events::ConnectWorldUpdateBegin(
            std::bind(&GazeboMapCreatorAutoPlugin::OnUpdate, this));
        
        std::cout << "Waiting for world to stabilize before generating map..." << std::endl;
    }
    
    void OnUpdate() {
        update_count_++;
        
        // Wait for 100 updates (~10 seconds at 10Hz) to ensure world is loaded
        if (!map_generated_ && update_count_ > 100) {
            update_connection_.reset();  // Disconnect from updates
            
            std::cout << "\nWorld stabilized. Starting map generation..." << std::endl;
            
            // Read configuration
            map_creator::MapConfig config = ReadConfig();
            
            // Generate map
            if (map_creator_.createMap(config)) {
                std::cout << "\n========================================" << std::endl;
                std::cout << "Map generation completed successfully!" << std::endl;
                std::cout << "========================================" << std::endl;
                map_generated_ = true;
                
                // Shutdown Gazebo after generation
                std::cout << "\nShutting down Gazebo in 3 seconds..." << std::endl;
                std::this_thread::sleep_for(std::chrono::seconds(3));
                gazebo::shutdown();
            } else {
                std::cerr << "Map generation failed: " 
                          << map_creator_.getLastError() << std::endl;
            }
        }
    }
    
    map_creator::MapConfig ReadConfig() {
        map_creator::MapConfig config;
        
        // Try to read from JSON file
        if (std::ifstream(config_file_path_).good()) {
            try {
                boost::property_tree::ptree pt;
                boost::property_tree::read_json(config_file_path_, pt);
                
                config.lower_right.x = pt.get<double>("lower_right.x", -10.0);
                config.lower_right.y = pt.get<double>("lower_right.y", -10.0);
                config.lower_right.z = pt.get<double>("lower_right.z", 0.05);
                
                config.upper_left.x = pt.get<double>("upper_left.x", 10.0);
                config.upper_left.y = pt.get<double>("upper_left.y", 10.0);
                config.upper_left.z = pt.get<double>("upper_left.z", 10.0);
                
                config.resolution = pt.get<double>("resolution", 0.01);
                config.range_multiplier = pt.get<double>("range_multiplier", 0.55);
                config.threshold_2d = pt.get<int>("threshold_2d", 255);
                config.skip_vertical_scan = pt.get<bool>("skip_vertical_scan", false);
                config.output_filename = pt.get<std::string>("output_filename", "map");
                
                std::cout << "Configuration loaded from: " << config_file_path_ << std::endl;
            } catch (const std::exception& e) {
                std::cerr << "Error reading config file: " << e.what() << std::endl;
                std::cerr << "Using default configuration" << std::endl;
            }
        } else {
            std::cout << "Config file not found, using default configuration" << std::endl;
            std::cout << "Create " << config_file_path_ << " to customize settings" << std::endl;
        }
        
        return config;
    }
};

// Register this plugin with the simulator
GZ_REGISTER_SYSTEM_PLUGIN(GazeboMapCreatorAutoPlugin)

} // namespace gazebo
