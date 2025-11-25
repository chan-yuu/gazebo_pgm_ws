#include <fstream>
#include <iostream>
#include <cmath>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include "gazebo/gazebo.hh"
#include "gazebo/common/common.hh"
#include "gazebo/physics/physics.hh"
#include "map_creator_core.h"

namespace gazebo {

struct MapRequest {
    map_creator::MapConfig config;
    bool processed;
    bool success;
    
    MapRequest() : processed(false), success(false) {}
};

class GazeboMapCreatorPlugin : public SystemPlugin {
private:
    physics::WorldPtr world_;
    event::ConnectionPtr world_created_event_;
    map_creator::MapCreatorCore map_creator_;
    
    std::queue<MapRequest> request_queue_;
    std::mutex queue_mutex_;
    std::condition_variable queue_cv_;
    std::thread worker_thread_;
    bool shutdown_;

public:
    GazeboMapCreatorPlugin() : shutdown_(false) {}
    
    ~GazeboMapCreatorPlugin() {
        shutdown_ = true;
        queue_cv_.notify_all();
        if (worker_thread_.joinable()) {
            worker_thread_.join();
        }
    }

    void Load(int /*argc*/, char ** /*argv*/) override {
        std::cout << "Gazebo Map Creator Plugin Loaded (Standalone Version)" << std::endl;
        
        // Get a callback when a world is created
        this->world_created_event_ = event::Events::ConnectWorldCreated(
            std::bind(&GazeboMapCreatorPlugin::OnWorldCreated, this, std::placeholders::_1));
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
        
        // Start worker thread for processing requests
        worker_thread_ = std::thread(&GazeboMapCreatorPlugin::ProcessRequests, this);
        
        std::cout << "Map Creator Plugin ready. Use map_creator_cli to generate maps." << std::endl;
    }
    
    void ProcessRequests() {
        while (!shutdown_) {
            std::unique_lock<std::mutex> lock(queue_mutex_);
            queue_cv_.wait(lock, [this] { return !request_queue_.empty() || shutdown_; });
            
            if (shutdown_) break;
            
            if (!request_queue_.empty()) {
                MapRequest req = request_queue_.front();
                request_queue_.pop();
                lock.unlock();
                
                // Process the request
                req.success = map_creator_.createMap(req.config);
                req.processed = true;
                
                if (!req.success) {
                    std::cerr << "Map creation failed: " 
                              << map_creator_.getLastError() << std::endl;
                }
            }
        }
    }
    
    bool CreateMapSync(const map_creator::MapConfig& config) {
        return map_creator_.createMap(config);
    }
};

// Register this plugin with the simulator
GZ_REGISTER_SYSTEM_PLUGIN(GazeboMapCreatorPlugin)

} // namespace gazebo
