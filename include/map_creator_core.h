#ifndef MAP_CREATOR_CORE_H
#define MAP_CREATOR_CORE_H

#include <string>
#include <memory>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace map_creator {

struct Point3D {
    double x, y, z;
    Point3D(double x_ = 0.0, double y_ = 0.0, double z_ = 0.0) 
        : x(x_), y(y_), z(z_) {}
};

struct MapConfig {
    Point3D lower_right;
    Point3D upper_left;
    double resolution;
    double range_multiplier;
    int threshold_2d;
    bool skip_vertical_scan;
    std::string output_filename;
    
    MapConfig() 
        : resolution(0.01)
        , range_multiplier(0.55)
        , threshold_2d(255)
        , skip_vertical_scan(false)
        , output_filename("map") {}
};

class MapCreatorCore {
public:
    MapCreatorCore();
    ~MapCreatorCore();
    
    // Initialize with Gazebo world
    bool initialize(gazebo::physics::WorldPtr world);
    
    // Create map with given configuration
    bool createMap(const MapConfig& config);
    
    // Get last error message
    std::string getLastError() const { return last_error_; }
    
private:
    void writeYamlFile(const std::string& filename, const MapConfig& config);
    void writePgmFile(const std::string& filename, 
                      const std::vector<std::vector<uint8_t>>& image_data,
                      int width, int height);
    
    gazebo::physics::WorldPtr world_;
    std::string last_error_;
};

} // namespace map_creator

#endif // MAP_CREATOR_CORE_H
