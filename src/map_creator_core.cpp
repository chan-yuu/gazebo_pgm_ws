#include "map_creator_core.h"
#include <fstream>
#include <iostream>
#include <cmath>
#include <boost/gil.hpp>
#include <boost/gil/extension/io/png.hpp>
#include <pcl/io/pcd_io.h>
#include <octomap/octomap.h>
#include <ignition/math/Vector3.hh>

namespace map_creator {

MapCreatorCore::MapCreatorCore() : world_(nullptr) {}

MapCreatorCore::~MapCreatorCore() {}

bool MapCreatorCore::initialize(gazebo::physics::WorldPtr world) {
    if (!world) {
        last_error_ = "Invalid world pointer";
        return false;
    }
    world_ = world;
    return true;
}

bool MapCreatorCore::createMap(const MapConfig& config) {
    if (!world_) {
        last_error_ = "World not initialized";
        return false;
    }

    // Calculate the size of the cubic area
    float size_x = config.upper_left.x - config.lower_right.x;
    float size_y = config.lower_right.y - config.upper_left.y;  // size_y to be -ve
    float size_z = config.upper_left.z - config.lower_right.z;
    
    // Check for coordinate validity
    if (size_x <= 0 || size_y >= 0 || size_z <= 0) {
        last_error_ = "Invalid coordinates: ensure upper_left.x > lower_right.x, "
                     "upper_left.y < lower_right.y, upper_left.z > lower_right.z";
        return false;
    }

    // Calculate the number of points in each dimension
    int num_points_x = static_cast<int>(std::abs(size_x) / config.resolution) + 1;
    int num_points_y = static_cast<int>(std::abs(size_y) / config.resolution) + 1;
    int num_points_z = static_cast<int>(std::abs(size_z) / config.resolution) + 1;
    
    // Calculate the step size in each dimension
    float step_x = size_x / num_points_x;
    float step_y = size_y / num_points_y;
    float step_z = size_z / num_points_z;   

    int dims = 6;

    if (config.skip_vertical_scan) {
        num_points_z = 2;
        step_z = size_z;
        dims = 4;
    }

    std::cout << "-----------------" << std::endl 
              << "Area Corners: (lower right, upper left)  (" 
              << config.lower_right.x << ", " << config.lower_right.y << ", " << config.lower_right.z << "), (" 
              << config.upper_left.x << ", " << config.upper_left.y << ", " << config.upper_left.z << ") " << std::endl 
              << "Area size : " << size_x << " x " << size_y << " x " << size_z << " (WxLxH)" << std::endl 
              << "Step size : " << step_x << ", " << step_y << ", " << step_z << " (stepx, stepy, stepz) " << std::endl 
              << "Resolution: (" << num_points_x << ", " << num_points_y << ", " << num_points_z << ") - " << config.resolution << std::endl 
              << "Map Mode: " << (config.skip_vertical_scan ? "Partial Scan" : "Full Scan") << std::endl 
              << "-----------------" << std::endl;

    // Create ray object for collision detection
    gazebo::physics::PhysicsEnginePtr engine = world_->Physics();
    engine->InitForThread();
    gazebo::physics::RayShapePtr ray =
        boost::dynamic_pointer_cast<gazebo::physics::RayShape>(
        engine->CreateShape("ray", gazebo::physics::CollisionPtr()));

    // Pixel values near to 0 means occupied and 255 means empty
    boost::gil::gray8_pixel_t fill(255 - config.threshold_2d);
    boost::gil::gray8_pixel_t blank(255);
    boost::gil::gray8_image_t image(num_points_x, num_points_y);

    // Initially fill all area with empty pixel value
    boost::gil::fill_pixels(image._view, blank);

    // Create a point cloud object
    pcl::PointCloud<pcl::PointXYZ> cloud;
    std::string entityName;
    ignition::math::Vector3d start, end;
    double dx, dy, dz, dist;
    cloud.width = num_points_x;
    cloud.height = num_points_y;
        
    struct PointMask {
        int x, y, z;
    };
    PointMask directions[6] = {
        {-1, 0, 0}, // Left
        {1, 0, 0},  // Right
        {0, 1, 0},  // Front
        {0, -1, 0}, // Back
        {0, 0, 1},  // Top
        {0, 0, -1}  // Bottom
    };

    for (int x = 0; x < num_points_x; ++x) {
        std::cout << "\rPercent complete: " << x * 100.0 / num_points_x << "%       " << std::flush;
        
        double cur_x = config.lower_right.x + x * step_x; 
        for (int y = 0; y < num_points_y; ++y) {
            double cur_y = config.upper_left.y + y * step_y;    
            
            ignition::math::Vector3d startV(cur_x, cur_y, config.lower_right.z);
            ignition::math::Vector3d endV(cur_x, cur_y, config.upper_left.z);
            
            // Detect collision upward until top of area. z direction
            ray->SetPoints(startV, endV);
            ray->GetIntersection(dist, entityName);              
            if (!entityName.empty()) { 
                image._view(x, y) = fill;                      
            }

            // Walk in z direction to check each point for any collision to its neighbours
            for (int z = 0; z < num_points_z; ++z) {
                double cur_z = config.lower_right.z + z * step_z;
                ignition::math::Vector3d start(cur_x, cur_y, cur_z);

                // Check for right, left, front, back, top and bottom points for collision.
                // 2d mode checks for right, left, front and back. see dims assignment.
                for (int i = 0; i < dims; ++i) {
                    dx = directions[i].x * step_x * config.range_multiplier;
                    dy = directions[i].y * step_y * config.range_multiplier;
                    dz = directions[i].z * step_z * config.range_multiplier;
                    ignition::math::Vector3d end(cur_x + dx, cur_y + dy, cur_z + dz);
                    
                    ray->SetPoints(start, end);
                    ray->GetIntersection(dist, entityName);
                    if (!entityName.empty()) { 
                        // Collision found. Push point to cloud and set in image
                        cloud.push_back(pcl::PointXYZ(cur_x, cur_y, cur_z));
                        image._view(x, y) = fill;
                        break;
                    }            
                }
            }
        }
    }
   
    std::cout << std::endl << "Completed calculations, writing to files" << std::endl;
    
    if (!config.output_filename.empty()) { 
        if (cloud.size() > 0) {
            // Save pcd file
            pcl::io::savePCDFileASCII(config.output_filename + ".pcd", cloud);
        
            // Save octomap file
            octomap::OcTree octree(config.resolution);
            for (auto p : cloud.points)
                octree.updateNode(octomap::point3d(p.x, p.y, p.z), true);
            octree.updateInnerOccupancy();
            octree.writeBinary(config.output_filename + ".bt");
        }

        // Save png file
        boost::gil::gray8_view_t view = image._view;
        boost::gil::write_view(config.output_filename + ".png", view, boost::gil::png_tag());

        // Convert view to vector for pgm writing
        std::vector<std::vector<uint8_t>> image_data(view.width(), 
                                                      std::vector<uint8_t>(view.height()));
        for (int x = 0; x < view.width(); ++x) {
            for (int y = 0; y < view.height(); ++y) {
                image_data[x][y] = view(x, y)[0];
            }
        }
        
        // Save pgm file
        writePgmFile(config.output_filename, image_data, view.width(), view.height());

        // Write down yaml file for nav2 usage.
        writeYamlFile(config.output_filename, config);

        std::cout << "Output location: " << config.output_filename 
                  << "[.pcd, .bt, .pgm, .png, .yaml]" << std::endl;
    }

    std::cout << std::endl;
    return true;
}

void MapCreatorCore::writeYamlFile(const std::string& filename, const MapConfig& config) {
    std::ofstream outputFile(filename + ".yaml");
    if (outputFile.is_open()) {
        outputFile << "image: " << filename << ".pgm" << std::endl;
        outputFile << "mode: trinary" << std::endl;
        outputFile << "resolution: " << config.resolution << std::endl;
        outputFile << "origin: [" << config.lower_right.x << ", " 
                   << config.lower_right.y << ", 0.0]" << std::endl;
        outputFile << "negate: 0" << std::endl;
        outputFile << "occupied_thresh: 0.95" << std::endl;
        outputFile << "free_thresh: 0.90" << std::endl;
        outputFile.close();
    } else {
        std::cerr << "Unable to open yaml file for writing." << std::endl;
    }
}

void MapCreatorCore::writePgmFile(const std::string& filename, 
                                   const std::vector<std::vector<uint8_t>>& image_data,
                                   int width, int height) {
    std::ofstream ofs(filename + ".pgm");
    if (!ofs.is_open()) {
        std::cerr << "Unable to open pgm file for writing." << std::endl;
        return;
    }

    ofs << "P2" << '\n';          // grayscale
    ofs << width << ' ' << height << '\n'; // width and height
    ofs << 255 << '\n';            // max value
    
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            ofs << (int)image_data[x][y] << ' ';
        }
        ofs << '\n';
    }
    ofs.close();
}

} // namespace map_creator
