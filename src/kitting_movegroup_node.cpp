//ros
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
//moveit
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/ApplyPlanningScene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
//standard library
#include <math.h>
#include <vector>
//custom
#include "competition.h"
#include "arm.h"
#include "agv.h"
#include "utils.h"
#include "nist_gear/Product.h"

/**
 * @brief Helper function to check if an item is in a vector
 *
 * @tparam T
 * @param vec Vector to check
 * @param elem Item to check is inside the vector
 * @return true The item is in the vector
 * @return false The item is not in the vector
 */
template <typename T>
bool contains(std::vector<T> vec, const T& elem)
{
    bool result = false;
    if (find(vec.begin(), vec.end(), elem) != vec.end()) {
        result = true;
    }
    return result;
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "rwa3_node_cpp");
    ros::NodeHandle node_handle;

    // 0 means that the spinner will use as many threads as there are processors on your machine.
    //If you use 3 for example, only 3 threads will be used.
    ros::AsyncSpinner spinner(0);
    spinner.start();

    // start the competition
    motioncontrol::Competition competition(node_handle);
    competition.init();


    // create an instance of the kitting arm
    motioncontrol::Arm arm(node_handle);
    arm.init();

    // Wait for order to be recieved
    ros::Rate rate = 2;
    while (competition.getKittingShipments().size() == 0) {
        ROS_INFO_STREAM("Waiting for order");
        rate.sleep();
    }

    // if we get to this point it means we started receiving orders
    auto kitting_shipments = competition.getKittingShipments();
    arm.goToPresetLocation("home1");
    arm.goToPresetLocation("home2");

    // this variable will be used to build frame names (see line 142)
    int counter{};
    // parse each kitting shipment
    for (const auto& kitting_shipment : kitting_shipments) {
        counter++;
        // list of different product types
        // if the shipment has 2 blue batteries then only 1 blue
        // battery is placed in this list.
        // the list will look like this:
        // [('assembly_pump_red', nist_gear::Product), ('assembly_battery_blue', nist_gear::Product)]
        std::vector<std::pair<std::string, nist_gear::Product>> product_list{};

        // Let's wait 10 s
        // data should be publishing after 5 seconds
        double outside_time = ros::Time::now().toSec();
        double inside_time = ros::Time::now().toSec();
        while (inside_time - outside_time < 5.0) {
            inside_time = ros::Time::now().toSec();
            // ROS_INFO_STREAM("Waiting for data");
        }

        auto camera1_data = competition.get_logical_camera1_data();
        auto camera2_data = competition.get_logical_camera2_data();

        if ((camera1_data.first.compare("") == 0) && (camera1_data.first.compare("") == 0)) {
            ROS_FATAL_STREAM("No data reported by any logical camera");
            ros::shutdown();
            return 1;
        }

        ROS_INFO_STREAM("Received data from camera");
        
        std::vector<std::pair<nist_gear::Product, std::string> > camera_for_product{};
        // If we got here it means cameras got some data
        // check which camera found products needed in this shipment
        for (const auto& product : kitting_shipment.products) {
            if (camera1_data.first.compare(product.type) == 0) {
                std::pair<nist_gear::Product, std::string> entry;
                entry.first = product;
                entry.second = camera1_data.second;
                camera_for_product.emplace_back(entry);
            }
            else if (camera2_data.first.compare(product.type) == 0) {
                std::pair<nist_gear::Product, std::string> entry;
                entry.first = product;
                entry.second = camera2_data.second;
                camera_for_product.emplace_back(entry);
            }
            else {
                ROS_FATAL_STREAM("No matching part found by any logical camera");
                ros::shutdown();
                return 1;
            }
        }

        if (camera_for_product.empty()) {
            ROS_FATAL_STREAM("Product not found by camera");
            ros::shutdown();
            return 1;
        }

        // keep track of how many products have been placed in this shipment
        int product_placed_in_shipment{};
        for (auto iter : camera_for_product) {
            std::string part_frame = iter.second + "_" + iter.first.type + "_" + std::to_string(counter)+ "_frame";
            arm.movePart(iter.first.type, part_frame, iter.first.pose, kitting_shipment.agv_id);
            product_placed_in_shipment++;
            // if we have placed all products in this shipment then ship the AGV
            if (product_placed_in_shipment == kitting_shipment.products.size()) {
                ros::Duration(sleep(1.0));
                motioncontrol::Agv agv{ node_handle, kitting_shipment.agv_id };
                if (agv.getAGVStatus()) {
                    agv.shipAgv(kitting_shipment.shipment_type, kitting_shipment.station_id);
                }
            }
        }
        
        
    }


    ros::waitForShutdown();
}