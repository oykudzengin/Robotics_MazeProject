#include <ros/ros.h>
#include <silver_fundamentals/Com.h>
#include "silver_fundamentals/MoveToPosition.h"
#include <vector>
#include <cstdint>
#include <LocalizeCommunication.h>


#include <playsong.h>
#include <fstream>
#include <string>
#include <regex>
#include <utility>
#include <stdexcept>

// Helper: parses a file containing coordinates in the form [[x1,y1], [x2,y2], ...]
void parseCoordinates(const std::string& filename,
                      std::vector<std::pair<int,int>>& coords)
{
    std::ifstream ifs(filename);
    if (!ifs.is_open()) {
        throw std::runtime_error("Cannot open file: " + filename);
    }

    std::string content((std::istreambuf_iterator<char>(ifs)),
                        std::istreambuf_iterator<char>());

    std::regex pairRegex(R"(\[\s*(-?\d+)\s*,\s*(-?\d+)\s*\])");
    std::smatch match;
    auto begin = content.cbegin();
    auto end   = content.cend();

    while (std::regex_search(begin, end, match, pairRegex)) {
        int x = std::stoi(match[1].str());
        int y = std::stoi(match[2].str());
        coords.emplace_back(x, y);
        begin = match.suffix().first;
    }
}

// Main loader: fills gold_locations and pickups from their respective files
void loadWaypoints(const std::string& goldFile,
                   const std::string& pickupsFile,
                   std::vector<std::pair<int,int>>& gold_locations,
                   std::vector<std::pair<int,int>>& pickups)
{
    gold_locations.clear();
    pickups.clear();

    parseCoordinates(goldFile,    gold_locations);
    parseCoordinates(pickupsFile, pickups);
}

// localaise communication to execute plan server 
namespace silver_fundamentals {
    PlanSuccessState success_state = PlanSuccessState::NONE;
    std::vector<geometry_msgs::Point> com_waypoints;
    bool plan_exits = false;
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "full_run_server");
    ros::NodeHandle nh;

    // Read in files and create waypoint vectors
    std::vector<std::pair<int,int>> gold_locations;
    std::vector<std::pair<int,int>> pickups;
    loadWaypoints("src/silver_fundamentals/maps/gold.txt", "src/silver_fundamentals/maps/pickup.txt", gold_locations, pickups);

    // Combine gold and pickups into goals
    std::vector<std::vector<uint8_t>> goals;
    std::vector<std::vector<uint8_t>> pickup_points;
    for (const auto &p : gold_locations) {
        goals.push_back({static_cast<uint8_t>(p.first), static_cast<uint8_t>(p.second)});
    }
    for (const auto &p : pickups) {
        pickup_points.push_back({static_cast<uint8_t>(p.first),static_cast<uint8_t>(p.second)});
    }
    //goals.push_back({static_cast<uint8_t>(pickups[0].first), static_cast<uint8_t>(pickups[0].second)});


    ros::ServiceClient comm_client = nh.serviceClient<silver_fundamentals::Com>("comm");
    silver_fundamentals::Com comm_srv;

    for (const auto &goal: goals) {

        bool success = false;
        // do this as long as we (might) fail
        while (!success) {
            // set request
            comm_srv.request.operation = silver_fundamentals::Com::Request::SET_DATA;
            comm_srv.request.success_state = static_cast<uint8_t>(silver_fundamentals::PlanSuccessState::NONE);
            comm_srv.request.goal_exists   = true;
            comm_srv.request.goal = goal;

            //send request
            while (!comm_client.call(comm_srv))
                ROS_ERROR("execute_plan_server: failed to call comm service for SET_DATA");

            // wait for execution
            comm_srv.request.operation = silver_fundamentals::Com::Request::GET_DATA;
            while (ros::ok() && (!comm_client.call(comm_srv) || static_cast<silver_fundamentals::PlanSuccessState>(comm_srv.response.success_state) == silver_fundamentals::PlanSuccessState::NONE)) {
                ROS_INFO("waiting for client to execute a plan...");
                ros::Duration(0.1).sleep();
            }

            // check success and maybe redo
            success = (static_cast<silver_fundamentals::PlanSuccessState>(comm_srv.response.success_state) == silver_fundamentals::PlanSuccessState::PLAN_DONE);
        }

        // play song
        silver_fundamentals::playSong3(nh);
        ros::Duration(5).sleep();
    }

    bool success = false;
     int current_pickup = 0;
    // do this as long as we (might) fail
    while (!success) {
        // set request
        comm_srv.request.operation = silver_fundamentals::Com::Request::SET_DATA;
        comm_srv.request.success_state = static_cast<uint8_t>(silver_fundamentals::PlanSuccessState::NONE);
        comm_srv.request.goal_exists   = true;
        comm_srv.request.goal = pickup_points[current_pickup];

        //send request
        while (!comm_client.call(comm_srv))
            ROS_ERROR("execute_plan_server: failed to call comm service for SET_DATA");

        // wait for execution
        comm_srv.request.operation = silver_fundamentals::Com::Request::GET_DATA;
        while (ros::ok() && (!comm_client.call(comm_srv) || static_cast<silver_fundamentals::PlanSuccessState>(comm_srv.response.success_state) == silver_fundamentals::PlanSuccessState::NONE)) {
            ROS_INFO("waiting for client to execute a plan...");
            ros::Duration(0.1).sleep();
        }

        // check success and maybe redo
        success = (static_cast<silver_fundamentals::PlanSuccessState>(comm_srv.response.success_state) == silver_fundamentals::PlanSuccessState::PLAN_DONE);
        current_pickup = (current_pickup + 1) % pickup_points.size();
    }

    // play song
    silver_fundamentals::playSong3(nh);
    ros::Duration(5).sleep();

    return 0;
}
