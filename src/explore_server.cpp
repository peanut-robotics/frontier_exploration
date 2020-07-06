#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>

#include <geometry_msgs/PolygonStamped.h>

#include <frontier_exploration/ExploreTaskAction.h>
#include <frontier_exploration/GetNextFrontier.h>
#include <frontier_exploration/UpdateBoundaryPolygon.h>

#include <tf/transform_listener.h>

#include <move_base_msgs/MoveBaseAction.h>

#include <frontier_exploration/geometry_tools.h>

namespace frontier_exploration{

/**
 * @brief Server for frontier exploration action, runs the state machine associated with a
 * structured frontier exploration task and manages robot movement through move_base.
 */
class FrontierExplorationServer
{

public:

    /**
     * @brief Constructor for the server, sets up this node's ActionServer for exploration and ActionClient to move_base for robot movement.
     * @param name Name for SimpleActionServer
     */
    FrontierExplorationServer(std::string name) :
        tf_listener_(ros::Duration(10.0)),
        private_nh_("~")
    {
        explore_costmap_ros_ = boost::shared_ptr<costmap_2d::Costmap2DROS>(new costmap_2d::Costmap2DROS("explore_costmap", tf_listener_));

        // Plugin service clients 
        update_boundary_polygon_client_ = private_nh_.serviceClient<frontier_exploration::UpdateBoundaryPolygon>("explore_costmap/explore_boundary/update_boundary_polygon");
        get_next_frontier_client_ = private_nh_.serviceClient<frontier_exploration::GetNextFrontier>("explore_costmap/explore_boundary/get_next_frontier");
        if (!update_boundary_polygon_client_.waitForExistence(ros::Duration(3))){
            ROS_ERROR("Could not find update_boundary_polygon ");
            return;
        }
        if (!get_next_frontier_client_.waitForExistence(ros::Duration(3))){
            ROS_ERROR("Could not find update_boundary_polygon");
            return;
        }

        // Start service servers 
        update_polygon_server_=  private_nh_.advertiseService("update_polygon", &FrontierExplorationServer::updatePolygonCb, this);
        get_next_frontier_server_=  private_nh_.advertiseService("get_next_frontier", &FrontierExplorationServer::getNextFrontierCb, this);

        explore_costmap_ros_->resetLayers();
        explore_boundary_set_ = false;

        ROS_INFO("Done");

    }

private:

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    tf::TransformListener tf_listener_;

    boost::shared_ptr<costmap_2d::Costmap2DROS> explore_costmap_ros_;
    geometry_msgs::PolygonStamped current_explore_boundary_;
    bool explore_boundary_set_;

    //  Service clients
    ros::ServiceClient update_boundary_polygon_client_;
    ros::ServiceClient get_next_frontier_client_;

    //  Service servers
    ros::ServiceServer update_polygon_server_;
    ros::ServiceServer get_next_frontier_server_;

    /**
     * @brief Update polygon boundary for explore server
     */
    
    bool updatePolygonCb(   frontier_exploration::UpdateBoundaryPolygon::Request& req, 
                            frontier_exploration::UpdateBoundaryPolygon::Response& res){
        ROS_INFO("Here");
        bool success;
        ROS_INFO("H 1");
        frontier_exploration::UpdateBoundaryPolygon srv;
        ROS_INFO("H 2");
        srv.request.explore_boundary = req.explore_boundary;
        ROS_INFO("H 4");
        if(update_boundary_polygon_client_.call(srv)){
            ROS_INFO("Sending");
            current_explore_boundary_ = req.explore_boundary;
            success = true;
            ROS_INFO("Region boundary set");
        }else{
            ROS_ERROR("Failed to set region boundary");
            success = false;
            
        }

        res.success = success;
        return true;
    }

     /**
     * @brief Get next frontier to explore
     */
    
    bool getNextFrontierCb( frontier_exploration::GetNextFrontier::Request& server_req, 
                            frontier_exploration::GetNextFrontier::Response& server_res){
        frontier_exploration::GetNextFrontier next_frontier_srv;
        geometry_msgs::PoseStamped goal_pose; //placeholder for next goal to be sent to move base
        tf::Stamped<tf::Pose> robot_pose;
        geometry_msgs::PoseStamped eval_pose;
        bool robot_in_polygon;
        bool success;

        // Check if boundary is set 
        if (!explore_boundary_set_){
            server_res.error_code = frontier_exploration::GetNextFrontier::Response::BOUNDARY_NOT_SET;
            return true;
        }

        // get current robot pose in frame of exploration boundary
        explore_costmap_ros_->getRobotPose(robot_pose);
        tf::poseStampedTFToMsg(robot_pose,next_frontier_srv.request.start_pose);

        // evaluate if robot is within exploration boundary using robot_pose in boundary frame
        eval_pose = next_frontier_srv.request.start_pose;
        if(eval_pose.header.frame_id != current_explore_boundary_.header.frame_id){
            tf_listener_.transformPose(current_explore_boundary_.header.frame_id, next_frontier_srv.request.start_pose, eval_pose);
        }

        //check if robot is not within exploration boundary and needs to return to center of search area
        robot_in_polygon = current_explore_boundary_.polygon.points.size() > 0 && !pointInPolygon(eval_pose.pose.position, current_explore_boundary_.polygon);
        if(!robot_in_polygon){
            ROS_ERROR("Robot not in exploration boundary, return to center");
        }
        else{ 
            // If in boundary, try to find next frontier to search
            if(get_next_frontier_client_.call(next_frontier_srv)){
                ROS_DEBUG("Found frontier to explore");
                goal_pose = next_frontier_srv.response.next_frontier;
                server_res.error_code = frontier_exploration::GetNextFrontier::Response::SUCCESS;
                server_res.next_frontier = goal_pose;
            }
            else{ 
                ROS_DEBUG("Couldn't find a frontier");
                server_res.error_code = frontier_exploration::GetNextFrontier::Response::NO_FRONTIER;
            }
        }

        return true;
    }

};

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "explore_server");

    frontier_exploration::FrontierExplorationServer server(ros::this_node::getName());
    ros::spin();
    return 0;
}
