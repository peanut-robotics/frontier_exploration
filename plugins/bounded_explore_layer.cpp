#include <frontier_exploration/bounded_explore_layer.h>

#include <pluginlib/class_list_macros.h>
#include <geometry_msgs/PolygonStamped.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/footprint.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>

#include <frontier_exploration/Frontier.h>
#include <frontier_exploration/UpdateBoundaryPolygon.h>
#include <frontier_exploration/GetNextFrontier.h>
#include <frontier_exploration/frontier_search.h>
#include <frontier_exploration/geometry_tools.h>
#include <pluginlib/class_loader.hpp>

PLUGINLIB_EXPORT_CLASS(frontier_exploration::BoundedExploreLayer, costmap_2d::Layer)

namespace frontier_exploration
{

    using costmap_2d::LETHAL_OBSTACLE;
    using costmap_2d::NO_INFORMATION;
    using costmap_2d::FREE_SPACE;

    BoundedExploreLayer::BoundedExploreLayer(){}

    BoundedExploreLayer::~BoundedExploreLayer(){
        polygonService_.shutdown();
        frontierService_.shutdown();
        delete dsrv_;
        dsrv_ = 0;
    }

    void BoundedExploreLayer::onInitialize(){

        ros::NodeHandle nh_("~/" + name_);
        frontier_cloud_pub = nh_.advertise<sensor_msgs::PointCloud2>("frontiers",5);
        frontier_search_marker_ = nh_.advertise<visualization_msgs::MarkerArray>("frontiers/markers", 5);

        configured_ = false;
        marked_ = false;

        bool explore_clear_space;
        nh_.param("explore_clear_space", explore_clear_space, true);
        if(explore_clear_space){
            default_value_ = NO_INFORMATION;
        }else{
            default_value_ = FREE_SPACE;
        }

        matchSize();

        nh_.param<bool>("resize_to_boundary", resize_to_boundary_, true);
        nh_.param<std::string>("frontier_travel_point", frontier_travel_point_, "closest");

        polygonService_ = nh_.advertiseService("update_boundary_polygon", &BoundedExploreLayer::updateBoundaryPolygonService, this);
        frontierService_ = nh_.advertiseService("get_next_frontier", &BoundedExploreLayer::getNextFrontierService, this);
        allFrontiersService_ = nh_.advertiseService("get_all_frontiers", &BoundedExploreLayer::getAllFrontiersService, this);
        clearLayerService_ = nh_.advertiseService("clear_costmap_layer", &BoundedExploreLayer::clearLayerService, this);

        dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh_);
        dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
                    &BoundedExploreLayer::reconfigureCB, this, _1, _2);
        dsrv_->setCallback(cb);

    }


    void BoundedExploreLayer::matchSize(){
        Costmap2D* master = layered_costmap_->getCostmap();
        resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
                  master->getOriginX(), master->getOriginY());
    }


    void BoundedExploreLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level){
        enabled_ = config.enabled;
    }

    bool BoundedExploreLayer::getNextFrontierService(frontier_exploration::GetNextFrontier::Request &req, frontier_exploration::GetNextFrontier::Response &res){
        return getNextFrontier(req.start_pose, res.next_frontier, res.error_code);
    }

    bool BoundedExploreLayer::getNextFrontier(geometry_msgs::PoseStamped start_pose, geometry_msgs::PoseStamped &next_frontier, int &error_code){

        //wait for costmap to get marked with boundary
        if(!marked_){
            ROS_ERROR("Costmap not marked");
            error_code = frontier_exploration::GetNextFrontier::Response::FAILURE;
            return false;
        }

        if(start_pose.header.frame_id != layered_costmap_->getGlobalFrameID()){
            //error out if no transform available
            if(!tf_listener_.waitForTransform(layered_costmap_->getGlobalFrameID(), start_pose.header.frame_id,ros::Time::now(),ros::Duration(10))) {
                ROS_ERROR_STREAM("Couldn't transform from "<<layered_costmap_->getGlobalFrameID()<<" to "<< start_pose.header.frame_id);
                error_code =  frontier_exploration::GetNextFrontier::Response::TF_ERROR;
                return true;
            }
            geometry_msgs::PoseStamped temp_pose = start_pose;
            tf_listener_.transformPose(layered_costmap_->getGlobalFrameID(),temp_pose,start_pose);
        }

        //initialize frontier search implementation
        FrontierSearch frontierSearch(*(layered_costmap_->getCostmap()));
        //get list of frontiers from search implementation
        std::list<Frontier> frontier_list = frontierSearch.searchFrom(start_pose.pose.position);

        if(frontier_list.size() == 0){
            ROS_DEBUG("No frontiers found, exploration complete");
            error_code = frontier_exploration::GetNextFrontier::Response::NO_FRONTIERS;
            return true;
        }

        //create placeholder for selected frontier
        Frontier selected;
        selected.min_distance = std::numeric_limits<double>::infinity();

        //pointcloud for visualization purposes
        pcl::PointCloud<pcl::PointXYZI> frontier_cloud_viz;
        pcl::PointXYZI frontier_point_viz(50);
        int max;

        BOOST_FOREACH(Frontier frontier, frontier_list){
            //load frontier into visualization poitncloud
            frontier_point_viz.x = frontier.initial.x;
            frontier_point_viz.y = frontier.initial.y;
            frontier_cloud_viz.push_back(frontier_point_viz);

            //check if this frontier is the nearest to robot
            if (frontier.min_distance < selected.min_distance){
                selected = frontier;
                max = frontier_cloud_viz.size()-1;
            }
        }

        //color selected frontier
        frontier_cloud_viz[max].intensity = 100;

        //publish visualization point cloud
        sensor_msgs::PointCloud2 frontier_viz_output;
        pcl::toROSMsg(frontier_cloud_viz,frontier_viz_output);
        frontier_viz_output.header.frame_id = layered_costmap_->getGlobalFrameID();
        frontier_viz_output.header.stamp = ros::Time::now();
        frontier_cloud_pub.publish(frontier_viz_output);

        //set goal pose to next frontier
        next_frontier.header.frame_id = layered_costmap_->getGlobalFrameID();
        next_frontier.header.stamp = ros::Time::now();

        //
        if(frontier_travel_point_ == "closest"){
            next_frontier.pose.position = selected.initial;
        }else if(frontier_travel_point_ == "middle"){
            next_frontier.pose.position = selected.middle;
        }else if(frontier_travel_point_ == "centroid"){
            next_frontier.pose.position = selected.centroid;
        }else{
            ROS_ERROR("Invalid 'frontier_travel_point' parameter, falling back to 'closest'");
            next_frontier.pose.position = selected.initial;
        }

        next_frontier.pose.orientation = tf::createQuaternionMsgFromYaw( yawOfVector(start_pose.pose.position, next_frontier.pose.position) );
        error_code = frontier_exploration::GetNextFrontier::Response::SUCCESS;
        return true;

    }

    bool BoundedExploreLayer::clearLayerService(frontier_exploration::ClearCostmapLayer::Request &req, 
                                                    frontier_exploration::ClearCostmapLayer::Response &res){
        bool valid_layer = false;
        std::vector<std::string> layer_names;
        std::vector < boost::shared_ptr<Layer> > *plugins = layered_costmap_->getPlugins();                                         
        for (std::vector<boost::shared_ptr<Layer> >::iterator plugin = plugins->begin(); plugin != plugins->end(); ++plugin)
        {
            std::string name = (*plugin)->getName();
            layer_names.push_back(name);
            if(name == req.layer_name){
                ROS_INFO_STREAM("Resetting layer " << name);   
                (*plugin)->reset();
                valid_layer = true;
            }
        }

        if(!valid_layer){
            ROS_WARN_STREAM("Could not find layer " << req.layer_name);
            std::string msg = "Available layers are:";
            for(auto const layer : layer_names){
                msg = msg + " " + layer + ",";
            }
            msg.pop_back();
            ROS_WARN_STREAM(msg);
        }

        res.success = valid_layer;
        return true;
    }  

    bool BoundedExploreLayer::getAllFrontiersService(frontier_exploration::GetAllFrontiers::Request &req, 
                                                    frontier_exploration::GetAllFrontiers::Response &res){
        int error_code;
        geometry_msgs::PoseStamped start_pose;

        // Wait for costmap to get marked with boundary
        if(!marked_){
            ROS_ERROR("Costmap not marked");
            res.error_code = frontier_exploration::GetNextFrontier::Response::FAILURE;
            return false;
        }

        start_pose = req.start_pose;
        if(start_pose.header.frame_id != layered_costmap_->getGlobalFrameID()){
            // error out if no transform available
            if(!tf_listener_.waitForTransform(layered_costmap_->getGlobalFrameID(), start_pose.header.frame_id,ros::Time::now(),ros::Duration(10))) {
                ROS_ERROR_STREAM("Couldn't transform from "<<layered_costmap_->getGlobalFrameID()<<" to "<< start_pose.header.frame_id);
                error_code =  frontier_exploration::GetNextFrontier::Response::TF_ERROR;
                return true;
            }
            geometry_msgs::PoseStamped temp_pose = start_pose;
            tf_listener_.transformPose(layered_costmap_->getGlobalFrameID(),temp_pose,start_pose);
        }

        // initialize frontier search implementation
        FrontierSearch frontierSearch(*(layered_costmap_->getCostmap()));
        // get list of frontiers from search implementation
        std::list<Frontier> frontier_list = frontierSearch.searchFrom(start_pose.pose.position);

        if(frontier_list.size() == 0){
            ROS_DEBUG("No frontiers found, exploration complete");
            error_code = frontier_exploration::GetNextFrontier::Response::NO_FRONTIERS;
            return true;
        }

        for(const Frontier& f : frontier_list){
            geometry_msgs::PoseStamped frontier;
            if(frontier_travel_point_ == "closest"){
                frontier.pose.position = f.initial;
            }
            else if(frontier_travel_point_ == "middle"){
                frontier.pose.position = f.middle;
            }
            else if(frontier_travel_point_ == "centroid"){
                frontier.pose.position = f.centroid;
            }
            else{
                ROS_ERROR("Invalid 'frontier_travel_point' parameter, falling back to 'closest'");
                frontier.pose.position = f.initial;
            }
            frontier.pose.orientation = tf::createQuaternionMsgFromYaw( yawOfVector(start_pose.pose.position, frontier.pose.position));
            frontier.header.frame_id = layered_costmap_->getGlobalFrameID();
            frontier.header.stamp = ros::Time::now();
            res.frontiers.push_back(frontier);
        }
        
        // Publish Markers 
        visualization_msgs::MarkerArray m_array; 
        visualization_msgs::Marker m_delete;
        m_delete.ns = "frontier_exploration/all_frontiers";
        m_delete.id = 0;
        m_delete.action = visualization_msgs::Marker::DELETEALL;
        m_array.markers.push_back(m_delete);

        visualization_msgs::Marker m;
        m.header.frame_id = "map";
        m.header.stamp = ros::Time::now();
        m.ns = "frontier_exploration/all_frontiers";
        m.id = 1;
        m.type = visualization_msgs::Marker::POINTS;
        m.action = visualization_msgs::Marker::ADD;
        m.frame_locked = true;
        m.lifetime = ros::Duration(10);
        m.scale.x = 0.05;
        m.scale.y = 0.05;
        m.scale.z = 0.05;
        for(const Frontier& f : frontier_list){
            geometry_msgs::Point p;
            p.x = f.centroid.x;
            p.y = f.centroid.y;
            p.z = 0;
            m.points.push_back(p);

            std_msgs::ColorRGBA c;
            c.a = 1;
            c.r = 0.0;
            c.g = 1.0;
            c.b = 0.0;
            m.colors.push_back(c);
        }
        m_array.markers.push_back(m);
        frontier_search_marker_.publish(m_array);

        res.error_code = frontier_exploration::GetNextFrontier::Response::SUCCESS;
        return true;
    }

    bool BoundedExploreLayer::updateBoundaryPolygonService(frontier_exploration::UpdateBoundaryPolygon::Request &req, frontier_exploration::UpdateBoundaryPolygon::Response &res){

        return updateBoundaryPolygon(req.explore_boundary);

    }

    void BoundedExploreLayer::reset(){

        //reset costmap_ char array to default values
        marked_ = false;
        configured_ = false;
        memset(costmap_, default_value_, size_x_ * size_y_ * sizeof(unsigned char));

    }

    bool BoundedExploreLayer::updateBoundaryPolygon(geometry_msgs::PolygonStamped polygon_stamped){

        //clear existing boundary, if any
        polygon_.points.clear();

        //error if no transform available between polygon and costmap
        if(!tf_listener_.waitForTransform(layered_costmap_->getGlobalFrameID(), polygon_stamped.header.frame_id,ros::Time::now(),ros::Duration(10))) {
            ROS_ERROR_STREAM("Couldn't transform from "<<layered_costmap_->getGlobalFrameID()<<" to "<< polygon_stamped.header.frame_id);
            return false;
        }

        //Transform all points of boundary polygon into costmap frame
        geometry_msgs::PointStamped in, out;
        in.header = polygon_stamped.header;
        BOOST_FOREACH(geometry_msgs::Point32 point32, polygon_stamped.polygon.points){
            in.point = costmap_2d::toPoint(point32);
            tf_listener_.transformPoint(layered_costmap_->getGlobalFrameID(),in,out);
            polygon_.points.push_back(costmap_2d::toPoint32(out.point));
        }

        //if empty boundary provided, set to whole map
        if(polygon_.points.empty()){
            geometry_msgs::Point32 temp;
            temp.x = getOriginX();
            temp.y = getOriginY();
            polygon_.points.push_back(temp);
            temp.y = getSizeInMetersY();
            polygon_.points.push_back(temp);
            temp.x = getSizeInMetersX();
            polygon_.points.push_back(temp);
            temp.y = getOriginY();
            polygon_.points.push_back(temp);
        }

        if(resize_to_boundary_){
            updateOrigin(0,0);

            //Find map size and origin by finding min/max points of polygon
            double min_x = std::numeric_limits<double>::infinity();
            double min_y = std::numeric_limits<double>::infinity();
            double max_x = -std::numeric_limits<double>::infinity();
            double max_y = -std::numeric_limits<double>::infinity();

            BOOST_FOREACH(geometry_msgs::Point32 point, polygon_.points){
                min_x = std::min(min_x,(double)point.x);
                min_y = std::min(min_y,(double)point.y);
                max_x = std::max(max_x,(double)point.x);
                max_y = std::max(max_y,(double)point.y);
            }

            //resize the costmap to polygon boundaries, don't change resolution
            int size_x, size_y;
            worldToMapNoBounds(max_x - min_x, max_y - min_y, size_x, size_y);
            layered_costmap_->resizeMap(size_x, size_y, layered_costmap_->getCostmap()->getResolution(), min_x, min_y);
            matchSize();
        }

        configured_ = true;
        marked_ = false;
        return true;
    }


    void BoundedExploreLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                           double* min_y, double* max_x, double* max_y){

        //check if layer is enabled and configured with a boundary
        if (!enabled_ || !configured_){ return; }

        //update the whole costmap
        *min_x = getOriginX();
        *min_y = getOriginY();
        *max_x = getSizeInMetersX()+getOriginX();
        *max_y = getSizeInMetersY()+getOriginY();

    }

    void BoundedExploreLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j){
        //check if layer is enabled and configured with a boundary
        if (!enabled_ || !configured_){ return; }

        //draw lines between each point in polygon
        MarkCell marker(costmap_, LETHAL_OBSTACLE);

        //circular iterator
        for(int i = 0, j = polygon_.points.size()-1; i < polygon_.points.size(); j = i++){

            int x_1, y_1, x_2, y_2;
            worldToMapEnforceBounds(polygon_.points[i].x, polygon_.points[i].y, x_1,y_1);
            worldToMapEnforceBounds(polygon_.points[j].x, polygon_.points[j].y, x_2,y_2);

            raytraceLine(marker,x_1,y_1,x_2,y_2);
        }
        //update the master grid from the internal costmap
        mapUpdateKeepObstacles(master_grid, min_i, min_j, max_i, max_j);


    }

    void BoundedExploreLayer::mapUpdateKeepObstacles(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j){
        if (!enabled_)
            return;

        unsigned char* master = master_grid.getCharMap();
        unsigned int span = master_grid.getSizeInCellsX();

        for (int j = min_j; j < max_j; j++)
        {
            unsigned int it = span*j+min_i;
            for (int i = min_i; i < max_i; i++)
            {
                //only update master grid if local costmap cell is lethal/higher value, and is not overwriting a lethal obstacle in the master grid
                if(master[it] != LETHAL_OBSTACLE && (costmap_[it] == LETHAL_OBSTACLE || costmap_[it] > master[it])){
                    master[it] = costmap_[it];
                }
                it++;
            }
        }
        marked_ = true;
    }
}
