#include "flightros/trajectory_visualization/trajectory_visualization.h"

TrajectoryVisualization::TrajectoryVisualization(ros::NodeHandle &n, ros::NodeHandle &pn) {
    std::string marker_color_str;
    std::string traj_type_str;
    pn.param("marker_color", marker_color_str, std::string("w"));
    pn.param("traj_type", traj_type_str, std::string("w"));
    if (marker_color_str.length()<1) 
        marker_color_str = std::string("w");
    if (traj_type_str.length()<1) 
        traj_type_str = std::string("R");
    marker_color_chr = marker_color_str[0];
    traj_type_chr = traj_type_str[0];
    switch (marker_color_chr) {
        case 'R': {
        marker_color = TrajectoryVisualization::RED;
        } break;
        case 'G': {
        marker_color = TrajectoryVisualization::GREEN;
        } break;
        case 'B': {
        marker_color = TrajectoryVisualization::BLUE;
        } break;
        case 'O': {
        marker_color = TrajectoryVisualization::ORANGE;
        } break;
        case 'W': {
        marker_color = TrajectoryVisualization::WHITE;
        } break;
        case 'D': {
        marker_color = TrajectoryVisualization::DEEP_BLUE;
        } break;
        default: {
        marker_color = TrajectoryVisualization::WHITE;
        } break;
    }
    switch (traj_type_chr) {
        case 'R':
            traj_type = TrajectoryVisualization::RAW;
            break;
        case 'S':
            traj_type = TrajectoryVisualization::SIMPLIFIED;
            break;
        default:
            traj_type = TrajectoryVisualization::RAW;
            break;
    }

    // ROS subscribers
    trajectory_sub = n.subscribe<mav_planning_msgs::PolynomialTrajectory4D>(
                "/path_planner/planned_trajectory", 10,
                &TrajectoryVisualization::trajectoryCallback, this);

    // ROS publishers
    if (traj_type == RAW) rviz_markers_publisher = pn.advertise<visualization_msgs::Marker>("trajectory_markers_raw", 10, true);
    else if (traj_type == SIMPLIFIED) rviz_markers_publisher = pn.advertise<visualization_msgs::Marker>("trajectory_markers", 10, true);
}

void TrajectoryVisualization::trajectoryCallback(const mav_planning_msgs::PolynomialTrajectory4D::ConstPtr &p_msg) {
    visualization_msgs::Marker::Ptr p_traj_points, p_traj_edges;
    p_traj_points = visualization_msgs::Marker::Ptr( new visualization_msgs::Marker() );
    p_traj_edges = visualization_msgs::Marker::Ptr( new visualization_msgs::Marker() );
    visualization_msgs::Marker &traj_points = *p_traj_points;
    visualization_msgs::Marker &traj_edges = *p_traj_edges;
    traj_points.header.frame_id = traj_edges.header.frame_id = "world"; // "odom"
    traj_points.header.stamp = traj_edges.header.stamp = ros::Time::now();
    traj_points.ns = traj_edges.ns = "planned_trajectory";
    traj_points.action = traj_edges.action = visualization_msgs::Marker::ADD;
    traj_points.pose.orientation.w = traj_edges.pose.orientation.w = 1.0;
    traj_points.id = 0;
    traj_edges.id = 1;

    traj_points.type = visualization_msgs::Marker::POINTS;
    traj_edges.type = visualization_msgs::Marker::LINE_STRIP;

    // POINTS markers use x and y scale for width/height respectively
    traj_points.scale.x = 0.05;
    traj_points.scale.y = 0.05;
    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    traj_edges.scale.x = 0.025;

    switch (marker_color) {
    case RED:
        traj_points.color.r = 1.0f;
        traj_points.color.g = 0.0f;
        traj_points.color.b = 0.0f;
        traj_edges.color.r = 1.0f;
        traj_edges.color.g = 0.0f;
        traj_edges.color.b = 0.0f;
        break;
    case GREEN:
        traj_points.color.r = 0.0f;
        traj_points.color.g = 1.0f;
        traj_points.color.b = 0.0f;
        traj_edges.color.r = 0.0f;
        traj_edges.color.g = 1.0f;
        traj_edges.color.b = 0.0f;
        break;
    case BLUE:
        traj_points.color.r = 0.0f;
        traj_points.color.g = 0.0f;
        traj_points.color.b = 1.0f;
        traj_edges.color.r = 0.0f;
        traj_edges.color.g = 0.0f;
        traj_edges.color.b = 1.0f;
        break;
    case ORANGE:
        traj_points.color.r = 1.00f;
        traj_points.color.g = 0.65f;
        traj_points.color.b = 0.00f;
        traj_edges.color.r = 1.00f;
        traj_edges.color.g = 0.65f;
        traj_edges.color.b = 0.00f;
        break;
    case WHITE:
        traj_points.color.r = 1.0f;
        traj_points.color.g = 1.0f;
        traj_points.color.b = 1.0f;
        traj_edges.color.r = 1.0f;
        traj_edges.color.g = 1.0f;
        traj_edges.color.b = 1.0f;
        break;
    case DEEP_BLUE:
        traj_points.color.r = 0.0f;
        traj_points.color.g = 0.749f;
        traj_points.color.b = 1.0f;
        traj_edges.color.r = 0.0f;
        traj_edges.color.g = 0.749f;
        traj_edges.color.b = 1.0f;
        break;
    // case DEEP_BLUE_2:
    //     traj_points.color.r = 0.0f;
    //     traj_points.color.g = 0.50f;
    //     traj_points.color.b = 1.0f;
    //     traj_edges.color.r = 0.0f;
    //     traj_edges.color.g = 0.50f;
    //     traj_edges.color.b = 1.0f;
    //     break;
    default:
        break;
    }
    traj_points.color.a = 1.0;
    traj_edges.color.a = 0.75;

    // *** Conversion to Rviz markers message ***
    // Create the vertices for the points and lines
    const mav_planning_msgs::PolynomialTrajectory4D &msg = *p_msg;
    size_t N = msg.segments.size();
    for (size_t i=0; i<N; i++)
    {
        const mav_planning_msgs::PolynomialSegment4D &segment = msg.segments[i];
        geometry_msgs::Point p;
        p.x = segment.x[0];
        p.y = segment.y[0];
        p.z = segment.z[0];
        traj_points.points.push_back(p);
        traj_edges.points.push_back(p);
    } 

    //case WHITE:
    rviz_markers_publisher.publish(p_traj_edges);
    rviz_markers_publisher.publish(p_traj_points);
}
