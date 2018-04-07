#include <mav_trajectory_generation/polynomial_optimization_linear.h>
#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include <mav_trajectory_generation/trajectory.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>
#include <ros/ros.h>
#include <iostream>
#include <sstream>
#include <fstream>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "testNode");
    ros::NodeHandle n;
    mav_trajectory_generation::Vertex::Vector vertices;
    const int dimension = 4;
    const int derivative_to_optimize = mav_trajectory_generation::derivative_order::SNAP;
    mav_trajectory_generation::Vertex start(dimension), middle(dimension), end(dimension);

    std::string line;
    std::ifstream waypointsFile("/home/sai/rrbt.txt");
    int lineNum = 0;
    double x, y, z, yaw;

    while (waypointsFile >> x >> y >> z >> yaw) {

        if (lineNum == 0) {
            start.makeStartOrEnd(Eigen::Vector4d(x, y, z, yaw), derivative_to_optimize);
            vertices.push_back(start);
        }

        else {
            middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector4d(x, y, z, yaw));
            vertices.push_back(middle);
        }
        ++lineNum;
    }

    ROS_INFO("Read %d waypoints", lineNum);

    end.makeStartOrEnd(Eigen::Vector4d(4, 0, 0, 0), derivative_to_optimize);
    vertices.push_back(end);

    std::vector<double> segment_times;
    const double v_max = 2.0;
    const double a_max = 2.0;
    const double magic_fabian_constant = 6.5; // A tuning parameter.
    segment_times = estimateSegmentTimes(vertices, v_max, a_max, magic_fabian_constant);

    mav_trajectory_generation::NonlinearOptimizationParameters parameters;
    parameters.max_iterations = 1000;
    parameters.f_rel = 0.05;
    parameters.x_rel = 0.1;
    parameters.time_penalty = 500.0;
    parameters.initial_stepsize_rel = 0.1;
    parameters.inequality_constraint_tolerance = 0.1;

    const int N = 10;
    mav_trajectory_generation::PolynomialOptimizationNonLinear<N> opt(dimension, parameters, false);
    opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
    opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::VELOCITY, v_max);                                
    opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, a_max);

    ROS_INFO("Performing optimization...");
    opt.optimize();

    mav_trajectory_generation::Segment::Vector segments;
    opt.getPolynomialOptimizationRef().getSegments(&segments);
    ROS_INFO("Done.");
    mav_trajectory_generation::Trajectory trajectory;
    opt.getTrajectory(&trajectory);

    ros::Publisher vis_pub = n.advertise<visualization_msgs::MarkerArray>( "smoothed_path", 10, true );
    visualization_msgs::MarkerArray markers;
    double distance = 1.0; // Distance by which to seperate additional markers. Set 0.0 to disable.
    std::string frame_id = "world";
    std_msgs::Header header;
    header.frame_id = frame_id;

    // From Trajectory class:
    mav_trajectory_generation::drawMavTrajectory(trajectory, distance, frame_id, &markers);
    while(ros::ok())
    {
        ROS_INFO("Publishing trajectory");
        header.stamp = ros::Time::now();
        vis_pub.publish(markers);
        header.seq++;

        ros::Duration(5.0).sleep();
    }
}