#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "path_publisher");
    ros::NodeHandle nh;

    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("/path_publisher/path", 10);

    nav_msgs::Path path_msg;
    path_msg.header.frame_id = "map";

    std::vector<geometry_msgs::PoseStamped> poses;

    double point[][2] = {{-30.0, 30.0}, {-20.0, 29.0}, {-12.0, 23.0},{-11.0, 10.0},{-22.0,14.0},
    {-30.0,13.0},{-22.0,2.0},{-19.0,-17.0},{-17.0,-21.0},{-15.0,-23.0},{-13.0,-24.0},{0,-25.0}};
    // double point[][2] = {{0,-25.0},{-13.0,-24.0},{-15.0,-23.0},{-17.0,-21.0},{-19.0,-17.0},{-22.0,2.0},
    // {-30.0,13.0},{-22.0,14.0},{-11.0, 10.0}, {-12.0, 23.0} ,{-20.0, 29.0},{-30.0, 30.0}};

    //将pose循环存入poses
    for (int i = 0; i < sizeof(point) / sizeof(point[0]); i++)
    {
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = point[i][0];
        pose.pose.position.y = point[i][1];
        pose.pose.position.z = 0.0;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 1.0;
        poses.push_back(pose);
    }

    path_msg.poses = poses;

    // Publish the path_msg
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        path_msg.header.stamp = ros::Time::now();
        path_pub.publish(path_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
