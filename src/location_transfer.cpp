#include <iostream>
#include <math.h>

#include <ros/ros.h>
#include <tf/tf.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>

#define PI 3.14159265

using namespace std; 

class Transfer{
    private:
        ros::NodeHandle n;

        ros::Subscriber sub_goal;
        ros::Subscriber sub_pose_mavros;
        ros::Subscriber sub_pose_ekf;
        ros::Publisher pub_drone2wamv;

        geometry_msgs :: PoseStamped pose_goal;
        geometry_msgs :: PoseStamped pose_mavros;
        geometry_msgs :: PoseStamped pose_ekf;

    public:
        Transfer();
        void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
        void mavrosCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
        void ekfCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
};

Transfer :: Transfer(){
    sub_goal = n.subscribe<geometry_msgs::PoseStamped>("move_base_simple/goal", 1,  &Transfer::goalCallback, this);
    sub_pose_mavros = n.subscribe<geometry_msgs::PoseStamped>("mavros/pose", 1,  &Transfer::mavrosCallback, this);
    sub_pose_ekf = n.subscribe<geometry_msgs::PoseStamped>("ekf/pose", 1,  &Transfer::ekfCallback, this);
    pub_drone2wamv = n.advertise<geometry_msgs::PoseStamped>("drone/goal", 10);
}

void Transfer :: goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    pose_goal = *msg;
    cout << "Recieve goal" << endl;
    geometry_msgs :: PoseStamped msg_pub_goal;
    msg_pub_goal.header = pose_goal.header;
    msg_pub_goal.pose.position.x = pose_goal.pose.position.x + pose_mavros.pose.position.x - pose_ekf.pose.position.x;
    msg_pub_goal.pose.position.y = pose_goal.pose.position.y + pose_mavros.pose.position.y - pose_ekf.pose.position.y;
    msg_pub_goal.pose.position.z = pose_goal.pose.position.z;
    msg_pub_goal.pose.orientation = pose_goal.pose.orientation;
    cout << " drone X: " << pose_mavros.pose.position.x << endl;
    cout << " ekf   X: " << pose_ekf.pose.position.x << endl;

    cout << " drone Y: " << pose_mavros.pose.position.y << endl;
    cout << " ekf   Y: " << pose_ekf.pose.position.y << endl;

    cout << " X: " << msg_pub_goal.pose.position.x << endl;
    cout << " Y: " << msg_pub_goal.pose.position.y << endl;
    pub_drone2wamv.publish(msg_pub_goal);

    return;
}

void Transfer :: mavrosCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    pose_mavros = *msg;
    return;
}

void Transfer :: ekfCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    pose_ekf = *msg;
    return;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "transfer");
    Transfer uav;
    ros::spin();
    return 0;
}