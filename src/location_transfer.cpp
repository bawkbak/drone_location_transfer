#include <iostream>
#include <math.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>

#define PI 3.14159265

using namespace std; 
void syncCallback(const geometry_msgs::PoseStamped::ConstPtr& msg_ekf, const geometry_msgs::PoseStamped::ConstPtr& msg_mavros){
    float tf_offset_x, tf_offset_y;
    tf_offset_x = msg_ekf->pose.position.x - msg_mavros->pose.position.x;
    tf_offset_y = msg_ekf->pose.position.y - msg_mavros->pose.position.y;
    cout << "X: " << tf_offset_x << endl;
    cout << "Y: " << tf_offset_y << endl;

    static tf::TransformBroadcaster broadcast;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(tf_offset_x,tf_offset_y, 0.0) );
    tf::Quaternion q;
    q.setRPY(0, 0, 0);
    transform.setRotation(q);
    broadcast.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "drone_origin"));
    
    return;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "transfer");

    ros::NodeHandle n;
    message_filters::Subscriber<geometry_msgs::PoseStamped> sub_ekf(n, "ekf/pose", 1);
    message_filters::Subscriber<geometry_msgs::PoseStamped> sub_mavros(n, "mavros/pose", 1);
    
    typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, geometry_msgs::PoseStamped> approxSync;
    message_filters::Synchronizer<approxSync> sync(approxSync(10), sub_ekf, sub_mavros);

    // message_filters::TimeSynchronizer <geometry_msgs::PoseStamped, geometry_msgs::PoseStamped> 
    // sync(sub_ekf, sub_mavros, 1); 
    sync.registerCallback(boost::bind(&syncCallback, _1, _2)); 

    ros::spin();

    return 0;
}