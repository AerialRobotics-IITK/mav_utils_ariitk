#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Point.h"
#include <mav_utils_msgs/BBPose.h>
#include <mav_utils_msgs/BBPoses.h>
#define E 0.2

geometry_msgs::PoseStamped curr_pose_, obj_pose_, setpt_, last_setpt_;
geometry_msgs::Point obj_loc_;
bool obj_vis_=false, last_vis_=false;
int count =0;

void odom_cb_(const nav_msgs::Odometry &msg)
{
    curr_pose_.pose= msg.pose.pose;
    curr_pose_.header=msg.header;
}

void obj_cb_(const mav_utils_msgs::BBPoses &msg)
{
    //obj_pose_= msg;
    std::vector<mav_utils_msgs::BBPose> obj_poses_ = msg.object_poses;
    obj_vis_=true;
    // select first box only for now
    obj_loc_ = obj_poses_[0].position;
    std::cout << "~Obj Found at x = " << obj_loc_.x << " and y ="<<obj_loc_.y << std::endl;
    //std::cout << "~Obj Found at x = " << obj_pose_.pose.position.x << " and y ="<<obj_pose_.pose.position.y << std::endl;
} 
 

bool permit_(const geometry_msgs::PoseStamped &msg)
{
    if(((msg.pose.position.x < (curr_pose_.pose.position.x)+E ) || (msg.pose.position.x > (curr_pose_.pose.position.x)-E )) && 
    ((msg.pose.position.y < (curr_pose_.pose.position.y)+E ) || (msg.pose.position.y > (curr_pose_.pose.position.y)-E )))
    {
        return true;
    }
    else 
    {
        return false;
    }
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "lander");
    ros::NodeHandle nh_;

    ros::Publisher setpt_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("command",100);
    ros::Subscriber odom_sub_ = nh_.subscribe("odom",20, odom_cb_);
    ros::Subscriber obj_sub_ =nh_.subscribe("poses", 10, obj_cb_);

    ros::Rate loop_rate(10);

    while(ros::ok())
    {
        if(!obj_vis_)
        {   
            // //If obj not visible then hovering at same position
            // std::cout << "No Obj Found " << std::endl;
            // if(!last_vis_)
            // {
            //     setpt_.header.stamp = ros::Time::now();
            //     setpt_.pose = curr_pose_.pose;
            //     std::cout << "  Waiting for obj .. " << std::endl;
            //     setpt_pub_.publish(setpt_);
            //     std::cout << "  pub1" << std::endl;
            // }

            // else
            // {
            //     setpt_.header.stamp = ros::Time::now();
            //     setpt_.pose = last_setpt_.pose;
            //     setpt_.pose.position.z=curr_pose_.pose.position.z -0.2;
            //     if (setpt_.pose.position.z > 1)
            //     {
            //         setpt_pub_.publish(setpt_);
            //         std::cout << "  pub2" << std::endl;
            //     }


            //     // while (!permit_(curr_pose_))
            //     // {
            //     //     std::cout << "  On the way to mailbox (last pose).. " << std::endl;
            //     //     ros::spinOnce();
            //     // }
            // }
            
            
            
        }
        
        else
        {
            //Obj visible
            std::cout << "Obj Found " << std::endl;
            // setpt_.header.stamp = ros::Time::now();
            // setpt_.pose.orientation=curr_pose_.pose.orientation;
            // setpt_.pose.position.x=curr_pose_.pose.position.x + obj_pose_.pose.position.x;
            // setpt_.pose.position.y = curr_pose_.pose.position.y + obj_pose_.pose.position.y;
            // setpt_.pose.position.z=curr_pose_.pose.position.z;

    
            // setpt_pub_.publish(setpt_);
            last_vis_=true;

            // while(!permit_(curr_pose_))
            // {
            //     std::cout<<"  On the way to mailbox .. "<<std::endl;
            //     ros::spinOnce();
            // }

            obj_vis_=false;
            
            if (count%2==0)
            {
                setpt_.header.stamp = ros::Time::now();
                setpt_.pose.orientation = curr_pose_.pose.orientation;
                setpt_.pose.position.x =  obj_loc_.x;
                setpt_.pose.position.y =  obj_loc_.y;
                setpt_.pose.position.z = curr_pose_.pose.position.z - 0.04;
                if (1)
                {
                    setpt_pub_.publish(setpt_);
                    std::cout << "  pub3" << std::endl;
                }
                std::cout<<"  Descending"<<std::endl;
                last_setpt_ = setpt_;
                
            }

            
            else
            {
                setpt_.header.stamp = ros::Time::now();
                setpt_.pose = last_setpt_.pose;
                setpt_.pose.position.z = curr_pose_.pose.position.z -0.04;
                if (1)
                {
                    setpt_pub_.publish(setpt_);
                    std::cout << "  pub4" << std::endl;
                }
            }
            
            //last_setpt_=setpt_;
            count = count + 1;
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

}

