
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "message_filters/subscriber.h"
#include "tf/message_filter.h"
#include <iostream>
#include <cstdio>


#define LASERLOG "laserlog"
#define ODOMLOG "odomlog"

using namespace std;

tf::Stamped<tf::Transform> parseOdomMsgtoFile(FILE * odom_file){

	   tf::Stamped<tf::Transform> odom_pose;


	   /*tfScalar x =  odom_pose.getRotation().getX();
		tfScalar y =  odom_pose.getRotation().getY();
		tfScalar z =  odom_pose.getRotation().getZ();
		tfScalar w =  odom_pose.getRotation().getW();


	   fread(&x,sizeof(tfScalar),1,odom_file);
	   fread(&y,sizeof(tfScalar),1,odom_file);
	   fread(&z,sizeof(tfScalar),1,odom_file);
	   fread(&w,sizeof(tfScalar),1,odom_file);

	   tf::Quaternion quat(x,y,z,w);

	   x = odom_pose.getOrigin().getX();
	   y = odom_pose.getOrigin().getY();
	   z = odom_pose.getOrigin().getZ();

	   fread(&x,sizeof(tfScalar),1,odom_file);
	   fread(&y,sizeof(tfScalar),1,odom_file);
	   fread(&z,sizeof(tfScalar),1,odom_file);


	   tf::Vector3 vector(x,y,z);

		*/

	   tf::Vector3 vector;
	   tf::Quaternion quat;

	   fread(&quat,sizeof(tf::Quaternion),1,odom_file);
	   fread(&vector,sizeof(tf::Vector3),1,odom_file);


	   odom_pose.setOrigin(vector);
	   odom_pose.setRotation(quat);


	   return odom_pose;

}


sensor_msgs::LaserScan parseLaserMessage(FILE *laserfile){

	       sensor_msgs::LaserScan scan;

		   //Header
		   fread(&scan.header.seq,sizeof(uint32_t),1,laserfile);
		   fread(&scan.header.stamp.sec,sizeof(uint32_t),1,laserfile);
		   fread(&scan.header.stamp.nsec,sizeof(uint32_t),1,laserfile);


		   fread(&scan.angle_min,sizeof(float),1,laserfile);
		   fread(&scan.angle_max,sizeof(float),1,laserfile);
		   fread(&scan.angle_increment,sizeof(float),1,laserfile);
		   fread(&scan.time_increment,sizeof(float),1,laserfile);
		   fread(&scan.scan_time,sizeof(float),1,laserfile);
		   fread(&scan.range_min,sizeof(float),1,laserfile);
		   fread(&scan.range_max,sizeof(float),1,laserfile);

		   //ranges size

		   unsigned long size;
		   fread(&size,sizeof(unsigned long),1,laserfile);

		   for(unsigned int i=0;i<size;i++){
		       float value;
			   fread(&value,sizeof(float),1,laserfile);
			   scan.ranges.push_back(value);
		   }

		   //intensities size
		   fread(&size,sizeof(unsigned long),1,laserfile);

		   for(unsigned int i=0;i<size;i++){
		       float value;
			   fread(&value,sizeof(float),1,laserfile);
			   scan.intensities.push_back(value);
		   }

		   ROS_INFO ("header.seq: %u",(scan).header.seq);
		   ROS_INFO ("header.stamp: %u",(scan).header.stamp.nsec);
		   //cout << "hea(.frame_id: "<<scan.header.frame_id << endl;
		   ROS_INFO ("angle_increment: %f ",(scan).angle_increment);
		   ROS_INFO ("angle_max: %f",(scan).angle_max);
		   ROS_INFO ("angle_min: %f",(scan).angle_min);
		   ROS_INFO ("range_max: %f",(scan).range_max);
		   ROS_INFO ("range_min: %f",(scan).range_min);
		   ROS_INFO ("ranges.size(): %lu",(scan).ranges.size());

		   return scan;

}


int main(){

    FILE * laserfile;
    FILE * odomfile;

    laserfile = fopen(LASERLOG,"rb");
    odomfile = fopen(ODOMLOG,"rb");

    sensor_msgs::LaserScan scan;

    while (!feof(laserfile)){
    	scan=parseLaserMessage(laserfile);
    }

    tf::Stamped<tf::Transform> odom_pose;

    while (!feof(odomfile)){
    	odom_pose=parseOdomMsgtoFile(odomfile);
        double yaw = tf::getYaw(odom_pose.getRotation());
        ROS_INFO("yaw: %f",yaw);
        ROS_INFO("x: %f",odom_pose.getOrigin().x());
        ROS_INFO("y: %f",odom_pose.getOrigin().y());
    }






    fclose(laserfile);
    fclose(odomfile);



    return 0;



}
