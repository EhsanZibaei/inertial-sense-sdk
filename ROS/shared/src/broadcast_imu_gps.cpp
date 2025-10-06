#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h> // For GPS data if available
#include <tf/transform_broadcaster.h>

// Include Inertial Sense SDK headers
#include <InertialSense.h>
#include <data_sets.h>
#include <com_manager.h> // Explicitly include com_manager.h

ros::Publisher imu_pub;
ros::Publisher gps_pub;
// Callback function for Inertial Sense data
void inertialSenseDataCallback(InertialSense* i, p_data_t* data, int pHandle, ros::NodeHandle* nh)
{


    // Example: Process IMU data
    if (data->hdr.id == DID_PIMU)
    {
        const pimu_t* pimu = (const pimu_t*)data->ptr; // Changed data->buf to data->ptr
        sensor_msgs::Imu imu_msg;
        imu_msg.header.stamp = ros::Time::now();
        imu_msg.header.frame_id = "imu_link"; // You might want to change this frame_id

        // Orientation is not directly available in pimu_t in this SDK version.
        // You might need to subscribe to DID_INS_2 for orientation.
        // imu_msg.orientation.x = pimu->qn[1];
        // imu_msg.orientation.y = pimu->qn[2];
        // imu_msg.orientation.z = pimu->qn[3];
        // imu_msg.orientation.w = pimu->qn[0];

        imu_msg.angular_velocity.x = pimu->theta[0] / pimu->dt;
        imu_msg.angular_velocity.y = pimu->theta[1] / pimu->dt;
        imu_msg.angular_velocity.z = pimu->theta[2] / pimu->dt;

        imu_msg.linear_acceleration.x = pimu->vel[0] / pimu->dt;
        imu_msg.linear_acceleration.y = pimu->vel[1] / pimu->dt;
        imu_msg.linear_acceleration.z = pimu->vel[2] / pimu->dt;

        imu_pub.publish(imu_msg);
    }
    // Example: Process GPS data
    else if (data->hdr.id == DID_GPS1_POS)
    {
        const gps_pos_t* gps_pos = (const gps_pos_t*)data->ptr; // Changed data->buf to data->ptr
        sensor_msgs::NavSatFix gps_msg;
        gps_msg.header.stamp = ros::Time::now();
        gps_msg.header.frame_id = "gps_link"; // You might want to change this frame_id

        gps_msg.latitude = gps_pos->lla[0];
        gps_msg.longitude = gps_pos->lla[1];
        gps_msg.altitude = gps_pos->lla[2];

        gps_msg.status.status = sensor_msgs::NavSatStatus::STATUS_FIX; // Assuming a fix
        gps_msg.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;

        gps_pub.publish(gps_msg);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "broadcast_imu_gps");
    ros::NodeHandle nh;

    imu_pub = nh.advertise<sensor_msgs::Imu>("imu/data_raw", 100);
    gps_pub = nh.advertise<sensor_msgs::NavSatFix>("gps/fix", 100);

    // Initialize Inertial Sense
    InertialSense inertialSense;
    if (!inertialSense.Open("/dev/ttyACM1", 921600)) // Adjust port and baud rate as needed (changed from /dev/ttyUSB0)
    {
        ROS_ERROR("Failed to open Inertial Sense device.");
        return 1;
    }

    // Enable desired data streams
    inertialSense.BroadcastBinaryData(DID_PIMU, 100, [&](InertialSense* i_ptr, p_data_t* data_ptr, int pHandle_val) {
        inertialSenseDataCallback(i_ptr, data_ptr, pHandle_val, &nh);
    });
    inertialSense.BroadcastBinaryData(DID_GPS1_POS, 1, [&](InertialSense* i_ptr, p_data_t* data_ptr, int pHandle_val) {
        inertialSenseDataCallback(i_ptr, data_ptr, pHandle_val, &nh);
    });

    ros::spin();

    inertialSense.Close();
    return 0;
}
