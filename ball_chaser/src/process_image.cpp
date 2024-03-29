#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;


// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // Request a service and pass the velocities to it to drive the robot
    ball_chaser::DriveToTarget srv;

    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

    client.call(srv);

    // Call the safe_move service and pass the requested joint angles
    if (!client.call(srv))
        ROS_ERROR("Failed to call service /command_robot");
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{

    int white_pixel = 255;

    // Loop through each pixel in the image and check if there's a bright white one

    for (int height = 350; height < img.height - 350; height +=10) 
    {
    for (int width = 0; width < img.step; width+= 10) 
    {

    //   ROS_INFO_STREAM("Pixel: " + std::to_string(i) + " Pixel value: " + std::to_string(img.data[i]));
       if (img.data[width*height] == 255)
        {
         // Then, identify if this pixel falls in the left, mid, or right side of the image
           ROS_INFO_STREAM("White pixel found here :" + std::to_string(width) + "Width/3 " + std::to_string(img.width/3));
          if(width <= (img.step)/3)
          {
            //move left
            ROS_INFO_STREAM("Moving left");
            drive_robot(5, 15);
            break;
          }
          else if(width >= (2*img.step)/3)
          {
            // move right
            ROS_INFO_STREAM("Moving right");
            drive_robot(5, -15);
            break;
          }
          else
          {
            // move fwd
            ROS_INFO_STREAM("Moving straight");
            drive_robot(45, 0.0);
            break;
          }
         }
        else
        {
         ROS_INFO_STREAM("Sitting tight");
   //        drive_robot(0.5, 0.5);
        }
    } 
    } 
}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;


    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}
