//message_filters
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// sensor_msgs
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

// Vision msgs
#include <vision_msgs/Detection2DArray.h>
#include <depthai_ros_msgs/SpatialDetectionArray.h>

// Convert image to cv
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


// using namespace sensor_msgs;
ros::Publisher pub;
static const std::string OPENCV_WINDOW = "Image window";

static const std::vector<std::string> labelMap = {"stairs"};

float roundForPrint(float var)
{
    // 37.66666 * 100 =3766.66
    // 3766.66 + .5 =3767.16    for rounding off value
    // then type cast to int so value is 3767
    // then divided by 100 so the value converted into 37.67
    float value = (int)(var * 100 + .5);
    return (float)value / 100;
}

void callback(const depthai_ros_msgs::SpatialDetectionArray::ConstPtr& detectionArray,
               const sensor_msgs::Image::ConstPtr& image_msg)
{
  // ROS_INFO("Synched messages");
  // convert the input (sensor_msgs) to cv image
 cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }


    //Check for detections 
    
    if(!detectionArray->detections.empty()){
      // Draw boxes for every detected staircse  
      for(int d_num =0; d_num < detectionArray->detections.size(); d_num ++){
        // int labelIndex = std::stoi(detectionArray->detections.at(d_num).results.at(0).id);
        int labelIndex = int(detectionArray->detections.at(d_num).results.at(0).id);
        // std::cerr<<  detectionArray->detections.at(d_num).results.at(0).id<< std::endl;
        std::string labelStr = std::to_string(labelIndex);
        if(labelIndex < labelMap.size()) {
                labelStr = labelMap[labelIndex];
        }
        float center_x = detectionArray->detections.at(d_num).bbox.center.x;
        float center_y = detectionArray->detections.at(d_num).bbox.center.y;
        float half_size_x = detectionArray->detections.at(d_num).bbox.size_x/2;
        float half_size_y = detectionArray->detections.at(d_num).bbox.size_y/2;
        std::string pos_x = std::to_string(detectionArray->detections.at(d_num).position.x);
        std::string pos_y = std::to_string(detectionArray->detections.at(d_num).position.y);
        std::string pos_z = std::to_string(detectionArray->detections.at(d_num).position.z);
        std::string position = "(" + pos_x.substr(3) + ", " + pos_y + ", " + pos_z + ")";
        // float pos_x = float( detectionArray->detections.at(d_num).position.x);
        // float pos_y = float(detectionArray->detections.at(d_num).position.y);
        // float pos_z = float(detectionArray->detections.at(d_num).position.z);
        cv::Point pt1(center_x-half_size_x,center_y-half_size_y);
        cv::Point pt2(center_x+half_size_x,center_y+half_size_y);
        cv::rectangle(cv_ptr->image, pt1,pt2,CV_RGB(255,0,0));
        // cv::putText(cv_ptr->image, "Stairs", pt1, cv::FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv::LINE_AA);
        

        cv::putText(cv_ptr->image, position, pt1, cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0,0,255));
        cv::putText(cv_ptr->image, labelStr, cv::Point(pt1.x + 10, pt1.y + 20), cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0,0,255));
        std::stringstream confStr;
        confStr << std::fixed << std::setprecision(2) << detectionArray->detections.at(d_num).results.at(0).score * 100;
        cv::putText(cv_ptr->image, confStr.str(), cv::Point(pt1.x + 10, pt1.y + 35), cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0,0,255));

        // std::stringstream depthX;
        // depthX << "X: " << pos_x << " m";
        // cv::putText(cv_ptr->image, depthX.str(), cv::Point(pt1.x + 10, pt1.y + 40), cv::FONT_HERSHEY_TRIPLEX, 0.5, 255);
        // std::stringstream depthY;
        // depthY << "Y: " << pos_y << " m";
        // cv::putText(cv_ptr->image, depthY.str(), cv::Point(pt1.x + 10, pt1.y + 45), cv::FONT_HERSHEY_TRIPLEX, 0.5, 255);
        // std::stringstream depthZ;
        // depthZ << "Z: " << pos_z << " m";
        // cv::putText(cv_ptr->image, depthZ.str(), cv::Point(pt1.x + 10, pt1.y + 50), cv::FONT_HERSHEY_TRIPLEX, 0.5, 255);
      }
    } 
    


    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);

    // Output modified video stream
    pub.publish(cv_ptr->toImageMsg());

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "view_oakd_detections");

  ros::NodeHandle nh;
  // TODO: add args to be able to subscribe to any topic
  message_filters::Subscriber<depthai_ros_msgs::SpatialDetectionArray> bb_sub(nh, "/yolov4_publisher/color/yolov4_spatial_detections", 30);
  // message_filters::Subscriber<sensor_msgs::CameraInfo> bb_sub(nh, "/mobilenet_publisher/color/camera_info", 30);
  message_filters::Subscriber<sensor_msgs::Image> cloud_sub(nh, "/yolov4_publisher/color/image", 30);
  // message_filters::Subscriber<sensor_msgs::Image> cloud_sub(nh, "/mobilenet_publisher/color/image", 30);

  typedef message_filters::sync_policies::ApproximateTime<depthai_ros_msgs::SpatialDetectionArray, 
                    sensor_msgs::Image> MySyncPolicy;


  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10),bb_sub,cloud_sub);

  sync.registerCallback(boost::bind(&callback, _1, _2));

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::Image> ("image_w_detections", 1);


  ros::spin();

  return 0;
}
