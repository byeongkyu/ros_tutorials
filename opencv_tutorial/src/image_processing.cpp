#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static const std::string OPENCV_WINDOW = "Result Image";

class SimpleImageProcessing
{
public:
    SimpleImageProcessing(): it_ (nh_)
    {
        image_sub_ = it_.subscribe("/usb_cam/image_raw", 1, &SimpleImageProcessing::imageCb, this);
        image_pub_ = it_.advertise("/output_video", 1);

        cv::namedWindow(OPENCV_WINDOW);
    }

    ~SimpleImageProcessing()
    {
        cv::destroyWindow(OPENCV_WINDOW);
    }

    void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat gray_img;
        cv::Mat dst, detected_edges;

        cv::cvtColor(cv_ptr->image, gray_img, cv::COLOR_BGR2GRAY);
        cv::blur(gray_img, detected_edges, cv::Size(3,3));
        cv::Canny(detected_edges, detected_edges, 0, 0, 3);

        dst = cv::Scalar::all(0);
        cv_ptr->image.copyTo(dst, detected_edges);

        cv::imshow(OPENCV_WINDOW, dst);
        cv::waitKey(1);

        image_pub_.publish(cv_ptr->toImageMsg());
    }

private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
};



int main(int argc, char** argv)
{
    ros::init(argc, argv, "simple_image_processing");
    SimpleImageProcessing n;
    ros::spin();
    return 0;
}