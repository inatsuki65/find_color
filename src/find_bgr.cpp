#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include "find_color/Ellipses.h"
#include "find_color/Ellipse.h"

using namespace std;
using namespace cv;

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  ros::Publisher point_pub_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  find_color::Ellipses centers;
  find_color::Ellipse center;
  cv::Mat image;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/usb_cam/image_raw", 1,
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);
    point_pub_ = nh_.advertise<find_color::Ellipses>("/ellipses_centers", 1);


	cv::namedWindow("inRange", CV_WINDOW_AUTOSIZE|CV_WINDOW_FREERATIO);
  cv::namedWindow("fit_ellipse", CV_WINDOW_AUTOSIZE|CV_WINDOW_FREERATIO);
  }

  ~ImageConverter()
  {
    cv::destroyWindow("inRange");
		cv::destroyWindow("fit_ellipse");
  }

void find_color_(cv::Mat& mat)
{
    Mat dst;
    Scalar s_min = Scalar(150,  0,  0);
    Scalar s_max = Scalar(255, 150, 150);

		inRange(mat,s_min,s_max,dst);

    //inRange(mat,Scalar(0,60,0),Scalar(255,255,255),dst);
    //inRange(hsv,Scalar(0,0,49),Scalar(47,165,111),dst);
    cout<<"dst channels:"<<dst.channels()<<endl;

    int c0=0,c255=0,other=0;
    for(int i=0;i<dst.rows;i++){
        for(int j=0;j<dst.cols;j++){
            int v = dst.at<uchar>(i,j);
            if(v == 0){
                c0++;
            }else if(v == 255){
                c255++;
            }else{
                other++;
            }
        }
    }

    //cout<<"0 count:"<<c0<<",255 count:"<<c255<<",other value count:"<<other<<"over!!!"<<endl;
    cv::erode(dst,dst,cv::Mat(),cv::Point(1,1),6);
    cv::dilate(dst,dst,cv::Mat(),cv::Point(1,1),6);

    cv::medianBlur(dst,dst,15);

    int x = dst.cols / 2;
    int y = dst.rows / 2;
    int B = dst.at<Vec3b>(y, x)[0];
    int G = dst.at<Vec3b>(y, x)[1];
    int R = dst.at<Vec3b>(y, x)[2];
    cout<<"~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~^"<<endl;
    cout << "B:" << B << "G:" << G << "R:" << R << endl;
    cv::imshow("inRange",dst);

    vector< vector<Point> > contours;
    vector<Vec4i> hierarchy;
    Mat temp;
    dst.copyTo(temp);

	// 輪郭の検出
    findContours(temp,contours,hierarchy,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE );

    cout<<"contours size:"<<contours.size()<<endl;
    cout<<"hierarchy size:"<<hierarchy.size()<<endl;



	for(int i = 0; i < contours.size(); ++i) {
    	size_t count = contours[i].size();
    	if(count < 150 || count > 1000) continue; // （小さすぎる|大きすぎる）輪郭を除外

    	cv::Mat pointsf;
    	cv::Mat(contours[i]).convertTo(pointsf, CV_32F);
    	// 楕円フィッティング
    	cv::RotatedRect box = cv::fitEllipse(pointsf);
    	// 楕円の描画
    	cv::ellipse(mat, box, cv::Scalar(0,0,255), 2, CV_AA);

			//image_pub_.publish(box.center);
      center.num = i + 1;
			center.xcenter = box.center.x;
			center.ycenter = box.center.y;
			centers.ellipses.push_back(center);
  	}
    point_pub_.publish(centers);

    //cv::imshow("fit_ellipse",mat);


  	//cv::waitKey(0);



 // }
};

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Draw an example circle on the video stream
    //if (image.rows > 60 && image.cols > 60)
     // cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

    //find color..............
    cout<<"~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~^"<<endl;
    find_color_(image);
    cv::imshow("fit_ellipse",image);

    cv::waitKey(3);

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "find_bgr");
  ImageConverter ic;
  ros::spin();
  return 0;
}
