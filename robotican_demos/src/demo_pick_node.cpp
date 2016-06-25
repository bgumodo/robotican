//
// Created by tom on 22/06/16.
//

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <cv_bridge/cv_bridge.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <robotican_common/searchForColor.h>
#include <actionlib/client/simple_action_client.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PointStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <tf/transform_broadcaster.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

using namespace cv;
using namespace std;
using namespace pcl;

int image_w=0,image_h=0;

image_transport::Publisher image_pub_;
image_transport::Subscriber image_sub;


/*
 * //blue
 * i *nt minH=90,maxH=130;
 * int minS=70,maxS=255;
 * int minV=105,maxV=255;
 */
//green
//int minH=55,maxH=95;
//int minS=50,maxS=255;
//int minV=0,maxV=255;

//yellow
//int minH=18,maxH=43;
//int minS=16,maxS=240;
//int minV=132,maxV=212;

//red
int minH=3,maxH=160;
int minS=70,maxS=255;
int minV=70,maxV=255;
int minA=20000,maxA=50000;
int gaussian_ksize=0;
int gaussian_sigma=0;
int morph_size=0;

class DemoNavNode {
private:
    ros::NodeHandle _nodeHandle;
    ros::ServiceClient _searchForColorClient;
    ros::Subscriber _setColor;
    MoveBaseClient _moveBaseClient;

    std::string _color;
    bool _gotMission;

    void onSetColor(const std_msgs::String::ConstPtr &msg) {

        std::string prevColor = _color;
        _color = msg->data;
        _gotMission = true;
        if (_color != prevColor)
            ROS_INFO("[%s]: color set to: %s", ros::this_node::getName().c_str(), _color.c_str());

    }

public:
    DemoNavNode() : _nodeHandle(), _moveBaseClient("move_base", true) {
        std::string serviceName = "";
        _gotMission = false;
        if (_nodeHandle.getParam("search_for_color_topic", serviceName)) {

            _searchForColorClient = _nodeHandle.serviceClient<robotican_common::searchForColor>(serviceName);
            _setColor = _nodeHandle.subscribe<std_msgs::String>("set_color", 10, &DemoNavNode::onSetColor, this);

            while (!_searchForColorClient.waitForExistence(ros::Duration(5.0)))
                ROS_WARN("[%s]: Waiting for the %s server to come up", ros::this_node::getName().c_str(),
                         serviceName.c_str());

            while (!_moveBaseClient.waitForServer(ros::Duration(5.0)))
                ROS_WARN("[%s]: Waiting for the move_base action server to come up", ros::this_node::getName().c_str());

        }
        else {
            ROS_ERROR("[%s]: can't find 'search_for_color_topic' parameter", ros::this_node::getName().c_str());
            ros::shutdown();
        }
    }

    void run() {
        ros::Rate loopRate(100);
        while(ros::ok()) {
            if(_gotMission) {
                robotican_common::searchForColorResponse response;
                robotican_common::searchForColorRequest request;
                request.color = _color;
                if(_searchForColorClient.call(request, response)) {
                    if(response.valid) {
                        _gotMission = false;
                        move_base_msgs::MoveBaseGoal goal;
                        goal.target_pose.header.frame_id = response.pose.header.frame_id;
                        goal.target_pose.header.stamp = ros::Time::now();
                        goal.target_pose.pose = response.pose.pose;
                        ROS_INFO("[%s]: Sending goal", ros::this_node::getName().c_str());
                        _moveBaseClient.sendGoal(goal);
                        _moveBaseClient.waitForResult();
                        if(_moveBaseClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
                            ROS_INFO("[%s]: SUCCEEDED", ros::this_node::getName().c_str());
                        }
                        else {
                            ROS_ERROR("[%s]: FAILED ", ros::this_node::getName().c_str());
                        }
                    }

                }
            }
            loopRate.sleep();
        }

    }

};


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

    Mat bgr=cv_ptr->image;

    if (image_w==0) image_w=bgr.cols;
    if (image_h==0) image_h=bgr.rows;
    //ROS_INFO("AA");

}

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input) {

    if ((image_w==0)||(image_h==0)) return;

    pcl::PointCloud<pcl::PointXYZRGBA> cloud;
    pcl::fromROSMsg (*input, cloud);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudp (new pcl::PointCloud<pcl::PointXYZRGBA> (cloud));

    cv::Mat result;

    result = cv::Mat(image_h, image_w, CV_8UC3);

    if (!cloudp->empty()) {
        for (int h=0; h<image_h; h++) {
            for (int w=0; w<image_w; w++) {
                int pcl_index = (h*image_w) + w;
                pcl::PointXYZRGBA point = cloudp->at(pcl_index);
                if (point.z>0.1) {
                    Eigen::Vector3i rgb = point.getRGBVector3i();
                    result.at<cv::Vec3b>(h,w)[0] = point.b;
                    result.at<cv::Vec3b>(h,w)[1] = point.g;
                    result.at<cv::Vec3b>(h,w)[2] = point.r;
                }
                else {
                    result.at<cv::Vec3b>(h,w)[0]=0;
                    result.at<cv::Vec3b>(h,w)[1]=0;
                    result.at<cv::Vec3b>(h,w)[2]=0;
                }
            }
        }

    }
    else {
        ROS_WARN("empty cloud");
        return;
    }


    Mat hsv,gray,bgr,filtered,bw;
    //HSV filtering:

    cvtColor(result,hsv,CV_BGR2HSV);

    // inRange(hsv,Scalar(minH,minS,minV),Scalar(maxH,maxS,maxV),mask);

    // Threshold the HSV image, keep only the red pixels
    Mat lower_red_hue_range;
    Mat upper_red_hue_range;
    inRange(hsv, cv::Scalar(0, minS, minV), cv::Scalar(minH, maxS, maxV), lower_red_hue_range);
    inRange(hsv, cv::Scalar(maxH, minS, minV), cv::Scalar(179, maxS, maxV), upper_red_hue_range);
    // Combine the above two images
    Mat red_hue_image;
    addWeighted(lower_red_hue_range, 1.0, upper_red_hue_range, 1.0, 0.0, red_hue_image);
    GaussianBlur(red_hue_image, red_hue_image, cv::Size(9, 9), 2, 2);

    hsv.copyTo(filtered,red_hue_image);

    cvtColor(filtered,filtered,CV_HSV2BGR);

    Mat filtered2;
    resize(filtered, filtered2, Size(), 0.5, 0.5, INTER_LINEAR);
    imshow("filtered",filtered2);

red_hue_image.copyTo(bw);
    //cvtColor(filtered, bw, COLOR_BGR2GRAY);                       //back to gray

    //  imshow("GRAY_WINDOW",gray);
    //threshold( gray, bw, 0, 255,  0 );


    if (gaussian_ksize>0) {
        if (gaussian_ksize % 2 == 0) gaussian_ksize++;
        GaussianBlur( bw, bw, Size(gaussian_ksize,gaussian_ksize), gaussian_sigma , 0);
    }



    if (morph_size>0) {
        Mat element = getStructuringElement( MORPH_ELLIPSE, Size( 2*morph_size + 1, 2*morph_size+1 ), Point( morph_size, morph_size ) );
        morphologyEx( bw, bw, MORPH_CLOSE, element, Point(-1,-1), 1 );
    }
    Mat bw2;
    resize(bw, bw2, Size(), 0.5, 0.5, INTER_LINEAR);
    imshow("BW_WINDOW",bw2);


    vector<vector<Point> > contours;
            vector<Vec4i> hierarchy;

    findContours(bw, contours,hierarchy,CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    //   markers.clear();
    for( int i = 0; i< contours.size(); i++ )
    {
        double area0 = contourArea(contours[i]);

        //cout << " Contour " << i << "   A="<<area0<< endl;
        if ((area0>minA)&&(area0<maxA)) {
            //
            drawContours(result, contours, (int)i,  Scalar(0,0,255), 1, 8, hierarchy, 0);
            Moments mu=moments( contours[i], true );
            Point2f mc = Point2f( mu.m10/mu.m00 , mu.m01/mu.m00 );
            circle( result, mc, 4, Scalar(0,0,255), -1, 8, 0 );
            // markers.push_back(mc);


            int pcl_index = ((int)(mc.y)*result.cols) + (int)(mc.x);
            circle( result, mc, 8, Scalar(0,255,0), -1, 8, 0 );
            Point3d pr;
            pr.x=cloud[pcl_index].x;
            pr.y=cloud[pcl_index].y;
            pr.z=cloud[pcl_index].z;
            char str[100];
            if (isnan (pr.x) || isnan (pr.y) || isnan (pr.z) ) sprintf(str,"NaN");
            else sprintf(str,"[%.3f,%.3f,%.3f] ",pr.x,pr.y,pr.z);
            putText( result, str, mc, CV_FONT_HERSHEY_COMPLEX, 1, Scalar(255,0,0), 1, 8);

            transform.setOrigin( tf::Vector3(pr.x,pr.y,pr.z) );
            tf::Quaternion q;
            q.setRPY(0, 0, 0);
            transform.setRotation(q);
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "kinect2_link", "red"));
            ROS_INFO("%d  -  A: %d",i,(int)area0);
        }
        //  else  drawContours(result, contours, (int)i,  Scalar(0,255,255), 0, 8, hierarchy, 0);

    }
    resize(result, bgr, Size(), 0.5, 0.5, INTER_LINEAR);
    imshow("Blobs",bgr);


    waitKey(10);







    /*
    cv_bridge::CvImage out_msg;
    out_msg.header.stamp =ros::Time::now(); // Same timestamp and tf frame as input image
    out_msg.encoding = sensor_msgs::image_encodings::BGR8;
    out_msg.image    = result; // Your cv::Mat
    // Output modified video stream
    image_pub_.publish(out_msg.toImageMsg());
*/
    // imshow("Cloud Image",result);

    /*
    geometry_msgs::PointStamped m;
    m.header.stamp=ros::Time::now();
    m.header.frame_id=input->header.frame_id;
    m.point.x=pr.x;
    m.point.y=pr.y;
    m.point.z=pr.z;
*/
}
void on_trackbar( int, void* )
{
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "demo_nav_node");
    ros::NodeHandle n;
    image_transport::ImageTransport it_(n);

    image_sub = it_.subscribe("kinect2/hd/image_color", 1,imageCb);
    image_pub_ = it_.advertise("demo_nav/output_video", 1);

    ros::Subscriber pcl_sub = n.subscribe("kinect2/hd/points", 1, cloud_cb);

    namedWindow("Trackbars",CV_WINDOW_AUTOSIZE);              // trackbars window
    createTrackbar( "H min", "Trackbars", &minH, 180, on_trackbar );
    createTrackbar( "H max", "Trackbars", &maxH, 180, on_trackbar );
    createTrackbar( "S min", "Trackbars", &minS, 255, on_trackbar );
    createTrackbar( "S max", "Trackbars", &maxS, 255, on_trackbar );
    createTrackbar( "V min", "Trackbars", &minV, 255, on_trackbar );
    createTrackbar( "V max", "Trackbars", &maxV, 255, on_trackbar );
    createTrackbar( "gaussian_ksize", "Trackbars", &gaussian_ksize, 255, on_trackbar );
    createTrackbar( "gaussian_sigma", "Trackbars", &gaussian_sigma, 255, on_trackbar );
    createTrackbar( "A min", "Trackbars", &minA, 50000, on_trackbar );
    createTrackbar( "A max", "Trackbars", &maxA, 50000, on_trackbar );
    createTrackbar( "morph_size", "Trackbars", &morph_size, 50, on_trackbar );

    while (ros::ok())
    {
        ros::spinOnce();
    }


    return 0;
}
