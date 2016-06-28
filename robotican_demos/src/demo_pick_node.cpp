


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
#include <std_msgs/Empty.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/GripperCommandAction.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

typedef actionlib::SimpleActionClient<control_msgs::GripperCommandAction> GripperClient;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

using namespace cv;
using namespace std;
using namespace pcl;

bool debug_vision=true;

Point3d find_object(Mat input, pcl::PointCloud<pcl::PointXYZRGBA> cloud);
void go(tf::Transform  dest);
bool gripper_cmd(double gap,double effort);
bool arm_cmd();
bool base_cmd(move_base_msgs::MoveBaseGoal goal);
void pick_go_cb(std_msgs::Empty);
void button_go_cb(std_msgs::Empty);

//tf::Transform *obj_transform_ptr=NULL,*dest_transform_ptr=NULL;
GripperClient *gripperClient_ptr;
MoveBaseClient *moveBaseClient_ptr;
moveit::planning_interface::MoveGroup *moveit_ptr;
tf::TransformListener *listener_ptr;

tf::StampedTransform obj_transform;

geometry_msgs::PoseStamped target_pose1;
bool have_goal=false;
bool tf_ok=false;
int image_w=0,image_h=0;
bool moving=false;
bool have_object=false;
image_transport::Publisher image_pub_;
image_transport::Subscriber image_sub;

 ros::Publisher planning_scene_diff_publisher;
ros::Publisher goal_pub;

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
int minA=7000,maxA=50000;
int gaussian_ksize=0;
int gaussian_sigma=0;
int morph_size=0;

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

}

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input) {

    if ((image_w==0)||(image_h==0)||(moving)) return;

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
                    // Eigen::Vector3i rgb = point.getRGBVector3i();
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


    Point3d obj = find_object(result,cloud);

    waitKey(1);

    if (!have_object) {
        tf_ok=false;
        return;
    }



    if (moving) return;
    obj_transform.setOrigin( tf::Vector3(obj.x,obj.y,obj.z) );
    tf::Quaternion q;
    q.setRPY(0.0, 0, 0);
    obj_transform.setRotation(q);
    obj_transform.frame_id_="kinect2_depth_optical_frame";
    obj_transform.child_frame_id_="red_object";
    obj_transform.stamp_=ros::Time::now();
    //dest_transform.setRotation(q);

    tf_ok=true;



}

Point3d find_object(Mat input, pcl::PointCloud<pcl::PointXYZRGBA> cloud) {
    have_object=false;
    Mat hsv,bgr,filtered,bw;
    //HSV filtering:

    Point3d pr = Point3d(NAN,NAN,NAN);
    cvtColor(input,hsv,CV_BGR2HSV);

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
   if (debug_vision) imshow("filtered",filtered2);

    red_hue_image.copyTo(bw);
    if (gaussian_ksize>0) {
        if (gaussian_ksize % 2 == 0) gaussian_ksize++;
        GaussianBlur( bw, bw, Size(gaussian_ksize,gaussian_ksize), gaussian_sigma , 0);
    }


    if (morph_size>0) {
        Mat element = getStructuringElement( MORPH_ELLIPSE, Size( 2*morph_size + 1, 2*morph_size+1 ), Point( morph_size, morph_size ) );
        morphologyEx( bw, bw, MORPH_CLOSE, element, Point(-1,-1), 1 );
    }
    Mat bw2;
    // resize(bw, bw2, Size(), 0.5, 0.5, INTER_LINEAR);
    // if (debug_vision)   imshow("BW_WINDOW",bw2);


    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;

    findContours(bw, contours,hierarchy,CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

    double largest_area=0;
    int largest_contour_index=0;
    for( int i = 0; i< contours.size(); i++ )
    {
        double area0 = contourArea(contours[i]);
        if(area0>largest_area){
            largest_area=area0;
            largest_contour_index=i;
        }
    }
    if ((largest_area>minA)&&(largest_area<maxA)) {

        //
        drawContours(input, contours, (int)largest_contour_index,  Scalar(255,0,0), 3, 8, hierarchy, 0);
        Moments mu=moments( contours[largest_contour_index], true );
        Point2f mc = Point2f( mu.m10/mu.m00 , mu.m01/mu.m00 );
        circle( input, mc, 4, Scalar(0,0,255), -1, 8, 0 );
        int pcl_index = ((int)(mc.y)*input.cols) + (int)(mc.x);
        circle( input, mc, 8, Scalar(0,255,0), -1, 8, 0 );

        pr.x=cloud[pcl_index].x;
        pr.y=cloud[pcl_index].y;
        pr.z=cloud[pcl_index].z;
        char str[100];
        if (isnan (pr.x) || isnan (pr.y) || isnan (pr.z) ) sprintf(str,"NaN");
        else sprintf(str,"[%.3f,%.3f,%.3f] A=%lf",pr.x,pr.y,pr.z,largest_area);
        putText( input, str, mc, CV_FONT_HERSHEY_COMPLEX, 1, Scalar(255,0,0), 1, 8);
        have_object=true;
    }


    resize(input, bgr, Size(), 0.5, 0.5, INTER_LINEAR);
    if (debug_vision) imshow("Blobs",bgr);

    return pr;
}

void on_trackbar( int, void* ){}

void button_go_cb(std_msgs::Empty) {


        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position.x=-9.558070;
        goal.target_pose.pose.position.y=4.108188;
        goal.target_pose.pose.position.z=0;
        goal.target_pose.pose.orientation= tf::createQuaternionMsgFromRollPitchYaw(0,0,CV_PI );

        if (base_cmd(goal)) {

            ROS_INFO("REACHED BUTTON");
            goal.target_pose.pose.position.x=-9.024430;
            goal.target_pose.pose.position.y=7.220850;
            if (base_cmd(goal)) {

                ROS_INFO("REACHED TABLE");
                std_msgs::Empty e;
                pick_go_cb(e);
            }
        }
}
void pick_go_cb(std_msgs::Empty) {

    if (moving) return;
    else if (!moving) {
        moving=true;
        tf_ok=false;
        destroyAllWindows();
    }

    tf::Transform dest_transform;
    try{

        tf::StampedTransform transform_obj,transform_base;
        listener_ptr->lookupTransform("map", "red_object", ros::Time(0), transform_obj);

        obj_transform.frame_id_="map";
        obj_transform.child_frame_id_="red_object";

        obj_transform.setData(transform_obj);

        listener_ptr->lookupTransform("map", "base_link", ros::Time(0), transform_base);
        tf::Vector3 v_obj =transform_obj.getOrigin();
        tf::Vector3 v_base =transform_base.getOrigin();



        tf::Vector3 v=v_obj-v_base;
        float yaw=atan2(v.y(),v.x());
        float away=0.55/sqrt(v.x()*v.x()+v.y()*v.y());
        tf::Vector3 dest=v_base+v*(1-away);
        dest.setZ(0);
        dest_transform.setOrigin( dest );
        tf::Quaternion q;
        q.setRPY(0.0, 0, yaw);
        dest_transform.setRotation(q);
    }

    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        return;
    }


    ROS_INFO("%f %f",dest_transform.getOrigin().x(),dest_transform.getOrigin().y());

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x=dest_transform.getOrigin().x();
    goal.target_pose.pose.position.y=dest_transform.getOrigin().y();
    goal.target_pose.pose.position.z=0;
    goal.target_pose.pose.orientation.x=dest_transform.getRotation().x();
    goal.target_pose.pose.orientation.y=dest_transform.getRotation().y();
    goal.target_pose.pose.orientation.z=dest_transform.getRotation().z();
    goal.target_pose.pose.orientation.w=dest_transform.getRotation().w();

    if (base_cmd(goal)) {
        if(gripper_cmd(0.14,0.0)) {
            if (arm_cmd()) {
                ROS_INFO("MOVING!");
                moveit_ptr->move();
                ROS_INFO("DONE MOVING!");
                 if(gripper_cmd(0.01,0.0)) {
                      ROS_INFO("HOLDING");
                      ros::Duration w(5);
                      w.sleep();
                       ROS_INFO("LIFTING");
                      target_pose1.pose.position.x = 0.4;
                      target_pose1.pose.position.y =  0.0;
                      target_pose1.pose.position.z =  0.845;
                      target_pose1.pose.orientation= tf::createQuaternionMsgFromRollPitchYaw(0,CV_PI/2.0,0 );
                      goal_pub.publish(target_pose1);
                      moveit_ptr->setPoseTarget(target_pose1);
                      moveit::planning_interface::MoveGroup::Plan my_plan;
                      bool success = moveit_ptr->plan(my_plan);
                      ROS_INFO("moveit final plan (pose goal) %s",success?"SUCCESS":"FAILED");
                     // WRIST      ROLL: -0.162121      PITCH: 1.24426      YAW: -0.192632
                     // WRIST    x: 0.407369, y: 0.0103332, z: 0.846887
                     if (success)  {
                         moveit_ptr->move();
                         ROS_INFO("ALL DONE");
                        // moving=false;
                     }

                 }


            }
        }
    }
}

bool arm_cmd() {

    tf::StampedTransform transform_base_obj;
    try{

        listener_ptr->lookupTransform("base_link", "red_object", ros::Time(0), transform_base_obj);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        return false;
    }

    //  double 	roll=0;
    //  double 	pitch=0;
    tf::Vector3 v= transform_base_obj.getOrigin();

    double 	yaw=atan2(v.y(),v.x());
    //tf::Quaternion tf::createQuaternionFromRPY(roll,pitch,yaw );
    // tf::Quaternion quat=transform_base_obj.getRotation();
    // tf::Quaternion q(quat.x(), quat.y(), quat.z(), quat.w());
    //  tf::Matrix3x3 m(q);
    //  m.getRPY(roll, pitch, yaw);
    // std::cout << "Yaw: " << yaw << std::endl;
    // roll=0;
    // pitch=0;


    moveit_ptr->setStartStateToCurrentState();

    target_pose1.header.frame_id="base_link";
    target_pose1.header.stamp=ros::Time::now();


    float away=0.05/sqrt(v.x()*v.x()+v.y()*v.y());
    tf::Vector3 dest=v*(1-away);


    target_pose1.pose.position.x = dest.x();
    target_pose1.pose.position.y =  dest.y();
    target_pose1.pose.position.z =  v.z()-0.02;
    // target_pose1.pose.orientation.w=1.0;
    target_pose1.pose.orientation= tf::createQuaternionMsgFromRollPitchYaw(-yaw,CV_PI/2.0,0 );
    goal_pub.publish(target_pose1);
    //std::cout << "goal in base_link frame    x: " << target_pose1.pose.position.x << ", y: " << target_pose1.pose.position.y << ", z: " << target_pose1.pose.position.z << std::endl;

    // std::cout << "GOAL      ROLL: " << roll << "      PITCH: " << pitch << "      YAW: " << yaw << std::endl;
    //
    //  tf::quaternionTFToMsg( transform_base_obj.getRotation(), target_pose1.pose.orientation);
    moveit_ptr->setPoseTarget(target_pose1);
have_goal=true;
    moveit::planning_interface::MoveGroup::Plan my_plan;
    // my_plan.planning_time_=10.0;
    bool success = moveit_ptr->plan(my_plan);
    ROS_INFO("moveit plan (pose goal) %s",success?"SUCCESS":"FAILED");

    return success;
}

bool base_cmd(move_base_msgs::MoveBaseGoal goal) {


    ROS_INFO("[%s]: Sending goal", ros::this_node::getName().c_str());

    moveBaseClient_ptr->sendGoal(goal);
    moveBaseClient_ptr->waitForResult();

    if(moveBaseClient_ptr->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("[%s]: BASE NAV SUCCEEDED", ros::this_node::getName().c_str());


        return true;


    }
    else {
        ROS_ERROR("[%s]: BASE NAV FAILED ", ros::this_node::getName().c_str());
        return false;
    }
}

bool gripper_cmd(double gap,double effort) {

    control_msgs::GripperCommandGoal openGoal;

    openGoal.command.position = gap;
    openGoal.command.max_effort = effort;
    gripperClient_ptr->sendGoal(openGoal);
  ROS_INFO("Sent gripper goal");
    gripperClient_ptr->waitForResult();

    if(gripperClient_ptr->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Gripper done");
        return true;
    }
    else {
        ROS_ERROR("Gripper fault");
       // return false;
    }
    return false;
}


int main(int argc, char **argv) {

    ros::init(argc, argv, "demo_pick_node");
    ros::AsyncSpinner spinner(3);

    ros::NodeHandle n;
    image_transport::ImageTransport it_(n);

    image_sub = it_.subscribe("kinect2/hd/image_color", 1,imageCb);
    image_pub_ = it_.advertise("demo_nav/output_video", 1);

    GripperClient gripperClient("/gripper_controller/gripper_cmd", true);
    if (!gripperClient.waitForServer(ros::Duration(5.0))) {
        ROS_WARN("[%s]: No gripper action server", ros::this_node::getName().c_str());
        return -1;
    }
    gripperClient_ptr=&gripperClient;

    MoveBaseClient moveBaseClient("move_base", true);
    if (!moveBaseClient.waitForServer(ros::Duration(5.0))) {
        ROS_WARN("[%s]: No move_base action server", ros::this_node::getName().c_str());
        return -1;
    }
    moveBaseClient_ptr=&moveBaseClient;


    moveit::planning_interface::MoveGroup group("arm");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;





    ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
    ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());
    group.allowReplanning(true);
    group.setPlanningTime(10.0);
    group.setNumPlanningAttempts(5);
    group.setPlannerId("RRTConnectkConfigDefault");
group.setMaxAccelerationScalingFactor(0.01);
group.setMaxVelocityScalingFactor(0.01);
    // group.setGoalPositionTolerance(0.2);
    // group.setGoalOrientationTolerance(0.5);

    moveit_ptr=&group;
    ros::Subscriber pcl_sub = n.subscribe("kinect2/hd/points", 1, cloud_cb);

    ros::Subscriber pick_sub = n.subscribe("pick_go", 1, pick_go_cb);
    ros::Subscriber button_sub = n.subscribe("button_go",1,button_go_cb);

    goal_pub=n.advertise<geometry_msgs::PoseStamped>("pick_moveit_goal", 2, true);
    ros::Publisher wr_pub=n.advertise<geometry_msgs::PoseStamped>("wrist", 2, true);

    if (debug_vision) {
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
}
    tf::TransformBroadcaster br;

    tf::TransformListener listener;
    listener_ptr=&listener;

    ros::Rate r(50); // 100 hz

    spinner.start();
    while (ros::ok())
    {
        //  ROS_INFO("%d %d",tf_ok,moving);
        if ((tf_ok)||(moving)) {
            //ROS_INFO("sendTransform");
            obj_transform.stamp_=ros::Time::now();
            br.sendTransform(obj_transform);
            //ROS_INFO("sendTransform done");
            // br.sendTransform(tf::StampedTransform(dest_transform, ros::Time::now(), "map", "dest"));

            if (have_goal) {
                tf::StampedTransform wr_goal;
                tf::Quaternion q;
                tf::quaternionMsgToTF(target_pose1.pose.orientation,q);
                wr_goal.setOrigin(tf::Vector3(target_pose1.pose.position.x,target_pose1.pose.position.y,target_pose1.pose.position.z));
                wr_goal.setRotation((q));


                wr_goal.stamp_=ros::Time::now();
                wr_goal.child_frame_id_="wrist_goal";
                wr_goal.frame_id_="base_link";
                br.sendTransform(wr_goal);
            }
        }
        // ros::spinOnce();
        tf::StampedTransform wr;
        try{
            listener_ptr->lookupTransform("base_link", "wrist_link", ros::Time(0), wr);
            double roll,pitch,yaw;
            tf::Quaternion quat=wr.getRotation();
            tf::Quaternion q(quat.x(), quat.y(), quat.z(), quat.w());
            tf::Matrix3x3 m(q);
            m.getRPY(roll, pitch, yaw);

        //    std::cout << "WRIST      ROLL: " << roll << "      PITCH: " << pitch << "      YAW: " << yaw << std::endl;

            geometry_msgs::PoseStamped target_pose2;
            target_pose2.header.frame_id="base_link";
            target_pose2.header.stamp=ros::Time::now();
            target_pose2.pose.position.x = wr.getOrigin().x();
            target_pose2.pose.position.y = wr.getOrigin().y();
            target_pose2.pose.position.z = wr.getOrigin().z();
            // target_pose1.pose.orientation.w=1.0;
            tf::quaternionTFToMsg( wr.getRotation(), target_pose2.pose.orientation);
            wr_pub.publish(target_pose2);
          //  std::cout << "WRIST    x: " << target_pose2.pose.position.x << ", y: " << target_pose2.pose.position.y << ", z: " << target_pose2.pose.position.z << std::endl;

        }
        catch (tf::TransformException ex){
           // ROS_ERROR("%s",ex.what());
        }

        r.sleep();
    }


    return 0;
}
