#include <opencv2/opencv.hpp>
#include "yolov8trt_ros/cpm.hpp"
#include "yolov8trt_ros/infer.hpp"
#include "yolov8trt_ros/yolo.hpp"
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "cv_bridge/cv_bridge.h"
#include "yolov8_ros_msgs/BoundingBox.h"
#include "yolov8_ros_msgs/BoundingBoxes.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int64.h"
using namespace std;
//权重路径
std::string model_path = "/home/lishuai/code/YOLOv8_Tensorrt/build/best_fp16.trt";
ros::Publisher image_pub;
ros::Publisher position_pub;
yolov8_ros_msgs::BoundingBoxes yoloboxes;
//类别标签（使用自己的数据集需要修改）
static const char *cocolabels[] = { "apple"};
void single_inference(cv::Mat &img_ori, cv::Mat &img_out, std::string model_path,std_msgs::Header header);
yolo::Image cvimg(const cv::Mat &image) { return yolo::Image(image.data, image.cols, image.rows); }
// 回调函数，处理接收到的图像消息，并发布新的图像消息
void imageCallbackAndPublish(const sensor_msgs::ImageConstPtr& msg) {
	cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
	cv::Mat image_result;
	

	// 设置图像消息的头信息
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = "d435_link";
	yoloboxes.header = header;
	yoloboxes.image_header = header;
	single_inference(image,image_result,model_path,header);
	

    // 使用cv_bridge将OpenCV图像转换为ROS图像消息
    cv_bridge::CvImage cv_image(header, sensor_msgs::image_encodings::BGR8, image_result);

    // 发布图像消息
    image_pub.publish(cv_image.toImageMsg());

    
}

//预测单张图像
void single_inference(cv::Mat &img_ori, cv::Mat &img_out, std::string model_path,std_msgs::Header header) {
	cv::Mat image = img_ori.clone();
	float confidence_threshold = 0.5f;
	float nms_threshold = 0.5f;
	auto yolo = yolo::load(model_path, yolo::Type::V8, confidence_threshold, nms_threshold);
	if (yolo == nullptr) return;

	auto objs = yolo->forward(cvimg(image));
	
	int i = 0;int j=0;
	for (auto &obj : objs) {
		uint8_t b, g, r;
		tie(b, g, r) = yolo::random_color(obj.class_label);
		cv::rectangle(image, cv::Point(obj.left, obj.top), cv::Point(obj.right, obj.bottom),
			cv::Scalar(b, g, r), 5);

		auto name = cocolabels[obj.class_label];
		auto caption = cv::format("%s %.2f", name, obj.confidence);
		int width = cv::getTextSize(caption, 0, 1, 2, nullptr).width + 10;
		cv::rectangle(image, cv::Point(obj.left - 3, obj.top - 33),
			cv::Point(obj.left + width, obj.top), cv::Scalar(b, g, r), -1);
		cv::putText(image, caption, cv::Point(obj.left, obj.top - 5), 0, 1, cv::Scalar::all(0), 2, 16);
		yolov8_ros_msgs::BoundingBox boundingBox;
        boundingBox.xmin = (int64)obj.left;
        boundingBox.ymin = (int64)obj.top;
        boundingBox.xmax = (int64)obj.right;
        boundingBox.ymax = (int64)obj.bottom;
        boundingBox.Class = (string)name;
		
		// cout<<"obj.left:"<<obj.left<<"obj.right:"<<obj.right<<"obj.bottom:"<<obj.bottom<<"obj.top:"<<obj.top<<"obj.confidence:"<<obj.confidence<<endl;
        boundingBox.probability = (double)obj.confidence;
		yoloboxes.bounding_boxes.push_back(boundingBox);
		position_pub.publish(yoloboxes);
		if (obj.seg) {
			cv::imwrite(cv::format("%d_mask.jpg", i),
				cv::Mat(obj.seg->height, obj.seg->width, CV_8U, obj.seg->data));
			i++;
		}
	}

	img_out = image.clone();
}

int main(int argc,char** argv) {
	//初始化ros节点
	ros::init(argc,argv,"yolov8trt_node");
	ros::NodeHandle nh;
	string topic_pub = "/yolov8/detection_image";
	string topic_sub = "/d435/color/image_raw";
	nh.param("model_path",model_path,model_path);                                   //模型路径
	 nh.param("topic_sub",topic_sub,std::string("/d435/color/image_raw")); 				//订阅的话题名称
	ros::Subscriber image_sub = nh.subscribe<sensor_msgs::Image>(topic_sub,30,imageCallbackAndPublish);
	image_pub = nh.advertise<sensor_msgs::Image>("/yolov8/detection_image", 30);
	position_pub = nh.advertise<yolov8_ros_msgs::BoundingBoxes>("/yolov8/BoundingBoxes",10);

	ros::spin();
	return 0;
}

