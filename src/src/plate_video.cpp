#include "plate_video.h"

// --------------------------------------------------------
/// \概要:	msleep
///
/// \参数:	image
// --------------------------------------------------------
void sleep_ms(unsigned int millisec)
{
	struct timeval tval;
	tval.tv_sec = millisec / 1000;
	tval.tv_usec = (millisec * 1000) % 1000000;
	select(0, NULL, NULL, NULL, &tval);
}

// --------------------------------------------------------
/// \概要:	制作文字图片
///
/// \参数:	image
// --------------------------------------------------------
void Make_text_pic(cv::Mat &image)
{
	std::string text = std::string("地点名称: ") + std::string("longitude: ") + std::to_string(lon_) + std::string(" latitude: ") + std::to_string(lat_);
	cv::Point origin1(10, 40);
	Add_text_to_pic(image, text, origin1);
	char time_char[30] = "/0";
	struct timeval tv;
	struct timezone tz;
	gettimeofday(&tv, &tz);
//	struct tm *tmp_ptr = Get_current_time()
	struct tm *tmp_ptr = localtime(&tv.tv_sec);
	sprintf(time_char, "%04d/%02d/%02d %02d:%02d:%02d:%03ld", 1900 + tmp_ptr->tm_year, tmp_ptr->tm_mon + 1, tmp_ptr->tm_mday, tmp_ptr->tm_hour, tmp_ptr->tm_min, tmp_ptr->tm_sec, tv.tv_usec / 1000);
	std::string time_str = time_char;
//	std::cout << "time " << time_str << std::endl;
	text = std::string("经过时间: ") + time_str;
	cv::Point origin2(10, 100);
	Add_text_to_pic(image, text, origin2);
	text = std::string("设备编号: ") + car_vin_;
	cv::Point origin3(10, 160);
	Add_text_to_pic(image, text, origin3);
}

// --------------------------------------------------------
/// \概要:	添加文字到图片
///
/// \参数:	image
// --------------------------------------------------------
void Add_text_to_pic(cv::Mat &image, std::string text, cv::Point origin)
{
	int fontHeight = 50;
	int thickness = -1;
	int linestyle = 8;
	int baseline = 0;

	cv::Ptr<cv::freetype::FreeType2> ft2;
	ft2 = cv::freetype::createFreeType2();
	ft2->loadFontData(ch_ttf_ + std::string("platech.ttf"), 0);

	ft2->putText(image, text, origin, fontHeight, cv::Scalar(0, 0, 255), thickness, linestyle, true);
}

// --------------------------------------------------------
/// \概要:	获得当前时间
///
/// \返回:	struct tm *
// --------------------------------------------------------
struct tm *Get_current_time()
{
	time_t local_tmp;

	time(&local_tmp);
	struct tm *tmp_ptr = NULL;
	tmp_ptr = localtime(&local_tmp);

	return tmp_ptr;
}

// --------------------------------------------------------
/// \概要:	添加信息到vector
///
/// \参数:	plate
/// \参数:	frame
/// \参数:	plate_name
// --------------------------------------------------------
void Vector_push_back(std::vector<Plate_Info> &plate_, cv::Mat frame, std::string plate_name)
{
	Plate_Info plate_info;

	struct timeval tv;
	gettimeofday(&tv, NULL);

	plate_info.capTime = (tv.tv_sec * 1000 + tv.tv_usec / 1000);
	plate_info.plate_frame = frame, plate_info.plate_str = plate_name;
	if(plate_.size() == vector_size_)
	{
		std::vector<Plate_Info>::iterator it_delete = plate_.begin();
		plate_.erase(it_delete);
	}

	plate_.push_back(plate_info);
}

// --------------------------------------------------------
/// \概要:	前摄像头接收线程
// --------------------------------------------------------
void Receive()
{
	cv::VideoCapture cap;
	cap.open(video_path_);
	if(!cap.isOpened())
		std::cerr << "open video failed!" << std::endl;
	else
		std::cout << "open video success!" << std::endl;

	cv::Mat frame;
	bool isSuccess = true;
	while(1)
	{
		isSuccess = cap.read(frame);
		if(!isSuccess)
		{
			std::cerr << "video ends!" << endl;
			break;
		}

		que_cv.push(frame);
		if(que_cv.size() > 1)
			// 注意 pop和front的区别
			que_cv.pop();
		else
			sleep_ms(750);
	}
}

// --------------------------------------------------------
/// \概要:	前摄像头显示线程
// --------------------------------------------------------
void Display()
{
	//定义模型文件
	pr::PipelinePR prc(model_xml_ + "cascade.xml", model_xml_ + "HorizonalFinemapping.prototxt", model_xml_ + "HorizonalFinemapping.caffemodel", model_xml_ + "Segmentation.prototxt", model_xml_ + "Segmentation.caffemodel", model_xml_ + "CharacterRecognization.prototxt", model_xml_ + "CharacterRecognization.caffemodel", model_xml_ + "SegmenationFree-Inception.prototxt", model_xml_ + "SegmenationFree-Inception.caffemodel");

	cv::Mat image, frame;
	std::vector<Plate_Info>::iterator it;
	while(1)
	{
		sleep_ms(1500);
		if(!que_cv.empty())
		{
			image = que_cv.front();
			que_cv.pop();
			frame = image.clone();
		}
		else
			continue;

		int weight = image.cols, height = image.rows;
		cv::Mat image_intercept = Image_intercept(image);
		//使用端到端模型模型进行识别 识别结果将会保存在res里面
		std::vector<pr::PlateInfo> res = prc.RunPiplineAsImage(image_intercept, pr::SEGMENTATION_FREE_METHOD);

		for(auto st:res)
		{
		//	std::cout << "front " << st.getPlateName() << " : " << st.confidence << std::endl;
			if(st.confidence > 0.95)
			{
				//输出识别结果 、识别置信度
				string plate_name = st.getPlateName();

				if(plate_name.size() >= 9)
				{
				//	std::cout << "front " << st.getPlateName() << " : " << st.confidence << std::endl;
					if(plate_vector_.empty())
					{
						Make_text_pic(frame);
						Vector_push_back(plate_vector_, frame, plate_name);
						Pub_plate_pic_message(plate_name, st.getPlateImage(), frame);
					}
					else
					{
						it = find(plate_vector_.begin(), plate_vector_.end(), plate_name);
						if(it == plate_vector_.end())
						{
							Make_text_pic(frame);
							Vector_push_back(plate_vector_, frame, plate_name);
							Pub_plate_pic_message(plate_name, st.getPlateImage(), frame);
						}
					}
				}

				//获取车牌位置
				cv::Rect region = st.getPlateRect();
				//画出车牌位置
				cv::rectangle(image, cv::Point(region.x + weight / 7, region.y + height / 3), cv::Point(region.x + region.width + weight / 7, region.y + region.height + height / 3), cv::Scalar(255, 255, 0), 2);
			}
		}

		cv::rectangle(image, cv::Point(weight / 7, height / 3), cv::Point(weight / 7 * 6, height), cv::Scalar(0, 0, 255), 2);

#if 0
		sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
		pub_image_.publish(msg);
#endif
	}
}

// --------------------------------------------------------
/// \概要:	发布车牌图片信息
///
/// \参数:	output_image
// --------------------------------------------------------
void Pub_plate_pic_message(std::string plate_str, cv::Mat license_plate_pic, cv::Mat license_plate_scense_pic)
{
	face_plate_msgs::Plate_pic plate_pic_msg;
	plate_pic_msg.vin = "as00030";
	plate_pic_msg.deviceId = "030车牌";
	plate_pic_msg.pictureType = 2;
	plate_pic_msg.licenseNum = plate_str;
	plate_pic_msg.plateColor = 0;
	plate_pic_msg.carColor = 0;
	plate_pic_msg.carType = 0;
	struct timeval tv;
	gettimeofday(&tv, NULL);
	plate_pic_msg.capTime = (tv.tv_sec * 1000 + tv.tv_usec / 1000);
	plate_pic_msg.licensePlatePicture = Mat_to_Base64(license_plate_pic, std::string("jpg"));
	plate_pic_msg.licensePlateScenePicture = Mat_to_Base64(license_plate_scense_pic, std::string("jpg"));

	pub_plate_pic_message_.publish(plate_pic_msg);
}

// --------------------------------------------------------
/// \概要:	图像截取函数
///
/// \参数:	middle_frame
///
/// \返回:	cv::Mat
// --------------------------------------------------------
cv::Mat Image_intercept(cv::Mat frame)
{
	int weight = frame.cols, height = frame.rows;
	cv::Rect rect(weight / 7, height / 3, weight / 7 * 5, height / 3 * 2);
	cv::Mat rect_frame = frame(rect);

	return rect_frame;
}

// --------------------------------------------------------
/// \概要:	回调函数
///
/// \参数:	msg
// --------------------------------------------------------
void LocationMsgCallback(const location::location::ConstPtr &msg)
{
	lon_ = msg->gps.lon, lat_ = msg->gps.lat;
}

// --------------------------------------------------------
/// \概要:	主函数
///
/// \参数:	argc
/// \参数:	argv[]
///
/// \返回:	int
// --------------------------------------------------------
int main(int argc, char *argv[])
{
	ros::init(argc, argv, "plate_detection");
	ros::NodeHandle nh_("~");
	ros::Time time = ros::Time::now();
	ros::Rate loop_rate(10);

	nh_.param("/model_xml", model_xml_, std::string("./models/mtcnn_frozen_model.pb"));
	nh_.param("/ch_ttf", ch_ttf_, std::string("./ttf/platech.ttf"));
	nh_.param("/output_image", output_image_, std::string("./output_image/output_image01.jpg"));
	nh_.param("video_path", video_path_, std::string("test.mp4"));
	nh_.param("cam", cam_, std::string("right_front"));

	video_path_ = video_path_ + video_sub_;

#if 0
	image_transport::ImageTransport it(nh_);
	pub_image_ = it.advertise("/camera/image_" + cam_, 1);
#endif

	pub_plate_pic_message_ = nh_.advertise<face_plate_msgs::Plate_pic>("/plate_pic_msg", 1);

	sub_gps_ = nh_.subscribe("/location", 10, LocationMsgCallback);

	std::thread thread_1(Receive);
	std::thread thread_2(Display);

	ros::spin();

	thread_1.join();
	thread_2.join();

	return 0 ;
}
