#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>

int main( int argc, char **argv )
{
	// Inicia o ROS
	ros::init( argc, argv, "camera_driver" );
	ros::NodeHandle nh;
	
	// Criar o image transport publisher
	image_transport::ImageTransport it(nh);
	image_transport::Publisher pub = it.advertise("camera/image", 100);
	
	// Capturar o video da camera
	int cam_id;
	if ( !ros::param::get("~cam_id", cam_id) )
	{
		ROS_FATAL("Nao foi possivel ler o parametro id da camera.");
		exit(1);
	}
	
	
	cv::VideoCapture cap(cam_id); // open the default camera
    if(!cap.isOpened())  // check if we succeeded
    {
        ROS_ERROR("Nao foi possivel abrir o dispositivo %d.", cam_id);
        ros::shutdown();
	}
	
	
	ros::Rate rate(30);
	while(nh.ok())
	{
		// Obter um frame da imagem
		cv::Mat frame;
		cap >> frame;
		
		// Transformar o frame em uma mensagem do tipo imagem e salvar em uma variavel
		sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
		
		// Publicar a mensagem
		pub.publish(msg);
		rate.sleep();
	}	
}
