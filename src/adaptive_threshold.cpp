/***********************************************************************
 universidade federal de itajuba - UNIFEI
 mestrado em ciencia e tecnologia da computacao
 ruan costa - ruan.scosta@gmail.com
 wander mendes martins - wandermendes@unifei.edu.br
 rafael gomes braga    - rafaelbraga@unifei.edu.br
************************************************************************
Este programa executa as seguintes tarefas:
	1) recebe uma imagem da camera embarcada em um VANT
	2) a trasnforma em escalas de cinza
	3) aplica binarização baseada em limiar médio
	4) divide a imagem em 3 quadrantes horizontais de mesmo tamanho
	5) identifica o quadrante com maior número de pixels brancos
	6) envia comandos ao DRONE para se dirigir a este quadrante 
***********************************************************************/

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <mavros_msgs/OverrideRCIn.h>
#include <mavros_msgs/State.h>

#include <AdaptiveThreshold.h>


using namespace cv;

#define FACTOR  0.6

#define MINRC   1100
#define BASERC  1500
#define MAXRC   1900

// Subscriber to bottom camera
image_transport::Subscriber sub;

// Subscriber to flight mode
ros::Subscriber mavros_state_sub;

// RC publisher
ros::Publisher pub;

double Roll, Pitch;

// Flight mode
std::string mode;
bool guided;
bool armed;










void adaptiveThreshold(unsigned char* input, unsigned char* output, int width, int height)
{
	long sum = 0;
	unsigned long* integralImage;
	
	int i, j;
	int count = 0;
	int index;
	int x1, y1, x2, y2;
	int s = width / 8;

	integralImage = (unsigned long*)malloc(width * height * sizeof(unsigned long*));

	for (i = 0; i < width; i++)
	{
		sum = 0;

		for (j = 0; j < height; j++)
		{
			index = j * width + i;

			sum += input[index];
			if (i == 0)
				integralImage[index] = sum;
			else
				integralImage[index] = integralImage[index - 1] + sum;
		}
	}

	for (i = 0; i < width; i++)
	{
		for (j = 0; j < height; j++)
		{
			index = j * width + i;

			x1 = i - s; x2 = i + s;
			y1 = j - s; y2 = j + s;

			if (x1 < 0) x1 = 0;
			if (x2 >= width) x2 = width - 1;
			if (y1 < 0) y1 = 0;
			if (y2 >= height) y2 = height - 1;

			count = (x2 - x1)*(y2 - y1);

			sum = integralImage[y2 * width + x2] - 
				integralImage[y1 * width + x2] -
				integralImage[y2 * width + x1] +
				integralImage[y1 * width + x1];

			if ((long)(input[index] * count) < (long)(sum * (1.0 - T)))
				output[index] = 0;
			else
				output[index] = 255;
		}
	}

	free(integralImage);
}






void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{	
	// Variáveis
	Mat imageOriginal, imageGrayscale;
	IplImage* imageBinary;
    int vTercaParte;
    int vmostrarimagens = 1; //0=nao 1=sim
    int rollAction, pitchAction; 
	int quadrante[3] = {0,0,0};
    int areaVANT;
    	
	//23/03/17-wander
	int limiar; // valor limite de cinza. 
	            // Acima dele pintar acender o pixel (255-branco), e 
	            // abaixo dele apagar o pixel (0-preto) 
	
    try
    {
        // Get the msg image
        imageOriginal = cv_bridge::toCvShare(msg, "bgr8")->image;
        imageGrayscale = cv_bridge::toCvShare(msg, "mono8")->image;
        
        
        
        int width = imageOriginal.cols;
		int height = imageOriginal.rows;

		imageBinary = cvCreateImage(cvSize(width, height), 8, 1);

		adaptiveThreshold((unsigned char*)imageGrayscale.data, (unsigned char*)imageBinary->imageData, width, height);
        
        
        
        
        
        
        
        //divide the photo by 3 areas (columns) 
		vTercaParte   = imageOriginal.cols / 9;

		if (vmostrarimagens == 1) {
		   namedWindow("Source Image", WINDOW_AUTOSIZE );
		   imshow("Source Image", imageOriginal);
		}   
		// convert the source image to a gray scale image
		//cvtColor(image, ImageGrayScale, CV_BGR2GRAY);
		// show the gray scale image 
		if (vmostrarimagens == 1) {
		  namedWindow("gray scale Image", WINDOW_AUTOSIZE );
		  imshow("gray scale Image", imageGrayscale);
		}  

        if (vmostrarimagens == 1) 
	    {
			namedWindow("Black and White Full Image", WINDOW_AUTOSIZE );
			cvShowImage("Black and White Full Image", imageBinary);
	    }       
      
        // descobre a quantidade de pixels pretos (obstáculos) em cada quadrante
	    for( int y = 0; y < height; y++ )
	     for( int x = 0; x < width; x++ )
	      for( int c = 0; c < 3; c++ )
			if (x < vTercaParte) 
			   quadrante[c]++; //Left Side
			else   
			if ( (x > vTercaParte) && (x < 2*vTercaParte) )
			   quadrante[c]++; //Center
			else   
			   quadrante[c]++; // Right Side
			   
			   
		// Create RC msg
        mavros_msgs::OverrideRCIn msg;
        
        
        // DECIDE PARA ONDE O DRONE DEVE SE MOVER
        // SUGERE O QUADRANTE DE MENOR OBSTÁCULOS (MENOS PIXELS PRETOS)
	    if (vmostrarimagens == 1) 
	    {
			printf("\nQuadrante da esquerda com %i pixels pretos\n",quadrante[0]);
			printf("Quadrante do centro com %i pixels pretos\n",quadrante[1]);
			printf("Quadrante da direita com %i pixels pretos\n",quadrante[2]);
	    }
	    if (  (quadrante[0] > (height*width*80/100))
			& (quadrante[1] > (height*width*80/100))
			& (quadrante[2] > (height*width*80/100))
		   )
	    {
			printf("\nParar");
			rollAction = 0;
			pitchAction = 0;
	    }
	    else 
	    {
       
			if ((quadrante[0] < quadrante[1]) && (quadrante[0] < quadrante[2]) )
			{
				printf("\nMover para a esquerda");
				rollAction = 100;
				pitchAction = 100;
			}
			else
				if ((quadrante[1] < quadrante[0]) && (quadrante[1] < quadrante[2]) )
				{
					printf("\nMover para frente");
					rollAction = 0;
					pitchAction = 200;
				}
				else
					if ((quadrante[2] < quadrante[0]) && (quadrante[2] < quadrante[1]) )
					{
						printf("\nMover para direita");
						rollAction = -100;
						pitchAction = 100;
					}           
					else
					{
						
						printf("\nSeguir em frente");
						rollAction = 0;
						pitchAction = 100;
					}
        
		}
		
	    if (vmostrarimagens == 1) 
			waitKey(50);

        // Calculate Roll and Pitch depending on the mode
        if (mode == "LOITER"){
            Roll = BASERC - rollAction * FACTOR;
            Pitch = BASERC - pitchAction * FACTOR;
        }else{
            Roll = BASERC;
            Pitch = BASERC;
        }  
         
        // Limit the Roll
        if (Roll > MAXRC)
        {
            Roll = MAXRC;
        } else if (Roll < MINRC)
        {
            Roll = MINRC;
        }

        // Limit the Pitch
        if (Pitch > MAXRC)
        {
            Pitch = MAXRC;
        } else if (Pitch < MINRC)
        {
            Pitch = MINRC;
        }

        msg.channels[0] = Roll;     //Roll
        msg.channels[1] = Pitch;    //Pitch
        msg.channels[2] = BASERC;   //Throttle
        msg.channels[3] = 0;        //Yaw
        msg.channels[4] = 0;
        msg.channels[5] = 0;
        msg.channels[6] = 0;
        msg.channels[7] = 0;

        pub.publish(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}


void mavrosStateCb(const mavros_msgs::StateConstPtr &msg)
{
    if(msg->mode == std::string("CMODE(0)"))
        return;
    //ROS_INFO("I heard: [%s] [%d] [%d]", msg->mode.c_str(), msg->armed, msg->guided);
    mode = msg->mode;
    guided = msg->guided==128;
    armed = msg->armed==128;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    sub = it.subscribe("/erlecopter/front/image_front_raw", 1, imageCallback);
    mavros_state_sub = nh.subscribe("/mavros/state", 1, mavrosStateCb);
    pub = nh.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override", 10);
    
    ros::spin();
    
    /*ros::Rate rate(0.5);
    while (nh.ok())
    {
		ros::spinOnce();
		rate.sleep();
	}*/
	return 0;
}
