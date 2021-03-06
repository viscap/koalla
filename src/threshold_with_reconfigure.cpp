/***********************************************************************
 universidade federal de itajuba - UNIFEI
 mestrado em ciencia e tecnologia da computacao
 wander mendes martins - wandermendes@unifei.edu.br
 rafael gomes braga - 
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
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <mavros_msgs/OverrideRCIn.h>
#include <mavros_msgs/State.h>

#include <dynamic_reconfigure/server.h>
#include <koalla/KoallaConfig.h>


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


void reconfigureCallback(koalla::KoallaConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Request: %d", 
            config.limiar);
}



void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{	
	// Variáveis
	Mat image, ImageGrayScale;
    int vTercaParte;
    int vmostrarimagens; //0=nao 1=sim
    int vBlackPixels1 = 0;
	int vBlackPixels2 = 0;
	int vBlackPixels3 = 0;
	int rollAction, pitchAction;
	
	
	if ( ros::param::get("~mostrarimagens", vmostrarimagens) ) vmostrarimagens = 0;
	
	
	//23/03/17-wander
	int limiar; // valor limite de cinza. 
	            // Acima dele pintar acender o pixel (255-branco), e 
	            // abaixo dele apagar o pixel (0-preto) 
	            
	if ( !ros::param::get("~limiar", limiar) )
	{
		ROS_FATAL("Nao foi possivel ler o parametro limiar.");
		exit(1);
	}
	
    try
    {
        // Get the msg image
        image = cv_bridge::toCvShare(msg, "bgr8")->image;
        ImageGrayScale = cv_bridge::toCvShare(msg, "mono8")->image;
        
        //divide the photo by 3 areas (columns) 
		vTercaParte   = image.cols / 9;

		if (vmostrarimagens == 1) {
		   namedWindow("Source Image", WINDOW_AUTOSIZE );
		   imshow("Source Image", image);
		}   
		// convert the source image to a gray scale image
		//cvtColor(image, ImageGrayScale, CV_BGR2GRAY);
		// show the gray scale image 
		if (vmostrarimagens == 1) {
		  namedWindow("gray scale Image", WINDOW_AUTOSIZE );
		  imshow("gray scale Image", ImageGrayScale);
		}  

        //23/03/17-wander
        // calculo do limiar
        //limiar = 0;                       
        // passa as cores para 0 ou 255
	    // make a copy in black and white of the source image
	    //ROS_INFO("\nImageGrayScale: %d x %d", ImageGrayScale.rows, ImageGrayScale.cols);
	    for( int y = 0; y < ImageGrayScale.rows; y++ )
	    { 
			for( int x = 0; x < ImageGrayScale.cols / 3; x++ )
		    { 
				//if ( x >= 638 && y >= 358 ) ROS_INFO("\nx = %d | y = %d", x, y);
				for( int c = 0; c < 3; c++ )
		        {
					// c is RGB colors => 0=R 1=G 2=B
					if (ImageGrayScale.at<Vec3b>(y,x)[c] < limiar) 
						ImageGrayScale.at<Vec3b>(y,x)[c] = 0;
					else
						ImageGrayScale.at<Vec3b>(y,x)[c] = 255;   
				}
			}
	    }   
	    if (vmostrarimagens == 1) 
	    {
			namedWindow("Black and White Full Image", WINDOW_AUTOSIZE );
			imshow("Black and White Full Image", ImageGrayScale);
	    }
        
       
        // descobre a quantidade de pixels pretos (obstáculos) em cada quadrante
		vBlackPixels1 = 0;
		vBlackPixels2 = 0;
		vBlackPixels3 = 0;
		
	    // quadrante 1 - LEFT
	    for( int y = 0; y < ImageGrayScale.rows; y++ )
	     for( int x = 0; x < vTercaParte; x++ )
	      for( int c = 0; c < 3; c++ )
		  if (ImageGrayScale.at<Vec3b>(y,x)[c]==0)
		     vBlackPixels1++;

	    // quadrante 2 - MIDDLE
	    for( int y = 0; y < ImageGrayScale.rows; y++ )
	     for( int x = vTercaParte+1; x < 2*vTercaParte; x++ )
	      for( int c = 0; c < 3; c++ )
		  if (ImageGrayScale.at<Vec3b>(y,x)[c]==0)
		     vBlackPixels2++;

	    // quadrante 3 - RIGHT
	    for( int y = 0; y < ImageGrayScale.rows; y++ )
	     for( int x = 2*vTercaParte+1; x < 3*vTercaParte; x++ )
	      for( int c = 0; c < 3; c++ )
		  if (ImageGrayScale.at<Vec3b>(y,x)[c]==0)
		     vBlackPixels3++;
		     
		     
		// Create RC msg
        mavros_msgs::OverrideRCIn msg;
        
        
        // DECIDE PARA ONDE O DRONE DEVE SE MOVER
        // SUGERE O QUADRANTE DE MENOR OBSTÁCULOS (MENOS PIXELS PRETOS)
	    if (vmostrarimagens == 1) 
	    {
			printf("\nQuadrante da esquerda com %i pixels pretos\n",vBlackPixels1);
			printf("Quadrante do centro com %i pixels pretos\n",vBlackPixels2);
			printf("Quadrante da direita com %i pixels pretos\n",vBlackPixels3);
	    }
	    if (  (vBlackPixels1 > (ImageGrayScale.rows*ImageGrayScale.cols*80/100))
			& (vBlackPixels2 > (ImageGrayScale.rows*ImageGrayScale.cols*80/100))
			& (vBlackPixels3 > (ImageGrayScale.rows*ImageGrayScale.cols*80/100))
		   )
	    {
			printf("\nParar");
			rollAction = 0;
			pitchAction = 0;
	    }
	    else 
	    {
        
			if ((vBlackPixels1 < vBlackPixels2) && (vBlackPixels1 < vBlackPixels3) )
			{
				printf("\nMover para a esquerda");
				rollAction = 100;
				pitchAction = 100;
			}
			else
				if ((vBlackPixels2 < vBlackPixels1) && (vBlackPixels2 < vBlackPixels3) )
				{
					printf("\nMover para frente");
					rollAction = 0;
					pitchAction = 200;
				}
				else
					if ((vBlackPixels3 < vBlackPixels1) && (vBlackPixels3 < vBlackPixels2) )
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
    
    
    
    dynamic_reconfigure::Server<koalla::KoallaConfig> server;
	dynamic_reconfigure::Server<koalla::KoallaConfig>::CallbackType f;

	f = boost::bind(&reconfigureCallback, _1, _2);
	server.setCallback(f);
    
    
    
    ros::spin();
    
    /*ros::Rate rate(0.5);
    while (nh.ok())
    {
		ros::spinOnce();
		rate.sleep();
	}*/
	return 0;
}
