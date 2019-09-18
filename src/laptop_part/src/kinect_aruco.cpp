#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>


#include <iostream>
#include <stdio.h>
#include <benjie/MarcadorArray.h>
#include "aruco/aruco.h"


using namespace std;
using namespace cv;
using namespace aruco;
using namespace benjie;

ros::Publisher result_pub;


void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    aruco::CameraParameters CamParam;
    MarkerDetector MDetector;					// Declaracion de detector
    vector<Marker> Markers;						// Declaracion de marcadores
    Marcador marcador;
    MarcadorArray marcadores;
    float MarkerSize=-1;
    
    cv::Mat InImage;							// Declaracion de imagen
	cv_bridge::CvImagePtr cv_ptr;

	cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");	// Conversion del mensaje recibido a imagen
	InImage = cv_ptr->image;


	MDetector.detect(InImage,Markers,CamParam,MarkerSize);	// Deteccion de marcadores en la imagen
    
    for (unsigned int i=0;i<Markers.size();i++) {
		
		marcador.id = Markers[i].id;
		Point2f center = Markers[i].getCenter();	// Calculo del centro del marcador
		marcador.cx = center.x*10;					// Almacenado de las coordenadas del centro con un decimal
		marcador.cy = center.y*10;
		
		
		// Calculo y almacenado de la orientacion del marcador, con 3 decimales
		marcador.alpha = atan2f(Markers[i][2].y - Markers[i][1].y, Markers[i][2].x - Markers[i][1].x)*1000;
		
		
		marcadores.marcadorArray.push_back(marcador); // Guardado del marcador en un vector de marcadores
		
        Markers[i].draw(InImage,Scalar(0,0,255),2); // Dibuja el cuadrado donde se situa el marcador
    }	

	// Dibuja la orientacion del marcador con un cubo
    if (  CamParam.isValid() && MarkerSize!=-1)
        for (unsigned int i=0;i<Markers.size();i++) {
			CvDrawingUtils::draw3dCube(InImage,Markers[i],CamParam);
    }				
	// Si hay al menos un marcador, se publica
	if ( 0 < Markers.size() )
	result_pub.publish( marcadores );
	imshow( "image", InImage); // Se muestra la imagen por pantalla
	waitKey(2);
	
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "robot_detector");
	ros::NodeHandle nh;
	
	image_transport::ImageTransport it(nh);
	image_transport::Subscriber sub = it.subscribe("/camera/rgb/image_rect", 1, imageCallback); // Recibe la imagen de la kinect
	result_pub = nh.advertise< MarcadorArray >("/SEPA_node",1); // Publica los marcadores detectados
    
  	ros::spin();
  	
	destroyWindow( "image" );

}
