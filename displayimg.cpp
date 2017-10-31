#include <opencv2/opencv.hpp>
#include <iostream>
#include <stdio.h>
#include <fstream>
#include <sstream>

using namespace std;
using namespace cv;

/** Function Headers */
void detectAndDisplay( Mat frame );

/** Global variables */
//-- Note, either copy these two files from opencv/data/haarscascades to your current folder, or change these locations
String face_cascade_name = "haarcascade_frontalface_alt.xml";
String eyes_cascade_name = "haarcascade_eye_tree_eyeglasses.xml";
CascadeClassifier face_cascade;
CascadeClassifier eyes_cascade;
string window_name = "Capture - Face detection";
RNG rng(12345);

//test for face detection

int main( int argc, const char** argv )
{
	Mat frame;

	//-- 1. Load the cascades
	if( !face_cascade.load( face_cascade_name ) ){ printf("--(!)Error loading\n"); return -1; };
	if( !eyes_cascade.load( eyes_cascade_name ) ){ printf("--(!)Error loading\n"); return -1; };

	//-- 2. Read the video stream

	VideoCapture cap(0); // open the default camera
    if(!cap.isOpened())  // check if we succeeded
        return -1;
 
    namedWindow("Video",1);
    while(1)
    {
        Mat frame;
        cap >> frame;         // get a new frame from camera
        imshow("Video", frame);

		//-- 3. Apply the classifier to the frame
		if( !frame.empty() )
		{ detectAndDisplay( frame ); }
		else
		{ printf(" --(!) No captured frame -- Break!"); break; }

		int c = waitKey(30);
		if( (char)c == 'c' )  break; 
        // Press 'c' to escape
    }
                
	return 0;
}

/**
* @function detectAndDisplay
*/
void detectAndDisplay( Mat frame )
{
	std::vector<Rect> faces;
	Mat frame_gray;

	cvtColor( frame, frame_gray, CV_BGR2GRAY );
	equalizeHist( frame_gray, frame_gray );
	//-- Detect faces
	face_cascade.detectMultiScale( frame_gray, faces, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, Size(30, 30) );

	for( size_t i = 0; i < faces.size(); i++ )
	{
		Point center( int(faces[i].x + faces[i].width*0.5), int(faces[i].y + faces[i].height*0.5) );
		ellipse( frame, center, Size( int(faces[i].width*0.5), int(faces[i].height*0.5)), 0, 0, 360, Scalar( 255, 0, 255 ), 2, 8, 0 );

		Mat faceROI = frame_gray( faces[i] );
	} 
	//-- Show what you got
	imshow( window_name, frame );
}


/*
int main()  
{  
    Mat image=imread("D:\cat.jpg", CV_LOAD_IMAGE_COLOR);  
    if(! image.data )  // Check for invalid input
    {
            cout <<  "Could not open or find the image" << std::endl ;
            return -1;
    }
 
    //DISPLAY image
    namedWindow( "window", CV_WINDOW_AUTOSIZE ); // Create a window for display.
    imshow( "window", image ); // Show our image inside it.
 
    //SAVE image
    imwrite("result.jpg",image);// it will store the image in name "result.jpg"
	
	Mat draw_img = Mat::zeros( 400, 400, CV_8UC3 );
   
	// Draw a line 
	line( draw_img, Point( 15, 20 ), Point( 70, 50), Scalar( 110, 220, 0 ),  2, 8 );
	// Draw a circle 
	circle( draw_img, Point( 200, 200 ), 32.0, Scalar( 0, 0, 255 ), 1, 8 );
	imshow("Image",draw_img);

    waitKey(0);                       // Wait for a keystroke in the window
    return 0;
} */ 


//edge detection

/*
int main( int argc, char** argv )
{
    Mat src, gray, dst, abs_dst;
	
	src=imread("D:\cat.jpg", CV_LOAD_IMAGE_COLOR);
    // Remove noise by blurring with a Gaussian filter
    //GaussianBlur( src, src, Size(3,3), 0, 0, BORDER_DEFAULT );
    //cvtColor( src, gray, CV_RGB2GRAY );
 
    // Apply Laplace function
    //Laplacian( gray, dst, CV_16S, 3, 1, 0, BORDER_DEFAULT );
	bilateralFilter ( src, dst, 15, 80, 80 );
    convertScaleAbs( dst, abs_dst );
    imshow( "result", abs_dst );	
	Mat draw_img = Mat::zeros( 400, 400, CV_8UC3 );
   
	// Draw a line 
	line( abs_dst, Point( 15, 20 ), Point( 70, 50), Scalar( 110, 220, 0 ),  2, 8 );
	// Draw a circle 
	circle( abs_dst, Point( 200, 200 ), 32.0, Scalar( 0, 0, 255 ), 1, 8 );
	imshow("Image",abs_dst);
    waitKey(0);
    return 0;
}
*/

