// Draws a white square in the center of a picture.

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <iostream>
#include <math.h>


using namespace cv;
using namespace std;

int main()
{
    // Name path for image here
    String path = "/Users/barendpoot/workspace/opencvtest/opencvtest/pic6.png";
    
    Mat image;
    image = imread(path, CV_LOAD_IMAGE_COLOR);   // Read the file
    
    if(! image.data )                              // Check for invalid input
    {
        cout <<  "Could not open or find the image" << endl ;
        return -1;
    }
    
    // Get Height and Width from image
    Size s = image.size();
    int imgHeight = s.height;
    int imgWidth = s.width;
    
    //Draw filled rectangle in center of image
    rectangle(image,cvPoint((imgWidth/2-50),imgHeight/2-50), cvPoint((imgWidth/2+50), imgHeight/2+50),CV_RGB(255,255,255),1,8);
    
    namedWindow( "Display window", CV_WINDOW_AUTOSIZE );// Create a window for display.
    imshow( "Display window", image );                   // Show our image inside it.
    
    waitKey(0);                                          // Wait for a keystroke in the window
    return 0;
    
}

