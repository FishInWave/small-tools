#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
    
    
int 
main (int argc, char** argv)
{
    cv::Mat src = cv::imread("map_out.pgm");
    cv::Mat dst;
    cv::morphologyEx(src,dst,cv::MORPH_CLOSE,cv::Mat(30,30,CV_8U),cv::Point(-1,-1),1);





    cv::imshow("source",dst);
    cv::imwrite("out.pgm",dst);
    cv::waitKey();
    return 0;
}