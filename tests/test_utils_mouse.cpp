#include "../../VS/include/mouse.hpp"

int main(int argc, const char *argv[])
{
    // Create event notifier (subject)
    utils::MouseEventNotifier mouse;    

    // Create handlers (observers) 
    utils::ClicksToROI clicks;

    // Bind handlers to notify
    mouse.m_notifer.connect(boost::bind(&utils::IMouseObserver::notify, &clicks, _1, _2, _3, _4));

    cv::namedWindow("image", 1);
    cvSetMouseCallback("image", utils::mouseCallback, &mouse);
    cv::Mat img = cv::imread(argv[1], 1);
    cv::Mat img2;

    while(true)
    {
      img2 = img.clone();
      cv::rectangle(img2, clicks.getROI(), cv::Scalar(0,0,255), 2, 8, 0);
      cv::imshow("image", img2);
      cv::waitKey(30);
    }

    return 0;
}

