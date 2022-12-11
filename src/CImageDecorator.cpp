#include <iostream>
#include <boost/lexical_cast.hpp>
#include "../include/CImageDecorator.hpp"

//#define DEBUG
#ifdef DEBUG
    #define dbg(msg) std::cout << "[DBG] " << __TIME__ << ":" << __FILE__ << ":" << __LINE__ << ":" << __func__ << ": " << msg << std::endl;
#else
    #define dbg(msg)
#endif

CImageDecorator::CImageDecorator()
: QUIET(false),
  colour(cv::Scalar(255,255,255))
{}


CImageDecorator::~CImageDecorator()
{}


/**
 * @brief 
 *
 * @param image_
 */
void
CImageDecorator::setImage(cv::Mat image_)
{
    dbg("")
    this->image = image_.clone();
}


/**
 * @brief
 */
cv::Mat
CImageDecorator::getImage()
{
    dbg("")
    return this->image.clone();
}



/**
 * @brief Display the current frame 
 */
void
CImageDecorator::display()
{
  dbg("")

  // If QUIET mode is not enabled ...
  if(!this->QUIET && !this->image.empty())
  {
      // ... then display the image in a new window.
      cv::namedWindow("decorated_image", CV_WINDOW_AUTOSIZE);
      cv::imshow("decorated_image", this->image);
      cv::waitKey(30); 
  }
} // display()


/**
 * @brief 
 */
void
CImageDecorator::quiet()
{
    this->QUIET = true;
}


/**
 * @brief 
 *
 * @param 
 * @param cv::Point2f
 */
void
CImageDecorator::addPoints(std::vector<cv::Point2f> p, bool label)
{
    dbg("")

    if(!this->image.empty())
    {
      for(std::vector<cv::Point2f>::iterator it = p.begin(); it != p.end(); ++it)
      {
        cv::circle(this->image, (*it), 5, this->colour, 1, 8);
        if(label)
        {
          std::string index = boost::lexical_cast<std::string>(it - p.begin());
          this->addText(index, (*it)); 
        }
      }
    }
    else
    {
      std::cout << "[!] No image assigned to decorator." << std::endl;
    }
}


/**
* @brief 
*
* @param p
* @param label
*/
void
CImageDecorator::addPoint(cv::Point2f p, std::string label)
{
    dbg("")

    if(!this->image.empty())
    {
        cv::circle(this->image, p, 5, this->colour, 1, 8);
        this->addText(label, p); 
    }
    else
    {
      std::cout << "[!] No image assigned to decorator." << std::endl;
    }
}


/**
 * @brief 
 *
 * @param std::string
 * @param cv::Point2f
 */
void
CImageDecorator::addText(std::string s, cv::Point2f p)    
{
    dbg("")

    if(!this->image.empty())
    {
        cv::putText(this->image, s, p, 1, 1, this->colour, 1, 8, false); 
    }
    else
    {
        std::cout << "[!] No image assigned to decorator." << std::endl;
    }
}
        

/**
 * @brief 
 *
 * @param c
 */
void
CImageDecorator::setColour(cv::Scalar c)
{
    this->colour = c;
}


/**
* @brief 
*
* @param value
*
* @return 
*/
cv::Scalar
CImageDecorator::getJetColor(double value) const
{
    double fourValue = 4 * value;
    double red   = cv::min(fourValue - 1.5, -fourValue + 4.5);
    double green = cv::min(fourValue - 0.5, -fourValue + 3.5);
    double blue  = cv::min(fourValue + 0.5, -fourValue + 2.5);
    return cv::Scalar(blue*255,green*255,red*255,100);
} 


