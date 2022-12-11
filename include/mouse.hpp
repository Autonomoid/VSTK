// OpenCV mouse callbacks

#ifndef UTILS_CV_MOUSE_H
#define UTILS_CV_MOUSE_H

#define DEBUG
/*#ifdef DEBUG
    #define dbg(msg) std::cout << "[DBG] " << __TIME__ << ":" << __FILE__ << ":" << __LINE__ << ":" << __func__ << ": " << msg << std::endl;
#else
    #define dbg(msg)
#endif*/

#include <iostream>
#include <cstdarg>

#include <boost/bind.hpp>
#include <boost/signals2.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

namespace utils
{

/**
 * @brief 
 */
class MouseEventNotifier
{
  public:
    boost::signals2::signal<void (const int event, const int x, const int y, const int flags)> m_notifer;

    void operator()(const int event, const int x, const int y, const int flags)
    {
      this->m_notifer(event, x, y, flags);
    }
};


/**
 * @brief 
 */
class IMouseObserver
{
  public:
    void notify(const int event, const int x, const int y, const int flags)
    {
      m_event = event;
      m_x = x;
      m_y = y;
      m_flags = flags;
      m_update();
    }

  protected:
    int m_event;
    int m_x;
    int m_y;
    int m_flags;

    virtual void m_update() = 0;
};


/**
 * @brief 
 */
class ClickObserver : public IMouseObserver
{
  protected:

    virtual void m_update()
    {
        switch(m_event)
        {
            case CV_EVENT_LBUTTONDOWN:
                this->leftClick();
                break;
                
            case CV_EVENT_RBUTTONDOWN:
                this->rightClick();
                break;
                
            case CV_EVENT_MBUTTONDOWN:
                this->middleClick();
                break;

            default:
                break;
        }
    }

    virtual void leftClick(){};
    virtual void rightClick(){};
    virtual void middleClick(){};

};


/**
 * @brief 
 */
class ClicksToROI : public ClickObserver
{
  public:
    ClicksToROI()
    : nClicks(0)
    {}

    cv::Rect getROI()
    {
      if(nClicks == 2)
        return cv::Rect(m_p1.x, m_p1.y, (m_p2.x-m_p1.x), (m_p2.y-m_p1.y));
      else
        return cv::Rect();
    }

    cv::Point getP1()
    {
      return m_p1;
    }

    cv::Point getP2()
    {
      return m_p2;
    }

    int getNClicks()
    {
      return nClicks;
    }

  private:
    cv::Point m_p1;
    cv::Point m_p2;
    int nClicks;

    void leftClick()
    {
      // First click
      if(nClicks == 0 || nClicks == 2)
      {
        m_p1 = cv::Point2f(m_x, m_y);
        nClicks = 1;
        return;
      }

      // Second click
      if(nClicks == 1)
      {
        m_p2 = cv::Point2f(m_x, m_y);

        // Check ordering of points
        if(m_p1.x > m_p2.x)
        {
          m_p2.x = m_p1.x;
          m_p1.x = m_x;
        }

        if(m_p1.y > m_p2.y)
        {
          m_p2.y = m_p1.y;
          m_p1.y = m_y;
        }

        // Check points aren't the same
        if((m_p1.x == m_p2.x) || (m_p1.y == m_p2.y))
        {
          return;
        }

        nClicks = 2;
      } // if
    } // leftClick()

    void rightClick()
    {
      nClicks = 0;
    }
};


/**
 * @brief
 */
class ClickToPoint : public ClickObserver
{
  public:
    ClickToPoint()
    {}

    cv::Point getP1()
    {
      return m_p1;
    }

  private:
    cv::Point m_p1;

    void leftClick()
    {
        m_p1 = cv::Point2f(m_x, m_y);
    }

};


/**
 * @brief
 */
class ClicksToPath : public ClickObserver
{
  public:
    ClicksToPath()
    {
        m_usingImage = false;
    }

    ClicksToPath(cv::Mat& image)
    {
        m_image = image;
        m_usingImage = true;
    }

    std::vector<cv::Point> getPath()
    {
      return m_path;
    }

    cv::Point getP1()
    {
      return m_p1;
    }

  private:
    std::vector<cv::Point> m_path;
    cv::Point m_p1;
    bool m_usingImage;
    cv::Mat m_image;

    void leftClick()
    {
        m_p1 = cv::Point2f(m_x, m_y);
        m_path.push_back(m_p1);

        if(m_usingImage)
        {
            cv::circle(m_image, m_p1, 5, cv::Scalar(255,255,0), 1, 8);
            cv::waitKey(30);
        }
    }

};


/**
 * @brief OpenCV-style mouse callback
 *
 * @param event
 * @param x
 * @param y
 * @param flags
 * @param params
 */
void mouseCallback(const int event, const int x, const int y, const int flags, void* params)
{
  static_cast<MouseEventNotifier*>(params)->operator()(event, x, y, flags);
}

} // namespace utils

#endif
