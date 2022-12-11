#ifndef CADAPTIVEGAIN_H
#define CADAPTIVEGAIN_H

#include <boost/scoped_ptr.hpp>

#include "IGain.hpp"

namespace vs
{

/**
 * @brief The CAdaptiveGain class
 */
class CAdaptiveGain : public IGain
{
  public:
    CAdaptiveGain(double gainAtZero, double gainAtInf, double gradientAtZero);
    ~CAdaptiveGain();
    virtual double operator()(double x);

  private:
    class pImpl;
    boost::scoped_ptr<pImpl> mPimpl;

};

} // namespace vs

#endif // CADAPTIVEGAIN_H
