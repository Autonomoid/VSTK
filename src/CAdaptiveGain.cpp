#include <cmath>
#include "../include/CAdaptiveGain.hpp"

namespace vs {

/**
 * @brief The CAdaptiveGain::pImpl class
 */
class CAdaptiveGain::pImpl {
	friend class CAdaptiveGain;

       private:
	pImpl(double gainAtZero, double gainAtInf, double gradientAtZero)
	    : mGainAtZero(gainAtZero),
	      mGainAtInf(gainAtInf),
	      mGradientAtZero(gradientAtZero) {}

	double mGain;
	double mGainAtZero;
	double mGainAtInf;
	double mGradientAtZero;
};

/**
 * @brief CAdaptiveGain::CAdaptiveGain
 * @param gainAtZero
 * @param gainAtInf
 * @param gradientAtZero
 */
CAdaptiveGain::CAdaptiveGain(double gainAtZero, double gainAtInf,
			     double gradientAtZero)
    : mPimpl(new pImpl(gainAtZero, gainAtInf, gradientAtZero)) {}

/**
 * @brief CAdaptiveGain::~CAdaptiveGain
 */
CAdaptiveGain::~CAdaptiveGain() {}

/**
 * @brief CAdaptiveGain::operator ()
 * @param x
 * @return
 */
double CAdaptiveGain::operator()(double x) {
	double a = mPimpl->mGainAtZero - mPimpl->mGainAtInf;
	double b = mPimpl->mGradientAtZero / a;
	double c = mPimpl->mGainAtInf;
	mPimpl->mGain = (a * std::exp(-b * x)) + c;
	return mPimpl->mGain;
}

}  // namespace vs
