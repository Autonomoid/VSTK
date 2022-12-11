#ifndef IGAIN_HPP
#define IGAIN_HPP

namespace vs
{

/**
 * @brief The IGain interface class
 */
class IGain
{
    virtual double operator()(double x) = 0;
};

} // namespace vs

#endif // IGAIN_HPP
