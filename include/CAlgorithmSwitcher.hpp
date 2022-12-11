/////////////////////////////////////////////////////////////////
//
// Name: CAlgorithmSwitcher
//


#ifndef CALGORITHMSWITCHER
#define CALGORITHMSWITCHER

//#define DEBUG
#ifdef DEBUG
    #define dbg(msg) std::cout << "[DBG] " << __TIME__ << ":" << __FILE__ << ":" << __LINE__ << ":" << __func__ << ": " << msg << std::endl;
#else
    #define dbg(msg)
#endif

#include <iostream>
#include <vector>

template<typename THRESHOLD_TYPE, typename ALGORITHM_ID_TYPE> 
class CAlgorithmSwitcher
{
  public:

    /**
     * @brief 
     *
     * @param startingAlgorithm
     */
    CAlgorithmSwitcher(ALGORITHM_ID_TYPE startingAlgorithm)
    : current_alg(startingAlgorithm)
    {}

    /**
     * @brief 
     *
     * @param threshold
     */
    ALGORITHM_ID_TYPE switchAlgorithm(THRESHOLD_TYPE threshold) 
    {
      unsigned int hierarchy_level = 0;
      dbg(this->current_alg << " score = " << threshold)

      // Iterate over the hierarchy of algorithm thresholds.
      typename std::vector<std::pair<THRESHOLD_TYPE, ALGORITHM_ID_TYPE> >::iterator it; 
      for(it=this->hierarchy.begin(); it!=this->hierarchy.end(); ++it)
      {
        // If the number of matches is less than
        // the threshold for algorithm i then check 
        // against algorithm i+1, i+2 etc.
        if(threshold < it->first)
        {
          dbg("< Failed threshold for hierarchy level " << hierarchy_level << " (" << threshold << " < " << it->first << ")")
          ++hierarchy_level;
          continue;
        }
            
        // If not already using this algorithm
        // then switch to it..
        dbg("<< Switching from " << current_alg << " to " << it->second << " (hierarchy level " << hierarchy_level << ")" << std::endl)
        this->current_alg = it->second;

        // Stop descending the hierarchy.
        break;
      }

      return this->current_alg;
    }


    /**
     * @brief 
     *
     * @param _threshold
     * @param _name
     */
    void addAlgorithm(THRESHOLD_TYPE _threshold, ALGORITHM_ID_TYPE _id)
    {
      hierarchy.push_back(std::pair<THRESHOLD_TYPE, ALGORITHM_ID_TYPE>(_threshold, _id));
    }

  private:
    std::vector<std::pair<THRESHOLD_TYPE, ALGORITHM_ID_TYPE> > hierarchy;
    ALGORITHM_ID_TYPE current_alg;

};

#endif
