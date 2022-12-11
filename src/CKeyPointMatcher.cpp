//#define DEBUG
#ifdef DEBUG
#define dbg(msg) std::cout << "[DBG] " << __TIME__ << ":" << __FILE__ << ":" << __LINE__ << ":" << __func__ << ": " << msg << std::endl;
#else
#define dbg(msg)
#endif

#include "../include/CKeyPointMatcher.hpp"

/**
 * @brief Constructor
 *
 * @param kp
 */
CKeyPointMatcher::CKeyPointMatcher()
    : ratio(0.65f),
      refineF(true),
      distance(3.0),
      confidence(0.99)
{
    // Initialize the std::map member
    // wih all available algorithms
    kpAlgorithm["Harris"] = 0;
    kpAlgorithm["ORB"] = 1;
    kpAlgorithm["SIFT"] = 2;
    kpAlgorithm["SURF"] = 3;
    kpAlgorithm["DENSE"] = 4;
    //kpAlgorithm["BRISK"] = 5;
}


/**
 * @brief
 *
 * @param kp
 */
void
CKeyPointMatcher::setAlgorithm(const std::string kp)
{
    // Only change if different from current algorithm.
    if(this->current_alg == kp)
    {
        dbg("Algorithm is the same.")
        return;
    }

    switch(kpAlgorithm[kp])
    {
        //HARRIS
    case 0:
    {
        int maxCorners = 100;
        double qualityLevel = 0.01;
        double minDistance = 1;
        int blockSize = 3;
        bool useHarrisDetector = true;
        double k = 0.04;
        this->detector = new cv::GoodFeaturesToTrackDetector(maxCorners, qualityLevel, minDistance,
                blockSize, useHarrisDetector, k);
        this->detector = cv::DescriptorExtractor::create("ORB");
        this->matcher = cv::DescriptorMatcher::create("BruteForce-Hamming(2)");
        this->current_alg = "HARRIS";
    }

    // ORB
    case 1:
    {
        int nfeatures = 500; // 200
        float scaleFactor = 1.2f; // 1.2f
        int nlevels = 8; // 8
        int edgeThreshold = 31; //31
        int firstLevel = 0; // 0
        int WTA_K = 4; // 2 --> 4
        int scoreType = cv::ORB::HARRIS_SCORE;
        int patchSize = 31; // 31
        this->detector = new cv::ORB(nfeatures, scaleFactor, nlevels, edgeThreshold,
                                     firstLevel, WTA_K, scoreType, patchSize);
        this->extractor = cv::DescriptorExtractor::create("ORB");
        this->matcher = cv::DescriptorMatcher::create("BruteForce-Hamming(2)");
        this->current_alg = "ORB";
        break;
    }

    // SIFT
    case 2:
    {
        int nfeatures = 200; // 500
        int nOctaveLayers = 3;
        double contrastThreshold = 0.04;
        double edgeThreshold = 10;
        double sigma = 1.6;
        this->detector = new cv::SIFT(nfeatures, nOctaveLayers, contrastThreshold,
                                      edgeThreshold, sigma);
        this->extractor = cv::DescriptorExtractor::create("SIFT");
        this->matcher = cv::DescriptorMatcher::create("BruteForce");
        //this->matcher = cv::DescriptorMatcher::create("FlannBased");
        this->current_alg = "SIFT";
        break;
    }

    // SURF
    case 3:
    {
        double hessianThreshold = 1000;
        int nOctaves = 4;
        int nOctaveLayers = 2;
        bool extended = true;
        bool upright = false;
        this->detector = new cv::SURF(hessianThreshold, nOctaves, nOctaveLayers, extended, upright);
        this->extractor = cv::DescriptorExtractor::create("SURF");
        //this->matcher = cv::DescriptorMatcher::create("BruteForce");
        this->matcher = cv::DescriptorMatcher::create("FlannBased");
        this->current_alg = "SURF";
        break;
    }

    // DENSE
    case 4:
    {
        float initFeatureScale = 1.f;
        int featureScaleLevels = 1;
        float featureScaleMul = 0.1f;
        int initXyStep = 6;
        int initImgBound = 0;
        bool varyXyStepWithScale = true;
        bool varyImgBoundWithScale = false;
        this->detector = new cv::DenseFeatureDetector(initFeatureScale, featureScaleLevels, featureScaleMul,
                initXyStep, initImgBound, varyXyStepWithScale, varyImgBoundWithScale);
        this->extractor = cv::DescriptorExtractor::create("ORB");
        this->matcher = cv::DescriptorMatcher::create("BruteForce-Hamming(2)");
        this->current_alg = "DENSE";
        break;
    }

    // TODO -- BRISK
    //case 5:
    //{
    //int thresh = 30;
    //int octaves = 3;
    //float patternScale = 1.0f;
    //this->detector = new cv::BRISK(thresh, octaves, patternScale);
    //this->extractor = cv::DescriptorExtractor::create("BRISK");
    //this->matcher = cv::DescriptorMatcher::create("BruteForce-Hamming(2)");
    //this->current_alg = "BRISK;
    //break;
    //}

    }
}


/**
 * @brief Set the feature detector member.
 *
 * @param detect
 */
void
CKeyPointMatcher::setFeatureDetector(const cv::Ptr<cv::FeatureDetector>& detect)
{
    this->detector = detect;
}


/**
 * @brief Set the descriptor extractor member.
 *
 * @param desc
 */
void
CKeyPointMatcher::setDescriptorExtractor(const cv::Ptr<cv::DescriptorExtractor>& desc)
{
    this->extractor = desc;
}


/**
 * @brief Set the matcher member.
 *
 * @param match
 */
void
CKeyPointMatcher::setDescriptorMatcher(const cv::Ptr<cv::DescriptorMatcher>& match)
{
    this->matcher = match;
}


/**
 * @brief
 *
 * @param conf
 */
void
CKeyPointMatcher::setConfidenceLevel(const double conf)
{
    this->confidence = conf;
}


/**
 * @brief
 *
 * @param dist
 */
void
CKeyPointMatcher::setMinDistanceToEpipolar(const double dist)
{
    this->distance = dist;
}


/**
 * @brief
 *
 * @param rat
 */
void
CKeyPointMatcher::setRatio(const float rat)
{
    this->ratio = rat;
}


/**
 * @brief
 *
 * Clear matches for which NN ratio is > than threshold
 * return the number of removed points
 * (corresponding entries being cleared,
 * i.e. size will be 0)
 *
 * @param matches
 *
 * @return
 */
int
CKeyPointMatcher::ratioTest(vec2DMatch_t& matches)
{
    int removed = 0;

    // for all matches
    for(uint i=0; i<matches.size(); ++i)
    {
        // if 2 NN has been identified
        if(matches[i].size() > 1)
        {
            // check distance ratio
            if(matches[i][0].distance/matches[i][1].distance > ratio)
            {
                matches[i].clear(); // remove match
                removed++;
            }
        }
        else
        {
            // does not have 2 neighbours
            matches[i].clear(); // remove match
            removed++;
        }
    }
    return removed;
}


/**
 * @brief Insert symmetrical matches in symMatches vector
 *
 * @param matches1
 * @param matches2
 * @param symMatches
 */
void
CKeyPointMatcher::symmetryTest(const vec2DMatch_t& matches1, const vec2DMatch_t& matches2, vecDMatch_t& symMatches)
{
    // for all matches from image 1 --> image 2
    for(vec2DMatch_t::const_iterator matchIter1 = matches1.begin(); matchIter1 != matches1.end(); ++matchIter1)
    {
        // ignore deleted matches
        if(matchIter1->size() < 2)
            continue;

        // for all matches from image 2 --> image 1
        for(vec2DMatch_t::const_iterator matchIter2 = matches2.begin(); matchIter2 != matches2.end(); ++matchIter2)
        {
            // ignore deleted matches
            if(matchIter2->size() < 2)
                continue;

            // Do symmetry test...
            if((*matchIter1)[0].queryIdx == (*matchIter2)[0].trainIdx &&
                    (*matchIter2)[0].queryIdx == (*matchIter1)[0].trainIdx)
            {
                // add symmetrical matches
                symMatches.push_back(cv::DMatch((*matchIter1)[0].queryIdx,
                                                (*matchIter1)[0].trainIdx,
                                                (*matchIter1)[0].distance));
                break; // try next match from image 1 --> image 2
            }
        }
    }
}


/**
 * @brief Identify good matches using RANSAC, return fundamental matrix
 *
 * @param matches
 * @param keypoints1
 * @param keypoints2
 * @param outMatches
 *
 * @return
 */
cv::Mat
CKeyPointMatcher::ransacTest(const vecDMatch_t& matches, const vecKeyPoint_t& keypoints1,
                             const vecKeyPoint_t& keypoints2, vecDMatch_t& outMatches)
{
    std::vector<cv::Point2f> points1, points2;
    cv::Mat fundamental;

    // Convert keypoints to Point2f
    float x = -1.0;
    float y = -1.0;

    for(vecDMatch_t::const_iterator it = matches.begin(); it != matches.end(); ++it)
    {
        // Get the position of left keypoints
        x = keypoints1[it->queryIdx].pt.x;
        y = keypoints1[it->queryIdx].pt.y;
        points1.push_back(cv::Point2f(x,y));

        // Get the position of right keypoints
        x = keypoints2[it->trainIdx].pt.x;
        y = keypoints2[it->trainIdx].pt.y;
        points2.push_back(cv::Point2f(x,y));
    }

    // Compute fundamental matrix using RANSAC
    std::vector<uchar> inliers(points1.size(),0);
    if((points1.size() > 0) && (points2.size() > 0))
    {
        cv::Mat fundamental = cv::findFundamentalMat(cv::Mat(points1),
                              cv::Mat(points2), // matching points
                              inliers,      // match status (inlier or outlier)
                              CV_FM_RANSAC, // RANSAC method
                              distance,     // distance to epipolar line
                              confidence);  // confidence probability

        // extract any surviving inliers (matches).
        std::vector<uchar>::const_iterator itIn = inliers.begin();
        vecDMatch_t::const_iterator itM = matches.begin();
        for(; itIn != inliers.end(); ++itIn, ++itM)
        {
            if(*itIn)
            {
                outMatches.push_back(*itM);
            }
        }

        // Recompute fundamental matrix using
        // the accepted matches?
        if(refineF)
        {
            points1.clear();
            points2.clear();

            // Convert accepted keypoints into Point2f
            for(vecDMatch_t::const_iterator it = outMatches.begin(); it != outMatches.end(); ++it)
            {
                // Get the position of left keypoints
                x = keypoints1[it->queryIdx].pt.x;
                y = keypoints1[it->queryIdx].pt.y;
                points1.push_back(cv::Point2f(x,y));

                // Get the position of right keypoints
                x = keypoints2[it->trainIdx].pt.x;
                y = keypoints2[it->trainIdx].pt.y;
                points2.push_back(cv::Point2f(x,y));
            }

            // Compute 8-point fundamental matrix.
            if((points1.size() > 0) && (points2.size() > 0))
            {
                fundamental = cv::findFundamentalMat(cv::Mat(points1), cv::Mat(points2), CV_FM_8POINT);
            }
        }
    }

    return fundamental;
}


/**
 * @brief Include only those matches within given ROI.
 *
 * @param vec2DMatch_t
 * @param cv::Point
 * @param cv::Point
 */
void
CKeyPointMatcher::ROIinclude(const vecDMatch_t& matches, vecDMatch_t& ROImatches,
                             const vecKeyPoint_t& keypoints1, const vecKeyPoint_t& keypoints2,
                             const cv::Point p1, const cv::Point p2)
{
    float x = -1.0;
    float y = -1.0;
    for(vecDMatch_t::const_iterator it = matches.begin(); it != matches.end(); ++it)
    {
        // Get the position of right keypoints
        x = keypoints2[it->trainIdx].pt.x;
        y = keypoints2[it->trainIdx].pt.y;

        // If match is in the ROI then store it
        if((x>p1.x) && (x<p2.x) && (y>p1.y) && (y<p2.y))
            ROImatches.push_back(*it);
    }
}


/**
 * @brief Match keypoints (no ROI specified).
 *
 * @param image1
 * @param image2
 * @param matches
 * @param keypoints1
 * @param keypoints2
 *
 * @return
 */
cv::Mat
CKeyPointMatcher::match(cv::Mat& image1, cv::Mat& image2, vecDMatch_t& matches,
                        vecKeyPoint_t& keypoints1, vecKeyPoint_t& keypoints2)
{
    // No ROI has been specified, therefore use
    // the entire image.
    cv::Point p1(0,0);
    cv::Point p2(image1.cols-1, image1.rows-1);
    return this->match(image1, image2, matches, keypoints1, keypoints2, p1, p2);
}


/**
 * @brief Match keypoints in ROI using ratio test, symmetry test and RANSAC.
 *
 * @param image1
 * @param image2
 * @param matches
 * @param keypoints1
 * @param keypoints2
 * @param p1
 * @param p2
 *
 * @return
 */
cv::Mat
CKeyPointMatcher::match(cv::Mat& image1, cv::Mat& image2, vecDMatch_t& matches,
                        vecKeyPoint_t& keypoints1, vecKeyPoint_t& keypoints2,
                        const cv::Point p1, const cv::Point p2)
{

    // 1a. Detect features in both images.
    detector->detect(image1,keypoints1);
    detector->detect(image2,keypoints2);

    dbg("Initial number of keypoints: " << keypoints1.size() << " (image 1)" << ", " << keypoints2.size() << " (image 2)")

    // 1b. Extract descriptors
    // from both sets of features.
    cv::Mat descriptors1, descriptors2;
    extractor->compute(image1,keypoints1,descriptors1);
    extractor->compute(image2,keypoints2,descriptors2);

    // 2a. Match descriptors from image 1 to image 2
    // based on k nearest neighbours (with k=2)
    vec2DMatch_t matches1;
    matcher->knnMatch(descriptors1, descriptors2, matches1, 2);

    // 2b. Match descriptors fromm image 2 to image 1
    // based on k nearest neighbours (with k=2)
    vec2DMatch_t matches2;
    matcher->knnMatch(descriptors2, descriptors1, matches2, 2);

    // 3. Bi-directional ratio test:
    // I.e. Remove matches for which NN ratio > threshold
    int removed = 0;

    // Clean the image 1 --> image 2 matches
    removed = ratioTest(matches1);

    // Clean image 2 --> image 1 matches
    removed = ratioTest(matches2);

    dbg("Number of matches after ratio test: " << matches1.size() - removed)

    // 4. Remove non-symmetrical matches
    vecDMatch_t symMatches;
    symmetryTest(matches1, matches2, symMatches);

    dbg("Number of matches after symmetry test: " << symMatches.size())

    // 5. Validate matches using RANSAC
    vecDMatch_t RANSACmatches;
    cv::Mat fundamental = ransacTest(symMatches, keypoints1, keypoints2, RANSACmatches);

    dbg("Number of matches after RANSAC/epi-polar test: " << RANSACmatches.size())

    // 6. Include only the matches inside ROI.
    ROIinclude(RANSACmatches, matches, keypoints1, keypoints2, p1, p2);

    dbg("Number of matches after ROI exclusion: " << matches.size())

    // return the found fundamental matrix
    return fundamental;
}

