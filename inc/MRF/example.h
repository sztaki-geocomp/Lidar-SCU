#include <opencv2/core/core.hpp>

namespace MyMRF {

  void segmentImage(std::vector<cv::Mat>& logProbs, cv::Mat &segmentMap, float i_lambda = 0.2f);

}
