#include "MRFWrapper.h"

#include "mrf.h"
#include "ICM.h"
#include "GCoptimization.h"
#include "MaxProdBP.h"
#include "TRW-S.h"
#include "BP-S.h"

namespace MyMRF {
  void segmentImage(std::vector<cv::Mat>& logProbs, cv::Mat& segmentMap, float i_lambda) {
  
  	int nLabels = logProbs.size();
	  if (nLabels < 2)
		  return;

	  int width = logProbs[0].size().width;
	  int height = logProbs[0].size().height;
  
    MRF::CostVal*	D = (MRF::CostVal *)malloc(width * height * nLabels * sizeof(MRF::CostVal));
  	
  	float lambda=i_lambda;
	  SmoothnessCost* sc = new SmoothnessCost (2, 1, lambda);

    MRF* mrf;
    EnergyFunction *energy;
    MRF::EnergyVal E;
    double lowerBound;
    float t,tot_t;
    int iter;

    int seed = 1124285485;
    srand(seed);

  	int n = 0;
	  for (int y = 0; y < height; y++) {
		  for (int x = 0; x < width; x++) {
			  for (int j = 0;j < nLabels; j++) {
				  D[n] = (MRF::CostVal) (logProbs[j].at<float>(y,x)); n++;
			  }
		  }
	  }

    DataCost* dc = new DataCost(D);
    energy = new EnergyFunction(dc,sc);
    
    mrf = new Expansion(width, height, nLabels, energy);
    mrf->initialize();
    mrf->clearAnswer();

    E = mrf->totalEnergy();


    tot_t = 0;
    for (iter=0; iter<6; iter++) {
      mrf->optimize(3, t);

      E = mrf->totalEnergy();
      tot_t = tot_t + t ;
    }

    n = 0;
    for (int y = 0; y < height; y++) {
      for (int x = 0; x < width; x++) {
	      segmentMap.at<uchar>(y,x)=mrf->getLabel(n);
	      n++;
      }
    }

    delete mrf;   
    delete energy;
    
    delete dc;
    delete sc;
    delete[] D;

  }
}
