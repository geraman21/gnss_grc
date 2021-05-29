#include "vector"

void calcloopCoef (float &coeff1, float &coeff2, short loopNoiseBandwidth, float zeta, float loopGain, float pdi ) {
    
    //     Calculates the loop coefficients tau1 and tau2. 
    //     This process is discussed in sections 7.1-7.3 of Borre.
    //     # Solve for the natural frequency
    
    float wn = loopNoiseBandwidth*8*zeta / (4*zeta*zeta + 1);
    
    // Solve for tau1 and tau2

    float tau1 = loopGain / (wn*wn);
    float tau2 = 2.0 * zeta / wn;

    coeff1 = tau2 / tau1;
    coeff2 = pdi / tau1;

}

std::vector<float> linspace(float start_in, float end_in, int num_in)
{

  std::vector<float> linspaced;

  if (num_in == 0) { return linspaced; }
  if (num_in == 1) 
    {
      linspaced.push_back(start_in);
      return linspaced;
    }

  float delta = (end_in - start_in) / (num_in - 1);

  for(int i=0; i < num_in-1; ++i)
    {
      linspaced.push_back(start_in + delta * i);
    }

  linspaced.push_back(end_in); 

  linspaced.shrink_to_fit();

  return linspaced;
}