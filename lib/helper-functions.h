#include "vector"

void calcloopCoef(float& coeff1,
                  float& coeff2,
                  short loopNoiseBandwidth,
                  float zeta,
                  float loopGain,
                  float pdi);

std::vector<float> linspace(float start_in, float end_in, int num_in);

template <class T>
void convolve(std::vector<T>* result, T* x, T* y, int lenx, int leny);
