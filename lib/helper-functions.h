#include "vector"

void calcloopCoef(float& coeff1,
                  float& coeff2,
                  short loopNoiseBandwidth,
                  float zeta,
                  float loopGain,
                  float pdi);

std::vector<float> linspace(float start_in, float end_in, int num_in);

// template <class T>
void convolve(std::vector<int>* result, int* x, int* y, int lenx, int leny);

int parityCheck(std::vector<int>& bits, int index);

int bin2dec(std::vector<int> vec);

int twosComp2dec(std::vector<int> vec);

std::vector<int> vecSelector(std::vector<int>& source, int start, int end);
std::vector<int>
vecSelector(std::vector<int>& source, int start, int end, int start1, int end1);