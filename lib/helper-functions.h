#include "acqResults.h"
#include "ephemeris.h"
#include <complex>
#include <deque>
#include <tuple>
#include <valarray>
#include <vector>

double fast_sin(double x);

void calcloopCoef(float &coeff1, float &coeff2, short loopNoiseBandwidth, float zeta,
                  float loopGain, float pdi);

std::vector<float> linspace(float start_in, float end_in, int num_in);

// template <class T>
void convolve(std::vector<int> *result, int *x, int *y, int lenx, int leny);
void convolve(std::vector<int> *result, std::deque<int> &x, int *y, int lenx, int leny);

int parityCheck(std::vector<int> &bits, int index);
int findSubframeStart(std::deque<int> &buffer);
unsigned int bin2dec(std::vector<int> vec);

int twosComp2dec(std::vector<int> vec);

std::vector<int> vecSelector(std::vector<int> &source, int start, int end);
std::vector<int> vecSelector(std::vector<int> &source, int start, int end, int start1, int end1);

void printEphemeris(Ephemeris *ephResults);

std::vector<double> getPseudoRanges(std::vector<double> &travelTime, double startOffset,
                                    long int c);

void custom_fft(std::valarray<std::complex<double>> &x);
void custom_ifft(std::valarray<std::complex<double>> &x);
// Returns codePhase and strength of a Channel provided its caCodeVector
std::tuple<int, float> doParallelCodePhaseSearch(float ts, float IF,
                                                 std::vector<std::complex<float>> &caCodeVector,
                                                 std::vector<float> &longSignal);
AcqResults performAcquisition(int PRN, float ts, float IF,
                              std::vector<std::complex<float>> &caCodeVector,
                              std::vector<float> &longSignal);
AcqResults checkIfChannelPresent(int PRN, float ts, float IF,
                                 std::vector<std::complex<float>> &caCodeVector,
                                 std::vector<float> &longSignal);