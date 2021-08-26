#ifndef GENERATE_L1_CA
#define GENERATE_L1_CA

#include <vector>
#include <complex>

std::vector<int> generateCa(int prn, int chip_shift = 0);
void makePaddedCaTable(std::vector<std::vector<int>> &table);
std::vector<std::vector<float>> makeCaTable(int sampleFreq);
std::vector<std::vector<std::complex<float>>> makeComplexCaTable(int samplesPerCode);

#endif