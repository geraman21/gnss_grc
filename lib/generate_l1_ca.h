#ifndef GENERATE_L1_CA
#define GENERATE_L1_CA

#include <complex>
#include <vector>

std::vector<int> generateCa(int prn, int chip_shift = 0);
void makePaddedCaTable(std::vector<std::vector<int>> &table);
std::vector<std::vector<float>> makeCaTable(int sampleFreq);
std::vector<std::vector<std::complex<float>>> makeComplexCaTable(int samplesPerCode);
std::vector<std::complex<float>> makeComplexCaVector(int samplesPerCode, int PRN);

#endif