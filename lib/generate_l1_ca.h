#ifndef GENERATE_L1_CA
#define GENERATE_L1_CA

#include <vector>

std::vector<int> generateCa(int prn, int chip_shift = 0);
std::vector<std::vector<int>> makeCaTable(int sampleFreq);

#endif