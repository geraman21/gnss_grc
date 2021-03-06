#include "generate_l1_ca.h"
#include "cmath"
#include <array>
#include <bitset>
#include <complex>
#include <iostream>
#include <vector>

std::vector<int> generateCa(int prn, int chip_shift) {
  std::vector<int> dest(1023, 0);
  const int code_length = 1023;
  std::bitset<code_length> G1{};
  std::bitset<code_length> G2{};
  auto G1_register = std::bitset<10>{}.set(); // All true
  auto G2_register = std::bitset<10>{}.set(); // All true
  int lcv;
  int lcv2;
  int delay;
  int prn_idx;
  bool feedback1;
  bool feedback2;
  bool aux;

  // G2 Delays as defined in GPS-ISD-200D
  const std::array<int, 51> delays = {
      5 /*PRN1*/, 6,   7,   8,   17,  18,  139, 140, 141, 251,           252,
      254,        255, 256, 257, 258, 469, 470, 471, 472, 473,           474,
      509,        512, 513, 514, 515, 516, 859, 860, 861, 862 /*PRN32*/, 145 /*PRN120*/,
      175,        52,  21,  237, 235, 886, 657, 634, 762, 355,           1012,
      176,        603, 130, 359, 595, 68,  386 /*PRN138*/};

  // compute delay array index for given PRN number
  if (120 <= prn && prn <= 138) {
    prn_idx = prn - 88; // SBAS PRNs are at array indices 31 to 50 (offset: -120+33-1 =-88)
  } else {
    prn_idx = prn - 1;
  }

  // A simple error check
  if ((prn_idx < 0) || (prn_idx > 51)) {
    return dest;
  }

  // Generate G1 & G2 Register
  for (lcv = 0; lcv < code_length; lcv++) {
    G1[lcv] = G1_register[0];
    G2[lcv] = G2_register[0];

    feedback1 = G1_register[7] ^ G1_register[0];
    feedback2 = G2_register[8] ^ G2_register[7] ^ G2_register[4] ^ G2_register[2] ^
                G2_register[1] ^ G2_register[0];

    for (lcv2 = 0; lcv2 < 9; lcv2++) {
      G1_register[lcv2] = G1_register[lcv2 + 1];
      G2_register[lcv2] = G2_register[lcv2 + 1];
    }

    G1_register[9] = feedback1;
    G2_register[9] = feedback2;
  }

  // Set the delay
  delay = code_length - delays[prn_idx];
  delay += chip_shift;
  delay %= code_length;

  // Generate PRN from G1 and G2 Registers
  for (lcv = 0; lcv < code_length; lcv++) {
    aux = G1[(lcv + chip_shift) % code_length] ^ G2[delay];
    if (aux == true) {
      dest.at(lcv) = 1;
    } else {
      dest.at(lcv) = -1;
    }
    delay++;
    delay %= code_length;
  }
  return dest;
}

void makePaddedCaTable(std::vector<std::vector<int>> &table) {
  for (int i = 0; i <= 32; i++) {
    if (i == 0) {
      table.push_back(std::vector<int>(1025, 0));
    } else {
      table.push_back(generateCa(i));
      table.at(i).insert(table.at(i).begin(), table.at(i).back());
      table.at(i).push_back(table.at(i).at(1));
    }
  }
};

std::vector<std::vector<float>> makeCaTable(int samplesPerCode) {
  float codePhaseStep = 1023.0 / samplesPerCode;
  std::vector<std::vector<float>> result(32);
  std::vector<float> temp(samplesPerCode);

  for (int p = 0; p < 32; p++) {
    std::vector<int> caCode = generateCa(p + 1);
    for (int i = 1; i <= samplesPerCode; i++) {
      temp.at(i - 1) = caCode.at(ceil(codePhaseStep * i) - 1);
    }
    result.at(p) = temp;
  }

  return result;
}

std::vector<std::vector<std::complex<float>>> makeComplexCaTable(int samplesPerCode) {
  float codePhaseStep = 1023.0 / samplesPerCode;
  std::vector<std::vector<std::complex<float>>> result(32);
  std::vector<std::complex<float>> temp(samplesPerCode);

  for (int p = 0; p < 32; p++) {
    std::vector<int> caCode = generateCa(p + 1);
    for (int i = 1; i <= samplesPerCode; i++) {
      temp.at(i - 1) = std::complex(caCode.at(ceil(codePhaseStep * i) - 1) * 1.0, 0.0);
    }
    result.at(p) = temp;
  }

  return result;
}

std::vector<std::complex<float>> makeComplexCaVector(int samplesPerCode, int PRN) {
  float codePhaseStep = 1023.0 / samplesPerCode;
  std::vector<std::complex<float>> result;
  result.reserve(samplesPerCode);

  std::vector<int> caCode = generateCa(PRN);
  for (int i = 1; i <= samplesPerCode; i++) {
    result.push_back(std::complex<float>(caCode.at(ceil(codePhaseStep * i) - 1) * 1.0, 0.0));
  }

  return result;
}