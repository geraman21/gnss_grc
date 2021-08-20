#include "ephemeris.h"
#include "helper-functions.h"
#include <algorithm>
#include <cmath>
#include <deque>
#include <iostream>
#include <vector>
#include <complex>
#include <valarray>

void calcloopCoef(float &coeff1,
                  float &coeff2,
                  short loopNoiseBandwidth,
                  float zeta,
                  float loopGain,
                  float pdi)
{

    //     Calculates the loop coefficients tau1 and tau2.
    //     This process is discussed in sections 7.1-7.3 of Borre.
    //     # Solve for the natural frequency

    float wn = loopNoiseBandwidth * 8 * zeta / (4 * zeta * zeta + 1);

    // Solve for tau1 and tau2

    float tau1 = loopGain / (wn * wn);
    float tau2 = 2.0 * zeta / wn;

    coeff1 = tau2 / tau1;
    coeff2 = pdi / tau1;
}

std::vector<float> linspace(float start_in, float end_in, int num_in)
{

    std::vector<float> linspaced;

    if (num_in == 0)
    {
        return linspaced;
    }
    if (num_in == 1)
    {
        linspaced.push_back(start_in);
        return linspaced;
    }

    float delta = (end_in - start_in) / (num_in - 1);

    for (int i = 0; i < num_in - 1; ++i)
    {
        linspaced.push_back(start_in + delta * i);
    }

    linspaced.push_back(end_in);

    linspaced.shrink_to_fit();

    return linspaced;
}
// https://www.allaboutcircuits.com/technical-articles/understanding-correlation/
// Call with two vectors of data in x and y. Let z point to memory for result.
// Allocate length_x + length_y-1 locations. lenx and leny are the respective
// lengths of data.
// performs straight convolution by Clay S. Turner
// for correlattion just reverse the order of sequence y;

void convolve(std::vector<int> *result, int *x, int *y, int lenx, int leny)
{
    int s, *xp, *yp;
    int lenz;
    int i, n, n_lo, n_hi;

    lenz = lenx + leny - 1;
    for (i = 0; i < lenz; i++)
    {
        s = 0.0;
        n_lo = 0 > (i - leny + 1) ? 0 : i - leny + 1;
        n_hi = lenx - 1 < i ? lenx - 1 : i;
        xp = x + n_lo;
        yp = y + i - n_lo;
        for (n = n_lo; n <= n_hi; n++)
        {
            s += *xp * *yp;
            xp++;
            yp--;
        }
        (*result).at(i) = s;
    }
}

void convolve(std::vector<int> *result, std::deque<int> &x, int *y, int lenx, int leny)
{
    int s, xp, *yp;
    int lenz;
    int i, n, n_lo, n_hi;

    lenz = lenx + leny - 1;

    for (i = 0; i < lenz; i++)
    {
        s = 0.0;
        n_lo = 0 > (i - leny + 1) ? 0 : i - leny + 1;
        n_hi = lenx - 1 < i ? lenx - 1 : i;
        xp = x[n_lo];
        yp = y + i - n_lo;
        for (n = n_lo; n <= n_hi; n++)
        {
            s += xp * *yp;
            xp = x[n + 1];
            yp--;
        }
        (*result).at(i) = s;
    }
}

int parityCheck(std::vector<int> &bits, int index)
{
    if (bits.size() != 33)
    {
        std::cout << "wrong input: [bits]" << std::endl;
        return 0;
    }

    // Check if the data bits must be inverted
    if (bits.at(2) != 1)
    {
        std::for_each(bits.begin() + 3, bits.begin() + 27, [](int &n)
                      { n *= -1; });
    }

    //  -- Calculate 6 parity bits ----------------------------------------------
    //  The elements of the ndat array correspond to the bits showed in the table
    //  20-XIV (ICD-200C document) in the following way:
    //  The first element in the ndat is the D29* bit and the second - D30*.
    //  The elements 3 - 26 are bits d1-d24 in the table.
    //  The elements 27 - 32 in the ndat array are the received bits D25-D30.
    //  The array "parity" contains the computed D25-D30 (parity) bits.
    std::vector<int> parity(6, 0);

    parity.at(0) = bits.at(1) * bits.at(3) * bits.at(4) * bits.at(5) * bits.at(7) *
                   bits.at(8) * bits.at(12) * bits.at(13) * bits.at(14) * bits.at(15) *
                   bits.at(16) * bits.at(19) * bits.at(20) * bits.at(22) * bits.at(25);

    parity.at(1) = bits.at(2) * bits.at(4) * bits.at(5) * bits.at(6) * bits.at(8) *
                   bits.at(9) * bits.at(13) * bits.at(14) * bits.at(15) * bits.at(16) *
                   bits.at(17) * bits.at(20) * bits.at(21) * bits.at(23) * bits.at(26);

    parity.at(2) = bits.at(1) * bits.at(3) * bits.at(5) * bits.at(6) * bits.at(7) *
                   bits.at(9) * bits.at(10) * bits.at(14) * bits.at(15) * bits.at(16) *
                   bits.at(17) * bits.at(18) * bits.at(21) * bits.at(22) * bits.at(24);

    parity.at(3) = bits.at(2) * bits.at(4) * bits.at(6) * bits.at(7) * bits.at(8) *
                   bits.at(10) * bits.at(11) * bits.at(15) * bits.at(16) * bits.at(17) *
                   bits.at(18) * bits.at(19) * bits.at(22) * bits.at(23) * bits.at(25);

    parity.at(4) = bits.at(2) * bits.at(3) * bits.at(5) * bits.at(7) * bits.at(8) *
                   bits.at(9) * bits.at(11) * bits.at(12) * bits.at(16) * bits.at(17) *
                   bits.at(18) * bits.at(19) * bits.at(20) * bits.at(23) * bits.at(24) *
                   bits.at(26);

    parity.at(5) = bits.at(1) * bits.at(5) * bits.at(7) * bits.at(8) * bits.at(10) *
                   bits.at(11) * bits.at(12) * bits.at(13) * bits.at(15) * bits.at(17) *
                   bits.at(21) * bits.at(24) * bits.at(25) * bits.at(26);

    if (std::equal(parity.begin(), parity.end(), bits.begin() + 27))
        return -1 * bits.at(2);
    else
        return 0;
}

int findSubframeStart(std::deque<int> &buffer)
{
    int subframeStart = 0;
    auto corrResult = std::vector<int>(buffer.size() + 159);
    // Invert and spread preamble over 20 to find correlation using convolve function: {
    // 1, 1, 0, 1, 0, 0, 0, 1 }

    int reversePreamble[160]{
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
        -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
        -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
        -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
        -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};

    std::vector<int> indices;
    convolve(&corrResult, buffer, reversePreamble, buffer.size(), 160);

    std::vector<int>::iterator it = corrResult.begin();

    while (it != corrResult.end())
    {
        it++;
        it =
            std::find_if(it, corrResult.end(), [](int a)
                         { return (std::abs(a) > 153); });
        int dist = std::distance(corrResult.begin(), it);
        indices.push_back(dist - 159);
    }

    for (int i = 0; i < indices.size(); i++)
    {
        for (int k = 0; k < i; k++)
        {
            if (indices.at(i) - indices.at(k) == 6000 && indices.at(k) - 40 >= 0 &&
                indices.at(k) + 60 * 20 < buffer.size())
            {
                // Re-read bit vales for preamble verification ==============
                // Preamble occurrence is verified by checking the parity of
                // the first two words in the subframe. Now it is assumed that
                // bit boundaries a known. Therefore the bit values over 20ms are
                // combined to increase receiver performance for noisy signals.
                // in Total 62 bits mast be read :
                // 2 bits from previous subframe are needed for parity checking;
                // 60 bits for the first two 30bit words (TLM and HOW words).
                // The index is pointing at the start of TLM word.

                // Initialize of size 33, add blank element at the start for
                // better indexing
                int index = indices.at(k);
                std::vector<int> bits;
                int sum{0};
                int counter{0};
                for (int i = index - 40; i < index + 60 * 20; i++)
                {
                    sum += buffer.at(i);
                    counter++;
                    if (counter == 20)
                    {
                        if (sum > 0)
                            bits.push_back(1);
                        else
                            bits.push_back(-1);
                        counter = 0;
                        sum = 0;
                    }
                }

                std::vector<int> split_lo(bits.begin(), bits.begin() + 32);
                std::vector<int> split_hi(bits.begin() + 30, bits.end());
                split_lo.insert(split_lo.begin(), -10);
                split_hi.insert(split_hi.begin(), -10);

                int parity1 = parityCheck(split_lo, index);
                int parity2 = parityCheck(split_hi, index);

                // std::cout << "parity low: " << parity1 << std::endl;
                // std::cout << "parity high: " << parity2 << std::endl;

                if (parity1 != 0 && parity2 != 0)
                {
                    subframeStart = index;
                    goto endloop;
                }
            }
        }
    }
endloop:
    return subframeStart;
}

unsigned int bin2dec(std::vector<int> vec)
{
    unsigned int decimal = 0;

    for (int i = 0; i < vec.size(); i++)
    {
        decimal = decimal * 2 + vec.at(i);
    }
    return decimal;
}

int twosComp2dec(std::vector<int> vec)
{
    if (vec.front() == 1)
    {
        std::for_each(vec.begin(), vec.end(), [](int &a)
                      { a == 0 ? a = 1 : a = 0; });
        return -(bin2dec(vec) + 1);
    }
    else
        return bin2dec(vec);
}

std::vector<int> vecSelector(std::vector<int> &source, int start, int end)
{
    if (start < 0 || start >= end || end > source.size())
        return std::vector<int>(1, -1);
    return std::vector<int>(source.begin() + start - 1, source.begin() + end);
}

std::vector<int>
vecSelector(std::vector<int> &source, int start, int end, int start1, int end1)
{
    if (start < 0 || start >= end || end > source.size() || start1 < 0 ||
        start1 >= end1 || end1 > source.size())
        return std::vector<int>(1, -1);

    std::vector<int> temp(source.begin() + start - 1, source.begin() + end);
    temp.insert(temp.end(), source.begin() + start1 - 1, source.begin() + end1);
    return temp;
}

std::vector<float>
getPseudoRanges(std::vector<const void *> &data, int index, float startOffset, long int c)
{
    std::vector<float> pseudoRanges(data.size());
    for (int i = 0; i < data.size(); i++)
    {
        const float *in = reinterpret_cast<const float *>(data[i]);
        pseudoRanges.at(i) = in[index];
    }

    int minimum = floor(*std::min_element(pseudoRanges.begin(), pseudoRanges.end()));
    // std::cout << "minimum: " << minimum << std::endl;
    std::transform(pseudoRanges.begin(),
                   pseudoRanges.end(),
                   pseudoRanges.begin(),
                   [minimum, startOffset, c](float a)
                   {
                       return a == 0 ? 0 : ((a - minimum + startOffset) * c / 1000);
                   });

    return pseudoRanges;
}

const double PI = 3.141592653589793238460;

void custom_fft(std::valarray<std::complex<double>> &x)
{
    const size_t N = x.size();
    if (N <= 1)
        return;

    // divide
    std::valarray<std::complex<double>> even = x[std::slice(0, N / 2, 2)];
    std::valarray<std::complex<double>> odd = x[std::slice(1, N / 2, 2)];

    // conquer
    custom_fft(even);
    custom_fft(odd);

    // combine
    for (size_t k = 0; k < N / 2; ++k)
    {
        std::complex<double> t = std::polar(1.0, -2 * PI * k / N) * odd[k];
        x[k] = even[k] + t;
        x[k + N / 2] = even[k] - t;
    }
}

// inverse fft (in-place)
void custom_ifft(std::valarray<std::complex<double>> &x)
{
    // conjugate the complex numbers
    x = x.apply(std::conj);

    // forward fft
    custom_fft(x);

    // conjugate the complex numbers again
    x = x.apply(std::conj);

    // scale the numbers
    x /= x.size();
}