#include "vector"

void calcloopCoef(float& coeff1,
                  float& coeff2,
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

    if (num_in == 0) {
        return linspaced;
    }
    if (num_in == 1) {
        linspaced.push_back(start_in);
        return linspaced;
    }

    float delta = (end_in - start_in) / (num_in - 1);

    for (int i = 0; i < num_in - 1; ++i) {
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

template <class T>
void convolve(std::vector<T>* result, T* x, T* y, int lenx, int leny)
{
    T s, *xp, *yp;
    int lenz;
    int i, n, n_lo, n_hi;

    lenz = lenx + leny - 1;

    for (i = 0; i < lenz; i++) {
        s = 0.0;
        n_lo = 0 > (i - leny + 1) ? 0 : i - leny + 1;
        n_hi = lenx - 1 < i ? lenx - 1 : i;
        xp = x + n_lo;
        yp = y + i - n_lo;
        for (n = n_lo; n <= n_hi; n++) {
            s += *xp * *yp;
            xp++;
            yp--;
        }
        (*result).push_back(s);
    }
}