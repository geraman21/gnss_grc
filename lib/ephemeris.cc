#include "ephemeris.h"
#include "helper-functions.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

Ephemeris::Ephemeris(std::vector<int>& navBits)
{
    int D30Star = navBits.at(0);
    navBits.erase(navBits.begin());

    for (int i = 1; i <= 5; i++) {
        std::vector<int> subframe(navBits.begin() + 300 * (i - 1),
                                  navBits.begin() + 300 * i);


        std::cout << std::endl;
        // Correct polarity of the data bits in all 10 words

        for (int j = 1; j <= 10; j++) {
            if (D30Star == 1) {
                for (int a = 0; a < 24; a++) {
                    subframe.at(30 * (j - 1) + a) == 0
                        ? subframe.at(30 * (j - 1) + a) = 1
                        : subframe.at(30 * (j - 1) + a) = 0;
                }
            }
            D30Star = subframe.at(30 * j - 1);
        }

        int subframeID = bin2dec(vecSelector(subframe, 50, 52));


        switch (subframeID) {
        case 1:
            weekNumber = bin2dec(std::vector<int>(subframe.begin() + 61 - 1,
                                                  subframe.begin() + 70)) +
                         1024;
            accuracy = bin2dec(vecSelector(subframe, 73, 76));

            for (auto i :
                 std::vector<int>(subframe.begin() + 73 - 1, subframe.begin() + 76)) {
                std::cout << i;
            };
            std::cout << std::endl;

            health = bin2dec(vecSelector(subframe, 77, 82));
            T_GD = twosComp2dec(vecSelector(subframe, 197, 204)) * pow(2, -31);

            IODC = bin2dec(vecSelector(subframe, 83, 84, 197, 204));

            t_oc = bin2dec(vecSelector(subframe, 219, 234)) * pow(2, 4);
            a_f2 = twosComp2dec(vecSelector(subframe, 241, 248)) * pow(2, -55);
            a_f1 = twosComp2dec(vecSelector(subframe, 249, 264)) * pow(2, -43);
            a_f0 = twosComp2dec(vecSelector(subframe, 271, 292)) * pow(2, -31);
            break;
        case 2:
            IODE_sf2 = bin2dec(vecSelector(subframe, 61, 68));
            C_rs = twosComp2dec(vecSelector(subframe, 69, 84)) * pow(2, -5);
            deltan = twosComp2dec(vecSelector(subframe, 91, 106)) * pow(2, -43) * gpsPi;
            M_0 = twosComp2dec(vecSelector(subframe, 107, 114, 121, 144)) * pow(2, -31) *
                  gpsPi;
            C_uc = twosComp2dec(vecSelector(subframe, 151, 166)) * pow(2, -29);
            e = bin2dec(vecSelector(subframe, 167, 174, 181, 204)) * pow(2, -33);
            C_us = twosComp2dec(vecSelector(subframe, 211, 226)) * pow(2, -29);
            sqrtA = bin2dec(vecSelector(subframe, 227, 234, 241, 264)) * pow(2, -19);
            t_oe = bin2dec(vecSelector(subframe, 271, 286)) * pow(2, 4);
            break;
        case 3:
            C_ic = twosComp2dec(vecSelector(subframe, 61, 76)) * pow(2, -29);
            omega_0 = twosComp2dec(vecSelector(subframe, 77, 84, 91, 114)) * pow(2, -31) *
                      gpsPi;
            C_is = twosComp2dec(vecSelector(subframe, 121, 136)) * pow(2, -29);
            i_0 = twosComp2dec(vecSelector(subframe, 137, 144, 151, 174)) * pow(2, -31) *
                  gpsPi;
            C_rc = twosComp2dec(vecSelector(subframe, 181, 196)) * pow(2, -5);
            omega = twosComp2dec(vecSelector(subframe, 197, 204, 211, 234)) *
                    pow(2, -31) * gpsPi;
            omegaDot =
                twosComp2dec(vecSelector(subframe, 241, 264)) * pow(2, -43) * gpsPi;
            IODE_sf3 = bin2dec(vecSelector(subframe, 271, 278));
            iDot = twosComp2dec(vecSelector(subframe, 279, 292)) * pow(2, -43) * gpsPi;
            break;
        default:
            break;
        }

        TOW = bin2dec(vecSelector(subframe, 37, 47)) * 60 - 30;
    }
}