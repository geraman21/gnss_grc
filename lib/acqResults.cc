#include <vector>
#include "acqResults.h"

AcqResults::AcqResults(int _PRN, float _carrFreq, float _codePhase, float _peakMetric) : PRN{_PRN}, carrFreq{_carrFreq}, codePhase{_codePhase}, peakMetric{_peakMetric}
{
}