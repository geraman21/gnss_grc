#include "channel.h"

Channel::Channel() 
 : prn{0}, acquiredFreq{0}, codePhase{0}, status{0} { }

Channel::Channel(int _prn, float _acquiredFreq, int _codePhase, char _status) 
 : prn{_prn}, acquiredFreq{_acquiredFreq}, codePhase{_codePhase}, status{_status} { }
 
