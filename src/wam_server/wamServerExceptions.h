#ifndef _WAMSERVEREXCEPTIONS
#define _WAMSERVEREXCEPTIONS

#include "exceptions.h"

#include <sstream>
#include <string.h>
#include <stdio.h>

/**
* WamDriver exceptions
*/
class WamServerException : public CException {
    public:
        /** constructor
         * @param where Set it to _HERE_
         * @param error_msg string with the error message */
        WamServerException(const std::string& where, const std::string& error_msg);
};

class CDisconnectedException : public CException {
    public:
        /** constructor
         * @param where Set it to _HERE_
         * @param error_msg string with the error message */
        CDisconnectedException(const std::string& where, const std::string& error_msg);
};

class CNewConnectionException : public CException {
    public:
        /** constructor
         * @param where Set it to _HERE_
         * @param error_msg string with the error message */
        CNewConnectionException(const std::string& where, const std::string& error_msg);
};
#endif
