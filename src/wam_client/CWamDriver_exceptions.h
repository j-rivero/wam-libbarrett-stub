#ifndef _WAMDRIVER_EXCEPTIONS
#define _WAMDRIVER_EXCEPTIONS

#include "exceptions.h"

#include <sstream>
#include <string.h>
#include <stdio.h>

/**
* WamDriver exceptions
*/
class CWamDriverException : public CException {
    public:
        /** constructor
         * @param where Set it to __HERE__
         * @param error_msg string with the error message */
        CWamDriverException(const std::string& where, const std::string& error_msg);
};

class CDisconnectedException : public CException {
    public:
        /** constructor
         * @param where Set it to _HERE_
         * @param error_msg string with the error message */
        CDisconnectedException(const std::string& where, const std::string& error_msg);
};

#endif
