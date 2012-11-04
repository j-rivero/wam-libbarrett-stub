#include "CWamDriver_exceptions.h"

const std::string error_msg="[CWamDriver class] - ";

CWamDriverException::CWamDriverException(const std::string& where,const std::string& error_msg) : CException(where,error_msg)
{
  this->error_msg+=error_msg;
}

CDisconnectedException::CDisconnectedException(const std::string& where,const std::string& error_msg):CException(where,error_msg)
{
  this->error_msg+=error_msg;
}

