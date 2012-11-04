#include "wamServerExceptions.h"

const std::string error_msg="[WamServer class] - ";

WamServerException::WamServerException(const std::string& where,const std::string& error_msg):CException(where,error_msg)
{
  this->error_msg+=error_msg;
}

CDisconnectedException::CDisconnectedException(const std::string& where,const std::string& error_msg):CException(where,error_msg)
{
    fprintf(stderr, "creating string\n");
  this->error_msg+=error_msg;
}

CNewConnectionException::CNewConnectionException(const std::string& where,const std::string& error_msg):CException(where,error_msg)
{
  this->error_msg+=error_msg;
}

