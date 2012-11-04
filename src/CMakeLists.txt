INCLUDE_DIRECTORIES(.)
#How can I fix the need to include .. for make install?
SET(headers ${headers} ../constants.h ../protocol.h)

ADD_SUBDIRECTORY(wam_server)
ADD_SUBDIRECTORY(wam_client)
