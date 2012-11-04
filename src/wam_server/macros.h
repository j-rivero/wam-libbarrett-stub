#include <errno.h>
#include <string.h>

#define SMS fprintf(stderr,"Checkpoint: %s:%d.\n",__FILE__,__LINE__)

#define FATAL(msg)   \
   do {  \
      fprintf(stderr,"%s:%d:%s: %s\n",__FILE__,__LINE__,msg,strerror(errno)); \
      exit(-1);   \
   } while(0)

#define DEBUG(format, ...) \
        fprintf(stderr,format,##__VA_ARGS__)

