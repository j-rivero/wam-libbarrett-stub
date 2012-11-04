/***************************************************************************
 *   Copyright (C) 2009 by Pol Monsó   *
 *   pmonso@iri.upc.edu   *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/


#define SMS fprintf(stderr,"Checkpoint: %s:%d.\n",__FILE__,__LINE__)

#define FATAL(msg)   \
   do {  \
      fprintf(stderr,"%s:%d:%s: %s\n",__FILE__,__LINE__,msg,strerror(errno)); \
      exit(-1);   \
   } while(0)

#define DEBUG(format, ...)

//#define DEBUG(format, ...) \
   fprintf(stdout,format,##__VA_ARGS__)

#define DEBUGE(format, ...) \
    fprintf(stderr,format,##__VA_ARGS__)

