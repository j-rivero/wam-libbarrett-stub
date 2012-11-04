/*
	Copyright 2010, 2011, 2012 Barrett Technology <support@barrett.com>

	This file is part of libbarrett.

	This version of libbarrett is free software: you can redistribute it
	and/or modify it under the terms of the GNU General Public License as
	published by the Free Software Foundation, either version 3 of the
	License, or (at your option) any later version.

	This version of libbarrett is distributed in the hope that it will be
	useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License along
	with this version of libbarrett.  If not, see
	<http://www.gnu.org/licenses/>.

	Further, non-binding information about licensing is available at:
	<http://wiki.barrett.com/libbarrett/wiki/LicenseNotes>
*/

/*
 * safety_module.h
 *
 *  Created on: Nov 4, 2010
 *      Author: dc
 */

#ifndef BARRETT_PRODUCTS_SAFETY_MODULE_H_
#define BARRETT_PRODUCTS_SAFETY_MODULE_H_


#include <string>

#include <barrett/products/puck.h>


namespace barrett {


class SafetyModule {
    public:
        enum SafetyMode {
                ESTOP, IDLE, ACTIVE
        };


        SafetyModule(Puck* puck = NULL)
        { }
        ~SafetyModule() {}

        void waitForMode(enum SafetyMode mode,
                         bool printMessage = true, double pollingPeriod_s = 0.25)
        {
            return;
        }
};


}


#endif /* BARRETT_PRODUCTS_SAFETY_MODULE_H_ */
