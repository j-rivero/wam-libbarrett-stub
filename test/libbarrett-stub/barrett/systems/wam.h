/*
	Copyright 2009, 2010 Barrett Technology <support@barrett.com>

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

/** Defines systems::Wam.
 *
 * @file systems/wam.h
 * @date Sep 25, 2009
 * @author Dan Cody
 */


#ifndef BARRETT_SYSTEMS_WAM_H_
#define BARRETT_SYSTEMS_WAM_H_


#include <vector>

#include <Eigen/Core>
#include <libconfig.h++>

#include <barrett/units.h>


namespace barrett {
namespace systems {


template<size_t DOF>
class Wam {
public:
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

    // genericPucks must be ordered by joint and must break into torque groups as arranged
    Wam()
    { };

    ~Wam()
    { };

    jp_type getJointPositions() const
    {
        jp_type foo;
        return foo;
    }

    pose_type getToolPose() const
    { 
        pose_type foo;
        return foo;
    }

    void gravityCompensate(bool compensate = true)
    { 
         return;
    }

    void moveHome(bool blocking = true)
    { 
         return;
    }

    void moveTo(const jp_type& destination, bool blocking = true, double velocity = 0.5, double acceleration = 0.5)
    {
         return;
    }

    bool moveIsDone() const
    {
        return true;
    }
    void idle()
    {
         return;
    }
};


}
}

#endif /* BARRETT_SYSTEMS_WAM_H_ */
