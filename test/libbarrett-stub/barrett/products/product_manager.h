/*
 * product_manager.h
 *
 *  Created on: Jan 3, 2011
 *      Author: dc
 */

#ifndef BARRETT_PRODUCTS_PRODUCT_MANAGER_H_
#define BARRETT_PRODUCTS_PRODUCT_MANAGER_H_


#include <barrett/products/puck.h>
#include <barrett/products/safety_module.h>
#include <barrett/systems/wam.h>


namespace barrett {

// Forward declarations
// namespace systems{
//   template<size_t DOF> class Wam;
// }

class ProductManager {
public:
        void wakeAllPucks() const 
        { 
                return;
        }

        SafetyModule * getSafetyModule()
        {
                return new SafetyModule(new Puck());
        }

        void waitForWam(bool promptOnZeroing = true)
        {
                return;
        }

        systems::Wam<7> * getWam7(bool waitForShiftActivate = true, const char* configPath = NULL)
        {
                return new systems::Wam<7>();
        }
};

}

#endif /* BARRETT_PRODUCTS_PRODUCT_MANAGER_H_ */
