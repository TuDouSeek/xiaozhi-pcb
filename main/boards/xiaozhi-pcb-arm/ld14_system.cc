#include "ld14_system.h"

#include "ld14_driver.h"
#include "ld14_web.h"

void ld14_system_init(void) {
    ld14_driver_init();
    ld14_web_init();
}
