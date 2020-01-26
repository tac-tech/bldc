#ifndef APP_MAVERICK_H_
#define APP_MAVERICK_H_

#include "soft_spi.h"

// Function Definitions
void maverick_configure(app_configuration *conf);
void maverick_init(app_configuration *config);
void maverick_stop();

#endif //APP_MAVERICK_H_