#ifndef _ESPIDF_APP_EVENT_H_
#define _ESPIDF_APP_EVENT_H_

#include "esp_event.h"

#define LULUPET_EVENT_BASE "LULUPET_EVENT"

#define FOREACH_EVENT(X)                                                       \
    X(LULUPET_EVENT_KEY)                                                       \
    X(LULUPET_EVENT_TAKE_PHOTO)                                                \
    X(LULUPET_EVENT_OTA)

#define GEN_ENUM(v) v,

enum lulupet_event { FOREACH_EVENT(GEN_ENUM) };

#endif // _ESPIDF_APP_EVENT_H_
