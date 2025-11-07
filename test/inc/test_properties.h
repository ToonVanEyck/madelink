#pragma once

#include "madelink_shared.h"

#define TEST_PROP_SIZE (10)

/* Define 6 example properties*/

#define PROP_STATIC_RW (mdl_prop_id_t)(0)
#define PROP_STATIC_RO (mdl_prop_id_t)(1)
#define PROP_STATIC_WO (mdl_prop_id_t)(2)
#define PROP_DYNAMIC_RW (mdl_prop_id_t)(3)
#define PROP_DYNAMIC_RO (mdl_prop_id_t)(4)
#define PROP_DYNAMIC_WO (mdl_prop_id_t)(5)
#define PROP_STATIC_RW_HALF_SIZE (mdl_prop_id_t)(6)

#define PROPERTY_CNT (7) /* Update this when adding new properties. */

/* Extern list of property used by both the master and the nodes. */
extern mdl_prop_t mdl_prop_list[PROPERTY_CNT];