#ifndef __FSC_VERSION_H__
#define __FSC_VERSION_H__

#include "platform.h"

/* Program Revision constant.  When updating firmware, change this.  */
#define FSC_TYPEC_CORE_FW_REV_UPPER  'F'
#define FSC_TYPEC_CORE_FW_REV_LOWER  '1'

UINT8 core_get_rev_lower(VOID);
UINT8 core_get_rev_upper(VOID);

#endif // __FSC_VERSION_H_
