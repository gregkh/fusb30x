/****************************************************************************
 * Company:         Fairchild Semiconductor
 *
 * Author           Date          Comment
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * G. Noblesmith
 *
 *
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 * Software License Agreement:
 *
 * The software is owned by the Company and/or its supplier, and is protected
 * under applicable copyright laws.  All rights are reserved. Any use in
 * violation of the foregoing restrictions may subject the user to criminal
 * sanctions under applicable laws, as well as to civil liability for the breach
 * of the terms and conditions of this license.
 *
 * THIS SOFTWARE IS PROVIDED IN AN “AS IS” CONDITION. NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
 * TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
 * IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
 * CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 *****************************************************************************/

#ifndef __VDM_BITFIELD_TRANSLATORS_H__
#define __VDM_BITFIELD_TRANSLATORS_H__

#include "platform.h"

/*
 * Functions that convert bits into internal header representations...
 */
// converts 32 bits into an ID Header struct
IdHeader getIdHeader(UINT32 in);

/*
 * Functions that convert internal header representations into bits...
 */
// converts ID Header struct into 32 bits
UINT32 getBitsForIdHeader(IdHeader in);

/*
 * Functions that convert bits into internal VDO representations...
 */
CertStatVdo getCertStatVdo(UINT32 in);
ProductVdo getProductVdo(UINT32 in);
CableVdo getCableVdo(UINT32 in);
AmaVdo getAmaVdo(UINT32 in);

/*
 * Functions that convert internal VDO representations into bits...
 */
// converts Product VDO struct into 32 bits
UINT32 getBitsForProductVdo(ProductVdo in);
// converts Cert Stat VDO struct into 32 bits
UINT32 getBitsForCertStatVdo(CertStatVdo in);
// converts Cable VDO struct into 32 bits
UINT32 getBitsForCableVdo(CableVdo in);
// converts AMA VDO struct into 32 bits
UINT32 getBitsForAmaVdo(AmaVdo in);

#endif // header guard
