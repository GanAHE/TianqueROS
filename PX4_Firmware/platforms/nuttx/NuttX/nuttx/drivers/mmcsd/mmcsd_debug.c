/****************************************************************************
 * drivers/mmcsd/mmcsd_debug.c
 *
 *   Copyright (C) 2008-2009 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include "mmcsd_csd.h"
#include "mmcsd.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mmcsd_dmpcsd
 *
 * Description:
 *   Dump the contents of the CSD
 *
 ****************************************************************************/

#if defined(CONFIG_DEBUG_INFO) && defined(CONFIG_DEBUG_FS)
void mmcsd_dmpcsd(FAR const uint8_t *csd, uint8_t cardtype)
{
  bool mmc = (cardtype == MMCSD_CARDTYPE_MMC);
#if defined(CONFIG_DEBUG_FS_INFO)
  bool sd2 = (MMCSD_CSD_CSDSTRUCT(csd) == 1);
#endif

  finfo("CSD\n");
  finfo("  CSD_STRUCTURE:           1.%d\n",   MMCSD_CSD_CSDSTRUCT(csd));
  if (mmc)
    {
      finfo("  MMC SPEC_VERS:           %d\n", MMC_CSD_SPECVERS(csd));
    }

  finfo("  TAAC:\n",
      sd2 ? SD20_CSD_TAC_TIMEVALUE(csd) : MMCSD_CSD_TAAC_TIMEVALUE(csd));
  finfo("    TIME_VALUE:            0x%02x\n",
      sd2 ? SD20_CSD_TAC_TIMEVALUE(csd) : MMCSD_CSD_TAAC_TIMEVALUE(csd));
  finfo("    TIME_UNIT:             0x%02x\n",
      sd2 ? SD20_CSD_TAC_TIMEUNIT(csd) : MMCSD_CSD_TAAC_TIMEUNIT(csd));
  finfo("  NSAC:                    0x%02x\n",
      sd2 ? SD20_CSD_NSAC(csd) : MMCSD_CSD_NSAC(csd));
  finfo("  TRAN_SPEED:\n");
  finfo("    TIME_VALUE:            0x%02x\n",
      sd2 ? SD20_CSD_TRANSPEED_TIMEVALUE(csd) : MMCSD_CSD_TRANSPEED_TIMEVALUE(csd));
  finfo("    RATE_UNIT:             0x%02x\n",
      sd2 ? SD20_CSD_TRANSPEED_TRANSFERRATEUNIT(csd) : MMCSD_CSD_TRANSPEED_TRANSFERRATEUNIT(csd));
  finfo("  CCC:                     0x%03x\n",
      sd2 ? SD20_CSD_CCC(csd) : MMCSD_CSD_CCC(csd));
  finfo("  READ_BL_LEN:             %d\n",
      sd2 ? SD20_CSD_READBLLEN(csd) : MMCSD_CSD_READBLLEN(csd));
  finfo("  READ_BL_PARTIAL:         %d\n",
      sd2 ? SD20_CSD_READBLPARTIAL(csd) : MMCSD_CSD_READBLPARTIAL(csd));
  finfo("  WRITE_BLK_MISALIGN:      %d\n",
      sd2 ? SD20_CSD_WRITEBLKMISALIGN(csd) : MMCSD_CSD_WRITEBLKMISALIGN(csd));
  finfo("  READ_BLK_MISALIGN:       %d\n",
      sd2 ? SD20_CSD_READBLKMISALIGN(csd) : MMCSD_CSD_READBLKMISALIGN(csd));
  finfo("  DSR_IMP:                 %d\n",
      sd2 ? SD20_CSD_DSRIMP(csd) : MMCSD_CSD_DSRIMP(csd));
  finfo("  C_SIZE:                  %d\n",
      sd2 ? SD20_CSD_CSIZE(csd) : MMCSD_CSD_CSIZE(csd));
  finfo("  VDD_R_CURR_MIN:          %d\n",
      sd2 ? SD20_CSD_VDDRCURRMIN(csd) : MMCSD_CSD_VDDRCURRMIN(csd));
  finfo("  VDD_R_CURR_MAX:          %d\n",
      sd2 ? SD20_CSD_VDDRCURRMAX(csd) : MMCSD_CSD_VDDRCURRMAX(csd));
  finfo("  VDD_W_CURR_MIN:          %d\n",
      sd2 ? SD20_CSD_VDDWCURRMIN(csd) : MMCSD_CSD_VDDWCURRMIN(csd));
  finfo("  VDD_W_CURR_MAX:          %d\n",
      sd2 ? SD20_CSD_VDDWCURRMAX(csd) : MMCSD_CSD_VDDWCURRMAX(csd));
  finfo("  C_SIZE_MULT:             %d\n",
      sd2 ? SD20_CSD_CSIZEMULT(csd) : MMCSD_CSD_CSIZEMULT(csd));
  if (mmc)
    {
      finfo("  MMC SECTOR_SIZE:        %d\n", MMC_CSD_SECTORSIZE(csd));
      finfo("  MMC ER_GRP_SIZE:        %d\n", MMC_CSD_ERGRPSIZE(csd));
      finfo("  MMC WP_GRP_SIZE:        %d\n",  MMC_CSD_WPGRPSIZE(csd));
      finfo("  MMC DFLT_ECC:           %d\n",  MMC_CSD_DFLTECC(csd));
    }
  else
    {
      finfo("  SD ER_BLK_EN:            %d\n",
          sd2 ? SD20_CSD_SDERBLKEN(csd) : SD_CSD_SDERBLKEN(csd));
      finfo("  SD SECTOR_SIZE:          %d\n",
          sd2 ? SD20_CSD_SECTORSIZE(csd) : SD_CSD_SECTORSIZE(csd));
      finfo("  SD WP_GRP_SIZE:          %d\n",
          sd2 ? SD_CSD_WPGRPSIZE(csd) : SD_CSD_WPGRPSIZE(csd));
    }
  finfo("  WP_GRP_EN:               %d\n",
      sd2 ? SD20_WPGRPEN(csd) : MMCSD_WPGRPEN(csd));
  finfo("  R2W_FACTOR:              %d\n",
      sd2 ? SD20_CSD_R2WFACTOR(csd) : MMCSD_CSD_R2WFACTOR(csd));
  finfo("  WRITE_BL_LEN:            %d\n",
      sd2 ? SD20_CSD_WRITEBLLEN(csd) : MMCSD_CSD_WRITEBLLEN(csd));
  finfo("  WRITE_BL_PARTIAL:        %d\n",
      sd2 ? SD20_CSD_WRITEBLPARTIAL(csd) : MMCSD_CSD_WRITEBLPARTIAL(csd));
  finfo("  FILE_FORMAT_GROUP:       %d\n",
      sd2 ? SD20_CSD_FILEFORMATGRP(csd) : MMCSD_CSD_FILEFORMATGRP(csd));
  finfo("  COPY:                    %d\n",
      sd2 ? SD20_CSD_COPY(csd) : MMCSD_CSD_COPY(csd));
  finfo("  PERM_WRITE_PROTECT:      %d\n",
      sd2 ? SD20_CSD_PERMWRITEPROTECT(csd) : MMCSD_CSD_PERMWRITEPROTECT(csd));
  finfo("  TMP_WRITE_PROTECT:       %d\n",
      sd2 ?SD20_CSD_TMPWRITEPROTECT(csd) : MMCSD_CSD_TMPWRITEPROTECT(csd));
  finfo("  FILE_FORMAT:             %d\n",
      sd2 ? SD20_CSD_FILEFORMAT(csd) : MMCSD_CSD_FILEFORMAT(csd));
  if (mmc)
    {
      finfo("  MMC ECC:                 %d\n",
          sd2 ? MMC_CSD_ECC(csd) : MMC_CSD_ECC(csd));
    }
  finfo("  CRC:                     %02x\n",
      sd2 ? SD20_CSD_CRC(csd) : MMCSD_CSD_CRC(csd));
}
#endif
