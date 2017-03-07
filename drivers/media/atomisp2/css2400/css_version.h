/*
 * Support for Intel Camera Imaging ISP subsystem.
 *
 * Copyright (c) 2010 - 2014 Intel Corporation. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

//
// CSS version file
//

#ifndef __CSS_VERSION_H
#define __CSS_VERSION_H

#define VER	0
#define SUBVER	8

/*
Changelog
---------


VER 0.5, released 05/02/14:
	- Added versioning

VER 0.6, released 11/02/14:
	- Version release mainly for driver and AIC bug fixes

	Changes/Fixes:
		AIC bugs  : 1399,1588,1590,1589,1703 (Linearization hang)
		FW versioning added
		FW tracer for debug in DMEM2
		Removed A0 support from code
		Fixed corrupted fields of gird x/y_end for AWB & AE


VER 0.7, released 17/02/14:
        - Version release mainly for driver and AIC bug fixes

        Changes/Fixes:
                AIC bugs  : 1703 RGB gamma enable
		1st striping pipe in smoketest - if_striped


VER 0.8, released 24/02/14:
        - Version release mainly for validation IGK and SVOS run  (patch 1667)

        Changes/Fixes:
		added missing file to css_skycam_a0t_system.proj.mk
		Updated reset function in mini_vid.c
		added prepare function to mini_vid.c


*/

#endif /* __CSS_VERSION_H */
