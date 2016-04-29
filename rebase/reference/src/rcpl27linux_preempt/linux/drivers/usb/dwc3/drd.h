/****
 * drd.c - DesignWare USB3 DRD Controller Dual Role Switch Funciton Header
 *
 * Copyright (C) 2014 Advanced Micro Devices, Inc.
 *
 * Author: Huang Rui <ray.huang@... <http://gmane.org/get-address.php?address=ray.huang%2d5C7GfCeVMHo%40public.gmane.org>>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2  of
 * the License as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __DRIVERS_USB_DWC3_DRD_H
#define __DRIVERS_USB_DWC3_DRD_H

#include "core.h"
#include "gadget.h"
#include "io.h"

extern int dwc3_drd_to_host(struct dwc3 *dwc);
extern int dwc3_drd_to_device(struct dwc3 *dwc);
extern int dwc3_drd_to_otg(struct dwc3 *dwc);

#endif /* __DRIVERS_USB_DWC3_CORE_H */

