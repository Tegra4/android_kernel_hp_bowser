/*
 * Copyright (c) 2011, NVIDIA CORPORATION, All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 * Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * Neither the name of NVIDIA CORPORATION nor the names of its contributors
 * may be used to endorse or promote products derived from this software
 * without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
 * IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 * TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __OV9740_H__
#define __OV9740_H__

#include <linux/ioctl.h>  /* For IOCTL macros */

#define OV9740_IOCTL_SET_MODE            _IOW('o', 1, struct ov9740_mode)
//#define OV9740_IOCTL_GET_STATUS          _IOR('o', 2, struct ov9740_status)
#define OV9740_IOCTL_SET_FRAME_LENGTH  _IOW('o', 2, __u32)
#define OV9740_IOCTL_SET_COARSE_TIME   _IOW('o', 3, __u32)
#define OV9740_IOCTL_SET_GAIN          _IOW('o', 4, __u16)
#define OV9740_IOCTL_GET_STATUS        _IOR('o', 5, __u8)
#define OV9740_IOCTL_SET_BINNING       _IOW('o', 6, __u8)
//#define OV9740_IOCTL_TEST_PATTERN    _IOW('o', 7, enum ov9740_test_pattern)
#define OV9740_IOCTL_GET_ISO           _IOW('o', 8, __u8)
#define OV9740_IOCTL_SET_CAMERA_MODE   _IOW('o', 10, __u32)
#define OV9740_IOCTL_SYNC_SENSORS      _IOW('o', 11, __u32)

#define OV9740_IOCTL_SET_WHITE_BALANCE _IOW('o', 12, __u32)
#define OV9740_IOCTL_SET_EXPOSURE_COMPENSATION  _IOW('o', 13, __u32)
#define OV9740_IOCTL_SET_3A_LOCK      _IOW('o', 14, __u32)
#define OV9740_IOCTL_GET_EXPOSURE_TIME _IOW('o', 15, struct ov9740_exposure_time)

struct ov9740_mode {
    int xres;
    int yres;
    int banding;
};

struct ov9740_exposure_time {
    int exposure_line;
    int vts;
    int max_fps;
};

#if 1
struct ov9740_status {
    int data;
    int status;
};
#endif
enum {
	OV9740_WB_AUTO,
	OV9740_WB_INCANDESCENT,
	OV9740_WB_DAYLIGHT,
	OV9740_WB_FLUORESCENT,
	OV9740_WB_CLOUDY,
};

enum {
	OV9740_EXPOSURE_NEGATIVE_3,
	OV9740_EXPOSURE_NEGATIVE_2,
	OV9740_EXPOSURE_NEGATIVE_1,
	OV9740_EXPOSURE_0,
	OV9740_EXPOSURE_1,
	OV9740_EXPOSURE_2,
	OV9740_EXPOSURE_3,
	OV9740_EXPOSURE_MAX,
};

#ifdef __KERNEL__
struct ov9740_platform_data {
    int (*power_on)(void);
    int (*power_off)(void);
    void (*synchronize_sensors)(void);
};
#endif /* __KERNEL__ */

#endif  /* __OV9740_H__ */

