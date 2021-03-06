# cython: language_level=3
# -*- coding: utf8 -*-
#
#   pyflycapture2 - python bindings for libflycapture2_c
#   Copyright (C) 2012 Robert Jordens <jordens@phys.ethz.ch>
#
#   This program is free software: you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation, either version 3 of the License, or
#   (at your option) any later version.
#
#   This program is distributed in the hope that it will be useful,
#   but WITHOUT ANY WARRANTY; without even the implied warranty of
#   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#   GNU General Public License for more details.
#
#   You should have received a copy of the GNU General Public License
#   along with this program.  If not, see <http://www.gnu.org/licenses/>.

from _FlyCapture2_C cimport *
include "flycapture2_enums.pxi"
from libc.stdlib cimport malloc, free

import numpy as np
cimport numpy as np

from cpython cimport PyObject, Py_INCREF

ctypedef np.uint8_t DTYPE_t

cdef extern from "numpy/arrayobject.h":
    object PyArray_NewFromDescr(object subtype, np.dtype descr,
                                int nd, np.npy_intp* dims,
                                np.npy_intp* strides,
                                void* data, int flags, object obj)

np.import_array()

class ApiError(Exception):
    pass

cdef raise_error(fc2Error e):
    if e != FC2_ERROR_OK:
        raise ApiError(e, fc2ErrorToDescription(e))

def get_library_version():
    cdef fc2Version v
    cdef fc2Error r
    with nogil:
        r = fc2GetLibraryVersion(&v)
    raise_error(r)
    return {"major": v.major, "minor": v.minor,
            "type": v.type, "build": v.build}


cdef class Context:
    cdef fc2Context ctx

    def __cinit__(self):
        cdef fc2Error r
        with nogil:
            r = fc2CreateContext(&self.ctx)
        raise_error(r)

    def __dealloc__(self):
        cdef fc2Error r
        with nogil:
            r = fc2DestroyContext(self.ctx)
        raise_error(r)

    def version(self):
        return 2.0

    def get_num_of_cameras(self):
        cdef unsigned int n
        cdef fc2Error r
        with nogil:
            r = fc2GetNumOfCameras(self.ctx, &n)
        raise_error(r)
        return n

    def get_num_of_devices(self):
        cdef unsigned int n
        cdef fc2Error r
        with nogil:
            r = fc2GetNumOfDevices(self.ctx, &n)
        raise_error(r)
        return n

    def get_camera_from_index(self, unsigned int index):
        cdef fc2PGRGuid g
        cdef fc2Error r
        with nogil:
            r = fc2GetCameraFromIndex(self.ctx, index, &g)
        raise_error(r)
        return g.value[0], g.value[1], g.value[2], g.value[3]

    def get_camera_from_serialnumber(self, unsigned int serialNumber):
        # fc2Error fc2GetCameraFromSerialNumber(fc2Context context, unsigned int serialNumber, fc2PGRGuid *pGuid) nogil
        cdef fc2PGRGuid g
        cdef fc2Error r
        with nogil:
            r = fc2GetCameraFromSerialNumber(self.ctx, serialNumber, &g)
        raise_error(r)
        return g.value[0], g.value[1], g.value[2], g.value[3]

    def get_camera_serialnumber_from_index(self, unsigned int index):
        # fc2Error fc2GetCameraSerialNumberFromIndex(fc2Context context, unsigned int index, unsigned int *pSerialNumber) nogil
        cdef unsigned int serial_number
        cdef fc2Error r
        with nogil:
            r = fc2GetCameraSerialNumberFromIndex(self.ctx, index, &serial_number)
        raise_error(r)
        return serial_number

    def get_camera_info(self):
        cdef fc2CameraInfo i
        cdef fc2Error r
        with nogil:
            r = fc2GetCameraInfo(self.ctx, &i)
        raise_error(r)
        ret = {"serial_number": i.serialNumber,
             "model_name": i.modelName,
             "vendor_name": i.vendorName,
             "sensor_info": i.sensorInfo,
             "sensor_resolution": i.sensorResolution,
             "firmware_version": i.firmwareVersion,
             "firmware_build_time": i.firmwareBuildTime,}
        return ret

    def connect(self, unsigned int a, unsigned int b,
            unsigned int c, unsigned int d):
        cdef fc2PGRGuid g
        cdef fc2Error r
        g.value[0], g.value[1], g.value[2], g.value[3] = a, b, c, d
        with nogil:
            r = fc2Connect(self.ctx, &g)
        raise_error(r)

    def disconnect(self):
        cdef fc2Error r
        with nogil:
            r = fc2Disconnect(self.ctx)
        raise_error(r)

    def get_video_mode_and_frame_rate_info(self, 
            fc2VideoMode mode, fc2FrameRate framerate):
        cdef fc2Error r
        cdef BOOL supp
        with nogil:
            r = fc2GetVideoModeAndFrameRateInfo(self.ctx, mode,
                    framerate, &supp)
        raise_error(r)
        return bool(supp)

    def get_video_mode_and_frame_rate(self):
        cdef fc2Error r
        cdef fc2VideoMode mode
        cdef fc2FrameRate framerate
        with nogil:
            r = fc2GetVideoModeAndFrameRate(self.ctx, &mode, &framerate)
        raise_error(r)
        return mode, framerate

    def set_video_mode_and_frame_rate(self, fc2VideoMode mode,
            fc2FrameRate framerate):
        cdef fc2Error r
        with nogil:
            r = fc2SetVideoModeAndFrameRate(self.ctx, mode, framerate)
        raise_error(r)

    def set_user_buffers(self, np.ndarray[np.uint8_t, ndim=2] buff not None):
        cdef fc2Error r
        r = fc2SetUserBuffers(self.ctx, <unsigned char *>buff.data,
            buff.shape[1], buff.shape[0])
        raise_error(r)
        # TODO: INCREF buff

    def start_capture(self):
        cdef fc2Error r
        with nogil:
            r = fc2StartCapture(self.ctx)
        raise_error(r)

    def stop_capture(self):
        cdef fc2Error r
        with nogil:
            r = fc2StopCapture(self.ctx)
        raise_error(r)

    def retrieve_buffer(self, Image img=None):
        cdef fc2Error r
        if img is None:
            img = Image()
        with nogil:
            r = fc2RetrieveBuffer(self.ctx, &img.img)
        raise_error(r)
        return img

    def get_property_info(self, fc2PropertyType prop):
        cdef fc2PropertyInfo pi
        pi.type = prop
        cdef fc2Error r
        with nogil:
            r = fc2GetPropertyInfo(self.ctx, &pi)
        raise_error(r)
        return {"type": pi.type,
                "present": bool(pi.present),
                "auto_supported": bool(pi.autoSupported),
                "manual_supported": bool(pi.manualSupported),
                "on_off_supported": bool(pi.onOffSupported),
                "one_push_supported": bool(pi.onePushSupported),
                "abs_val_supported": bool(pi.absValSupported),
                "read_out_supported": bool(pi.readOutSupported),
                "min": pi.min,
                "max": pi.max,
                "abs_min": pi.absMin,
                "abs_max": pi.absMax,
                "units": pi.pUnits,
                "unit_abbr": pi.pUnitAbbr,}

    def get_property(self, fc2PropertyType type):
        cdef fc2Error r
        cdef fc2Property p
        p.type = type
        with nogil:
            r = fc2GetProperty(self.ctx, &p)
        raise_error(r)
        return {"type": p.type,
                "present": bool(p.present),
                "auto_manual_mode": bool(p.autoManualMode),
                "abs_control": bool(p.absControl),
                "on_off": bool(p.onOff),
                "one_push": bool(p.onePush),
                "abs_value": p.absValue,
                "value_a": p.valueA,
                "value_b": p.valueB,}

    def set_property(self, type, present, on_off, auto_manual_mode,
            abs_control, one_push, abs_value, value_a, value_b):
        cdef fc2Error r
        cdef fc2Property p
        p.type = type
        p.present = present
        p.autoManualMode = auto_manual_mode
        p.absControl = abs_control
        p.onOff = on_off
        p.onePush = one_push
        p.absValue = abs_value
        p.valueA = value_a
        p.valueB = value_b
        with nogil:
            r = fc2SetProperty(self.ctx, &p)
        raise_error(r)
    
    def set_property_abs_value(self, fc2PropertyType prop_type, value):  
        prop = self.get_property( prop_type )        
        # prop['auto_manual_mode'] = False
        prop ={'on_off': True, 'type': prop_type, 'value_a': 3L, 'abs_value': 0.0, 'one_push': False, 'abs_control': True, 'auto_manual_mode': False, 'present': True, 'value_b': 0L}
        prop['abs_value'] = value
        self.set_property( **prop )
        return self.get_property( prop_type )['abs_value']


    def get_trigger_mode(self):
        cdef fc2Error r
        cdef fc2TriggerMode tm
        with nogil:
            r = fc2GetTriggerMode(self.ctx, &tm)
        return {"on_off": bool(tm.onOff),
                "polarity": tm.polarity,
                "source": tm.source,
                "mode": tm.mode,
                "parameter": tm.parameter }


    def set_trigger_mode(self, on_off, polarity, source, mode, parameter):
        cdef fc2Error r
        cdef fc2TriggerMode tm
        tm.onOff = on_off
        tm.polarity = polarity
        tm.source = source
        tm.mode = mode
        tm.parameter = parameter
        with nogil:
            r = fc2SetTriggerMode(self.ctx, &tm)
        raise_error(r)

    
    def fire_software_trigger(self):
        cdef fc2Error r
        with nogil:
            r = fc2FireSoftwareTrigger(self.ctx)
        raise_error(r)
        

    # fc2Error fc2WriteRegister(fc2Context context, unsigned int address, unsigned int value) nogil
    def set_register(self, unsigned int address, unsigned int value):
        cdef fc2Error r
        with nogil:
            r = fc2WriteRegister(self.ctx, address, value)
        raise_error(r)

    # fc2Error fc2ReadRegister(fc2Context context, unsigned int address, unsigned int *pValue) nogil
    def get_register(self, unsigned int address):
        cdef fc2Error r
        cdef unsigned int value
        with nogil:
            r = fc2ReadRegister(self.ctx, address, &value)
        raise_error(r)
        return value
   
    # fc2Error fc2GetEmbeddedImageInfo(fc2Context context, fc2EmbeddedImageInfo *pInfo) nogil
    # see: https://github.com/jordens/pyflycapture2/issues/12
    def get_embedded_image_info(self):
        cdef fc2Error r
        cdef fc2EmbeddedImageInfo pInfo
        
        with nogil:
            r = fc2GetEmbeddedImageInfo(self.ctx, &pInfo)
            # pInfo.timestamp( &pInfoProperty)
            # r = fc2GetEmbeddedImageTimeStamp(self.ctx)
            # r = fc2GetEmbeddedImageInfoProperty(self.ctx, &pInfoProperty)
        raise_error(r)
        return {'timestamp': pInfo.timestamp,
                'gain': pInfo.gain,
                'shutter': pInfo.shutter,
                'brightness': pInfo.brightness,
                'exposure': pInfo.exposure,
                'white_balance': pInfo.whiteBalance,
                'frame_counter': pInfo.frameCounter,
                'strobe_pattern': pInfo.strobePattern,
                'GPIO_pin_state': pInfo.GPIOPinState,
                'roi_position': pInfo.ROIPosition}
        
    # fc2Error fc2SetEmbeddedImageInfo(fc2Context context, fc2EmbeddedImageInfo *pInfo) nogil
    def set_embedded_image_info(self, fc2EmbeddedImageInfo pInfo):
        cdef fc2Error r
        with nogil:
            r = fc2SetEmbeddedImageInfo(self.ctx, &pInfo)
        raise_error(r)     


    def set_default_output_format(self, fc2PixelFormat pf):
        cdef fc2Error r
        with nogil:
            r = fc2SetDefaultOutputFormat(pf)
        raise_error(r)


    def get_format7_info(self):
        cdef fc2Error r
        cdef fc2Format7Info info
        cdef BOOL supported
        with nogil:
            r = fc2GetFormat7Info(self.ctx, &info, &supported)
        raise_error(r)
        return {"mode": info.mode,
                "max_width": info.maxWidth,
                "max_height": info.maxHeight,
                "offset_h_step_size": info.offsetHStepSize,
                "offset_v_step_size": info.offsetVStepSize,
                "image_h_step_size": info.imageHStepSize,
                "image_v_step_size": info.imageVStepSize,
                "pixel_format_bit_field": info.pixelFormatBitField,
                "vendor_pixel_format_bit_field": info.vendorPixelFormatBitField,
                "packet_size": info.packetSize,
                "min_packet_size": info.minPacketSize,
                "max_packet_size": info.maxPacketSize,
                "percentage": info.percentage,}, supported  
    
    
    def get_format7_configuration(self):
        cdef fc2Error r
        cdef fc2Format7ImageSettings s
        cdef unsigned packetSize
        cdef float percentage
        with nogil:
            r = fc2GetFormat7Configuration(self.ctx, &s, &packetSize, &percentage)
        raise_error(r)
        return {"mode": s.mode,
                "offset_x": s.offsetX,
                "offset_y": s.offsetY,
                "width": s.width,
                "height": s.height,
                "pixel_format": s.pixelFormat,}


    def set_format7_configuration(self, mode, offset_x, offset_y, width, height, pixel_format):
        cdef fc2Error r
        cdef fc2Format7ImageSettings s
        cdef float f = 100.0
        s.mode = mode
        s.offsetX = offset_x
        s.offsetY = offset_y
        s.width = width
        s.height = height
        s.pixelFormat = pixel_format
        with nogil:
            r = fc2SetFormat7Configuration(self.ctx, &s, f)
        raise_error(r)


    def get_configuration(self):
        cdef fc2Error r
        cdef fc2Config config
        with nogil:
            r = fc2GetConfiguration(self.ctx, &config)
        raise_error(r)        
        return {
            "num_buffers": config.numBuffers,
            "num_image_notifications": config.numImageNotifications,
            "min_num_image_notifications": config.minNumImageNotifications,
            "grab_timeout": config.grabTimeout,
            "grab_mode": config.grabMode,
            # "high_performance_retrieve_buffer": config.highPerformanceRetrieveBuffer,
            "isoch_bus_speed": config.isochBusSpeed,
            "async_bus_speed": config.asyncBusSpeed,
            "bandwidth_allocation": config.bandwidthAllocation,
            "register_timeout_retries": config.registerTimeoutRetries,
            "register_timeout": config.registerTimeout,
        }

    def set_configuration(self, num_buffers,
                          num_image_notifications, min_num_image_notifications,
                          grab_timeout, grab_mode, #high_performance_retrieve_buffer,
                          isoch_bus_speed, async_bus_speed,
                          bandwidth_allocation,
                          register_timeout_retries, register_timeout):
        cdef fc2Error r
        cdef fc2Config config
        config.numBuffers = num_buffers
        config.numImageNotifications = num_image_notifications
        config.minNumImageNotifications = min_num_image_notifications
        config.grabTimeout = grab_timeout
        config.grabMode = grab_mode
        # config.highPerformanceRetrieveBuffer = high_performance_retrieve_buffer
        config.isochBusSpeed = isoch_bus_speed
        config.asyncBusSpeed = async_bus_speed
        config.bandwidthAllocation = bandwidth_allocation
        config.registerTimeoutRetries = register_timeout_retries
        config.registerTimeout = register_timeout
        with nogil:
            r = fc2SetConfiguration(self.ctx, &config)
        raise_error(r)

    def set_cfg_value(self, key, value):
        cfg = self.get_configuration()
        cfg[key] = value
        self.set_configuration(**cfg)

cdef class Image:
    cdef fc2Image img
    cdef object fmt


    def __cinit__(self):
        cdef fc2Error r
        with nogil:
            r = fc2CreateImage(&self.img)
        raise_error(r)

    def __init__(self):
        self.fmt = None


    def __dealloc__(self):
        cdef fc2Error r
        with nogil:
            r = fc2DestroyImage(&self.img)
        raise_error(r)

    # NEW VERSION - TEST
    def __array__(self):
        cdef np.ndarray r
        cdef np.npy_intp shape[3]
        cdef np.npy_intp stride[3]
        cdef np.dtype dtype
        fmt = self.fmt or self.img.format
        ndim = 2
        if fmt == PIXEL_FORMAT_MONO8 or fmt == PIXEL_FORMAT_RAW8:
            dtype = np.dtype("uint8")
            stride[1] = 1
        elif fmt == PIXEL_FORMAT_MONO16 or fmt == PIXEL_FORMAT_RAW16:
            dtype = np.dtype("uint16")
            stride[1] = 2
        elif fmt == PIXEL_FORMAT_RGB8 or fmt == PIXEL_FORMAT_444YUV8:
            dtype = np.dtype("uint8")
            ndim = 3
            stride[1] = 3
            stride[2] = 1
            shape[2] = 3
        elif fmt == PIXEL_FORMAT_422YUV8:
            dtype = np.dtype("uint8")
            ndim = 3
            stride[1] = 2
            stride[2] = 1
            shape[2] = 2
        else:
            dtype = np.dtype("uint8")
            stride[1] = int(self.img.stride/self.img.cols)
        Py_INCREF(dtype)
        shape[0] = self.img.rows
        shape[1] = self.img.cols
        stride[0] = self.img.stride
        #assert stride[0] == stride[1]*shape[1]
        #assert shape[0]*shape[1]*stride[1] == self.img.dataSize
        r = PyArray_NewFromDescr(np.ndarray, dtype,
                ndim, shape, stride,
                self.img.pData, np.NPY_DEFAULT, None)
        r.base = <PyObject *>self
        Py_INCREF(self)
        return r


    def get_bits_per_pixel(self):
        cdef fc2Error r
        cdef fc2PixelFormat pixel_format
        cdef unsigned int bitsPerPixel

        pixel_format = self.img.format

        with nogil:
            r = fc2DetermineBitsPerPixel(pixel_format, &bitsPerPixel)
        raise_error(r)
        return bitsPerPixel


    def get_info(self):
         return {"stride": self.img.stride,
                "cols": self.img.cols,
                "rows": self.img.rows,
                "format": self.img.format,
                "bitsPerPixel": self.get_bits_per_pixel(),
                "dataSize": self.img.dataSize,
                "pData": self.img.pData}

  
    def convert(self, pf, Image dst=None):
        cdef fc2Error r
        cdef fc2PixelFormat _pf
        _pf = pf
        if dst is None:
            dst = Image()
        with nogil:
            r = fc2ConvertImageTo(_pf, &self.img, &dst.img)
        raise_error(r)
        return dst

    def timestamp(self):
        cdef fc2TimeStamp ts
        with nogil:
            ts = fc2GetImageTimeStamp(&self.img)
        return {"seconds": ts.seconds,
                "cycleCount": ts.cycleCount,
                "cycleSeconds": ts.cycleSeconds,
                "cycleOffset": ts.cycleOffset,
                "microSeconds": ts.microSeconds}


    def set_format(self, fmt):
        self.fmt = fmt


    def get_format(self):
        return self.fmt or self.img.format

