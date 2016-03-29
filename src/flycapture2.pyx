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

cdef class VideoWriter:
    cdef fc2AVIContext avi

    def __cinit__(self):
        cdef fc2Error r
        with nogil:
            r = fc2CreateAVI(&self.avi)
        raise_error(r)

    def __dealloc__(self):
        cdef fc2Error r
        with nogil:
            r = fc2DestroyAVI(self.avi)
        raise_error(r)

    def openAVI(self, char *fileName, unsigned int frameRate):
        cdef fc2Error r
        cdef fc2AVIOption AVIOption
        AVIOption.frameRate = frameRate
        with nogil:
            # r = fc2AVIOpen(self.avi, &fileName, &AVIOption)   
            r = fc2AVIOpen(self.avi, fileName, &AVIOption)   
        raise_error(r)
    
    def openH264(self, char *fileName, unsigned int frameRate):
        cdef fc2Error r
        cdef fc2H264Option H264Option
        H264Option.frameRate = frameRate
        H264Option.bitrate = 1000000
        H264Option.width = 1280
        H264Option.height = 960
        with nogil:
            r = fc2H264Open(self.avi, fileName, &H264Option)   
        raise_error(r)
    
    def openMJPG(self, char *fileName, unsigned int frameRate):
        cdef fc2Error r
        cdef fc2MJPGOption MJPGOption
        MJPGOption.frameRate = frameRate
        MJPGOption.quality = 75
        with nogil:
            r = fc2MJPGOpen(self.avi, fileName, &MJPGOption)   
        raise_error(r)
    
    def close(self):
        cdef fc2Error r
        with nogil:
            r = fc2AVIClose(self.avi)
        raise_error(r)

    # fc2Error fc2AVIAppend(fc2AVIContext AVIContext, fc2Image *pImage) nogil
    def append(self, Image img):
        cdef fc2Error r
        with nogil:
            r = fc2AVIAppend(self.avi, &img.img)
        raise_error(r)



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
        prop['auto_manual_mode'] = False
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

    def set_trigger_mode(self, on_off, polarity, source,
            mode, parameter):
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
    def get_embedded_image_info(self):
        cdef fc2Error r
        cdef fc2EmbeddedImageInfo pInfo
        
        # cdef fc2EmbeddedImageInfoProperty pInfoProperty
        print('test')
        with nogil:
            r = fc2GetEmbeddedImageInfo(self.ctx, &pInfo)
            # pInfo.timestamp( &pInfoProperty)
            # r = fc2GetEmbeddedImageTimeStamp(self.ctx)
            # r = fc2GetEmbeddedImageInfoProperty(self.ctx, &pInfoProperty)
        print(pInfo)
        return {"timeStamp": pInfo.timestamp,
                "brightness": pInfo.brightness}

    # # fc2Error fc2SetEmbeddedImageInfo(fc2Context context, fc2EmbeddedImageInfo *pInfo) nogil
    # # fc2Error fc2GetDefaultOutputFormat(fc2PixelFormat *pFormat) nogil
    
    # fc2Error fc2SetDefaultOutputFormat(fc2PixelFormat format) nogil
    def set_default_output_format(self, fc2PixelFormat pf):
        cdef fc2Error r
        with nogil:
            r = fc2SetDefaultOutputFormat(pf)
        raise_error(r)

cdef class Image:
    cdef fc2Image img
    
    def __cinit__(self):
        cdef fc2Error r
        with nogil:
            r = fc2CreateImage(&self.img)
        raise_error(r)

    def __dealloc__(self):
        cdef fc2Error r
        with nogil:
            r = fc2DestroyImage(&self.img)
        raise_error(r)

    def __array__(self):
        cdef np.ndarray r
        cdef np.npy_intp shape[2]
        cdef np.npy_intp stride[2]
        cdef np.dtype dtype

        shape[0] = self.img.rows
        shape[1] = self.img.cols
        stride[0] = self.img.stride
        
        if self.img.format == PIXEL_FORMAT_MONO8:
            dtype = np.dtype("uint8")
            stride[1] = 1
        elif self.img.format == PIXEL_FORMAT_MONO16:
            dtype = np.dtype("uint16")
            stride[1] = 2
        elif self.img.format == PIXEL_FORMAT_RGB:
            dtype = np.dtype("uint8")
            stride[1] = 1#self.img.stride/self.img.cols
            shape[1] = self.img.cols*3
        elif self.img.format == PIXEL_FORMAT_422YUV8:
            dtype = np.dtype("uint8")
            stride[1] = 1            
        else:
            dtype = np.dtype("uint8")
            stride[1] = self.img.stride/self.img.cols
        Py_INCREF(dtype)
        #assert stride[0] == stride[1]*shape[1]
        #assert shape[0]*shape[1]*stride[1] == self.img.dataSize
        r = PyArray_NewFromDescr(np.ndarray, dtype,
                2, shape, stride,
                self.img.pData, np.NPY_DEFAULT, None)
        r.base = <PyObject *>self
        Py_INCREF(self)
        return r

    def getInfo(self):
         return {"stride": self.img.stride,
                "cols": self.img.cols,
                "rows": self.img.rows,
                "format": self.img.format,
                "dataSize": self.img.dataSize,
                "pData": self.img.pData}

    def convertImage(self, fc2PixelFormat pf, Image img):
        cdef fc2Error r
        with nogil:
            r = fc2ConvertImageTo(pf, &self.img, &img.img)                                                
        raise_error(r)

    def getTimeStamp(self):
        cdef fc2TimeStamp ts
        with nogil:
            ts = fc2GetImageTimeStamp(&self.img)
        return {"seconds": ts.seconds,
                "cycleCount": ts.cycleCount,
                "cycleSeconds": ts.cycleSeconds,
                "cycleOffset": ts.cycleOffset,
                "microSeconds": ts.microSeconds}

