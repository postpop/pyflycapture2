#!/usr/bin/python
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

import flycapture2 as fc2
import numpy as np

print(fc2.get_library_version())
c = fc2.Context()
print(c.get_num_of_cameras())
c.connect(*c.get_camera_from_index(0))
print(c.get_camera_info())
c.set_video_mode_and_frame_rate(fc2.VIDEOMODE_1280x960Y16, fc2.FRAMERATE_60)
m, f = c.get_video_mode_and_frame_rate()
print(m, f)
print(c.get_video_mode_and_frame_rate_info(m, f))

print(c.set_property_abs_value(fc2.SHUTTER, 20.0))

c.start_capture()
im = fc2.Image()
print([np.array(c.retrieve_buffer(im)).sum() for i in range(80)])
a = np.array(im)
print(a.shape, a.base)
i= c.get_embedded_image_info();
print(i)

cfg = c.get_configuration()
print(cfg)
cfg['num_buffers'] = 100
cfg = c.set_configuration(**cfg)
cfg = c.get_configuration()
print(cfg)

c.set_cfg_value('num_buffers', 21)
print(c.get_configuration())


c.stop_capture()
c.disconnect()
