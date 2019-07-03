'''
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 * MIT License                                                             *
 *                                                                         *
 * @authors     Mithun Diddi <diddi.m@husky.neu.edu>                       *
 * @maintainer  Mithun Diddi <diddi.m@husky.neu.edu>                       *
 * @website   https://web.northeastern.edu/fieldrobotics                   *
 * @copyright (c) 2019, Northeastern University Field Robotics Lab(NEUFRL),*
 *             All rights reserved.                                        *
 *                                                                         *
 * Permission is hereby granted, free of charge, to any person obtaining   *
 * copy of this software and associated documentation files                *
 * (the "Software"), to deal in the Software without restriction,          *
 * including *without limitation the rights to use, copy, modify, merge,   *
 * publish, distribute, sublicense, and/or sell copies of the Software,    *
 * and to permit persons to whom the Software is furnished to do so,       *
 * subject to the following conditions:                                    *
 *                                                                         *
 * The above copyright notice and this permission notice shall be included *
 * in all copies or substantial portions of the Software.                  *
 *                                                                         *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS *
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF              *
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  *
 *                                                                         *
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR        *
 * ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF          *
 * CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH *
 * THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.              *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
'''

#! /usr/bin/env python
import numpy as np
import struct

def extract_type_int(data,struct_type):
    # here struct_type mentions data_type formatter required
    # example s8 struct_type = b
    # u8/uint8 struct_type = B refer this link
    # https://docs.python.org/2/library/struct.html    
    data_int = [data[-i] for i in range(1,len(data)+1)]
    data_string = ''.join(x_ for x_ in data_int)
    return struct.unpack('!'+struct_type,data_string)[0]

def extract_type_float(data):
    data_float = [data[-i] for i in range(1,len(data)+1)]
    data_string = ''.join(x_ for x_ in data_float)
    return struct.unpack('!f',data_string)[0]

def extract_type_double(data):
    data_double = [data[-i] for i in range(1,len(data)+1)]
    data_string = ''.join(x_ for x_ in data_double)
    return struct.unpack('!d',data_string)[0]

def extract_vectorN_float(data):
    data_out = []
    for i in range(len(data)):
        data_out.append(extract_type_float(data[4*i:4*i+4]))
    return data_out

def extract_vectorN_double(data):
    data_out = []
    for i in range(len(data)):
        data_out.append(extract_type_double(data[8*i:8*i+8]))
    return data_out

def extract_UTC(data):
    # year offset from 2000, 2013 is formatted as 13
    year = struct.unpack('!b',data[0])[0]
    month = struct.unpack('!B',data[1])[0]
    day = struct.unpack('!B',data[2])[0]
    hour = struct.unpack('!B',data[3])[0]
    minute = struct.unpack('!B',data[4])[0]
    sec = struct.unpack('!B',data[5])[0]
    millisec = struct.unpack('!H','{0}{1}'.format(data[7],data[6]))[0]

    #epoch_utc = ((30+year)*31,556,952)+(month*2592000)+(day*86400)+(hour*3600)+(minute*60)+sec+(millisec/1000.0)
    # write utc_struct and return it if you need
    return [year,month,day,hour,minute,sec,millisec]

def extract_Tow(data):
    ## in nano seconds  
    data_tow = [data[-i] for i in range(1,len(data)+1)]
    tow_string = ''.join(x_ for x_ in data_tow)
    #print struct.unpack('!Q',tow_string)[0]
    return struct.unpack('!Q',tow_string)[0]

def extract_Week(data):
    return struct.unpack('!H','{0}{1}'.format(data[1],data[0]))[0]

def extract_NumSats(data):
    return struct.unpack('!B',data[0])[0]

def extract_Fix(data):
    return struct.unpack('!B',data[0])[0]

def extract_PosLla(data):
    return extract_vec3_double(data)

def extract_PosEcef(data):
    return extract_vec3_double(data)

def extract_VelNed(data):
    return extract_vectorN_float(data)

def extract_VelEcef(data):
    return extract_vectorN_float(data)

def extract_PosU(data):
    return extract_vectorN_float(data)

def extract_VelU(data):
    return extract_type_float(data)

def extract_TimeU(data):
    return extract_type_float(data)

def extract_TimeInfo(data):
    status = struct.unpack('!B',data[0])[0]
    status_mask = format(bin(status)[2:],'0>8')
    leap_seconds = struct.unpack('!b',data[1])[0]
    return [status,status_mask,leap_seconds]

def extract_DOP(data):
    return extract_vectorN_float(data)

def extract_SatInfo(data):
    # length = 1+ 8*(num_of_sats)
    if len(data) == 1:
        #if there are no visible sats, return
        print 'no sats'
        return ''
        # neglect the first reserved one byte
    data = data[1:]

    sys_array   = [] # GNSS constellation indicator 
    svld_array  = [] # Space vehicle Id
    flags_array = [] # Tracking info flags #8bit flags
    flags_mask_array = [] # bitmask based on uint8 of flags
    cno_array   = [] # Carrier-to-noise density ratio (signal strength) [dB-Hz]
    qi_array    = [] # Quality Indicator
    el_array    = [] # Elevation in degrees
    az_array    = [] # Azimuth angle in degrees

    for i in range(len(data)/8):
        # sys is s8
        sys = struct.unpack('!b',data[8*i + 0])[0]
        sys_array.append(sys)

        # svld is uint8
        svld = struct.unpack('!B',data[8*i + 1])[0]
        svld_array.append(svld)

        # flags is uint8 but it is bitmask actually
        flags = struct.unpack('!B',data[8*i + 2])[0]
        flags_mask = format(bin(flags)[2:],'0>8')
        flags_array.append(flags)
        flags_mask_array.append(flags_mask)
        
        # cno is uint8
        cno = struct.unpack('!B',data[8*i + 3])[0]
        cno_array.append(cno)

        # qi is uint8
        qi = struct.unpack('!B',data[8*i + 4])[0]
        qi_array.append(qi)

        # el is s8
        el = struct.unpack('!b',data[8*i + 5])[0]
        el_array.append(el)

        # az is s16
        az =struct.unpack('!h','{0}{1}'.format(data[8*i + 7],data[8*i + 6]))[0]
        az_array.append(az)
    # print 'num_of_sats ', len(data)/8
    # print sys_array
    # print svld_array
    # print flags_array
    # print flags_mask_array
    # print cno_array
    # print qi_array
    # print el_array
    # print az_array
    return sys_array,svld_array,flags_array,flags_mask_array,cno_array,qi_array,el_array,az_array

def extract_RawMeas(data):

    # length = 1+ 28*(num_of_sats)
    if len(data) == 1:
        #if there are no visible sats, return
        print 'no sats'
        return ''
    # neglect the first reserved one byte
    data = data[1:]

    sys_array   = [] # GNSS constellation indicator 
    svld_array  = [] # Space vehicle Id
    freq_array  = [] # Frequency indicator
    chan_array  = [] # Channel Indicator
    slot_array  = [] # Slot Id
    cno_array   = [] # Carrier-to-noise density ratio (signal strength) [dB-Hz]
    flags_array = [] # Tracking info flags #16 bit flag
    flags_mask_array = [] # bitmask based on uint8 of flags
    pr_array    = [] # Pseudorange measurement in meters
    cp_array    = [] # Carrier phase measurement in cycles
    dp_array    = [] # Doppler measurement in Hz

    for i in range(len(data)/28):
        # sys is uint8 as mentioned in raw_measurement.(not s8 as in satinfo doc of user manual)
        sys = struct.unpack('!B',data[28*i + 0])[0]
        sys_array.append(sys)

        # svld is uint8
        svld = struct.unpack('!B',data[28*i + 1])[0]
        svld_array.append(svld)

        #freq is uint8
        freq = struct.unpack('!B',data[28*i + 2])[0]
        freq_array.append(freq)

        #chan is uint8
        chan = struct.unpack('!B',data[28*i + 3])[0]
        chan_array.append(chan)

        #slot is s8
        slot = struct.unpack('!b',data[28*i + 4])[0]
        slot_array.append(slot)

        # cno is uint8
        cno = struct.unpack('!B',data[28*i + 5])[0]
        cno_array.append(cno)

        # flags is uint16, but it is bitmask actually
        flags = struct.unpack('!H','{0}{1}'.format(data[28*i + 7],data[28*i + 6]))[0]
        flags_mask = format(bin(flags)[2:],'0>16')
        flags_array.append(flags)
        flags_mask_array.append(flags_mask)

        #pr is double 64 bit/8 bytes
        pr = extract_type_double(data[28*i+8:28*i +16])
        pr_array.append(pr)

        #cp is double 64 bit/8 bytes
        cp = extract_type_double(data[28*i+16:28*i +24])
        cp_array.append(cp)

        #dp is float 32 bit/4 bytes
        dp = extract_type_float(data[28*i+24:28*i +28])
        dp_array.append(dp)

    # print 'num_of_sats ', len(data)/28
    # print sys_array
    # print svld_array
    # print freq_array
    # print chan_array
    # print slot_array
    # print cno_array
    # print flags_array
    # print flags_mask_array
    # print pr_array
    # print cp_array
    # print dp_array
    #print 'data'
    return sys_array, svld_array, freq_array, chan_array, slot_array, cno_array, flags_array, flags_mask_array, pr_array, cp_array, dp_array

gnss_group_dict = {    
0: extract_UTC,
1: extract_Tow,
2: extract_Week,
3: extract_NumSats,
4: extract_Fix,
5: extract_PosLla,
6: extract_PosEcef,
7: extract_VelNed,
8: extract_VelEcef,
9: extract_PosU,
10: extract_VelU,
11: extract_TimeU,
12: extract_TimeInfo,
13: extract_DOP,
14: extract_SatInfo,
15: extract_RawMeas
}

def extract_quaternion(data):
    return extract_vectorN_float(data)

def extract_Accel(data):
    return extract_vectorN_float(data)

def extract_AngularRate(data):
    return extract_vectorN_float(data)

def extract_Imu(data):
    return extract_vectorN_float(data)

def extract_Temp(data):
    return extract_vectorN_float(data)

def extract_Pres(data):
    return extract_vectorN_float(data)
