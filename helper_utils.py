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
import binascii

def extract_UTC(data):
    return data
def extract_Tow(data):
    return data
def extract_Week(data):
    return data
def extract_NumSats(data):
    return data
def extract_Fix(data):
    return data
def extract_PosLla(data):
    return data
def extract_PosEcef(data):
    return data
def extract_VelNed(data):
    return data
def extract_VelEcef(data):
    return data
def extract_PosU(data):
    return data
def extract_VelU(data):
    return data
def extract_TimeU(data):
    return data
def extract_TimeInfo(data):
    return data
def extract_DOP(data):
    return data
def extract_SatInfo(data):
    # length = 1+ 8*(num_of_sats)
    if len(data) == 1:
        #if there are no visible sats, return
        return

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
        # sys is uint8 as mentioned in raw_measurement.(not s8 as in satinfo doc of user manual)
        sys = int(data[8*i + 0],16)
        sys_array.append(sys)

        # svld is uint8
        svld = int(data[8*i + 1],16)
        svld_array.append(svld)

        # flags is uint8 but it is bitmask actually
        flags = int(data[8*i + 2],16)
        flags_mask = format(bin(flags)[2:],'0>8')
        flags_array.append(flags)
        flags_mask_array.append(flags_mask)
        
        # cno is uint8
        cno = int(data[8*i + 3],16)
        cno_array.append(cno)

        # qi is uint8
        qi = int(data[8*i + 4],16)
        qi_array.append(qi)

        # el is s8 # is it float8 or unit8?
        el = int(data[8*i + 5],16)
        el_array.append(el)

        # az is s16 # could be a float16 or uint16?
        az = int('{0}{1}'.format(data[8*i + 7],data[8*i + 6]),16)
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
        return
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
        sys = int(data[28*i + 0],16)
        sys_array.append(sys)

        # svld is uint8
        svld = int(data[28*i + 1],16)
        svld_array.append(svld)
        #freq is uint8
        freq = int(data[28*i + 2],16)
        freq_array.append(freq)

        #chan is uint8
        chan = int(data[28*i + 3],16)
        chan_array.append(chan)

        #slot is s8
        slot = data[28*i + 4].decode("hex")
        print slot
        #slot_array.append(slot)

        # cno is uint8
        cno = int(data[28*i + 5],16)
        cno_array.append(cno)

        # flags is uint16, but it is bitmask actually
        flags = int('{0}{1}'.format(data[28*i + 7],data[28*i + 6]),16)
        flags_mask = format(bin(flags)[2:],'0>16')
        flags_array.append(flags)
        flags_mask_array.append(flags_mask)

        #pr is double 64 bit/8 bytes
        data_pr = data[28*i+8:28+i +15]
        data_pr = [data_pr[-i] for i in range(1,len(data_pr)+1)]
        hex_string_pr = ''.join(x_ for x_ in data_pr)
        pr = struct.unpack('!d',hex_string_pr.decode("hex"))
        pr_array.append(pr)

        #cp is double 64 bit/8 bytes
        data_cp = data[28*i+16:28+i +23]
        data_cp = [data_pr[-i] for i in range(1,len(data_cp)+1)]
        hex_string_cp = ''.join(x_ for x_ in data_cp)
        cp = struct.unpack('!d',hex_string_cp.decode("hex"))
        cp_array.append(cp)

        #dp is double 32 bit/4 bytes
        data_dp = data[28*i+24:28+i +27]
        data_dp = [data_dp[-i] for i in range(1,len(data_dp)+1)]
        hex_string_dp = ''.join(x_ for x_ in data_dp)
        dp = struct.unpack('!f',hex_string_dp.decode("hex"))
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