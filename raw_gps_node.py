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

import serial
import time
import rospy
import traceback
import sys
import numpy as np

import binascii
from helper_utils import *

import struct

_sync_byte = 0xFA
_binarygroup_datasize = np.array(np.mat(\
    '8 8 2 8 2 2 8 0; \
    8 8 12 8 12 24 8 0; \
    8 8 12 2 16 24 2 0; \
    12 2 12 1 36 12 1 0; \
    16 8 4 1 12 12 1 0; \
    12 8 4 24 12 12 24 0; \
    24 8 16 24 12 12 24 0; \
    12 4 12 12 12 12 12 0; \
    12 4 12 12 12 12 12 0; \
    24 1 12 12 0 4 12 0; \
    20 0 12 4 0 4 4 0; \
    28 0 0 4 0 0 4 0; \
    2 0 0 2 0 0 2 0; \
    4 0 0 28 0 0 28 0; \
    8 0 0 0 0 0 0 0; \
    0 0 0 0 0 0 0 0')).transpose()

class VnBinaryParser:
    """docstring for VnBinaryParser"""
    def __init__(self, port, baudrate, timeout):
        try:
            self.serial_handler = serial.Serial(port, baudrate, timeout=1/200)
            print 'serial port connected'
            time.sleep(3)
        except Exception as init_ex:
            print(traceback.format_exc(init_ex))
            sys.exit()

    def parse_header(self):
        """ parser for header in binary msg to determine length of msg and groups present in the msg"""
        """ once sync_byte is recieved, parse_header is to be called. so here I assume that the you just received sync_byte and about to parse rest of header """

        binary_groups=[False,False,False,False,False,False,False,False]
        # get groups
        while (self.serial_handler.inWaiting()) <= 0 :
            continue
        groups = self.serial_handler.read(1)
        groups = format(bin(ord(groups))[2:],'0>8')
        for i,j in enumerate(groups):
            if j == '1' :
                binary_groups[7-i]=True
                if i == 0:
                    print("binary_groups 8 - 16 seems to be set, not yet supported in vecnav lib 1.14 as of june 2019 ")
                    sys.exit()
        #print binary_groups
        active_groups_count = binary_groups.count(True)
        active_fields = np.zeros([active_groups_count,16], dtype = bool) 
        # payload length is fixed unless set flase when variable data fields 15,16 are set in group 4 or 7
        fixed_length = True
        
        for i in range(active_groups_count):
            while (self.serial_handler.inWaiting()) < 2 :
                continue
            fields = self.serial_handler.read(2)
            fields = self.hex_2_binary(fields)
            for j,k in enumerate(fields):
                if k == '1':
                    active_fields[i,15-j]= True

                    # determine if payload is fixed or variable
                    if j == 0 :
                        # field 16 for bit offset 15
                        fixed_length = False
                    elif j == 1:
                        # bit offset 14 for fields 15 is active
                        # only group 1, fields 15 has fixed length payload
                        # group 4 and group 7 have variable length payload
                        if binary_groups[3] is True or binary_groups[6] is True:
                            fixed_length = False
        return binary_groups, active_fields, fixed_length
    
    def hex_2_binary(self,data_hex,length=16):
        data_bin =''
        for i in range(0,len(data_hex)):
            data_string = format(bin(ord(data_hex[i]))[2:],'0>8')
            data_bin = data_string + data_bin
        return data_bin

    def binary_2_ascii(self,data_binary):
        data_ascii = data_binary.decode("hex")
        return data_ascii
    def payload_parser(self,binary_groups_array,active_fields_array):
        #active_fields_idx_x, active_fields_idx_y = np.where(active_fields_array)
        #print('binary_groups_array', binary_groups_array)
        #print('active_fields_array', active_fields_array)
        active_idx = np.where(binary_groups_array)[0]
        for idx, i in enumerate(active_idx):
            if i == 3:
                self.parser_binary_group_4(active_fields_array[idx])
            elif i == 6:
                self.parser_binary_group_7(active_fields_array[idx])
            else:
                print('parser for group ',i,' not supported as of now')


    def parser_binary_group_4(self,active_fields_group4):
        # gnss 1 receiver
        self.parser_gnss_group(group_idx = 4, active_fields_group = active_fields_group4)
        
    def parser_binary_group_7(self,active_fields_group7):
        self.parser_gnss_group(group_idx=7,active_fields_group =  active_fields_group7)


    def parser_gnss_group(self,group_idx,active_fields_group):   
        active_fields_idx = np.where(active_fields_group)[0]
        
        for i in active_fields_idx:
            # decide buffer size for specific field
            if i <14:
                # group_idx=3+1, field_idx=i
                mem_size = _binarygroup_datasize[group_idx-1][i]
            elif i ==14:
                # first byte reveals numOf Sats
                while (self.serial_handler.inWaiting()) < 1 :
                    continue
                num_sats_raw = self.serial_handler.read(1)
                num_sats_hex = [binascii.hexlify(x) for x in num_sats_raw]
                num_sats = int(num_sats_hex[0],16)
                # not 2+(num_sats* 8) as in usermanual as we are reading 10 byte above to determine num of sats
                if num_sats > 0:
                    mem_size = 1+(num_sats*8)
                else:
                    mem_size = 1
            elif i==15:
                # first set of 8 bytes[0-7] are for timeofweek(Tow)
                while (self.serial_handler.inWaiting()) < 8 :
                    continue
                tow_raw = self.serial_handler.read(8)
                tow_hex = [binascii.hexlify(x) for x in tow_raw]
                print tow_hex
                # second set of 2 bytes[8-9] are for week
                while (self.serial_handler.inWaiting()) < 2 :
                    continue
                weeks_raw = self.serial_handler.read(2)
                weeks_hex = [binascii.hexlify(x) for x in weeks_raw]
                print weeks_hex
                # third set of 1 byte   [10] is for numofsats
                while (self.serial_handler.inWaiting()) < 1 :
                    continue
                num_sats_raw = self.serial_handler.read(1)
                num_sats_hex = [binascii.hexlify(x) for x in num_sats_raw]
                num_sats = int(num_sats_hex[0],16) 
                # not 12+(num_sats* 28) as in usermanual as we are reading 11bytes    above to determine num of sats
                if num_sats > 0:
                    mem_size = 1+(num_sats*28)
                else:
                    mem_size = 1

            # get data for sepcific field
            while (self.serial_handler.inWaiting()) < mem_size :
                continue
            data_field_i_raw = self.serial_handler.read(mem_size)
            data_field_i_hex = [binascii.hexlify(x) for x in data_field_i_raw]
            #data_field_i_bin = self.hex_2_binary(data_field_i_hex)
            #data_field_i     = self.binary_2_ascii(data_field_i_bin)
            #print 'sats',num_sats,len(data_field_i_hex)-1,data_field_i_hex

            extract_field = gnss_group_dict.get(i, "invalid_sequence")
            #if function_ == "invalid_sequence":
            #    print('Invalid Sequence Requested')
            data = extract_field(data_field_i_hex)
            #publish the data




    def run(self):
        try:
            while(1):
                if (self.serial_handler.inWaiting()) > 0 : 
                    data = self.serial_handler.read(1)
                    #print data
                    if ord(data)==_sync_byte:
                        print ord(data), ' sync bit received'
                        binary_groups_array, active_fields_array, fixed_length = self.parse_header()
                        # parse payload as required
                        self.payload_parser(binary_groups_array, active_fields_array)
                        # wait for 16 bit checksum
                        while (self.serial_handler.inWaiting()) < 2 :
                            continue
                        # not parsing crc as I dont require it, 
                        # if you need, uncomment below lines
                        crc_hex = self.serial_handler.read(2)
                        crc_bin = self.hex_2_binary(crc_hex)
                        #print('crc ',[binascii.hexlify(x) for x in crc_hex],   crc_bin)
        except Exception as run_ex:
            print(traceback.format_exc(run_ex))
        
if __name__ == '__main__':
    port = sys.argv[1]
    baudrate = sys.argv[2]
    timeout = float(1/200.0)
    vnparser_obj = VnBinaryParser(port, baudrate, timeout)
    vnparser_obj.run()