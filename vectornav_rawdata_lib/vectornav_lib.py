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
import traceback
import sys
import numpy as np
import struct
from vectornav_utils import *
import time


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

def sample_cb(data):
    #process / feed data into some other place
    print 'data',data[0]
    return True

class VnBinaryParser:
    """doc for VnBinaryParser"""
    def __init__(self, port, baudrate, timeout=0,callback_fn=sample_cb,callback_var_dict=''):
        try:
            self.serial_handler = serial.Serial(port, baudrate,timeout=timeout)
            print 'serial port connected'
            time.sleep(3)
            self.callback_fn = callback_fn
            self.callback_var_dict = callback_var_dict
        except Exception as init_ex:
            print(traceback.format_exc(init_ex))
            sys.exit()

    # helper functions
    def get_serial_shifted_data(self,length):
        while self.serial_handler.inWaiting() < length :
            continue
        data = self.serial_handler.read(length)
        data_shift = [data[-i] for i in range(1,len(data)+1)]
        data_out = ''.join(x_ for x_ in data_shift)
        return data_out

    def get_serial_data(self,length):
        while self.serial_handler.inWaiting() < length :
            continue
        data = self.serial_handler.read(length)
        return data

    def raw_2_binary(self,data_raw):
        binary_list =[format(bin(ord(x))[2:],'0>8') for x in data_raw]
        data_binary =''.join(x_ for x_ in binary_list)
        return data_binary

    def parse_dict_set_bor(self):
        """
        For Future purpose, implement setting/writing register for BOR 
        based on user requirments in self.callback_var_dict and read specific BOR to confirm again
        1. parse self.callback_var_dict to map Binary groups and fields
        2. set/write BOR register and serial port
        3. Read/verify BOR  
        """
        pass

    def run_driver(self):
        run_loop = True
        while(run_loop):
            try:
                if self.serial_handler.inWaiting() > 0 : 
                    data = self.serial_handler.read(1)
                    #print data
                    if ord(data)==_sync_byte:
                        print ord(data), ' sync bit received'
                        
                        # parse header
                        status, binary_groups_array, active_fields_array, fixed_length = self.parse_header()
                        if status is False:
                            # if header parsing fails
                            print 'skip at header'
                            continue
                        
                        if fixed_length is True:
                            # parse payload as required
                            status = self.fixed_payload_parser(binary_groups_array, active_fields_array)
                            if status is False:
                                # if payload_parser fails
                                print 'skip at payload'
                                continue
                        else:
                            # parse payload as required
                            status = self.variable_payload_parser(binary_groups_array, active_fields_array)
                            if status is False:
                                # if payload_parser fails
                                print 'skip at payload'
                                continue
                        # wait for 16 bit checksum
                        crc_raw = self.get_serial_shifted_data(2)
                        crc = struct.unpack('!H',crc_raw)[0]
                        #print('crc ', crc)

            except serial.serialutil.SerialException as serial_ex:
                run_loop = False
                print(traceback.format_exc(serial_ex))

            except Exception as run_ex:
                run_loop = False
                print(traceback.format_exc(run_ex))
        
    def parse_header(self):
        """ parser for header in binary msg to determine length of msg and groups present in the msg"""
        """ once sync_byte is recieved, parse_header is to be called. so here I assume that the you just received sync_byte and about to parse rest of header """

        # get groups
        while self.serial_handler.inWaiting() < 1 :
            continue
        groups = self.serial_handler.read(1)
        groups = format(bin(ord(groups))[2:],'0>8')
        # converting byte to bitmask in reverse order
        binary_groups = [bool(int(groups[-x])) for x in range(1,len(groups)+1)]

        if binary_groups[-1] is True:
            # bit 7 is set indicating presence of higher groups
            while (self.serial_handler.inWaiting()) < 1:
                continue
            higher_groups = self.serial_handler.read(1)
            higher_groups = format(bin(ord(higher_groups))[2:],'0>8')
            # converting byte to bitmask in reverse order
            higher_binary_groups = [bool(int(higher_groups[-x])) for x in range(1,len(higher_groups)+1)]
            binary_groups = binary_groups[0:-1]+higher_binary_groups
        # get active group count
        active_groups_count = binary_groups.count(True)
        #print active_groups_count, binary_groups
        
        active_fields = []
        # payload length is fixed unless set False, when variable data fields 15,16 are set in group 4 or 7
        fixed_length = True
        
        for i in range(active_groups_count):
            fields = self.get_serial_shifted_data(length=2)
            fields = self.raw_2_binary(fields)
            # converting byte to bitmask in reverse order
            fields = [bool(int(fields[-x])) for x in range(1,len(fields)+1)]
            if fields[-1] is True:
                # bit 15 is set, indicating fields 16-30 are set 
                higher_fields = self.get_serial_shifted_data(length=2)
                higher_fields = self.raw_2_binary(higher_fields)
                # converting byte to bitmask in reverse order
                higher_fields = [bool(int(higher_fields[-x])) for x in range(1,len(higher_fields)+1)]
                fields = fields[0:-1]+higher_fields
            
            active_fields.append(fields)
            # only group 1, fields 14 has fixed length payload
            if fields[14] is True and i != 0:
                fixed_length = False
            # field 15 is raw_measurement set in gnss groups 4,7 is variable packet.    
            if fields[15] is True:
                fixed_length = False

        status = True
        return status, binary_groups, active_fields, fixed_length  

    def variable_payload_parser(self,binary_groups_array,active_fields_array):
        # create new data_frame upon sync_byte and delete at end of callback
        data_frame = []
        active_idx = np.where(binary_groups_array)[0]
        for idx, i in enumerate(active_idx):
            if i == 3:
                status = self.parser_binary_group_4(active_fields_array[idx])
            elif i == 6:
                status = self.parser_binary_group_7(active_fields_array[idx])
            else:
                print('parser for group ',i,' not supported as of now')
                status = False
            
            # once in a while, the active groups parsing in header is messed up, probably due to sync byte getting repeated in one of the data and starting at a wrong place.
            # if parsing of even one group is messed up, ill negelct all data in the current payload
            # so i'll skip this data completely until next sync_byte
            if status is False:
                del data_frame
                return False
        # load data into data_frame based on self.callback_var_dict
        data_frame.append(1)
        # send data to callback here at end of parsing of all groups
        a = self.callback_fn(data_frame)
        # release data_frame at end of cb
        del data_frame
        return True 

    def fixed_payload_parser(self,binary_groups_array,active_fields_array):
        """
        For future purpose
        if only fixed data types are present in header,
        buffer to required length and parse at one go
        we know the size from _binarygroup_datasize,
        if we have _binarygroup_struct_datatype then we can create a string from activefiled_types and total buffer length and struct.unpack at one go and map them later for faster processing. 
        """
        pass     

    def parser_binary_group_4(self,active_fields_group4):
        # gnss 1 receiver
        return self.parser_gnss_group(group_idx = 4, active_fields_group = active_fields_group4)

    def parser_binary_group_7(self,active_fields_group7):
    # gnss 2 receiver
        return self.parser_gnss_group(group_idx=7,active_fields_group =  active_fields_group7)

    def parser_gnss_group(self,group_idx,active_fields_group):   
        active_fields_idx = np.where(active_fields_group)[0]
        
        for i in active_fields_idx:
            # decide buffer size for specific field
            if i <14:
                # group_idx=3+1, field_idx=i
                mem_size = _binarygroup_datasize[group_idx-1][i]
            
            elif i ==14:
                # satinfo
                # first byte reveals numOf Sats
                num_sats_raw = self.get_serial_shifted_data(1)
                num_sats = struct.unpack('!B',num_sats_raw)[0]
                #print 'sats ',num_sats
                # user manual: 2+(num_sats* 8)
                # we are read first 1 bytes above to determine num of sats
                # so it is 1 +( num_sats* 8)
                mem_size = 1+(num_sats*8)

            elif i==15:
                #raw measurement
                # first set of 8 bytes[0-7] are for timeofweek(Tow)
                # tow in raw_measurement is double
                # tow in gnss field is uint64
                tow_raw = self.get_serial_shifted_data(8)
                tow = struct.unpack('!d',tow_raw)[0]
                #print 'tow',tow

                # second set of 2 bytes[8-9] are for week
                week_raw = self.get_serial_shifted_data(2)
                week = struct.unpack('!H',week_raw)[0]
                #print 'week',week

                # third set of 1 byte   [10] is for numofsats
                num_sats_raw = self.get_serial_shifted_data(1)
                num_sats = struct.unpack('!B',num_sats_raw)[0]
                #print 'sats ',num_sats

                # user manual: 12+(num_sats* 28)
                # we are read first 11bytes above to determine num of sats
                # so it is 1 +(num_sats* 28)
                mem_size = 1+(num_sats*28)
            
            elif i>15:
                print 'un supported field recieved'
                # skipping the complete gnss data frame and in payload parser it will skip till next sync_byte
                return False

            # get data for sepcific field
            data_field_i_raw = self.get_serial_data(mem_size)
            extract_field = gnss_group_dict.get(i, "invalid_sequence")
            #if function_ == "invalid_sequence":
            #    print('Invalid Sequence Requested')
            data = extract_field(data_field_i_raw)
            #publish the data
        return True

if __name__ == '__main__':
    port = sys.argv[1]
    baudrate = sys.argv[2]
    #VnBinaryParser(self, port, baudrate, timeout=0,callback_fn,callback_var_dict='')
    callback_var_dict = ''
    vnparser_obj = VnBinaryParser(port, baudrate)
    vnparser_obj.run_driver()