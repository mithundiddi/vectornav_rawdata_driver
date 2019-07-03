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

import sys
from scipy.io import savemat
from vectornav_rawdata_lib.vectornav_lib import VnBinaryParser

def test_cb(data):
    print 'test',data[0]
    return True

if __name__ == '__main__':
    port = sys.argv[1]
    baudrate = sys.argv[2]
    #VnBinaryParser(self, port, baudrate, timeout=0,callback_fn,callback_var_dict='')
    callback_var_dict = ''
    vnparser_obj = VnBinaryParser(port=port, baudrate=baudrate, callback_fn = test_cb)
    vnparser_obj.run_driver()