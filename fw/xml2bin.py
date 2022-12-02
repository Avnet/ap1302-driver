#!/usr/bin/python3
# SPDX-License-Identifier: GPL-2.0-or-later
# Copyright 2020 Laurent Pinchart <laurent.pinchart@ideasonboard.com>

import binascii
import struct
import sys
from xml.parsers import expat


class Parser(object):

    def __init__(self):
        self.__parser = expat.ParserCreate()
        self.__parser.StartElementHandler = self.__startElement
        self.__parser.EndElementHandler = self.__endElement
        self.__parser.CharacterDataHandler = self.__characterData

        self.__in_release = False
        self.__in_fwnotes = False
        self.__in_date = False
        self.__in_firmware = False
        self.__attrs = {}
        self.__data = ''
        self.__release = ''
        self.__date = ''
        self.__fwnotes = ''

    def parse(self, filename):
        f = open(filename, 'rb')
        self.__parser.ParseFile(f)

    def data(self):
        return self.__data

    def date(self):
        return self.__date

    def release(self):
        return self.__release

    def fwnotes(self):
        return self.__fwnotes


    def __getitem__(self, name):
        return self.__attrs[name]

    def __startElement(self, name, attrs):
        if name == 'dump':
            self.__in_firmware = True
            self.__attrs = attrs
        if name == 'release':
            self.__in_release = True
        if name == 'fwnotes':
            self.__in_fwnotes = True
        if name == 'date':
            self.__in_date = True

    def __endElement(self, name):
        if name == 'dump':
            self.__in_firmware = False
        if name == 'release':
            self.__in_release = False
        if name == 'fwnotes':
            self.__in_fwnotes = False
        if name == 'date':
            self.__in_date = False

    def __characterData(self, data):
        if self.__in_firmware:
            self.__data += data
        if self.__in_release:
            self.__release += data
        if self.__in_fwnotes:
            self.__fwnotes += data
        if self.__in_date:
            self.__date += data


def main(argv):
    if len(argv) != 3:
        print(f'Usage: {argv[0]} input.xml output.bin')
        return 1

    parser = Parser()
    parser.parse(argv[1])

    fw = parser.data()
    fw = ''.join([c for c in fw if c in '0123456789abcdefABCDEF'])
    pll_init_size = int(parser['pll_init_size'])
    crc = int(parser['crc'], 16)

    #
    # Header
    # 32-Bit Magic : represent AP13
    # 32-Bit Version :
    # Description : 256-byte for an ASCII string
    # 16-Bit pll size
    # 16-Bit CRC

    desc = "Release %s @ %s - %s" % (parser.release().strip() ,
                                    parser.date().strip(),
                                    parser.fwnotes().strip())

    print(desc)

    header = struct.pack('<4cL256sHH',
        b'A',b'P',b'1',b'3',
        1,
        bytes(desc,'ascii'),
        pll_init_size, crc)

    out = open(argv[2], 'wb')
    out.write(header)
    out.write(binascii.a2b_hex(fw))


if __name__ == '__main__':
    sys.exit(main(sys.argv))
