#! /usr/bin/env python
# Copyright (C) 2015 Marvell International Ltd.
# All Rights Reserved.

# Load application to ram helper script
# Note: sys.stdout.flush() and sys.stderr.flush() are required for proper
# console output in eclipse

import os, sys, re, struct, platform, getopt, subprocess, hashlib
from sys import platform as _platform

def CRC16(bytes):
    wcrc = 0
    byte_array = bytearray(bytes)
    for i in byte_array:
        c = i
        for j in range(8):
            treat = c & 0x80
            c <<= 1
            bcrc = (wcrc >> 8) & 0x80
            wcrc <<= 1
            wcrc = wcrc & 0xffff
            if (treat != bcrc):
                wcrc ^= 0x1021

#print "wcrc=%04x"%(wcrc)
    return wcrc

def MD5(bytes):
    md5 = hashlib.md5()
    md5.update(bytes)
    return md5.hexdigest().decode('hex')

def print_usage():
    print ""
    print "Usage: Generate MOC user APP file and OTA file"
    print sys.argv[0]
    print "Optional Usage:"
    print " [<-u> <moc app binary file>]"
    print "          MOC application binary file generated by compiler."
    print " [<-d> <moc app binary file offset>]"
    print "          MOC application binary offset at ota output file."
    print " [<-k> <moc kernel binary file>]"
    print "          MOC kernel binary file provided by MXCHIP Inc."
    print " [-h | --help]"
    print "          Display usage"
    sys.stdout.flush()

def main():
    try:
        opts, args = getopt.getopt(sys.argv[1:], "u:k:d:h")
    except getopt.GetoptError as err:
        print str(err)
        print_usage()
        sys.exit(2)

    MOC_APP_OFFSET = 0

    for opt, arg in opts:
        if opt == "-u" :
            MOC_APP_FILE = arg
        elif opt == "-k" :
            MOC_KERNEL_FILE = arg
        elif opt == "-d" :
            MOC_APP_OFFSET = int(arg, 16)
        elif opt == "-h":
            print_usage()
            sys.exit()

    if not os.path.exists(MOC_APP_FILE):
        print "MOC application binary file not found"
        sys.exit()

    if not os.path.exists(MOC_KERNEL_FILE):
        print "MOC kernel binary file not found"
        sys.exit()

    if MOC_APP_OFFSET == 0:
        print "MOC application offset not set"
        sys.exit()

    MOC_APP_OUTPUT_FILE = re.sub(r'.bin$', '.usr.bin', MOC_APP_FILE)
    MOC_OTA_OUTPUT_FILE = re.sub(r'.bin$', '.ota.bin', MOC_APP_FILE)

#print "MOC APP BIN: " + MOC_APP_FILE
#print "MOC KERNEL BIN: " + MOC_KERNEL_FILE
#print "MOC APP OUTPUT BIN: " + MOC_APP_OUTPUT_FILE
#print "MOC OTA OUTPUT BIN: " + MOC_OTA_OUTPUT_FILE

    if os.path.exists(MOC_APP_OUTPUT_FILE):
        os.remove(MOC_APP_OUTPUT_FILE)
    if os.path.exists(MOC_OTA_OUTPUT_FILE):
        os.remove(MOC_OTA_OUTPUT_FILE)

    moc_app = open(MOC_APP_FILE, 'rb')
    moc_kernel = open(MOC_KERNEL_FILE, 'rb')
    moc_app_output = open(MOC_APP_OUTPUT_FILE, 'wb')
    moc_ota_output = open(MOC_OTA_OUTPUT_FILE, 'wb')

    app = moc_app.read()
    kernel = moc_kernel.read()
    
    crc = CRC16( bytes(app[8::]) )

    moc_app_output.write( struct.pack('<L', os.path.getsize(MOC_APP_FILE)-8 ) )
    moc_app_output.write( struct.pack('<HH', crc, crc ) )
    moc_app_output.write( bytes(app[8::]) )

    moc_ota_output.write( bytes(kernel) )
    moc_ota_output.write( '\xFF'*(MOC_APP_OFFSET-len(kernel)) )
    moc_ota_output.seek(MOC_APP_OFFSET)
    moc_ota_output.write( struct.pack('<L', os.path.getsize(MOC_APP_FILE)-8 ) )
    moc_ota_output.write( struct.pack('<HH', crc, crc ) )
    moc_ota_output.write( bytes(app[8::]) )

    moc_app.close()
    moc_kernel.close()
    moc_app_output.close()
    moc_ota_output.close()

    # os.system('cp '+MOC_OTA_OUTPUT_FILE+' '+MOC_OTA_OUTPUT_FILE+'.origin')
    with open(MOC_OTA_OUTPUT_FILE, 'rb') as file:
        ota_md5_bytes = MD5(file.read())
    with open(MOC_OTA_OUTPUT_FILE, 'ab') as file:
        file.write(ota_md5_bytes)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass