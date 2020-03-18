#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys, struct, time, os, subprocess, datetime, platform, math
from MAVProxy.modules.lib import multiproc
from pymavlink import mavutil
from io import StringIO
from termcolor import colored, cprint
from argparse import ArgumentParser
from mavflightview import *


# dictionaries of data decompilation
param_list = ["BATT_CAPACITY", "COMPASS_USE", "RTL_ALT_FINAL", "RTL_ALT", "TELEM_DELAY", "SERIAL0_BAUD", "SYSID_MYGCS", "SYSID_THISMAV"]
mode_dict = {0: "Stabilize", 1: "Acro", 2: "AltHold", 3: "Auto", 4: "Guided", 5: "Loiter", 6: "RTL", 7: "Circle", 8: "Position", 9: "Land", 10: "OF_Loiter",
            11: "Drift", 13: "Sport", 14: "Flip", 15: "AutoTune", 16: "PosHold", 17: "Brake"}
event_dict = {10: "ARMED", 11: "DISARMED", 15: "AUTO ARMED", 16: "TAKEOFF", 18: "LAND COMPETE", 25: "SET HOME"}
gps_desc_dict = {0:"no GPS", 1:"GPS but no fix", 2:"GPS with 2D fix", 3:"GPS with 3D fix"}
err_dict = {1: "Main", 2: "Radio", 3: "Compass", 4: "Optical flow", 5: "Throttle failsafe", 6: "Battery failsafe", 7: "GPS failsafe", 8: "GCS failsafe", 9: "Fence",
            10: "Flight Mode", 11: "GPS", 12: "Crash Check"}

# FMT header declared as types for usage in the proper function FORMAT: Name , Format , ColumsVals
gpstypes = set(['GPS', 'QBIHBcLLefffB', 'GMS', 'GWk', 'NSats', 'HDop', 'Lat', 'Lng', 'Alt', 'Spd', 'GCrs', 'VZ', 'U'])
msgtypes = set(['MSG', 'Z', 'Message'])
evtypes = set(['EV', 'B', 'Id'])
parmtypes = set(['PARM', 'Nf', 'Name', 'Value'])
modetypes = set(['MODE', 'Mh', 'ModeNum', 'Rsn'])
errtypes = set(['ERR', 'BB', 'Subsys','ECode'])
cmdtypes = set(['CMD', 'IHHHfffffff', 'TimeMS', 'CTot', 'CNum', 'CId', 'Prm1', 'Prm2', 'Prm3', 'Prm4', 'Lat', 'Lng', 'Alt'])
currtypes = set(['CURR', 'IhIhhhf', 'TimeMS', 'ThrOut', 'ThrInt', 'Volt', 'Curr', 'Vcc', 'CurrTot'])

# global variables for later data validation
ext_crc = None
hash_list = []
extdata_list = []
cmd_list = []
gps_list = []
curr_list = []

# object similar to the C# stringbuilder, for nice clean output
class StringBuilder:
     _file_str = None

     def __init__(self):
         self._file_str = StringIO()

     def append(self, str):
         self._file_str.write(str)

     def __str__(self):
         return self._file_str.getvalue()

# ----- Helper functions: get_eucledian_dist, find_letter / and unused but usefull functions log_info, fmt_info

# get the euclidean distance between 2 coordinate arrays
def get_eucledian_dist(a, b):
    dist = math.sqrt((float(a[0]) - float(b[0])) ** 2 + (float(a[1]) - float(b[1])) ** 2 + (float(a[2]) - float(b[2])) ** 2)
    return dist

# function to find letters inside lists
def find_letter(let, lst):
    if lst:
        e = lst[0]
        if isinstance(e, str) and let in e:
            return True
        return find_letter(let, lst[1:])
    return False

# display the full log life
def log_info(tlog, types = None):
    while True:
        output = StringBuilder()
        data = []
        mavmsg = tlog.recv_match(type=types, condition=None)
        if mavmsg is None:
            break
        print(str(mavmsg))
    tlog.rewind()

# display fmt headers
def fmt_info(tlog, types = None):
    while True:
        output = StringBuilder()
        data = []
        mavmsg = tlog.recv_match(type=types, condition=None)
        if mavmsg is None:
            break
        if mavmsg.get_type() == 'FMT':
            print(mavmsg)
    tlog.rewind()

# -----

# each of the _info function takes a types arg, of the type as declared in the FMT headers
# get the FMT type of PARM and look for the specific ones declared in the param_list
def parm_info(tlog, types = parmtypes):
    while True:
        data = []
        # create an output in a stringbuilder like format
        output = StringBuilder()
        # get the MavLink Message
        mavmsg = tlog.recv_match(type=types, condition=None)
        if mavmsg is None:
            break
        # if the header is PARM , and belongs to the param_list
        if mavmsg.get_type() == 'PARM':
            # get the value as declared in the FMT set list
            parm_name = mavmsg.Name
            if parm_name in param_list:
                parm_val = mavmsg.Value
                # timestamp extraction
                tmstmp = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(mavmsg._timestamp))
                output.append(str(tmstmp))
                output.append("  ")
                output.append(str(parm_name))
                output.append("\t")
                output.append(str(parm_val))
                print(output)
                # timelining
                data.append(tmstmp)
                data.append(parm_name)
                data.append(parm_val)
                extdata_list.append(data)
    # reset back to the begin of log.bin file
    tlog.rewind()

# get the output of messages
def msg_info(tlog, types = msgtypes):
    while True:
        data = []
        # create an output in a stringbuilder like format
        output = StringBuilder()
        # get the MavLink Message
        mavmsg = tlog.recv_match(type=types, condition=None)
        if mavmsg is None:
            break
        if mavmsg.get_type() == 'MSG':
            # get the value as declared in the FMT set list
            msg = mavmsg.Message
            # timestamp extraction
            tmstmp = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(mavmsg._timestamp))
            output.append(str(tmstmp))
            output.append("  ")
            output.append(colored(str(msg), 'green'))
            print(output)
            # timelining
            data.append(tmstmp)
            data.append(msg)
            extdata_list.append(data)
            # get the checksum, spit the msg and check if the () appear which means that firmware version is included
            if len(msg.split(" ")) == 3 and find_letter("(", msg.split(" ")):
                msg = msg.split(" ")
                # refer to the global variable , since python keeps the scope visible for the function only
                global ext_crc
                # keep the list item and remove the first and last "(" ")"
                ext_crc = msg[2][1:-1]
    # reset back to the begin of log.bin file
    tlog.rewind()

# get the events of the flight
def ev_info(tlog, types = evtypes):
    while True:
        data = []
        output = StringBuilder()
        mavmsg = tlog.recv_match(type=types, condition=None)
        if mavmsg is None:
            break
        if mavmsg.get_type() == 'EV':
            # get the value as declared in the FMT set list
            eventNo = mavmsg.Id
            # timestamp extraction
            tmstmp = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(mavmsg._timestamp))
            output.append(str(tmstmp))
            output.append("  ")
            output.append(str(event_dict.get(eventNo)))
            print(output)
            # timelining
            data.append(tmstmp)
            data.append(str(event_dict.get(eventNo)))
            extdata_list.append(data)
    # reset back to the begin of log.bin file
    tlog.rewind()

# get the errors happened during the flight
def err_info(tlog, types = errtypes):
    while True:
        data = []
        output = StringBuilder()
        mavmsg = tlog.recv_match(type=types, condition=None)
        if mavmsg is None:
            break
        if mavmsg.get_type() == 'ERR':
            err = mavmsg.Subsys
            ecode = mavmsg.ECode
            tmstmp = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(mavmsg._timestamp))
            output.append(str(tmstmp))
            output.append("  ")
            output.append(err_dict.get(err))
            output.append("\t")
            output.append(str(ecode))
            print(output)
            # timelining
            data.append(tmstmp)
            data.append(str(err_dict.get(err)))
            data.append(ecode)
            extdata_list.append(data)
    tlog.rewind()

# get the mode changes during the flight
def mode_info(tlog, types = modetypes):
    while True:
        data = []
        output = StringBuilder()
        mavmsg = tlog.recv_match(type=types, condition=None)
        if mavmsg is None:
            break
        if mavmsg.get_type() == 'MODE':
            # get the value as declared in the FMT set list
            mode = mode_dict.get(mavmsg.Mode)
            modenum = mavmsg.ModeNum
            # timestamp extraction
            tmstmp = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(mavmsg._timestamp))
            output.append(str(tmstmp))
            output.append("  ")
            output.append(str(mode))
            output.append("\t")
            output.append(str(modenum))
            print(output)
            # timelining
            data.append(tmstmp)
            data.append(mode)
            data.append(modenum)
            extdata_list.append(data)
    # reset back to the begin of log.bin file
    tlog.rewind()

# get current, voltage and consumption info
def curr_info(tlog, types = currtypes):
    while True:
        output = StringBuilder()
        data = []
        mavmsg = tlog.recv_match(type=types, condition=None)
        if mavmsg is None:
            break
        if mavmsg.get_type() == 'CURR':
            tmstmp = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(mavmsg._timestamp))
            volt = mavmsg.Volt
            curr = mavmsg.Curr
            currtot = mavmsg.CurrTot
            output.append(str(tmstmp))
            output.append("  ")
            output.append("CURR")
            output.append("\t")
            output.append(str(volt))
            output.append("\t")
            output.append(str(curr))
            output.append("\t")
            output.append(str(currtot))
            # timelining
            data.append(tmstmp)
            data.append("CURR")
            data.append(str(volt))
            data.append(curr)
            data.append(currtot)
            extdata_list.append(data)
            # record Board voltage and Total current drawn from battery to monitor for Anomalies
            curr_list.append([tmstmp,curr])
            print(output)
    tlog.rewind()

# Detect Anomalies in Board voltage and Total current drawn from battery above decalred thresshold
def curr_anomaly_detection():
    # total current thresshold percentage
    thres_per_curr = 0.1
    # median values curr
    medcurr = 0
    # list of values exceding thresshold
    c_ex_list = []
    # find the median value of each one
    for c in zip(curr_list):
        medcurr += c[1]
    # foramt float to 0 pressision
    medcurr = medcurr / len(curr_list)
    # find anomalies in declared thresshold
    for c in zip(curr_list):
        thres_curr = medcurr*thres_per_curr
        if (c[1] < medcurr - thres_curr) or (c[1] > medcurr + thres_curr):
            c_ex_list.append([c[0],c[1]])
    if not c_ex_list:
        print(colored("No Current drawn from the battery Anonmaly Detected",'green'))
    else:
        print(colored("Current drawn from the battery  Anonmaly Detected",'red'))
        for c in c_ex_list:
            print(c[0], "\t", c[1])

# get gps info and coordinates
def gps_info(tlog, types = gpstypes):
    while True:
        output = StringBuilder()
        data = []
        mavmsg = tlog.recv_match(type=types, condition=None)
        gps_status_err = False
        if mavmsg is None:
            break
        if mavmsg.get_type() == 'GPS':
            # get the value as declared in the FMT set list
            lat = mavmsg.Lat
            lng = mavmsg.Lng
            status = mavmsg.Status
            # format float val to 2 decimal
            alt = "{0:.2f}".format(mavmsg.Alt)
            # timestamp extraction
            tmstmp = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(mavmsg._timestamp))
            output.append(str(tmstmp))
            output.append("  ")
            # according to the GPS status a proper color is displayed
            if status == 0:
                gps_status_err = True
                output.append(str(tmstmp))
                output.append("  ")
                output.append(colored(str(gps_desc_dict.get(status)),'red'))
                print(output)
            elif status == 1:
                gps_status_err = True
                output.append(str(tmstmp))
                output.append("  ")
                output.append(colored(str(gps_desc_dict.get(status)),'yellow'))
                print(output)
            # timelining
            data.append(tmstmp)
            data.append(gps_desc_dict.get(status))
            data.append(lat)
            data.append(lng)
            data.append(alt)
            extdata_list.append(data)
            gps_list.append(data)
    if not gps_status_err:
      output.append(colored("No GPS signal loss",'green'))
      print(output)
    # reset back to the begin of log.bin file
    tlog.rewind()

# get the recorded transmited commands
def cmd_info(tlog, types = cmdtypes):
    while True:
        output = StringBuilder()
        data = []
        mavmsg = tlog.recv_match(type=types, condition=None)
        if mavmsg is None:
            break
        if mavmsg.get_type() == 'CMD':
            tmstmp = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(mavmsg._timestamp))
            cid = mavmsg.CId
            lat = mavmsg.Lat
            lng = mavmsg.Lng
            alt = mavmsg.Alt
            output.append(str(tmstmp))
            output.append("  ")
            output.append(str(cid))
            output.append("\t")
            output.append(str(lat))
            output.append("\t")
            output.append(str(lng))
            output.append("\t")
            output.append(str(alt))
            print(output)
            # timelining
            data.append(tmstmp)
            data.append(str(cid))
            data.append(lat)
            data.append(lng)
            data.append(float(alt))
            extdata_list.append(data)
            cmd_list.append(data)
    # reset back to the begin of log.bin file
    tlog.rewind()

# verify the cmd execution
def cmd_execution():
    # allowed offset of location to mark a cmd as executed
    offset = 0.2
    if not gps_list:
        print("No GPS Data recorded during flight")
    elif not cmd_list:
        print("No Command Data recorded during flight")
    else:
        # check foe every recoeded command
        for command in cmd_list:
            output = StringBuilder()
            data = []
            # flag for command execution
            executed = False
            # cmd_lat = command[2]  cmd_lng = command[3]  cmd_alt = command[4]
            cmd_coords = [command[2], command[3], command[4]]
            # crossvalidate with the recorded gps coordinates
            for gpslocation in gps_list:
                # gps_lat = gpslocation[3]  gps_lng = gpslocation[4]  gps_alt = gpslocation[5]
                gps_coords = [gpslocation[2], gpslocation[3], gpslocation[4]]
                # get the 3D euclidean distance
                dist = get_eucledian_dist(cmd_coords, gps_coords)
                # check the correct execution
                if dist <= offset:
                    # set the flag true and get the timestamp of the execution
                    executed = True
                    tmstmp = gpslocation[0]
                    print(gpslocation)
                    print(command)
            if executed:
                output.append(tmstmp)
                output.append("  ")
                output.append(colored("EXECUTED",'green'))
                output.append("\t")
                output.append(command[1])
                print(output)
                # timelining
                data.append(tmstmp)
                data.append("EXECUTED")
                data.append(command[1])
                extdata_list.append(data)
            else:
                # if not executed get the timestamp of cmd transmission
                tmstmp = command[0]
                output.append(tmstmp)
                output.append("  ")
                output.append(colored("NOT EXECUTED",'red'))
                output.append("\t")
                output.append(command[1])
                print(output)
                # timelining
                data.append(command[0])
                data.append("NOT EXECUTED")
                data.append(command[1])
                extdata_list.append(data)

# get the gps height locations where the next coord is way far from the current
def gps_altD_anomaly_detection():
    # max allowed alt offset in meters
    offset = 3
    # flag to display alt anomaly
    alt_anomaly = False
    for i in range(len(gps_list)-1):
        output = StringBuilder()
        data = []
        # get the current and next timestamp
        cur_tmstmp = gps_list[i][0]
        next_tmstmp = gps_list[i+1][0]
        # get the current and next relalt
        cur_relalt = float(gps_list[i][4])
        next_relalt = float(gps_list[i+1][4])
        # get the alt diff and display the anomaly
        diff = abs(cur_relalt - next_relalt)
        if diff >= offset:
            alt_anomaly = True
            output.append(cur_tmstmp)
            output.append("  ")
            output.append(next_tmstmp)
            output.append("\t")
            output.append(colored("Alt Anomaly Detected",'red'))
            output.append("\t")
            output.append("{0:.2f}".format(diff))
            print(output)
            #timelining
            data.append(cur_tmstmp)
            data.append(next_tmstmp)
            data.append("Alt Anomaly Detected")
            data.append("{0:.2f}".format(diff))
            extdata_list.append(data)
    if not alt_anomaly:
        output = StringBuilder()
        output.append(colored("No Alt Anomaly Detected",'green'))
        print(output)

# function to check if the extracted checksum corresponds to the one of ArduPilot official repo
def crc_verification():
    output = StringBuilder()
    data = []
    # get the branches hash code that include tags, which means they are released versions
    command = ["git ls-remote git://github.com/ArduPilot/ardupilot.git | grep refs/tags"]
    # send the command
    p = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    # read the output
    for line in p.stdout.readlines():
        line = line.decode("utf-8")
        #keep the 8 first digits
        line = line[:8]
        hash_list.append(line)
    if ext_crc in hash_list:
        print(colored("CRC Found",'yellow'),"\t",ext_crc)
    else:
        print(colored("Extracted CRC does not match with Ardupilot",'red'))

# function to sort and display timeline events and create an timeline file
def timeline_analysis(args):
    filename = args.strip('../')
    filename = filename.replace("/","__")
    extdata_list.sort()
    with open(filename+".analysis", 'w+') as log:
        for item in extdata_list:
            output = StringBuilder()
            for i in item:
                output.append(str(i))
                output.append('\t')
            log.write(str(output)+"\n")
    print("\n>Timeline Analysis file Created")

# core function to handle the data extraction
def get_MAVmsgs(args):
    # create an object out if the log file
    tlog = mavutil.mavlink_connection(args, notimestamps=False,
                                      zero_time_base=False)
    # create a map object to store the options from mavflightview
    map_options = mavflightview_options()

    # input validatiion
    if len(args) > 0:
        wildcard = args[0]
        if wildcard.find('*') == -1 and wildcard.find('?') == -1:
            wildcard = "*" + wildcard + "*"
    else:
        wildcard = '*'
    tlog.rewind()

    ##begin info extraction
    #log_info(tlog)
    #fmt_info(tlog)
    print("\n>PARM Extraction")
    #parm_info(tlog)
    print("\n>MSG Extraction")
    #msg_info(tlog)
    print("\n>EVENT Extraction")
    #ev_info(tlog)
    print("\n>MODE Extraction")
    #mode_info(tlog)
    print("\n>ERROR Extraction")
    #err_info(tlog)
    print("\n>CURRENT Extraction")
    #curr_info(tlog)
    print("\n>CMD Extraction")
    #cmd_info(tlog)
    print("\n>GPS Status Extraction")
    gps_info(tlog)
    print("\n>CMD Execution")
    #cmd_execution()
    print("\n>CRC Verification")
    #crc_verification()
    #print("\n>CURR Anomaly Detection")
    #curr_anomaly_detection()
    print("\n>GPS Alt Anomaly Detection")
    gps_altD_anomaly_detection()
    timeline_analysis(args)
    print("\n>MAP View")
    mavflightview(args,map_options)


def __main__():
    # parse the input data
    print('                               888                      ')
    print('                               888                      ')
    print(' .d88b. 888d888888  88888888b. 88888b.  .d88b. 88888b.  ')
    print('d88P"88b888P"  888  888888 "88b888 "88bd88""88b888 "88b ')
    print('Y88b 888888    Y88b 888888 d88P888  888Y88..88P888  888 ')
    print(' "Y88888888     "Y8888888888P" 888  888 "Y88P" 888  888 ')
    print('     888            888888                              ')
    print('Y8b d88P       Y8b d88P888                              ')
    print(' "Y88P"         "Y88P" 888                              ')
    parser = ArgumentParser(description=__doc__)
    parser.add_argument("files", metavar="<FILE>", nargs="?")
    args = parser.parse_args()
    if args.files is not None and len(args.files) != 0:
        get_MAVmsgs(args.files)
    else:
        print("Usage: gryphon.py <LOGFILE...>")
        sys.exit(1)

if __name__ == "__main__":
    __main__()
