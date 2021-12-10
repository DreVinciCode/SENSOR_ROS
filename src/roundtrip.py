#!/usr/bin/python3

'''
roundtrip.py
SENSAR
Andre Cleaver
Tufts University
'''

import os
import argparse

parser = argparse.ArgumentParser(description="A script to command the Turtlebot2 to make a round trip on selected points on a map.")
parser.add_argument('-n', metavar='NumPoints' , help="Specify the number of points for the path.", default='eth0')
args=parser.parse_args()


numPoints = 0

def Main():

    if args.n > 1 :
        try:
            print("Selected %(filename)s..." % {"filename" : args.n})
            
        except:
            print("Can\'t read PCAP file %(filename)s!" % {"filename" : args.r})
    else



if __name__ == "__main__":
    Main()