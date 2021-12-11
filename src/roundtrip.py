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
parser.add_argument('--n', metavar='NumPoints', type=int , help="Specify the number of points (2-10) for the robot to travel towards.")
args=parser.parse_args()

numPoints = 0
minPoints = 2
maxPoints = 10

def Main():            
    userInput = 2
    
    try:
        if args.n >= 2 :
            print("Entered in %(value)s points." % {"value" : args.n})
            userInput = args.n        
    
    except:
        while True:
            userInput = input("Enter number of points between 2 and 10: ")
            try:
                if(isinstance(int(userInput), int) and int(userInput) >= minPoints and int(userInput) < maxPoints):
                    print("Entered in %(value)s points." % {"value" : userInput})
                    break      
            except:
                continue


if __name__ == "__main__":
    Main()