#!/usr/bin/python
import sys

usage = 'scaleInput.py inputFile scaleFactor'

if len(sys.argv) != 3:
    sys.exit(usage)

f = open(sys.argv[1])
scale = float(sys.argv[2])
for line in f:
    newvals = [scale*float(x) for x in line.split()]
    print "%f %f %f" % tuple(newvals)
