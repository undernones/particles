#!/usr/bin/python

import sys

f = open(sys.argv[1])

verts = []
faces = []

for line in f.readlines():
    if line[0] == 'v':
        verts.append(line)
    elif line[0] == 'f':
        faces.append(line)

for v in verts:
    print v,
for f in faces:
    print f,
