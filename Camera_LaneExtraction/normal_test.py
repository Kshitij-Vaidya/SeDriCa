import numpy as np
import matplotlib.pyplot as plt
import cv2
import math

meanSum = 0
stdSum = 0

f = open("ValsMeanStdDev.txt", 'r')
Lines = f.readlines()
for line in Lines:
    line = line[:len(line)-2]
    line = line.split(' ')
    meanSum += float(line[0])
    stdSum += float(line[1])

print(meanSum / 103)
print(stdSum / 103)


Mean = 131.2963
StdDev = 10.25457564659083

LowerBound = Mean - (StdDev * 0.25)

UpperLimit = (1 / math.sqrt(2 * math.pi * (StdDev ** 2))) + 0.0001

LowerLimit = UpperLimit * math.exp((-1) * (((LowerBound - Mean) ** 2) / (2 * (StdDev ** 2))))


print(UpperLimit)
print(LowerLimit)


'''''
RIGHT LANE
0.5 Sigma
Upper Limit : 0.3900383124084344
Lower Limit : 0.034332510568718146

0.75 Sigma
Upper : 0.039003831240843445
Lower : 0.029441636449884682
'''




