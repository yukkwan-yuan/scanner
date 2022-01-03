#!/usr/bin/env python2

for i in range(0, 479, 1):
    f1 = open("./tea0_addr/"+str(i)+".txt", "r")
    lines = f1.readlines()
    f2 = open("./data/tea0/cfg/train.txt", "a")

    for j in range(len(lines)):
        f2.write("%s" %lines[j])