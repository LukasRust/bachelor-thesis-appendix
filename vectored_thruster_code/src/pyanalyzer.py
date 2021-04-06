import numpy as np
import matplotlib.pyplot as plt
import csv

x = []
modelInput = []
controllerInput = []
input = []
current = []
signals = []
kindlist = []
kindlist.append(x)
kindlist.append(modelInput)
kindlist.append(controllerInput)
kindlist.append(input)
kindlist.append(current)
kindlist.append(signals)
linecounter = 0
kindcounter = 0
with open('../files/thesis/trimmed/s_curve_left_up/calog_.csv') as csv_file:
    reader = csv.reader(csv_file ,delimiter=';')
    for row in reader:
        if(len(row) == 0):
            print('empty line')
            linecounter = linecounter + 1
            if(linecounter == 4):
                linecounter = 0
                kindcounter = kindcounter + 1
        else:
            vector = []
            for v in row:
                vector.append(v)
            kindlist[kindcounter].append(vector)

print('x length: ', len(kindlist[0]))
print('modelInput length: ', len(kindlist[1]))
print('controllerInput length: ', len(kindlist[2]))
print('input length: ', len(kindlist[3]))
print('current length: ', len(kindlist[4]))
print('signal length: ', len(kindlist[5]))

idxs = range(len(modelInput))
atod = []
plotarr = []
tx = []
for a in range(6):
    plotarr.append([])
for i in idxs:
    v = []
    if(float(input[i][0]) != 0.):
        v.append(float(current[i][0])/float(input[i][0]))
        #tx.append(float(input[i][0])/float(current[i][0]))
    else:
        v.append(0.)
        #tx.append(0.)
    plotarr[0].append(v[0])

    if(float(input[i][1]) != 0.):
        v.append(float(current[i][1])/float(input[i][1]))
    else:
        v.append(0.)
    plotarr[1].append(v[1])

    if(float(input[i][2]) != 0.):
        v.append(float(current[i][2])/float(input[i][2]))
    else:
        v.append(0.)
    plotarr[2].append(v[2])

    if(float(input[i][3]) != 0.):
        v.append(float(current[i][3])/float(input[i][3]))
    else:
        v.append(0.)
    plotarr[3].append(v[3])

    if(float(input[i][4]) != 0.):
        v.append(float(current[i][4])/float(input[i][4]))
    else:
        v.append(0.)
    plotarr[4].append(v[4])

    if(float(input[i][5]) != 0.):
        v.append(float(current[i][5])/float(input[i][5]))
    else:
        v.append(0.)
    plotarr[5].append(v[5])

    atod.append(v)

print('overall average ratiob: ',sum(plotarr[5])/float(len(plotarr[5])))

r = range(len(plotarr[5]))
#plt.axis('equal')
plt.figure(figsize = (18,10)) #18, 10 normalerweise
#fig.set_figheight(1)
#fig.set_figwidth(1)
#plot = fig.add_subplot(111)

plt.plot(r, plotarr[0], 'r-')
#plot.xlabel("X")
#plot.ylabel("Y")
plt.grid(axis='y',alpha=0.3,zorder=0,linestyle='--')
plt.grid(axis='x',alpha=0.3,zorder=0,linestyle='--')
plt.xlabel('X',fontsize=30)
plt.ylabel('Z',fontsize=30)
plt.xticks(fontsize=20)
plt.yticks(fontsize=20)

plt.show()
