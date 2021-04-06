import numpy as np
import matplotlib.pyplot as plt
import csv
#from sklearn import preprocessing

x = []
modelInput = []
controllerInput = []
input = []
current = []
signals = []
linecounter = 0
kindcounter = 0
with open('../files/thesis/s_curve_left_up/trimmed/calog_.csv') as csv_file:
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
                vector.append(float(v))
            if kindcounter == 0:
                x.append(vector)
            if kindcounter == 1:
                modelInput.append(vector)
            if kindcounter == 2:
                controllerInput.append(vector)
            if kindcounter == 3:
                input.append(vector)
            if kindcounter == 4:
                current.append(vector)
            if kindcounter == 5:
                signals.append(vector)

cx = []
cy = []
cz = []
cpitch = []
cyaw = []
for v in current:
    cx.append(float(v[0]))
    cy.append(float(v[1]))
    cz.append(float(v[2]))
    cpitch.append(float(v[4]))
    cyaw.append(float(v[5]))

ix = []
iy = []
iz = []
ipitch = []
iyaw = []
for v in input:
    ix.append(float(v[0]))
    iy.append(float(v[1]))
    iz.append(float(v[2]))
    ipitch.append(float(v[4]))
    iyaw.append(float(v[5]))

ex = []
ey = []
ez = []
epitch = []
eyaw = []
for idx in range(len(current)):
    ex.append(float(current[idx][0]) - float(input[idx][0]))
    ey.append(float(current[idx][1]) - float(input[idx][1]))
    ez.append(float(current[idx][2]) - float(input[idx][2]))
    epitch.append(float(current[idx][4]) - float(input[idx][4]))
    eyaw.append(float(current[idx][5]) - float(input[idx][5]))

for idx in range(len(eyaw)):
    if idx < 3 or idx > len(eyaw) -4:
        if idx < 3:
            eyaw[idx] = (eyaw[idx] + eyaw[idx+1])/2.
        if idx > len(eyaw) -4:
            eyaw[idx] = (eyaw[idx-1] + eyaw[idx])/2.
    else:
        eyaw[idx] = (eyaw[idx-3] +eyaw[idx-2] +eyaw[idx-1] + eyaw[idx] + eyaw[idx+1] + eyaw[idx+2] + eyaw[idx+3])/7.


ixr = range(len(ix))
cxr = range(len(cx))
exr = range(len(ex))

ipitchr = range(len(ipitch))
cpitchr = range(len(cpitch))
epitchr = range(len(epitch))

iyawr = range(len(iyaw))
cyawr = range(len(cyaw))
eyawr = range(len(eyaw))
#plt.axis('equal')
plt.figure(figsize = (18,10))
plt.plot(exr, eyaw, 'r-', linewidth = 3.5,label = 'error')#,exr, iz, 'b-')
#plt.plot(exr, cx, 'g-', linewidth = 0.5,label = 'generated output')
#plt.grid(axis='y',alpha=0.3,zorder=0,linestyle='--')
#plt.grid(axis='x',alpha=0.3,zorder=0,linestyle='--')
plt.xlabel('n',fontsize=30)
plt.ylabel('e',fontsize=30)
plt.xticks(fontsize=20)
plt.yticks(fontsize=20)
plt.legend(fontsize=20)
plt.savefig('../plots/ca_eyaw.pdf')
plt.show()

#def flatten()
