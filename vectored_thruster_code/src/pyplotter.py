import numpy as np
import matplotlib.pyplot as plt
import csv

foldername = 'curve_left_horizon/trimmed'
filename = 'curve_left_horizon'
savename = 'curve_left_horizon_quad'
yorz = 1

actual_x = []
actual_y = []
desired_x = []
desired_y = []
uncontrolled_x = []
uncontrolled_y = []
with open('../files/thesis/' + foldername + '/actual_trajectories_' + filename + '.csv') as csv_file:
    reader = csv.reader(csv_file ,delimiter=';')
    counter = 0
    for row in reader:
        if counter % 10 == 1:
            actual_x.append(float(row[0]))
            actual_y.append(float(row[yorz]))
        counter += 1
with open('../files/thesis/' + foldername + '/desired_trajectories_' + filename + '.csv') as csv_file:
    reader = csv.reader(csv_file ,delimiter=';')
    counter = 0
    for row in reader:
        if counter % 10 == 1:
            desired_x.append(float(row[0]))
            desired_y.append(float(row[yorz]))
        counter += 1

#with open('../files/thesis/actual_trajectories_curve_left_without_controller.csv') as csv_file:
#    reader = csv.reader(csv_file ,delimiter=';')
#    counter = 0
#    for row in reader:
#        if counter % 10 == 1:
#            uncontrolled_x.append(float(row[0]))
#            uncontrolled_y.append(float(row[2]))
#        counter += 1

plt.figure(figsize = (18,18)) #18, 10 normalerweise
#fig.set_figheight(1)
#fig.set_figwidth(1)
#plot = fig.add_subplot(111)

plt.plot(actual_x, actual_y, 'r-', label = 'actual trajectory', linewidth = 3.5)
plt.plot(desired_x, desired_y, 'g--', label = 'planned trajectory', linewidth = 3.5)#,uncontrolled_x, uncontrolled_y, 'b-')
#plot.xlabel("X")
#plot.ylabel("Y")
plt.grid(axis='y',alpha=0.3,zorder=0,linestyle='--')
plt.grid(axis='x',alpha=0.3,zorder=0,linestyle='--')
plt.xlabel('X',fontsize=30)
plt.ylabel('Y',fontsize=30)
plt.xticks(fontsize=20)
plt.yticks(fontsize=20)
plt.legend(fontsize=20)
plt.savefig('../plots/' + savename + '.pdf')
plt.show()

#plt.axis('equal')
#plt.plot(actual_x, actual_y, 'r-', desired_x, desired_y, 'g--',uncontrolled_x, uncontrolled_y, 'b-')
#plt.set_figheight(6)
#plt.set_figwidth(8)
#plt.xlabel("X")
#plt.ylabel("Y")
#plt.show()


#    axes = plt. gca()
#    axes. xaxis. label. set_size(20)
#    axes. yaxis. label. set_size(20)
#https://tex.stackexchange.com/questions/196481/problem-on-subfigure-2x2
