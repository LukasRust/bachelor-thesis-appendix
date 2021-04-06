import numpy as np
import matplotlib.pyplot as plt
import csv


actual_x = []
actual_y = []
actual_t = []
desired_x = []
desired_y = []
desired_t = []
uncontrolled_x = []
uncontrolled_y = []
with open('../files/thesis/actual_trajectories_s_curve_without_model.csv') as csv_file:
    reader = csv.reader(csv_file ,delimiter=';')
    counter = 0
    for row in reader:
        if counter % 10 == 1:
            actual_x.append(float(row[0]))
            actual_y.append(float(row[1]))
            actual_times.append(float(row[9]))
        counter += 1
with open('../files/thesis/desired_trajectories_s_curve_without_model.csv') as csv_file:
    reader = csv.reader(csv_file ,delimiter=';')
    counter = 0
    for row in reader:
        if counter % 10 == 1:
            desired_x.append(float(row[0]))
            desired_y.append(float(row[1]))
            desired_times.append(row[9])
        counter += 1

former_time
error = []
time = 0.
first = true
for di in range(len(desired_times)):
    for ai in range(len(actual_times)):
        if first:
            time = abs(desired_times[di] - actual_times[ai])
            former_time = 5000.

        new_time = abs(desired_times[di] - actual_times[ai])
        if new_time > former_time:
            error[di][] = desired
            break

        if  new_time < time:
            time = new_time
            error



#with open('../files/thesis/actual_trajectories_curve_left_without_controller.csv') as csv_file:
#    reader = csv.reader(csv_file ,delimiter=';')
#    counter = 0
#    for row in reader:
#        if counter % 10 == 1:
#            uncontrolled_x.append(float(row[0]))
#            uncontrolled_y.append(float(row[2]))
#        counter += 1


plt.axis('equal')
plt.plot(actual_x, actual_y, 'r-', desired_x, desired_y, 'g--',uncontrolled_x, uncontrolled_y, 'b-')
plt.xlabel("X")
plt.ylabel("Y")
plt.show()
