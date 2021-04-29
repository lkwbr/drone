import numpy as np

wd = r'C:\Users\lukedottec\Documents\Education\Udacity\Flying Car and Autonmous Vehicles Nanodegree\FCND-Estimation-CPP\config\log'

# Grab data
gps_x = np.loadtxt('{}\Graph1.txt'.format(wd),delimiter=',',dtype='Float64',skiprows=1)[:,1]
acc_x = np.loadtxt('{}\Graph2.txt'.format(wd),delimiter=',',dtype='Float64',skiprows=1)[:,1]

# Analyze data
gps_x_std = np.std(gps_x)
acc_x_std = np.std(acc_x)

print(f'GPS X Std: {gps_x_std}')
print(f'Accelerometer X Std: {acc_x_std}')
