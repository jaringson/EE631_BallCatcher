import numpy as np
import matplotlib.pyplot as plt

data_file = open('build/pos_data.txt')

time = []
X = []
Y = []
Z = []

t = 0
dt = 1.0 / 60.0

for line in data_file.readlines(): 
    data = line.split('\t') 
    x = float(data[0])
    y = float(data[1])
    z = float(data[2])

    time.append(t)
    t += dt
    
    X.append(x) 
    Y.append(y) 
    Z.append(z) 

length = len(time)
time = np.array(time).reshape((length,1))
X = np.array(X).reshape((length,1))
Y = np.array(Y).reshape((length,1))
Z = np.array(Z).reshape((length,1))
ones = np.ones((length,1))

new_z = np.linspace(0,400,401).reshape((401,1))

# batch LS 1st time
A = np.hstack([Z**2,Z,ones])
c = np.linalg.inv(A.T @ A) @ A.T @ Y
yz_fit = c[0]*new_z**2 + c[1]*new_z + c[2]

# batch LS 2nd time
c = np.linalg.inv(A.T @ A) @ A.T @ X
xz_fit = c[0]*new_z**2 + c[1]*new_z + c[2]

print('Fitting with z')
print('Calculated x: ',-xz_fit[0])
print('Calculated y: ',yz_fit[0])

new_time = np.linspace(0,0.7)

# least squares with time
A = np.hstack([time**2,time,ones])
c = np.linalg.inv(A.T @ A) @ A.T @ X
x_fit = c[0]*new_time**2 + c[1]*new_time + c[2]

c = np.linalg.inv(A.T @ A) @ A.T @ Y
y_fit = c[0]*new_time**2 + c[1]*new_time + c[2]

c = np.linalg.inv(A.T @ A) @ A.T @ Z
z_fit = c[0]*new_time**2 + c[1]*new_time + c[2]

print('Fitting with time')
print('Calculated x: ',-x_fit[-1])
print('Calculated y: ',y_fit[-1])

plt.figure(1)
plt.plot(time,X,'r',label='x')
plt.plot(time,Y,'b',label='y')
plt.plot(time,Z,'g',label='z')
plt.xlabel('time (s)')
plt.ylabel('pos (in)')
plt.legend()
plt.title('Gathered Data')

plt.figure(2)
plt.plot(Z,X,'b',label='xz_data')
plt.plot(new_z,xz_fit,'r--',label='xz_fit')
plt.plot(new_z[0],xz_fit[0],'k.',label='goal')
plt.xlabel('z (in)')
plt.ylabel('x (in)')
plt.legend()
plt.title('X-Z Data')

plt.figure(3)
plt.plot(Z,Y,'b',label='yz_data')
plt.plot(new_z,yz_fit,'r--',label='yz_fit')
plt.plot(new_z[0],yz_fit[0],'k.',label='goal')
plt.xlabel('z (in)')
plt.ylabel('y (in)')
plt.legend()
plt.title('Y-Z Data')

plt.figure(4)
plt.plot(Z,X,'b',label='xz_data')
plt.plot(z_fit,x_fit,'r--',label='xz_fit')
plt.plot(z_fit[-1],x_fit[-1],'k.',label='goal')
plt.xlabel('z (in)')
plt.ylabel('x (in)')
plt.legend()
plt.title('X-Z Data')

plt.figure(5)
plt.plot(Z,Y,'b',label='yz_data')
plt.plot(z_fit,y_fit,'r--',label='yz_fit')
plt.plot(z_fit[-1],y_fit[-1],'k.',label='goal')
plt.xlabel('z (in)')
plt.ylabel('y (in)')
plt.legend()
plt.title('Y-Z Data')

plt.show()
