import matplotlib.pyplot as plt
import numpy as np
import re

nondecimal = re.compile(r'[^\d.]+')
with open("log_torque_rpm.txt", "r") as ins:
    torque = []
    speed = []
    for line in ins:
        tmp = line.split('\t')
        torque.append(tmp[0])
        speed.append(nondecimal.sub('',tmp[1]))
        # print speed

plt.plot(speed, torque, 'b*-')
plt.grid(True)
plt.title('Torque/speed curve')
plt.ylabel('Torque')
plt.xlabel('RPM')
plt.show()
