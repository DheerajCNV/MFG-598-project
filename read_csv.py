import pandas as pd
import matplotlib.pyplot as plt 
import numpy as np
data = pd.read_csv("/home/dheeraj/catkin_ws/src/drone_controller/plots/1/x-iris_demo:world_pose_position_x.csv")

print(data.shape)
data = np.array(data)
# x axis values
x = data[:,0]
# corresponding y axis values
y = data[:,1]
  
# plotting the points 
plt.plot(x, y)
  
# naming the x axis
plt.xlabel('x - axis')
# naming the y axis
plt.ylabel('y - axis')
  
# giving a title to my graph
plt.title('My first graph!')
  
# function to show the plot
plt.show()