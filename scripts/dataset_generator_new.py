# Import the necessary modules
import rosbag
import rospy
import numpy as np
import csv
import math
import pandas as pd
import matplotlib.pyplot as plt
import pickle

class Bag2CSV:
  def __init__(self):
    bag = rosbag.Bag('/home/camillo/workspace/Learning/scripts/my_bag.bag')

    print("Initialising Bag2CSV for Bag")

    JS = np.empty(12)
    Time = np.empty(1)
    for topic1, msg1, t1 in bag.read_messages(topics=['/joint_states']):
      p = msg1.position
      v = msg1.velocity
      js = [p[0], p[1], p[2], p[3], p[4], p[5], v[0], v[1], v[2], v[3], v[4], v[5]]
      time=t1.secs+(t1.nsecs/1000000000)
      # print(js[6:])
      JS = np.row_stack((JS, js))
      Time = np.row_stack((Time, time))
    JS=JS[1:,:]
    Time=Time[1:,:]

    # print("JS Done")


    Cart = np.empty(7)
    for topic3, msg3, t3 in bag.read_messages(topics=['/Current_EE_position']):
          cart = msg3.data
          Cart = np.row_stack((Cart, cart))
    Cart = Cart[1:,:]

    # print("Cart Done")
    

    self.target_length = max(len(JS), len(Cart), )

    # print("Starting synchrinisation...")
    JS_synced = self.Upsample(target_length= self.target_length, Input_array= JS)
    Cart_synced = self.Upsample(target_length = self.target_length,Input_array= Cart)
    Time_synced = self.Upsample(target_length = self.target_length,Input_array= Time)


    Dataset = np.concatenate((Time_synced, Cart_synced, JS_synced), axis = 1)

    with open("/home/camillo/workspace/Learning/scripts/Memory_1.pkl", 'rb') as f:
        memory = pickle.load(f)

    memory['Dataset'] = Dataset
    
    with open("/home/camillo/workspace/Learning/scripts/Memory_1.pkl", "wb") as fp:
      pickle.dump(memory, fp)

    
    print("Dataset ready")

    #upsampling function

  def Upsample(self, target_length, Input_array):
    a = (target_length/len(Input_array))
    i=0
    o=0
    # print(a)
    synced_array= np.empty((target_length,len(Input_array[1])))
    synced_array[:] = np.NaN
    for i in range(len(Input_array)):
        # print(o,",",i)
        synced_array[round(o),:] = Input_array[i,:]
        o=o+a
        i = i+1
        
    synced_array = pd.DataFrame(synced_array)
    synced_array = synced_array.interpolate(method ='linear', limit_direction ='forward')
    synced_array = synced_array.iloc[:].values

    return synced_array

  #downsampling function

  def Downsample(self, target_length, Input_array):
      aa = (len(Input_array)/target_length)
      # print(aa)
      ii=0
      oo=0
      synced_array= np.empty((target_length,len(Input_array[1])))
      for oo in range(len(synced_array)):
          # print(ii, ",", oo)
          synced_array[oo,:] = Input_array[round(ii),:]
          oo=oo+1
          ii = ii+aa
      # synced_array = pd.DataFrame(synced_array)
      # synced_array = synced_array.interpolate(method ='linear', limit_direction ='forward')
      # synced_array = synced_array.iloc[:].values

      return synced_array

if __name__ == "__main__":
  Bag2CSV()