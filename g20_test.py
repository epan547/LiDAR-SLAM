import numpy as np
import g2o
import os
import pickle

optimizer = g2o.SparceOptimizer()
load_file = open('data','rb')
data = pickle.load(load_file)
print(data)
