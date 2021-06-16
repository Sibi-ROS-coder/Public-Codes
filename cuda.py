from numba import vectorize, jit, cuda 
import numpy as np 
# to measure exec time 
from timeit import default_timer as timer 

# normal function to run on cpu 
def func(a):                                 
    for i in range(10000000): 
        a[i]+= 1    

# function optimized to run on gpu 
@vectorize(['float64(float64)'], target ="cuda")                         
def func2(x): 
    return x+1

# kernel to run on gpu
@cuda.jit
def func3(a, N):
    tid = cuda.grid(1)
    if tid < N:
        a[tid] += 1


if __name__=="__main__": 
    n = 10000000                            
    a = np.ones(n, dtype = np.float64) 

    for i in range(0,5):
         start = timer() 
         func(a) 
         print(i, " without GPU:", timer()-start)     

    for i in range(0,5):
         start = timer() 
         func2(a) 
         print(i, " with GPU ufunc:", timer()-start) 
    
    

    threadsperblock = 1024
    blockspergrid = (a.size + (threadsperblock - 1)) // threadsperblock
    for i in range(0,5):
         start = timer() 
         func3[blockspergrid, threadsperblock](a, n) 
         print(i, " with GPU kernel:", timer()-start)