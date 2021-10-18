import alsaaudio
import numpy as np
import serial
import time
import threading
import sys
import pyopencl as cl

device="hw:2,1,0"
PERIOD_SIZE=128
RATE=44100*2
inp=alsaaudio.PCM(alsaaudio.PCM_CAPTURE,alsaaudio.PCM_NONBLOCK,device)
inp.setrate(RATE)
inp.setformat(alsaaudio.PCM_FORMAT_S16_LE)
inp.setperiodsize(PERIOD_SIZE)
BLOCK_SIZE=256
THREADS=32
OVER=2
CLSAMPLES=int(BLOCK_SIZE/THREADS)
BUFFER_SIZE=BLOCK_SIZE*10
VOLUME=256
a_np=np.zeros((BUFFER_SIZE+1000,4),dtype=np.uint8)
a_np=np.zeros((BUFFER_SIZE+1000,2),dtype=np.int16)
b=np.zeros((BUFFER_SIZE+1000,2),dtype=np.int16)
d=np.zeros(BUFFER_SIZE+1000,dtype=np.uint32)
print(d)
c=np.zeros((BUFFER_SIZE+1000,2),dtype=np.int16)

k=0
samples=0
newsamples=0
transfers_left=0
nb=0

volume=256
arduino=serial.Serial(sys.argv[1],timeout=0)

while 1:
    l,a=inp.read()
    if l:
      if len(a)%2==0:
          try:
              arduino.write(a)
#              a=np.frombuffer(a,dtype='int16').reshape(l,2).astype(np.float32)
#              a_np[samples:samples+l]=0.99*a.astype(np.int16)
#              print(a_np[samples:samples+l])
#              samples+=l
#              if(samples>BLOCK_SIZE):
#                  arduino.write(a_np[:BLOCK_SIZE].tobytes())
#                  samples-=BLOCK_SIZE
#                  a_np[:samples]=a_np[BLOCK_SIZE:BLOCK_SIZE+samples]
          except:
              pass
