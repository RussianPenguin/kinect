#!/usr/bin/env python
from freenect import sync_get_depth as get_depth, sync_get_video as get_video
import cv
import numpy as np
  
def doloop():
    global depth, rgb
    while True:
        # Get a fresh frame
        (depth,_) = get_depth()
        print depth
        # Simple Downsample
        cv.WaitKey(5)
        
doloop()
