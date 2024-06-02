import numpy as np

def calibrate_magnetometer(mx, my, mz):
    mx_offset = (mx.min() + mx.max()) / 2
    my_offset = (my.min() + my.max()) / 2
    mz_offset = (mz.min() + mz.max()) / 2
    
    mx_calibrated = mx - mx_offset
    my_calibrated = my - my_offset
    mz_calibrated = mz - mz_offset
    
    mx_scale = (mx.max() - mx.min()) / 2
    my_scale = (my.max() - my.min()) / 2
    mz_scale = (mz.max() - mz.min()) / 2
    
    avg_scale = (mx_scale + my_scale + mz_scale) / 3
    
    mx_calibrated = avg_scale / (mx - mx_offset)
    my_calibrated = avg_scale / (my - my_offset)
    mz_calibrated = avg_scale / (mz - mz_offset)
    
    return mx_calibrated, my_calibrated, mz_calibrated
