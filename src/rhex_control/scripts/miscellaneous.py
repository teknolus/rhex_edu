import numpy as np
import re

def constrain_angle(angle):
    x = (angle+np.pi) % (2*np.pi)
    x[x<0] += 2*np.pi
    return x - np.pi


def remove_spaces(str):
    # Remove unnecessary spaces around commas
    str = re.sub(r'\s*,\s*', ', ', str)

    # Remove spaces within square brackets
    str = re.sub(r'\[\s*', '[', str)
    str = re.sub(r'\s*\]', ']', str)

    # Replace consecutive spaces with a single space
    str = re.sub(r'\s+', ' ', str)
    
    return str
