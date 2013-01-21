# This is a message that holds data about the pixel locations of detected features.
# These are to be used for visual servoing.
#
# Defined by Isura Ranatunga (isura@ieee.org) - 12/03/2012
#
# Each element is uniquely identified by its name
# The header specifies the time at which the features were recorded. All the features
# in one message have to be recorded at the same time.
#
# This message consists of a multiple arrays, one for each component of the image information. 
# The goal is to make each of the fields optional. When e.g. you do not have
# camera callibration values, you can leave the callibration array empty.
#
# All arrays in this message should have the same size, or be empty.
# This is the only way to uniquely associate the state name with the correct
# states.


Header header

float64[] points
float64 fKu
float64 fKv
float64 u0
float64 v0
float64 Z
string state
