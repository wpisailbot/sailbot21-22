# placeholder PID loop
import math
# replace angle with encoder positons or do conversions after the fact
prevErrors = [0, 0, 0, 0, 0]
currentHeelAngle = 90
desiredHeelAngle = 110
kp = 0.1
ki = 0.01
kd = 0.1

angleError = desiredHeelAngle - currentHeelAngle

prevErrors[4] = prevErrors[3]
prevErrors[3] = prevErrors[2]
prevErrors[2] = prevErrors[1]
prevErrors[1] = prevErrors[0]
prevErrors[0] = angleError

integralAngleError = sum(prevErrors)
derivativeAngleError = (angleError - prevErrors[1])/0.5 # change 0.5 to avg time between changes

errorSum = angleError * kp + integralAngleError * ki + derivativeAngleError * kd

