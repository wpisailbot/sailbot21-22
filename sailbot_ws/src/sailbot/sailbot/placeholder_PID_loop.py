# Replace angle with encoder positons or do conversions after the fact
prevErrors = [0, 0, 0, 0, 0]
errorSum = 0
currentHeelAngle = 80
desiredHeelAngle = 120
kp = 0.1
ki = 0.01
kd = 0.1


def runSim():
    
    while(1):
        calculateError()
        print("Error: " + str(errorSum))
        if (errorSum > 3):
            currentHeelAngle += 3
        elif (errorSum < -3):
            currentHeelAngle -= 3
        else:
            currentHeelAngle += errorSum
        print("")
        print("Current angle: " + str(currentHeelAngle))
        print("Desired angle: " + str(desiredHeelAngle) + "\n")
            
def calculateError():
    angleError = desiredHeelAngle - currentHeelAngle

    # make more efficient with rolling overwrite
    prevErrors[4] = prevErrors[3]
    prevErrors[3] = prevErrors[2]
    prevErrors[2] = prevErrors[1]
    prevErrors[1] = prevErrors[0]
    prevErrors[0] = angleError

    integralAngleError = sum(prevErrors)
    derivativeAngleError = (angleError - prevErrors[1])/0.5 # Change 0.5 to avg time between changes [or just delete derivative lol]

    errorSum = angleError * kp + integralAngleError * ki + derivativeAngleError * kd
    
    
runSim()
