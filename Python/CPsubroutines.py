import serial
import time

def hypertan(q):
    #input q - angle in radians
    
    from math import e 

    tanh =  (e**(2*q) - 1)/(e**(2*q) + 1)

    return tanh
    

def encodestring(inputlist,decimal_places):
    #inputs
    # Input list of [Array1, Array2, Array3, Time value]
    # Time - scalar, float or int
    #All values will be clipped to specified decimal places
    
    #unpack inputlist
    M = inputlist[0]
    P = inputlist[1]
    A = inputlist[2]
    Time = inputlist[3]
    Batt = inputlist[4]
    #sanitize the inputs (turn all into float)
    for i in range(len(M)):
        A[i] = float(A[i])
        M[i] = float(M[i])
        P[i] = float(P[i])

    Time = float(Time)
    Batt = float(Batt)
    #start the output string
    outputstring=''
    
    #decimal place string 
    decstring = '{:.' + str(decimal_places) + 'f}'
    
  
    #get length of everything
    len1 = len(M)
    len2 = len(P)
    len3 = len(A)
    
    #for each vector, convert to string and delimit with colons
    for i in range(len1):
     
        if i != len1-1:   
            outputstring += decstring.format(M[i]) + ':'
          
        else:
            outputstring += decstring.format(M[i])
           
    
    #delimit between vectors with a semi colon
    outputstring += ';'
    
    for i in range(len2):
     
        if i != len2-1:   
            outputstring += decstring.format(P[i]) + ':'
          
        else:
            outputstring += decstring.format(P[i])
           
    outputstring += ';'
    
    for i in range(len3):
     
        if i != len3-1:   
            outputstring += decstring.format(A[i]) + ':'
          
        else:
            outputstring += decstring.format(A[i])
    
    #add time 
    outputstring+=';' + decstring.format(Time) + ';' + decstring.format(Batt)
    
    #string is ready for printing
    return outputstring


def decodestring(outputstring):
#now lets decode this 

    #split string up by delimiters
    outputvectors = outputstring.split(';')
    
    #get each output
    output1 = outputvectors[0].split(':')
    output2 = outputvectors[1].split(':')
    output3 = outputvectors[2].split(':')
    output4 = outputvectors[3]
    output5 = outputvectors[4]
    
    #convert to float and place in export vector   
    vec1 = [0,0,0]
    for k in range(len(output1)):
        vec1[k] = float(output1[k])
    
    vec2 = [0,0,0]
    for k in range(len(output2)):
        vec2[k] = float(output2[k])
        
    vec3 = [0,0,0]
    for k in range(len(output3)):
        vec3[k] = float(output3[k])
        
        
    xportvec = [vec1,vec2,vec3, float(output4),float(output5)]
    
    return xportvec

def writeToSerial(inputlist, decimal_places):
    WRITECOUNT = 1
    VERBOSE = True
    ser = serial.Serial('/dev/cu.usbserial-1440', 9600, timeout = 1) # ttyACM1 for Feather board, CHANGE WITH EACH COMPUTER
    command_to_write = encodestring(inputlist, decimal_places)

    count = 0
    while count < WRITECOUNT:
        if VERBOSE:
            print ("Writing: ",  command_to_write)
        ser.write(str(command_to_write).encode('utf-8'))
        time.sleep(4)
        ser.flush() #flush the buffer
        count += 1

    ser.close()


