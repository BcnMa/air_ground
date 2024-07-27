string sender  #sender of fault indication message
string label   #fault label
bool fault     #is fault
bool fatal     #is fatal
float32 timeout #s
                #no fault message is received after timeout, the fault has been eliminated 
                #if timeout is 0, indicates that the fault will not time out