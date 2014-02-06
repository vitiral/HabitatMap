def temp(SOt): # in celcius
    d1 = -40.1
    d2 = .01
    return d1 + d2 * SOt
    
def hum(SOrh):
    c1 = -2.0468
    c2 = .0367
    c3 = -1.5955E-6
    return c1 + c2*SOrh + c3* (SOrh**2) # note that ** means "to the power of"


