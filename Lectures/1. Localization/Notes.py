import numpy as np

p = []

p =np.array([.2,.2,.2,.2,.2])

#also
n = 5
p = np.array([1/n]*n)




def sense(p, Z, world, pZ, pnonZ):
    Zv = world == Z
    nonZv = world != Z
    Zprob = Zv*pZ
    nonZprob = nonZv*pnonZ
    totProb = Zprob + nonZprob
    q = p*totProb
    sumq = np.sum(q)
    norm_p = q/sumq
    return norm_p

def move(p,U, pOvershoot, pUndershoot, pExact):
    states = len(p)
    all = np.zeros(states)
    for idx,x in enumerate(p):

        newp = np.zeros(states) #initialize probability vector

        #Calculate new positional indexes, taking into account cyclical world
        newp[(idx-1+U) % states] = pUndershoot*p[idx]
        newp[(idx + U) % states] = pExact*p[idx]
        newp[(idx + 1 + U) % states] = pOvershoot*p[idx]
        all = np.vstack([all,newp])

    total_prob = np.sum(all, axis=0)
    return total_prob

p = np.array([.2,.2,.2,.2,.2])

pHit = 0.6
pMiss = 0.2

world = np.array(['green' , 'red', 'red','green','green'])
Z = np.array(['red','red'])
motions = np.array([1,1])

pOvershoot = .1
pUndershoot = .1
pExact = .8

for idx,x in enumerate(Z):
    p = sense(p,x,world,0.6,0.2)
    p = move(p, motions[idx], pOvershoot, pUndershoot, pExact)



