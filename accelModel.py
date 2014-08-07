# -*- coding: utf-8 -*-
"""
Created on Wed Aug 06 21:34:15 2014

@author: Brody Ringler 

NOTE: This model assumes rigid suspension and no weight transfer due to rider movement
        
"""

import math as m 
import numpy as np

#Specify bike model file to load parameters from 
bikeModel = np.genfromtxt("D:\documents\GitHub\Braking-Dynamics\CBR600RR_bikeModel.csv", delimiter=",")

CGyFactor = 0.9   #determines ratio of lat. accel to long. accel (decreasing raises CGy [requirements specify 0.9])

#Input parameters pulled from vehicle parameter file
mass = bikeModel[0,1]
whlBase = bikeModel[1,1]
rwTq_sys = bikeModel[14,1]     #powertrain limited rear wheel torque

#tire
muTire = bikeModel[3,1]
Tx = bikeModel[4,1]   #tire profile x value
Ty = bikeModel[5,1]    #tire profile y value 
#suspension system
#fComp = 0.0        #(0-10) scalar value signifying suspension stroke where 0 is fully extended and 10 is fully compressed  
#rComp = 0.0        #(0-10) scalar value signifying suspension stroke where 0 is fully extended and 10 is fully compressed
#Brake system 
Flever = bikeModel[6,1]    #force applied to brake lever by rider
Xlever = bikeModel[7,1]
Ylever = bikeModel[8,1]
Asc = bikeModel[9,1]      #cross sectional area of caliper slave cylinder
Amc = bikeModel[10,1]      #cross sectional area of master cylinder
Npist = bikeModel[11,1]    #number of caliper pistons
radRot = bikeModel[12,1]   #radius of rotor 
radWhl = bikeModel[2,1]    #radius of wheel
muRot = bikeModel[13,1]        #frictional constant between brake pads and rotor (approx.)

#vehcicle CoG location
CGy = whlBase / 2 * CGyFactor * muTire
CGx = whlBase / 2

#Braking limitations
Fin = Flever * (Xlever / Ylever)
Fout = Fin * Npist * (Asc / Amc)
brakeTq_sys = Fout*muRot*radRot
brakeF_sys = brakeTq_sys / radWhl       #system limited braking
brakeF_lift = mass * 9.81 * (CGx/CGy)   #lift limited braking
brakeF_slide = mass * 9.81 * muTire     #traction limited braking
brakeF_lim = min(brakeF_slide, brakeF_lift, brakeF_sys)
decel_lim = brakeF_lim / mass
brakeTq_lim = brakeF_lim * radWhl 

if brakeF_lim == brakeF_sys:
    brake_lim = 'by the vehicle braking system.'
elif brakeF_lim == brakeF_lift:
    brake_lim = 'due to the rear wheel lifting.'
elif brakeF_lim == brakeF_slide:
    brake_lim = 'by traction.'

#Torque limitations
rwTq_lift = mass * 9.81 * (whlBase - CGx) / CGy * radWhl  #wheelie limited rear wheel torque     
rwTq_spin = mass * 9.81 * muTire * radWhl   #wheelspin limited rear wheel torque
rwTq_lim = min(rwTq_sys, rwTq_lift, rwTq_spin)

if rwTq_lim == rwTq_sys:
    rw_lim = 'by the vehicles powertrain.'
elif rwTq_lim == rwTq_lift:
    rw_lim = 'due to a vehicle wheelie.'
elif rwTq_lim == rwTq_spin:
    rw_lim = 'by traction.'

#required lean angle due to bike CoG and muTire
leanAngle = m.atan(muTire)*180/m.pi + m.atan(Tx/(CGy-Ty))*180/m.pi 

print '' 
print 'Braking is limited', brake_lim
print 'Braking force at front wheel is limited at', brakeF_lim, 'N' 
print 'Braking torque front wheel is limited at', brakeTq_lim, 'Nm' 
#print 'Braking acceleration =', decel_lim
print '' 
print 'Acceleration is limited', rw_lim 
print 'Rear wheel torque is limited at', rwTq_lim, 'Nm' 
print ''
print 'Required lean angle =', leanAngle, 'Degrees'


'Notes'
