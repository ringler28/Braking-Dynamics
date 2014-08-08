"""
Created on Wed Aug 06 21:34:15 2014

@author: Brody Ringler 

NOTE: This model assumes rigid suspension and no weight transfer due to rider movement
        
"""



import math as m 
import numpy as np
import matplotlib.pyplot as plt


#Specify bike model file to load parameters from 
bikeModel = np.genfromtxt("D:\documents\GitHub\Braking-Dynamics\CBR600RR_bikeModel.csv", delimiter=",")

cg_Max_pctVariation = 0.25 #specifies max pct shift of CG location
CGyFactor_options = np.array([-1.0, -0.9, -0.8, -0.7, -0.6, -0.5, -0.4, -0.3, -0.2, -0.1, 0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0])   #Scalar value that shifts CG up or down(max shift is cg_Max_pctVariation [-1 is low and 1 is high]) 
CGxFactor_options = np.array([-1.0, -0.9, -0.8, -0.7, -0.6, -0.5, -0.4, -0.3, -0.2, -0.1, 0.0,0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0])   #Scalar value that shifts CG forward or backwards(max shift is cg_Max_pctVariation [-1 is forward and 1 is backwards])
whlBase_options = np.array([1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7])


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

rwTorque_array = np.array([])
rwTorque_Sysarray = np.array([])
rwTorque_Spinarray = np.array([])
rwTorque_Liftarray = np.array([])
brakeTorque_array = np.array([])
brakeTorque_Liftarray = np.array([])
brakeTorque_Slidearray = np.array([])
brakeTorque_Sysarray = np.array([])
leanAngle_array = np.array([])
CGy_array = np.array([])
CGx_array = np.array([])
CGxFactor_array = np.array([])
CGyFactor_array = np.array([])
whlBase_array = np.array([])
minLean_array = np.array([])
optimalWhlBase_array = np.array([])

for w in whlBase_options: 
    whlBase = w
    
    for y in CGyFactor_options:
        #vehcicle CoG location
        CGy = whlBase / (2 * muTire)
        CGx = whlBase / 2
        y_pctShift = y * cg_Max_pctVariation 
        CGy = CGy + CGy*y_pctShift              
            
        for x in CGxFactor_options:
            #vehcicle CoG location
            CGx = whlBase / 2            
            x_pctShift = x*cg_Max_pctVariation            
            CGx = CGx + CGx * x_pctShift
                
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
                brakeTorque_Sysarray = np.append(brakeTorque_Sysarray, brakeTq_lim)
                brakeTorque_Slidearray = np.append(brakeTorque_Slidearray, 0.)
                brakeTorque_Liftarray = np.append(brakeTorque_Liftarray, 0.)
            elif brakeF_lim == brakeF_lift:
                brake_lim = 'due to the rear wheel lifting.'
                brakeTorque_Liftarray = np.append(brakeTorque_Liftarray, brakeTq_lim)
                brakeTorque_Slidearray = np.append(brakeTorque_Slidearray, 0.)
                brakeTorque_Sysarray = np.append(brakeTorque_Sysarray, 0.)
            elif brakeF_lim == brakeF_slide:
                brake_lim = 'by traction.'
                brakeTorque_Slidearray = np.append(brakeTorque_Slidearray, brakeTq_lim)
                brakeTorque_Liftarray = np.append(brakeTorque_Liftarray, 0.)
                brakeTorque_Sysarray = np.append(brakeTorque_Sysarray, 0.)
        
            #Torque limitations
            rwTq_lift = mass * 9.81 * (whlBase - CGx) / CGy * radWhl  #wheelie limited rear wheel torque     
            rwTq_spin = mass * 9.81 * muTire * radWhl   #wheelspin limited rear wheel torque
            rwTq_lim = min(rwTq_sys, rwTq_lift, rwTq_spin)
        
            if rwTq_lim == rwTq_sys:
                rw_lim = 'by the vehicles powertrain.'
                rwTorque_Sysarray = np.append(rwTorque_Sysarray, rwTq_lim)
                rwTorque_Spinarray = np.append(rwTorque_Spinarray, 0.)
                rwTorque_Liftarray = np.append(rwTorque_Liftarray, 0.)
            elif rwTq_lim == rwTq_lift:
                rw_lim = 'due to a vehicle wheelie.'
                rwTorque_Liftarray = np.append(rwTorque_Liftarray, rwTq_lim)
                rwTorque_Sysarray = np.append(rwTorque_Sysarray, 0.)
                rwTorque_Spinarray = np.append(rwTorque_Spinarray, 0.)
            elif rwTq_lim == rwTq_spin:
                rw_lim = 'by traction.'
                rwTorque_Spinarray = np.append(rwTorque_Spinarray, rwTq_lim)
                rwTorque_Liftarray = np.append(rwTorque_Liftarray, 0.)
                rwTorque_Sysarray = np.append(rwTorque_Sysarray, 0.)
                
            #required lean angle due to bike CoG and muTire
            leanAngle = m.atan(muTire)*180/m.pi + m.atan(Tx/(CGy-Ty))*180/m.pi 
                
            CGy_array = np.append(CGy_array, CGy)
            CGx_array = np.append(CGx_array, CGx)
            CGyFactor_array = np.append(CGyFactor_array, y_pctShift)
            CGxFactor_array = np.append(CGxFactor_array, x_pctShift)
            whlBase_array = np.append(whlBase_array, whlBase)
            rwTorque_array = np.append(rwTorque_array, rwTq_lim)
            brakeTorque_array = np.append(brakeTorque_array, brakeTq_lim)
            leanAngle_array = np.append(leanAngle_array, leanAngle)
                
    CG_array = np.column_stack((CGx_array, CGy_array))
    CGpctShift_array = np.column_stack((CGxFactor_array, CGyFactor_array))
  
'''''''''
    #plots lean angle for each wheelbase option
    plt.figure('Required lean Angle when wheelbase = ' + str(whlBase) + 'm.')
    plt.scatter(CGpctShift_array[:, 0], CGpctShift_array[:, 1], c = leanAngle_array, marker = 's', s = 200, vmin = min(leanAngle_array), vmax = max(leanAngle_array))
    plt.colorbar()
    plt.title('Required lean Angle when wheelbase = ' + str(whlBase) + 'm.')
    plt.xlabel('CG x-distance % shift')
    plt.ylabel('CG y-height % shift')
'''''''''

#plots rear wheel torque and braking
plt.figure('Effects of CG location shift')
plt.subplot(211)
plt.scatter(CGpctShift_array[:, 0], CGpctShift_array[:, 1], c = rwTorque_array, marker = 's', s = 200, vmin = min(rwTorque_array), vmax = max(rwTorque_array))
plt.colorbar()
plt.title('Rear Wheel Torque')
plt.xlabel('CG x-distance % shift')
plt.ylabel('CG y-height % shift')
plt.subplot(212)
plt.scatter(CGpctShift_array[:, 0], CGpctShift_array[:, 1], c = brakeTorque_Liftarray, marker = 's', s = 200, vmin = min(brakeTorque_array), vmax = max(brakeTorque_array))
plt.colorbar()
plt.subplots_adjust(hspace = 0.4)
plt.title('Braking Torque')
plt.xlabel('CG x-distance % shift')
plt.ylabel('CG y-height % shift')


#determines optimal cg location and wheelbase for maxmimum torques smallest leanangle 
combinedTorque = rwTorque_array + brakeTorque_Liftarray
Torque_CGlookup = np.column_stack((combinedTorque, CGpctShift_array, leanAngle_array))
maxCombinedTorque = max(Torque_CGlookup[:,0])


k = 0
n = 0
for i in Torque_CGlookup[:,0]:
    if Torque_CGlookup[k,0] == maxCombinedTorque:
        n = k
    k += 1


print ''   
print 'Optimization of useable torque in both propulsion and braking yields:'
if Torque_CGlookup[n,1] > 0.0:
    print 'C.o.G. x-location shifted ' + str(abs(Torque_CGlookup[n,1])*100) + '% reaward' 
elif Torque_CGlookup[n,1] < 0.0:
    print 'C.o.G. x-location shifted ' + str(abs(Torque_CGlookup[n,1])*100) + '% forward' 
elif Torque_CGlookup[n,1] == 0.0:
    print 'C.o.G. x-location remains centered at x = w/2' 
if Torque_CGlookup[n,2] > 0.0:
    print 'C.o.G. y-location shifted ' + str(abs(Torque_CGlookup[n,2])*100) + '% upward' 
elif Torque_CGlookup[n,2] < 0.0:
    print 'C.o.G. y-location shifted ' + str(abs(Torque_CGlookup[n,2])*100) + '% downward' 
elif Torque_CGlookup[n,2] == 0.0:
    print 'C.o.G. y-location remains unshifted at y= w/(2mu)'   
print 'Lean angle required = ' + str(Torque_CGlookup[n,3]) + 'degrees'
print ''

#combined torque and braking plot
#tqplt = plt.figure('Torque Limitations due to CG changes')
#plt.scatter(CG_array[:, 0], CG_array[:, 1], c = rwTorque_array, marker = 's', s = 200, vmin = min(rwTorque_array), vmax = max(rwTorque_array))
#plt.colorbar()
#plt.scatter(CG_array[:, 0], CG_array[:, 1], c = brakeTorque_array, marker = 'o', s = 75, vmin = min(brakeTorque_array), vmax = max(brakeTorque_array))
#tqplt.add_subplot(111).set_xlabel('CG x-distance')
#tqplt.add_subplot(111).set_ylabel('CG y-height')


