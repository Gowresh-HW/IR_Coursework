from controller import Robot
from datetime import datetime
import math
import numpy as np


class Controller:
    def __init__(self, robot):        
        # Robot Parameters
        self.robot = robot
        self.time_step = 32 # ms
        self.max_speed = 1  # m/s
 
        # Enable Motors
        self.left_motor = self.robot.getDevice('left wheel motor')
        self.right_motor = self.robot.getDevice('right wheel motor')
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)
        self.velocity_left = 0
        self.velocity_right = 0
    
        # Enable Proximity Sensors
        self.proximity_sensors = []
        for i in range(8):
            sensor_name = 'ps' + str(i)
            self.proximity_sensors.append(self.robot.getDevice(sensor_name))
            self.proximity_sensors[i].enable(self.time_step)
       
        # Enable Ground Sensors
        self.left_ir = self.robot.getDevice('gs0')
        self.left_ir.enable(self.time_step)
        self.center_ir = self.robot.getDevice('gs1')
        self.center_ir.enable(self.time_step)
        self.right_ir = self.robot.getDevice('gs2')
        self.right_ir.enable(self.time_step)
        
        # Data
        self.inputs = []
        self.dist = []
        self.inputsPrevious = []
        
        # Flag
        self.flag_turn = 0
        
    def clip_value(self,value,min_max):
        if (value > min_max):
            return min_max;
        elif (value < -min_max):
            return -min_max;
        return value;

    def sense_compute_and_actuate(self):
          
        if(len(self.inputs) > 0 and len(self.inputsPrevious) > 0):
            # Check for any possible collision
            print(self.dist[0], self.dist[2], self.dist[5], self.dist[7])
            if(self.inputs[0] < 0.4 and self.inputs[1] < 0.4 and self.inputs[2] < 0.4):
                print("Clue Detected");
                self.flag_turn = 1;
            if(self.dist[0] < 0.2 and self.dist[7] < 0.2):
                print("No obs in front");
                # Time
                time = datetime.now()
                #print("({} - {}) Object or walls detected!".format(time.second, time.microsecond))
                if(np.max(self.dist[0:2]) > 0.2):
                    #self.flag_turn = 1
                    self.velocity_left = -0.3;
                    self.velocity_right = 0.3;
                elif(np.max(self.dist[6:7]) > 0.2):
                    self.velocity_left = 0.3;
                    self.velocity_right = -0.3;
                else:
                    self.velocity_left = 1;
                    self.velocity_right = 1;
                    
            elif(self.dist[0] > 0.1 and self.dist[2] > 0.1 and self.dist[5] > 0.1 and self.dist[7] > 0.1):
                print("Stop");
                self.velocity_left = 0;
                self.velocity_right = 0;
                
            elif(self.dist[0] > 0.2 and self.dist[7] > 0.2) :
                if(self.flag_turn):
                    print("Turn Right");
                    self.velocity_left = 0.3;
                    self.velocity_right = -0.3;
                else:
                    print("Turn Left");
                    self.velocity_left = -0.3;
                    self.velocity_right = 0.3;
            
            
           
     
        self.left_motor.setVelocity(self.velocity_left)
        self.right_motor.setVelocity(self.velocity_right)

    def run_robot(self):        
        # Main Loop
        count = 0;
        inputs_avg = []
        while self.robot.step(self.time_step) != -1:
            
            # Read Ground Sensors
            self.inputs = []
            self.dist = []
            left = self.left_ir.getValue()
            center = self.center_ir.getValue()
            right = self.right_ir.getValue()

            # Adjust Values
            min_gs = 0
            max_gs = 1000
            if(left > max_gs): left = max_gs
            if(center > max_gs): center = max_gs
            if(right > max_gs): right = max_gs
            if(left < min_gs): left = min_gs
            if(center < min_gs): center = min_gs
            if(right < min_gs): right = min_gs
            
            # Save Data
            self.inputs.append((left-min_gs)/(max_gs-min_gs))
            self.inputs.append((center-min_gs)/(max_gs-min_gs))
            self.inputs.append((right-min_gs)/(max_gs-min_gs))
            #print("Ground Sensors \n    left {} center {} right {}".format(self.inputs[0],self.inputs[1],self.inputs[2]))
            
            # Read Distance Sensors
            for i in range(8):
                if(i==0 or i==1 or i==2 or i==3 or i==4 or i==5 or i==6 or i==7):        
                    temp = self.proximity_sensors[i].getValue()
                    #print(temp)
                    # Adjust Values
                    min_ds = 0
                    max_ds = 2400
                    if(temp > max_ds): temp = max_ds
                    if(temp < min_ds): temp = min_ds
                    # Save Data
                    self.inputs.append((temp-min_ds)/(max_ds-min_ds))
                    self.dist.append((temp-min_ds)/(max_ds-min_ds))
                    #print("Distance Sensors - Index: {}  Value: {}".format(i,self.proximity_sensors[i].getValue()))
                    #print(self.dist)
      
            # Smooth filter (Average)
            smooth = 30
            if(count == smooth):
                inputs_avg = [sum(x) for x in zip(*inputs_avg)]
                self.inputs = [x/smooth for x in inputs_avg]
                # Compute and actuate
                self.sense_compute_and_actuate()
                # Reset
                count = 0
                inputs_avg = []
                self.inputsPrevious = self.inputs
            else:
                inputs_avg.append(self.inputs)
                count = count + 1
                
            
if __name__ == "__main__":
    my_robot = Robot()
    controller = Controller(my_robot)
    controller.run_robot()
    