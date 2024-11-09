# import RPi.GPIO as GPIO
# from time import sleep
# GPIO.setmode(GPIO.BCM)
# GPIO.setwarnings(False)
 
 
# class Motor():
#     def __init__(self,EnaA,In1A,In2A,EnaB,In1B,In2B):
#         self.EnaA= EnaA
#         self.In1A = In1A
#         self.In2A = In2A
#         self.EnaB= EnaB
#         self.In1B = In1B
#         self.In2B = In2B
#         GPIO.setup(self.EnaA,GPIO.OUT);GPIO.setup(self.In1A,GPIO.OUT);GPIO.setup(self.In2A,GPIO.OUT)
#         GPIO.setup(self.EnaB,GPIO.OUT);GPIO.setup(self.In1B,GPIO.OUT);GPIO.setup(self.In2B,GPIO.OUT)
#         self.pwmA = GPIO.PWM(self.EnaA, 100);
#         self.pwmB = GPIO.PWM(self.EnaB, 100);
#         self.pwmA.start(0);
#         self.pwmB.start(0);
#         self.mySpeed=0
 
#     def move(self,speed=0.5,turn=0,t=0):
#         speed *=100
#         turn *=70
#         leftSpeed = speed-turn
#         rightSpeed = speed+turn
 
#         if leftSpeed>100: leftSpeed =100
#         elif leftSpeed<-100: leftSpeed = -100
#         if rightSpeed>100: rightSpeed =100
#         elif rightSpeed<-100: rightSpeed = -100
#         #print(leftSpeed,rightSpeed)
#         self.pwmA.ChangeDutyCycle(abs(leftSpeed))
#         self.pwmB.ChangeDutyCycle(abs(rightSpeed))
#         if leftSpeed>0:GPIO.output(self.In1A,GPIO.HIGH);GPIO.output(self.In2A,GPIO.LOW)
#         else:GPIO.output(self.In1A,GPIO.LOW);GPIO.output(self.In2A,GPIO.HIGH)
#         if rightSpeed>0:GPIO.output(self.In1B,GPIO.HIGH);GPIO.output(self.In2B,GPIO.LOW)
#         else:GPIO.output(self.In1B,GPIO.LOW);GPIO.output(self.In2B,GPIO.HIGH)
#         sleep(t)
 
#     def stop(self,t=0):
#         self.pwmA.ChangeDutyCycle(0);
#         self.pwmB.ChangeDutyCycle(0);
#         self.mySpeed=0
#         sleep(t)
 
# def main():
#     motor.move(0.5,0,2)
#     motor.stop(2)
#     motor.move(-0.5,0,2)
#     motor.stop(2)
#     motor.move(0,0.5,2)
#     motor.stop(2)
#     motor.move(0,-0.5,2)
#     motor.stop(2)
 
# if __name__ == '__main__':
#     motor= Motor(2,3,4,17,22,27)
#     main()


from gpiozero import Motor, PWMOutputDevice
from time import sleep

class MotorController:
    def __init__(self, EnaA, In1A, In2A, EnaB, In1B, In2B):
        # Initialize motors using gpiozero Motor and PWMOutputDevice
        self.left_motor = Motor(In1A, In2A)
        self.right_motor = Motor(In1B, In2B)
        self.pwmA = PWMOutputDevice(EnaA)
        self.pwmB = PWMOutputDevice(EnaB)
        
    def set_speed(self, speed, turn=0):
        # Calculate speed for each motor
        left_speed = speed - turn
        right_speed = speed + turn

        # Clamp speed between -1.0 and 1.0
        left_speed = max(min(left_speed, 1.0), -1.0)
        right_speed = max(min(right_speed, 1.0), -1.0)

        # Set PWM duty cycle to control speed
        self.pwmA.value = abs(left_speed)
        self.pwmB.value = abs(right_speed)

        # Set motor directions
        if left_speed > 0:
            self.left_motor.forward(left_speed)
        elif left_speed < 0:
            self.left_motor.backward(-left_speed)
        else:
            self.left_motor.stop()

        if right_speed > 0:
            self.right_motor.forward(right_speed)
        elif right_speed < 0:
            self.right_motor.backward(-right_speed)
        else:
            self.right_motor.stop()

    def move(self, speed=0.5, turn=0, t=0):
        self.set_speed(speed, turn)
        sleep(t)
        self.stop()

    def stop(self, t=0):
        self.left_motor.stop()
        self.right_motor.stop()
        sleep(t)

def main():
    motor = MotorController(EnaA=2, In1A=3, In2A=4, EnaB=17, In1B=22, In2B=27)
    
    motor.move(0.5, 0, 2)
    motor.stop(2)
    motor.move(-0.5, 0, 2)
    motor.stop(2)
    motor.move(0, 0.5, 2)
    motor.stop(2)
    motor.move(0, -0.5, 2)
    motor.stop(2)

if __name__ == '__main__':
    main()
