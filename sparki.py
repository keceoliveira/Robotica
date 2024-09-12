import sim
import time

class SparkiRobot:
    def __init__(self, port=19999, robot_name='Full_Robot__v2_recovered_', motor_L='Left_Gear', motor_R='Right_Gear'):
        self.clientID, self.robot, self.motorL, self.motorR = self.connect_to_simulator(port, robot_name, motor_L, motor_R)
        self.position = [0, 0, 0]  

    def connect_to_simulator(self, port, robot_name, motor_L, motor_R):
        sim.simxFinish(-1)  # Fecha qualquer conexão anterior
        clientID = sim.simxStart('127.0.0.1', port, True, True, 2000, 5)
        if clientID == -1:
            raise Exception("Falha na conexão com CoppeliaSim")
        
        # handle
        returnCode, robot_handle = sim.simxGetObjectHandle(clientID, robot_name, sim.simx_opmode_blocking)
        print(f"Handle do robô returnCode: {returnCode}")

        # handle do motor esquerdo
        returnCode, motorL_handle = sim.simxGetObjectHandle(clientID, motor_L, sim.simx_opmode_blocking)
        print(f"Motor esquerdo returnCode: {returnCode}")

        # handle do motor direito
        returnCode, motorR_handle = sim.simxGetObjectHandle(clientID, motor_R, sim.simx_opmode_blocking)
        print(f"Motor direito returnCode: {returnCode}")

        return clientID, robot_handle, motorL_handle, motorR_handle

    def get_position(self):
        sim.simxGetObjectPosition(self.clientID, self.robot, -1, sim.simx_opmode_streaming)
        time.sleep(0.1)
        returnCode, position = sim.simxGetObjectPosition(self.clientID, self.robot, -1, sim.simx_opmode_buffer)
        self.position = position
        return position

    def set_velocities(self, left_velocity, right_velocity):
        sim.simxSetJointTargetVelocity(self.clientID, self.motorL, left_velocity, sim.simx_opmode_streaming)
        sim.simxSetJointTargetVelocity(self.clientID, self.motorR, right_velocity, sim.simx_opmode_streaming)

    def stop_robot(self):
        self.set_velocities(0, 0)

    def wait(self, seconds):
        time.sleep(seconds)


# Exemplo de uso
if __name__ == "__main__":
    sparki = SparkiRobot()
    
    sparki.set_velocities(25, 25)
    sparki.wait(3)
    sparki.stop_robot()
    
    # Posição do robô após o movimento
    print("Posição atual do robô: ", sparki.get_position())
