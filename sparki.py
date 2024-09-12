import sim
import time

class SparkiRobot:
    def __init__(self, port=19999, robot_name='Full_Robot__v2_recovered_', motor_L='Left_Gear', motor_R='Right_Gear'):
        # Estabelece a conexão com o simulador e obtém handles do robô e motores
        self.clientID, self.robot, self.motorL, self.motorR = self.connect_to_simulator(port, robot_name, motor_L, motor_R)
        self.position = [0, 0, 0]  # Inicializa a posição do robô

    def connect_to_simulator(self, port, robot_name, motor_L, motor_R):
        # Fecha qualquer conexão anterior e inicia uma nova
        sim.simxFinish(-1)
        clientID = sim.simxStart('127.0.0.1', port, True, True, 2000, 5)
        if clientID == -1:
            raise Exception("Falha na conexão com CoppeliaSim")
        
        # Obtém handles do robô e dos motores
        returnCode, robot_handle = sim.simxGetObjectHandle(clientID, robot_name, sim.simx_opmode_blocking)
        print(f"Handle do robô returnCode: {returnCode}")

        returnCode, motorL_handle = sim.simxGetObjectHandle(clientID, motor_L, sim.simx_opmode_blocking)
        print(f"Motor esquerdo returnCode: {returnCode}")

        returnCode, motorR_handle = sim.simxGetObjectHandle(clientID, motor_R, sim.simx_opmode_blocking)
        print(f"Motor direito returnCode: {returnCode}")

        return clientID, robot_handle, motorL_handle, motorR_handle

    def get_position(self):
        # Obtém a posição atual do robô no simulador
        sim.simxGetObjectPosition(self.clientID, self.robot, -1, sim.simx_opmode_streaming)
        time.sleep(0.1)  # Aguarda o simulador atualizar a posição
        returnCode, position = sim.simxGetObjectPosition(self.clientID, self.robot, -1, sim.simx_opmode_buffer)
        self.position = position
        return position

    def set_velocities(self, left_velocity, right_velocity):
        # Define as velocidades das rodas esquerda e direita
        sim.simxSetJointTargetVelocity(self.clientID, self.motorL, left_velocity, sim.simx_opmode_streaming)
        sim.simxSetJointTargetVelocity(self.clientID, self.motorR, right_velocity, sim.simx_opmode_streaming)
        # Cinemática diferencial: a velocidade do robô é calculada a partir das velocidades das rodas

    def stop_robot(self):
        # Para o robô definindo a velocidade das rodas como 0
        self.set_velocities(0, 0)

    def wait(self, seconds):
        # Aguarda um número específico de segundos
        time.sleep(seconds)


# Exemplo de uso
if __name__ == "__main__":
    sparki = SparkiRobot()
    
    sparki.set_velocities(25, 25)  # Define velocidades das rodas para 25 unidades
    sparki.wait(3)  # Aguarda 3 segundos
    sparki.stop_robot()  # Para o robô
    
    # Exibe a posição do robô após o movimento
    print("Posição atual do robô: ", sparki.get_position())
