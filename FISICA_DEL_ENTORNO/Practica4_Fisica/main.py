import math
import time 
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

def main():
    client = RemoteAPIClient()
    sim = client.require('sim')

    sim.setStepping(True)
    sim.startSimulation()

    sim.addLog(sim.verbosity_scriptinfos, "Obteniendo manejadores")
    right_motor = sim.getObject("/PioneerP3DX/rightMotor")
    left_motor = sim.getObject("/PioneerP3DX/leftMotor")
    robot = sim.getObject("/PioneerP3DX")

    base_vel = 2.0
    
    pos_anterior = sim.getObjectPosition(robot, sim.handle_world)
    ultima_comprobacion_tiempo = sim.getSimulationTime()
    distancia_minima = 0.02
    intervalo_comprobacion = 0.5

    sim.addLog(sim.verbosity_scriptinfos, "Bucle principal iniciado")
    
    while True:
        # 1. CONTROL DE DIRECCIÓN 
        orientation = sim.getObjectOrientation(robot, sim.handle_world)
        angle = math.degrees(orientation[2]) 
        
        kp = 0.15  
        error = 0 - angle
        correccion = error * kp

        sim.setJointTargetVelocity(left_motor, base_vel - correccion)
        sim.setJointTargetVelocity(right_motor, base_vel + correccion)

        # 2. FUNCIÓN DE DETECCIÓN POR DISTANCIA
        tiempo_actual = sim.getSimulationTime()
        
        if tiempo_actual - ultima_comprobacion_tiempo > intervalo_comprobacion:
            pos_actual = sim.getObjectPosition(robot, sim.handle_world)
            
            distancia_recorrida = math.sqrt(
                (pos_actual[0] - pos_anterior[0])**2 + 
                (pos_actual[1] - pos_anterior[1])**2
            )

            if distancia_recorrida < distancia_minima:
                print(f"Bloqueo detectado (Avance: {distancia_recorrida:.4f}m). Parando...")
                sim.setJointTargetVelocity(left_motor, 0)
                sim.setJointTargetVelocity(right_motor, 0)
                break
            
            pos_anterior = pos_actual
            ultima_comprobacion_tiempo = tiempo_actual

        sim.step()

    time.sleep(1)
    sim.stopSimulation()
    print("Simulación terminada correctamente.")

if __name__ == "__main__":
    main()