from coppeliasim_zmqremoteapi_client import RemoteAPIClient

RECTO = 1
EMERGENCIA = -1

def main():
    state = RECTO

    client = RemoteAPIClient()
    sim = client.require('sim')

    sim.setStepping(True)
    sim.startSimulation()

    sim.addLog(sim.verbosity_scriptinfos, "Obteniendo manejadores de objetos")
    right_motor = sim.getObject("/PioneerP3DX/rightMotor")
    left_motor = sim.getObject("/PioneerP3DX/leftMotor")
    sensor = sim.getObject("/PioneerP3DX/ultrasonicSensor", {'index': 4})

    sim.addLog(sim.verbosity_scriptinfos, "Iniciando bucle principal")
    
    distancia_parada = 0.3 

    while True:

        result, distance, _, _, _ = sim.readProximitySensor(sensor)

        if result > 0 and distance < distancia_parada:
            state = EMERGENCIA

        match state:
            case 1:
                sim.setJointTargetVelocity(right_motor, 1.5)
                sim.setJointTargetVelocity(left_motor, 1.5)
            
            case -1: 
                sim.setJointTargetVelocity(right_motor, 0)
                sim.setJointTargetVelocity(left_motor, 0)
                
                mensaje = f"COLISION EVITADA. Robot detenido a: {distance:.4f} metros"
                sim.addLog(sim.verbosity_scriptinfos, mensaje)
                print(mensaje)
        
                break

            case _: 
                sim.addLog(sim.verbosity_scripterrors, "Se ha producido un error inesperado")
                break

        sim.step()

    import time
    time.sleep(2)
    sim.stopSimulation()
    print("Simulacion finalizada.")

if __name__ == "__main__":
    main()