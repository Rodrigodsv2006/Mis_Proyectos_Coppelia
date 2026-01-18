from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import cv2 as cv
import numpy as np

def main():
    client = RemoteAPIClient()
    sim = client.require('sim')
    sim.setStepping(True)
    sim.startSimulation()


    right_motor = sim.getObject("/PioneerP3DX/rightMotor")
    left_motor = sim.getObject("/PioneerP3DX/leftMotor")
    camera = sim.getObject("/PioneerP3DX/camera")

    fase = 1  
    center_x = 0

    print("Iniciando algoritmo")

    while True:
        raw_image, resolution = sim.getVisionSensorImg(camera)
        image = np.frombuffer(raw_image, dtype=np.uint8).reshape([resolution[1], resolution[0], 3])
        image = cv.cvtColor(image, cv.COLOR_RGB2HSV)
        image = np.fliplr(np.rot90(image, 2))

        mask = cv.bitwise_or(
            cv.inRange(image, np.array([0, 100, 100]), np.array([10, 255, 255])),
            cv.inRange(image, np.array([160, 100, 100]), np.array([180, 255, 255]))
        )

        width = resolution[0]
        hit_izquierda = np.any(mask[:, int(width * 0.10)] > 0)
        hit_derecha = np.any(mask[:, int(width * 0.90)] > 0)

        moments = cv.moments(mask)
        if moments["m00"] > 0:
            center_x = int(moments["m10"] / moments["m00"])
   
        # FASE 1: Buscar el objeto dando vueltas sobre sí mismo
        if fase == 1:
            sim.setJointTargetVelocity(left_motor, -0.3)
            sim.setJointTargetVelocity(right_motor, 0.3)
            if moments["m00"] > 500: 
                fase = 2

        # FASE 2: Encontrar el objeto 
        elif fase == 2:
            if center_x > (width * 0.5):
                print("Objeto centrado.Fase 3.")
                fase = 3

        # FASE 3: Avanzar al objeto en línea recta
        elif fase == 3:
            sim.setJointTargetVelocity(left_motor, 0.5)
            sim.setJointTargetVelocity(right_motor, 0.5)
            if hit_izquierda and hit_derecha:
                fase = 4

        # FASE 4: Detenerse antes de colisionar
        elif fase == 4:
            sim.setJointTargetVelocity(left_motor, 0)
            sim.setJointTargetVelocity(right_motor, 0)
            print("Objetivo alcanzado. Robot detenido.")

        cv.imshow("Mascara Rojo", mask)
        if cv.waitKey(5) == 27: break
        sim.step()

    sim.stopSimulation()

if __name__ == "__main__":
    main()