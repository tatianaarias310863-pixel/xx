
"""
Visualización en coordenadas cartesianas (x, y) del LIDAR A1M8

el escaner emite pulsos de laser infrarrojo
y mide el tiempo que tarda en reflejarse en un objeto y volver al sensor.
Con esta información, calcula la distancia al objeto y el ángulo de reflexión,
permitiendo así mapear el entorno en 2D.

El LIDAR A1M8 tiene un alcance máximo de 12 metros y una precisión de ±2 cm.
El escaneo se realiza en un rango de 0° a 360°, con una resolución angular de 1°.
El dispositivo puede operar a una velocidad de hasta 5.5 Hz (5.5 escaneos por segundo).
El puerto USB del LIDAR A1M8 está orientado hacia atrás (180°),
por lo que se realiza una corrección de 180° en los ángulos medidos para alinear el 0° hacia adelante.

El código utiliza las librerías numpy, matplotlib y rplidar para:
- Conectar y comunicarse con el LIDAR.
- Recibir y procesar los datos de escaneo.
- Visualizar los puntos en un gráfico cartesiano en tiempo real.
- Manejar la interrupción del programa de manera segura.
El programa incluye un filtro anti-saturación para ignorar mediciones inválidas
(o con calidad baja o distancia fuera del rango operativo).
"""



#--------------------IMPORTAMOS LAS LIBRERIAS QUE VAMOS A USAR -------------------------
import numpy as np
import matplotlib.pyplot as plt
from rplidar import RPLidar
import signal
import sys
# PREVIAMENTE CONOCEMOS QUE ESTE ES EL PUERTO DONDE ESTA CONECTADO NUESTRO LIDAR 

PORT = '/dev/ttyUSB0'


#ESTA FUNDIION NOS PERMITE DETENER EL LIDAR DE MANERA SEGURA 
"""
usando signal para capturar la señal de interrupción (Ctrl+C)
y ejecutar una función de limpieza antes de salir del programa.
"""

def signal_handler(sig, frame): # 
    print("\n🛑 Deteniendo LIDAR...")
    lidar.stop() #detiene el escaneo del lidar (funcion de la libreria rplidar)
    lidar.disconnect()
    plt.close('all')
    sys.exit(0)

# aqui establecemos que al recibir la señal SIGINT (Ctrl+C) se llame a la funcion signal_handler
signal.signal(signal.SIGINT, signal_handler)
# es como una interriptcion que permite detener el programa de manera segura ( vanidad)


print(f"Conectando al LIDAR en {PORT}...") #ver donde esta conectado el lidar
lidar = RPLidar(PORT, timeout=3) #
#creamos un objeto lidar de la clase RPLidar, que maneja la comunicacion con el dispositivo
try:
    lidar.stop() #detiene cualquier escaneo en curso
    print("LIDAR listo.")
except Exception as e:
    print(f"Error: {e}") #si hay un error al conectar o iniciar el lidar, se captura la excepcion y se muestra el error
    sys.exit(1)






# --- Gráfico cartesiano ---
plt.ion() #modo interactivo de matplotlib, permite actualizar el grafico en tiempo real
fig, ax = plt.subplots(figsize=(10, 10))
ax.set_xlim(-12, 12) #establecemos los limites del grafico en x e y, considerando el alcance maximo del lidar (12 metros)
ax.set_ylim(-12, 12)
ax.set_xlabel("X (metros)") 
ax.set_ylabel("Y (metros)")
ax.set_title("LIDAR A1M8")
ax.grid(True, linestyle='--', alpha=0.5) 
ax.set_aspect('equal', adjustable='box') #mantiene la proporción entre los ejes x e y

scatter = ax.scatter([], [], s=8, c='blue', alpha=0.7)
# puntos del escaneo (inicialmente vacíos)
# s: tamaño de los puntos, c: color, alpha: transparencia
lidar_point = ax.scatter([0], [0], s=100, c='red', marker='o', label='LIDAR')
#lidar_point representa la posición del LIDAR en el gráfico (origen)
print("Visualización en coordenadas cartesianas...")








#________________bucle principal para el escaneo de los datos___________________


try:
    for scan in lidar.iter_scans(max_buf_meas=2000): #iter_scans es un generador que devuelve escaneos completos del LIDAR
        x_vals, y_vals = [], [] #listas para almacenar las coordenadas x e y de los puntos escaneados

        for (quality, angle, distance) in scan: # cada medicion en el escaneo contiene calidad, angulo y distancia
            # quality: calidad de la medicion (0-15)
            # angle: angulo en grados (0-360)
            # distance: distancia en mm (0-12000)

            # filtramos para obtener solo mediciones validas

            if distance == 0 or quality == 0:
                continue
            # mediciones con distancia 0, calidad 0 son ignoradas


            # Corrección: 0° hacia adelante (opuesto al USB)
            corrected_angle = (angle + 180) % 360 # corregimos el angulo sumando 180° y usando modulo 360 para mantenerlo en el rango [0, 360)
            theta = np.radians(corrected_angle) # convertimos a radianes para las funciones trigonométricas
            r = distance / 1000.0  # convertimos distancia a metros (dividimos entre 1000)

            x = r * np.cos(theta) # convertimos de coordenadas polares (r, theta) a cartesianas (x, y)
            y = r * np.sin(theta)
            x_vals.append(x)
            y_vals.append(y)

        # Actualizar puntos
        scatter.set_offsets(np.c_[x_vals, y_vals])
        plt.pause(0.05)

except KeyboardInterrupt:
    pass
finally:
    lidar.stop()
    lidar.disconnect()
    plt.ioff()
    plt.close()
    print("Listo.")