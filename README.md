# arduino_proyect

# Seguidor de Línea E-puck con Control PID

## Descripción
Este proyecto implementa un controlador PID (Proporcional-Integral-Derivativo) para un robot E-puck que sigue una línea negra sobre un fondo blanco en el simulador Webots.

## Características
- Control PID balanceado para seguimiento suave y preciso de líneas
- Gestión de pérdida de línea con memoria de última dirección conocida
- Normalización automática de sensores de suelo
- Sistema anti-windup para prevenir saturación del término integral
- Velocidades limitadas para operación segura

## Requisitos
- Webots R2023b o superior
- Robot E-puck
- Compilador C++ compatible con C++11 o superior

## Componentes del Robot

### Sensores Utilizados
- `gs0`: Sensor de suelo izquierdo
- `gs2`: Sensor de suelo derecho

Los sensores de suelo del E-puck retornan aproximadamente:
- **~1000**: Superficie blanca/clara
- **~300-400**: Superficie negra (línea)

### Actuadores
- `left wheel motor`: Motor de la rueda izquierda
- `right wheel motor`: Motor de la rueda derecha

## Parámetros del Controlador

### Constantes PID
```cpp
Kp = 3.5    // Ganancia proporcional (respuesta inmediata al error)
Ki = 0.0005 // Ganancia integral (corrección de error acumulado)
Kd = 1.2    // Ganancia derivativa (amortiguación y estabilidad)
```

### Parámetros de Velocidad
```cpp
baseSpeed = 4.0         // Velocidad base de avance
maxCorrection = 3.5     // Corrección máxima permitida
MAX_SPEED = 6.28        // Velocidad máxima del motor (rad/s)
```

### Normalización de Sensores
```cpp
sensorMax = 1000.0  // Valor en superficie blanca
sensorMin = 300.0   // Valor en superficie negra
```

## Funcionamiento del Algoritmo

### 1. Lectura y Normalización
Los valores de los sensores se normalizan al rango [0, 1], donde:
- **1** = Superficie negra (línea)
- **0** = Superficie blanca (fondo)

### 2. Cálculo del Error
```
error = rightNorm - leftNorm
```
- **Error positivo**: El sensor derecho detecta más línea → Girar a la izquierda
- **Error negativo**: El sensor izquierdo detecta más línea → Girar a la derecha

### 3. Control PID
El controlador calcula tres componentes:

**Proporcional (P)**: Respuesta inmediata al error actual
```cpp
P = Kp × error
```

**Integral (I)**: Corrige errores acumulados en el tiempo
```cpp
integral += error × ΔT
I = Ki × integral
```
*Incluye anti-windup limitando el integral a [-50, 50]*

**Derivativo (D)**: Anticipa cambios futuros
```cpp
D = Kd × (error - previousError) / ΔT
```

**Salida total**:
```cpp
pidOutput = P + I + D
```

### 4. Gestión de Pérdida de Línea
Cuando ambos sensores detectan superficie blanca (perdió la línea):
```cpp
error = lastValidError × 1.2
```
Esto permite que el robot continúe buscando la línea en la última dirección conocida.

### 5. Cálculo de Velocidades
```cpp
leftSpeed = baseSpeed - pidOutput
rightSpeed = baseSpeed + pidOutput
```

Las velocidades se limitan al rango [0, MAX_SPEED] para operación segura.

## Compilación y Ejecución

### En Webots
1. Crear un nuevo proyecto de controlador en C++
2. Copiar el código en el archivo del controlador
3. Compilar desde Webots (Build → Build)
4. Asignar el controlador al robot E-puck
5. Ejecutar la simulación

### Compilación Manual
```bash
g++ -std=c++11 -I/path/to/webots/include/controller/cpp \
    -L/path/to/webots/lib/controller \
    -lCppController -lCppDriver \
    tu_controlador.cpp -o tu_controlador
```

## Ajuste de Parámetros

### Para Mayor Velocidad
- Aumentar `baseSpeed`
- Reducir `Kp` ligeramente

### Para Giros Más Cerrados
- Aumentar `Kp`
- Aumentar `maxCorrection`

### Para Mayor Estabilidad
- Aumentar `Kd`
- Reducir `Ki`

### Para Curvas Suaves
- Reducir `Kp`
- Aumentar `Kd`

## Salida de Debug
El programa imprime información en tiempo real:
```
L:valor_izq R:valor_der | Error:X.XX | P:X.XX I:X.XX D:X.XX | PID:X.XX | Vel L:X.XX R:X.XX
```
