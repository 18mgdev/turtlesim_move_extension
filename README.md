# 🐢 Turtlesim Controller - ROS2

Este repositorio contiene una extensión del simulador **Turtlesim** de ROS2, diseñada para proporcionar un control avanzado de la tortuga virtual mediante un teclado. La aplicación incluye funcionalidades adicionales como habilitar/deshabilitar el pintado, reiniciar la posición y borrar la pantalla, además de un archivo de lanzamiento dedicado que simplifica la ejecución de los nodos.

## ✨ Funcionalidades

- **🎮 Control de movimiento con teclas**:
  - **W**: Avanza hacia adelante.
  - **S**: Retrocede.
  - **A**: Gira hacia la izquierda.
  - **D**: Gira hacia la derecha.
- **🖌️ Habilitar/deshabilitar pintado**: Usa la tecla **ESPACIO** para activar o desactivar el pintado.
- **🧹 Borrar pantalla**: Presiona **C** para borrar todo lo dibujado.
- **🔄 Reiniciar posición**: Presiona **R** para mover la tortuga al centro de la pantalla y orientarla hacia arriba.
- **⚙️ Parámetros dinámicos**:
  - `speed`: Ajusta la velocidad de la tortuga en tiempo de ejecución.
  - `log_level`: Cambia el nivel de logging dinámicamente.

## 📋 Requisitos

- **ROS2 (Foxy o superior)** instalado en Ubuntu.
- **Dependencias de Python**:
  - [`pynput`](https://pynput.readthedocs.io/): Biblioteca para capturar eventos del teclado.
  
  Si no está instalada, puedes hacerlo con:
  ```bash
  pip3 install pynput
  ```

## 🚀 Instalación

1. Clona este repositorio en el *workspace* de ROS2:
   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/18mgdev/turtlesim_move_extension.git
   ```

2. Compila el paquete:
   ```bash
   cd ~/ros2_ws
   colcon build
   ```

3. Fuente el *workspace*:
   ```bash
   source install/setup.bash
   ```

## 🐾 Uso

### 1. 🏁 Ejecutar la aplicación

Usa el archivo de lanzamiento para iniciar el simulador Turtlesim junto con el controlador:

```bash
ros2 launch turtlesim_move_extension_pkg turtle_launch.py
```

### 2. 🔧 Cambiar parámetros dinámicos

Puedes modificar los parámetros en tiempo de ejecución utilizando los siguientes comandos:

- Cambiar la velocidad de la tortuga:
  ```bash
  ros2 param set /turtle_control speed 3.5
  ```

- Cambiar el nivel de logging:
  ```bash
  ros2 param set /turtle_control log_level DEBUG
  ```

## 📂 Estructura del Proyecto

```
turtlesim_move_extension_pkg/
├── launch/
│   ├── turtle_launch.py    # Archivo de lanzamiento
├── turtlesim_move_extension_pkg/
│   ├── turtle_control.py   # Nodo principal de control
├── package.xml             # Declaración de dependencias
├── setup.py                # Configuración de instalación
```
