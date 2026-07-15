# Dispositivo conectado para la estimación de cubierta vegetal

Código fuente desarrollado como parte del Trabajo Fin de Grado:

**Diseño y evaluación de un dispositivo conectado para la estimación del porcentaje de cubierta vegetal mediante procesamiento de imágenes**

Grado en Ingeniería en Tecnologías de Telecomunicación  
Universidad Católica San Antonio de Murcia (UCAM)

## Descripción

Este proyecto implementa un dispositivo conectado capaz de capturar imágenes y estimar localmente el porcentaje de cubierta vegetal mediante técnicas de procesamiento digital de imágenes.

El prototipo está basado en la M5Stack TimerCamera-F, que integra un microcontrolador ESP32 y un sensor de imagen OV3660. El sistema captura imágenes en formato JPEG, las convierte a RGB888 y analiza una región de interés para clasificar los píxeles como vegetación o fondo.

Los resultados pueden consultarse mediante una interfaz web local y enviarse a la plataforma IoT Thinger.io para su almacenamiento, visualización y control remoto.

## Archivo principal

El código completo del proyecto se encuentra en el archivo:

`codigo_base_funcional_v9.ino`

Esta es la versión final utilizada en el Trabajo Fin de Grado.

## Hardware utilizado

- M5Stack TimerCamera-F.
- Microcontrolador ESP32.
- Sensor de imagen OV3660.
- Memoria PSRAM de 8 MB.
- Conectividad Wi-Fi.
- Batería integrada.
- Temporizador RTC para el funcionamiento con Deep Sleep.

## Entorno de desarrollo

- Arduino IDE.
- Lenguaje C/C++.
- Perfil de placa: `M5TimerCAM / M5Stack-Timer-CAM`.

## Librerías principales

El proyecto utiliza las siguientes librerías:

- `esp_camera.h`
- `img_converters.h`
- `WiFi.h`
- `WiFiProv.h`
- `WebServer.h`
- `ThingerESP32.h`
- `ThingerConsole.h`
- `esp_sleep.h`
- `esp_timer.h`
- `Preferences.h`

Las librerías relacionadas con la cámara, la conectividad Wi-Fi, el aprovisionamiento Bluetooth y el modo Deep Sleep forman parte del entorno ESP32. Las librerías de Thinger.io deben instalarse desde el gestor de bibliotecas de Arduino IDE.

## Estructura general del código

El programa se organiza en los siguientes bloques funcionales:

1. Definición de constantes y parámetros de configuración.
2. Configuración del sensor de imagen OV3660.
3. Gestión de la memoria PSRAM.
4. Configuración y conexión a la red Wi-Fi mediante aprovisionamiento Bluetooth y almacenamiento en memoria NVS.
5. Conexión con la plataforma Thinger.io.
6. Captura de imágenes en formato JPEG.
7. Conversión de las imágenes a RGB888.
8. Configuración de la región de interés.
9. Clasificación de píxeles mediante Exceso de Verde normalizado.
10. Clasificación de píxeles mediante el espacio de color HSV.
11. Cálculo del porcentaje de cubierta vegetal.
12. Generación de la máscara binaria.
13. Generación de la superposición sobre la imagen original.
14. Implementación de la interfaz web local.
15. Envío de resultados y variables de estado a Thinger.io.
16. Funcionamiento automático.
17. Funcionamiento con Deep Sleep.
18. Conservación de parámetros mediante memoria RTC.

## Configuración inicial

Antes de compilar el programa deben sustituirse las etiquetas genéricas correspondientes a la plataforma Thinger.io por los datos de cada usuario.

### Credenciales de Thinger.io

Deben configurarse:

- Nombre de usuario.
- Identificador del dispositivo.
- Credencial del dispositivo.

Por motivos de seguridad, las credenciales reales utilizadas durante el desarrollo no se incluyen en este repositorio.

### Configuración de la red Wi-Fi

Las credenciales de la red Wi-Fi no se introducen directamente en el código.

En el primer arranque, el dispositivo inicia un proceso de aprovisionamiento mediante Bluetooth Low Energy. El nombre y la contraseña de la red se introducen desde la aplicación **ESP BLE Provisioning**, desarrollada por Espressif.

Una vez comprobada la conexión, las credenciales se almacenan en la memoria NVS del ESP32 y se reutilizan en los siguientes arranques.

El dispositivo aparece en la aplicación con el nombre `PROV_TimerCamF` y utiliza el PIN o Proof of Possession `CAMBIAR1`. Se recomienda sustituir este valor genérico por uno propio antes de utilizar el sistema.

## Carga del programa en la placa

Para cargar el programa en la M5Stack TimerCamera-F deben seguirse estos pasos:

1. Instalar Arduino IDE.
2. Instalar el soporte para placas ESP32.
3. Instalar las librerías necesarias para Thinger.io.
4. Abrir el archivo `codigo_base_funcional_v9.ino`.
5. Introducir las credenciales propias de Thinger.io.
6. Conectar la M5Stack TimerCamera-F al ordenador mediante USB.
7. Seleccionar en Arduino IDE la placa `M5TimerCAM / M5Stack-Timer-CAM`.
8. Seleccionar el puerto correspondiente.
9. Compilar el programa.
10. Cargar el programa en el dispositivo.
11. Abrir el monitor serie para comprobar la inicialización.
12. Abrir la aplicación **ESP BLE Provisioning** en un teléfono móvil.
13. Seleccionar el dispositivo `PROV_TimerCamF`.
14. Introducir el PIN o Proof of Possession configurado en el código.
15. Seleccionar una red Wi-Fi de 2,4 GHz e introducir su contraseña.
16. Esperar a que el dispositivo guarde las credenciales y se reinicie.
17. Consultar en el monitor serie la dirección IP asignada para acceder a la interfaz web local.

## Funcionamiento general

### Captura y procesamiento

La cámara captura inicialmente una imagen en formato JPEG. Posteriormente, el programa convierte la imagen a RGB888 para acceder a los componentes rojo, verde y azul de cada píxel.

El análisis puede realizarse sobre la imagen completa o sobre una región de interés configurable.

### Algoritmos de detección

El sistema permite seleccionar dos métodos de segmentación:

- Exceso de Verde normalizado.
- Segmentación mediante el espacio de color HSV.

A partir de la clasificación se obtiene:

- Número de píxeles clasificados como vegetación.
- Número total de píxeles analizados.
- Porcentaje de cubierta vegetal.

### Resultados visuales

El sistema genera:

- Imagen original.
- Máscara binaria.
- Superposición de los píxeles detectados sobre la imagen original.
- Porcentaje de cubierta vegetal.

### Interfaz web local

La interfaz web local permite:

- Realizar capturas manuales.
- Consultar el porcentaje calculado.
- Visualizar la imagen original.
- Visualizar la máscara binaria.
- Visualizar la superposición.
- Seleccionar el algoritmo.
- Ajustar los umbrales de ExG y HSV.
- Configurar la región de interés.
- Activar o detener el modo automático.
- Consultar el nivel de batería y el tiempo activo.

Para acceder a la interfaz, el ordenador o dispositivo móvil debe encontrarse conectado a la misma red Wi-Fi que la TimerCamera-F.

### Integración con Thinger.io

El dispositivo envía a Thinger.io los principales resultados y variables de estado:

- Porcentaje de cubierta vegetal.
- Píxeles clasificados como vegetación.
- Número total de píxeles analizados.
- Algoritmo seleccionado.
- Nivel de batería.
- Tiempo activo.
- Estado del modo automático.
- Estado de Deep Sleep.
- Contadores de capturas y despertares.

Las mediciones pueden almacenarse en un data bucket y visualizarse mediante indicadores y gráficas en un dashboard.

### Funcionamiento automático

El sistema permite realizar mediciones automáticas con distintos intervalos de tiempo.

En el modo automático normal, el dispositivo permanece encendido y conectado a la red entre capturas.

### Funcionamiento con Deep Sleep

En el modo Deep Sleep, el dispositivo:

1. Despierta mediante el temporizador RTC.
2. Recupera la configuración almacenada.
3. Se conecta a la red Wi-Fi.
4. Se conecta a Thinger.io.
5. Captura y procesa una imagen.
6. Envía los resultados.
7. Permanece activo durante una ventana configurable.
8. Vuelve a entrar en Deep Sleep.

Los parámetros principales y los contadores se conservan entre ciclos mediante memoria RTC.

## Seguridad

El código publicado utiliza etiquetas genéricas en lugar del nombre de usuario, el identificador y la credencial reales de Thinger.io.

Las credenciales de la red Wi-Fi tampoco se incluyen directamente en el código. Se introducen mediante la aplicación ESP BLE Provisioning y se almacenan en la memoria NVS del ESP32.

Cada usuario debe introducir sus propias credenciales de Thinger.io, configurar su red Wi-Fi mediante el proceso de aprovisionamiento y sustituir el PIN genérico por uno adecuado.

## Autor

Álvaro Martínez López

## Contexto académico

Trabajo Fin de Grado  
Grado en Ingeniería en Tecnologías de Telecomunicación  
Universidad Católica San Antonio de Murcia  
Julio de 2026
