# roomba-grid-b — Propuesta B

Robot diferencial tipo Roomba que recorre metas en una cuadrícula de 6×6 celdas de 50 cm.
ESP32 + PlatformIO, 2× BTS7960, encoders de un canal, BNO055 y una página web servida por la propia ESP32
para marcar inicio, obstáculos y metas, y elegir A\*, BFS, DFS o UCS.

## Qué hay en la carpeta

```text
roomba-grid-b/
├── platformio.ini            # entornos esp32dev (robot) y native (pruebas en la PC)
├── include/roomba_config.h          # pines, medidas del robot, ganancias y límites
├── include/roomba_settings.h        # ajustes que se cambian desde la web y se guardan en NVS
├── src/main.cpp              # arranque y tareas de FreeRTOS
├── lib/
│   ├── Bts7960/              # PWM 20 kHz, sentido, freno activo
│   ├── SingleEncoder/        # PCNT + periodo entre pulsos; el signo lo pone Motion
│   ├── Imu/                  # BNO055 IMUPLUS, reintentos, RST, offsets en NVS, rumbo continuo
│   ├── Pose/                 # x, y, θ y detección de deslizamiento
│   ├── Motion/               # control de ruedas, avance, giro, frenado
│   ├── Safety/               # atasco (por encoder) y latido del navegador
│   ├── Planner/              # Grid, BFS, DFS, UCS, A*, Held–Karp, 2-opt, primitivas (sin Arduino)
│   ├── Mission/              # máquina de estados y cola de primitivas
│   └── WebApi/               # Wi-Fi, REST y WebSocket
├── data/                     # index.html, app.js, style.css (se suben a LittleFS)
└── test/test_planner/        # 18 pruebas Unity del planificador
```

## Paso a paso

Necesitas VS Code con la extensión PlatformIO, o PlatformIO por consola. Todos los comandos se escriben
**dentro de la carpeta `roomba-grid-b`**. En VS Code: menú *Terminal → New Terminal*.

**1. Pruebas del planificador en tu laptop** (no necesita la ESP32):

```bash
pio test -e native
```

Debe terminar con `18 Tests 0 Failures`. En Windows, `native` necesita un compilador `gcc`;
si no lo tienes, córrelo desde WSL (Ubuntu) con `pip install platformio` y el mismo comando.

**2. Compilar el firmware:**

```bash
pio run -e esp32dev
```

**3. Subir el firmware** (ESP32 conectada por USB):

```bash
pio run -e esp32dev -t upload
```

**4. Subir la página web** a la memoria de la ESP32 (hazlo una vez, y cada vez que cambies algo en `data/`):

```bash
pio run -e esp32dev -t uploadfs
```

**5. Ver los mensajes de arranque:**

```bash
pio device monitor
```

Debes ver algo como:

```text
[roomba-grid-b] arrancando
[imu] BNO055 en 0x29, IMUPLUS, offsets sin guardar (calibra desde la web)
[web] AP Roomba-UAMM-B 192.168.4.1
[roomba-grid-b] listo: AP Roomba-UAMM-B 192.168.4.1
```

**6. Abrir la interfaz:** conecta el teléfono o la laptop a la red Wi-Fi `Roomba-UAMM-B`
(clave `roomba-grid`, cámbiala en `roomba_config.h`) y abre `http://192.168.4.1`.

## Primer arranque con motores (hazlo con las ruedas en el aire)

1. En la web, sección *Pruebas*: PWM izq `0.3`, der `0.3`, 1000 ms → **Probar PWM**. Las dos ruedas deben
   girar hacia adelante. Si una gira al revés, cambia `LEFT_MOTOR_INVERT` o `RIGHT_MOTOR_INVERT` en `roomba_config.h`.
2. Mira *Cuentas izq / der* en Telemetría: las dos deben subir. Si una se queda en 0, revisa el cable del
   encoder y el pull-up de 4.7 kΩ (GPIO 34 y 35 no tienen pull-up interno).
3. **Calibrar BNO055**: deja el robot quieto 3 s y luego ponlo en 6 posiciones (cada cara hacia arriba).
   Cuando giroscopio y acelerómetro lleguen a 3, se guarda solo y en el siguiente arranque dice «restaurada».
4. Robot en el piso, en una celda: marca esa celda como Inicio con la orientación real y pulsa
   **Colocar robot en el inicio marcado**. Luego prueba `AVANZAR(1)` y `GIRAR(+90°)`.
5. Con eso bien, marca metas, elige algoritmo, **Planear** y **Ejecutar**.

El botón físico (GPIO 18) hace: paro si está en marcha, ejecutar si hay un plan listo, reanudar si está pausado.
La barra espaciadora en la página también manda PARO.

## Decisiones donde la propuesta era ambigua o no coincidía con tu robot

| Tema | Propuesta | Lo que hace el código | Por qué |
| --- | --- | --- | --- |
| Dirección I²C del BNO055 | 0x28 | **0x29** (`IMU_I2C_ADDRESS`) | Así responde tu módulo |
| Radio de rueda | 35 mm | **35.7 mm** | Tu medición; k = 0.883 mm/cuenta, ≈ 566 cuentas por celda |
| Separación entre ruedas | 234 mm | **235 mm** | Tu medición |
| Encoder | PCNT en ambos flancos | **Un flanco**, 254 cuentas por vuelta | Las 254 que mediste son de un flanco. Para ambos: `ENC_BOTH_EDGES = true` y 508 cuentas |
| Arranque del BNO055 | — | Espera 800 ms y reintenta `begin()` 5 veces; si se pierde (cable flojo), reinicia por RST cada 2 s | En tu robot falla el primer intento y hubo falsos contactos |
| Ejes | «N, E, S, O» sin definir | N = +y = fila +1, E = +x = columna +1; θ = 0° al E, +90° al N; +90° = giro a la izquierda | Coincide con «E, E, E, N → GIRAR(+90°)» de la propuesta |
| Matriz de costos entre metas | Con el algoritmo elegido | Desde el inicio se usa la orientación real; entre metas, orientación libre. El recorrido final se vuelve a planear tramo por tramo con la orientación real | No se sabe con qué rumbo se llega a cada meta antes de fijar el orden |
| DFS | Orden N, E, S, O | Expande N primero, luego E, S, O | Es lo que dice la tabla |
| Nodos expandidos | Uno | Dos cifras: tramos finales y matriz de costos | La matriz pesa mucho con varias metas y ocultaría la diferencia entre algoritmos |
| Heurística de A\* | Manhattan y, opcional, + c_g·τ | Las dos, elegibles en la web | Para compararlas |
| Pérdida de latido (2 s) | «el robot frena» | Paro (freno + EN en bajo) → INACTIVO; se puede desactivar en Ajustes | Más seguro que reanudar solo |
| Ejecutar con metas inalcanzables | «pide confirmar» | La web pregunta; la API exige `{"confirmar": true}` | |
| Pruebas de primitivas | Plan de pruebas | Ruta extra `POST /api/prueba` (avanzar, girar, PWM) y botones en la web | Para las 20 repeticiones sin escribir un plan |
| Corriente IS, batería y zumbador | GPIO 36, 39, 32 y 33 | **No se usan por ahora** (quitados del código) | Decisión tuya del 23-nov. El atasco se detecta solo con el encoder |
| Ajustes en NVS | — | `include/roomba_settings.h` (no estaba en la lista de carpetas) | Lo usan Motion, Safety y la web |
| Versión de la plataforma | Sin fijar | `espressif32@6.9.0` (Arduino-ESP32 2.0.17) | Builds reproducibles; usa el driver PCNT clásico |

## Cómo funciona el control (resumen)

- **Avanzar**: perfil trapezoidal (0.30 m/s, 0.5 m/s²), últimos 10 cm a 0.08 m/s. El rumbo lo corrige el BNO055
  y el error lateral la línea entre centros de celda. Corta cuando faltan menos de 2 cuentas más la distancia de
  frenado aprendida, y aplica freno activo.
- **Girar**: PD sobre el rumbo del BNO055, con ω máx 90 °/s y α 180 °/s². Termina con |e| < 1° y la rueda quieta
  100 ms. Si se pasa, frena, espera y corrige (hasta 3 veces).
- **Sin segundo canal**: una rueda nunca invierte el sentido dentro de una primitiva; antes de cambiarlo,
  frena y espera 100 ms sin pulsos. Los pulsos que llegan después del freno se suman con el último signo.
- **Seguridad**: atasco = PWM mayor a 0.25 y sin pulsos del encoder durante 500 ms → freno y FALLA.
  Por ahora no hay sensor de corriente (pines IS), ni medición de batería, ni zumbador: vigila tú el voltaje
  y usa el fusible de 5 A. La prealimentación del PWM ya no se corrige por voltaje; el PI lo compensa.

## API (para probar con `curl` desde la laptop conectada al robot)

Todas las rutas POST esperan JSON. Si no llevan datos, manda `{}`.

```bash
curl http://192.168.4.1/api/estado
curl -X POST -H "Content-Type: application/json" -d '{"obstaculos":[[2,2],[2,3]]}' http://192.168.4.1/api/mapa
curl -X POST -H "Content-Type: application/json" -d '{"algoritmo":"astar","inicio":[0,0],"orientacion":"E","metas":[[4,4],[0,5]],"regresar":false}' http://192.168.4.1/api/plan
curl http://192.168.4.1/api/plan
curl -X POST -H "Content-Type: application/json" -d '{}' http://192.168.4.1/api/ejecutar
curl -X POST http://192.168.4.1/api/paro
```

| Ruta | Uso |
| --- | --- |
| GET /api/estado | Pose, celda, estado, velocidades, cuentas, calibración del BNO055 |
| GET, POST /api/mapa | Obstáculos `{obstaculos:[[f,c],…]}` (se guardan en NVS) |
| GET, POST /api/plan | `{algoritmo, heuristica?, inicio, orientacion, metas, regresar, obstaculos?}`; el resultado llega por `/ws` o con GET |
| POST /api/ejecutar | `{confirmar?: true}` |
| POST /api/pausa | Pausa al terminar la primitiva en curso, o reanuda |
| POST /api/paro | Freno activo y EN en bajo |
| POST /api/imu/calibrar | Calibrar y guardar offsets |
| POST /api/imu/cero | `{celda:[f,c], orientacion:"E"}`: coloca el robot y toma ese rumbo como referencia |
| GET, POST /api/config | Ganancias, velocidades, c_g; `{restablecer:true}` vuelve a `roomba_config.h` |
| POST /api/prueba | `{tipo:"avanzar",celdas:1}`, `{tipo:"girar",grados:90}` o `{tipo:"pwm",izq:0.3,der:0.3,ms:1000}` |
| POST /api/wifi | `{modo:"sta",ssid:"…",clave:"…"}` o `{modo:"ap"}`; reinicia. En estación: `roomba-b.local` |
| WS /ws | Telemetría cada 100 ms, plan, avisos. El navegador manda `{"tipo":"latido"}` cada segundo |

## Qué está probado y qué no

- **Planificador**: las 18 pruebas pasan (mapa vacío, pasillo, U, meta encerrada, 300 mapas aleatorios donde
  BFS da el mínimo de celdas, UCS = A\* con las dos heurísticas, A\* expande ≤ UCS, DFS encuentra ruta si existe,
  Held–Karp = fuerza bruta hasta 7 metas, primitivas E,E,E,N). Se corrieron con g++; falta confirmarlo con
  `pio test -e native`.
- **Firmware**: no se pudo compilar para ESP32 donde se escribió (no había acceso al registro de PlatformIO).
  Se revisaron los tipos con g++ y eso encontró y corrigió dos errores, pero **el primer paso es correr
  `pio run -e esp32dev`** y, si sale algún error, mandarlo tal cual.
- **Interfaz**: se revisó en un navegador, en laptop y en ancho de teléfono.
- **En el robot real**: nada aún. Las ganancias de `roomba_config.h` son un punto de partida; se ajustan desde la web.
