
---

# 🤖 Guía de Inicio Rápido: Proyecto UTEC-PFC-RPi5-RA

*(Generado por Gemini AI, para usuarios no tan comunes en Linux)*

Esta guía es para configurar el entorno de trabajo en sus laptops (Ubuntu 24.04 + ROS 2 Jazzy). Sigan los pasos en orden.

## 🛑 Paso 0: Entender "Git" (Leéme primero)

Como vamos a trabajar varios en el mismo código, es **peligroso** trabajar todos directamente en la versión principal (llamada `main`). Si uno rompe algo, nos rompemos todos.

Para evitar esto, usaremos **Branches (Ramas)**.

* **¿Qué es una Branch?**: Imagina que es un "universo paralelo" del código. Puedes hacer cambios, romper cosas y experimentar en tu rama sin afectar al código principal.
* **La Regla de Oro**: Nunca trabajes directo en `main`. Crea tu propia rama.

---

## 🚀 Paso 1: Descargar el Código (Clonar)

Abre tu terminal y ejecuta estos comandos. Esto descargará el proyecto a tu computadora.

```bash
# 1. Ve a tu carpeta de usuario (o donde quieras guardar el proyecto)
cd ~

# 2. Clona el repositorio principal
git clone [https://github.com/alexander-rod-2049/UTEC-PFC-RPi5-RA.git](https://github.com/alexander-rod-2049/UTEC-PFC-RPi5-RA.git)

# 3. Entra en la carpeta del proyecto
cd UTEC-PFC-RPi5-RA

```

---

## 🛡️ Paso 2: Crear tu "Universo Paralelo" (Branch)

**¡IMPORTANTE!** Antes de cambiar cualquier línea de código, crea tu rama personal.

```bash
# Crea una rama nueva con tu nombre (ejemplo: juan-dev) y entra en ella
git checkout -b <tu-nombre>-dev

```

* **Ejemplo:** `git checkout -b rodrigo-dev`
* **¿Cómo sé si funcionó?** Escribe `git status`. Debería decir `On branch rodrigo-dev` (o el nombre que elegiste).

---

## 📦 Paso 3: Instalar Dependencias Externas

Algunas carpetas (como la cámara) no se suben al repositorio para no hacerlo pesado. Necesitamos descargarlas manualmente una sola vez.

```bash
# 1. Entra a la carpeta de código fuente
cd src

# 2. Descarga el SDK de la cámara Orbbec (necesario para compilar)
git clone [https://github.com/orbbec/OrbbecSDK_ROS2.git](https://github.com/orbbec/OrbbecSDK_ROS2.git)

# 3. Regresa a la raíz del proyecto
cd ..

```

---

## 🏗️ Paso 4: Compilar el Proyecto (Build)

Ahora vamos a traducir el código C++ y Python para que ROS 2 lo entienda.

```bash
# Ejecuta este comando desde la carpeta UTEC-PFC-RPi5-RA
colcon build --symlink-install

```

* **¿Qué hace `--symlink-install`?**: Nos ahorra tiempo. Si modifican un archivo de Python, no tendrán que volver a compilar todo de nuevo; el cambio se aplica al instante.
* **Nota:** Si ven mensajes amarillos (Warnings), no se preocupen. Si ven mensajes rojos (Errors), avisen.

---

## 🔌 Paso 5: Activar el Entorno

Para que tu terminal reconozca los comandos de nuestro robot, debes "activar" el espacio de trabajo. **Debes hacer esto en cada terminal nueva que abras.**

```bash
source install/setup.bash

```

---

## 💾 Paso 6: Guardar tus cambios (Git Flow)

Cuando hayas terminado de trabajar por hoy y quieras guardar tus cambios en la nube (GitHub):

1. **Ver qué modificaste:**
```bash
git status

```


2. **Preparar los archivos:**
```bash
git add .

```


3. **Guardar con un mensaje (Commit):**
```bash
git commit -m "Descripción breve de lo que hice"

```


4. **Subir a la nube (Push):**
*La primera vez que subas tu rama, usa este comando:*
```bash
git push --set-upstream origin <nombre-de-tu-branch>

```


*(Las siguientes veces solo bastará con `git push`).*

---

## 🆘 Resumen de Comandos Diarios

Cada vez que empiecen a trabajar:

```bash
cd UTEC-PFC-RPi5-RA
source install/setup.bash

```

**¡A programar!** 🤖