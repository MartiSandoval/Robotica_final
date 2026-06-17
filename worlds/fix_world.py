import os
import re

# =============================================================================
# CONFIGURACIÓN
# =============================================================================
# Pon aquí el nombre exacto de tu archivo del mundo
ARCHIVO_ORIGINAL = "escenario_complejo.wbt" 
ARCHIVO_NUEVO = "escenario_corregido.wbt"

# =============================================================================
# EJECUCIÓN
# =============================================================================
def ensanchar_pasillos():
    if not os.path.exists(ARCHIVO_ORIGINAL):
        print(f"❌ Error: No encuentro el archivo '{ARCHIVO_ORIGINAL}'. Asegúrate de correr este script en la misma carpeta de tus mundos.")
        return

    with open(ARCHIVO_ORIGINAL, 'r', encoding='utf-8') as f:
        contenido = f.read()

    # 1. Mover el bloque diagonal (b2) hacia la izquierda y abajo
    # Original: translation -0.303736 -0.449011 0.13
    contenido = re.sub(
        r'translation -0\.303736 -0\.449011 0\.13\n(.*?)name "b2"', 
        r'translation -0.403736 -0.549011 0.13\n\1name "b2"', 
        contenido, flags=re.DOTALL
    )

    # 2. Mover el bloque cuadrado de la derecha (obs1(6)) más a la derecha
    # Original: translation 0.31 -0.41 0.05
    contenido = re.sub(
        r'translation 0\.31 -0\.41 0\.05\n(.*?)name "obs1\(6\)"', 
        r'translation 0.45 -0.41 0.05\n\1name "obs1(6)"', 
        contenido, flags=re.DOTALL
    )

    # 3. Mover el bloque cuadrado del centro (obs1(7)) hacia arriba a la izquierda
    # Original: translation 0.02 0.11 0.05
    contenido = re.sub(
        r'translation 0\.02 0\.11 0\.05\n(.*?)name "obs1\(7\)"', 
        r'translation -0.15 0.25 0.05\n\1name "obs1(7)"', 
        contenido, flags=re.DOTALL
    )

    # 4. Mover la pared vertical (b3) más a la derecha
    # Original: translation 0.402578 -0.214011 0.13
    contenido = re.sub(
        r'translation 0\.402578 -0\.214011 0\.13\n(.*?)name "b3"', 
        r'translation 0.502578 -0.214011 0.13\n\1name "b3"', 
        contenido, flags=re.DOTALL
    )

    with open(ARCHIVO_NUEVO, 'w', encoding='utf-8') as f:
        f.write(contenido)

    print(f"✅ ¡Éxito! Se ha creado el nuevo mundo: '{ARCHIVO_NUEVO}'")
    print("   Abre este nuevo archivo en Webots y corre tu controlador.")

if __name__ == '__main__':
    ensanchar_pasillos()