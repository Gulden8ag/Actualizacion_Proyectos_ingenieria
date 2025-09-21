# Tutorial: Creación de Usuarios y Acceso Remoto por SSH en Raspberry Pi 5

## 1. Crear un usuario en Raspberry Pi

Conéctate a tu Raspberry Pi con teclado y monitor, o conéctate vía SSH usando el usuario por defecto si ya está habilitado.

```bash
# Crear un nuevo usuario
sudo adduser nombre_usuario

# Agregar el usuario al grupo sudo (permisos administrativos)
sudo usermod -aG sudo nombre_usuario
```

Durante `adduser` se pedirá la contraseña y algunos datos opcionales.

---

## 2. Habilitar el servicio SSH

En Raspberry Pi OS (Debian Bookworm):

```bash
# Habilitar el servicio SSH
sudo systemctl enable ssh
sudo systemctl start ssh

# Verificar estado
systemctl status ssh
```

---

## 3. Obtener la dirección IP de la Raspberry Pi

Ejecuta en la Raspberry Pi:

```bash
hostname -I
```

Ejemplo de salida:
```
192.168.1.45
```

Esta IP se usará para conectarse desde Windows.

---

## 4. Configurar conexión desde Windows con VS Code y Remote-SSH

### 4.1 Requisitos previos
1. Instalar [Visual Studio Code](https://code.visualstudio.com/).
2. Instalar la extensión **Remote - SSH** en VS Code.
3. Asegurarse de tener instalado el cliente **OpenSSH** en Windows:
   - En PowerShell:
   ```powershell
   Get-WindowsCapability -Online | Where-Object Name -like 'OpenSSH.Client*'
   ```

### 4.2 Configurar archivo `config` de SSH en Windows
Editar (o crear) el archivo `config` en:

```
C:\Users\TU_USUARIO\.ssh\config
```

Agregar:

```
Host rpi5
    HostName 192.168.1.45
    User nombre_usuario
    Port 22
```

---

## 5. Probar la conexión

En terminal de Windows (PowerShell o CMD):

```powershell
ssh rpi5
```

Si funciona, en **VS Code**:
1. Presiona `F1`.
2. Selecciona **Remote-SSH: Connect to Host...**.
3. Elige `rpi5`.

---

## 6. Configuración opcional: Claves SSH en lugar de contraseña

En Windows:

```powershell
ssh-keygen -t ed25519 -C "tu_correo@example.com"
ssh-copy-id nombre_usuario@192.168.1.45
```

Esto permite entrar sin escribir contraseña en cada conexión.

---

## 7. Posibles errores por protección de red universitaria

En entornos universitarios es común que la red tenga protecciones adicionales:

1. **Aislamiento de clientes (AP isolation)**: aunque la PC y la Raspberry estén en el mismo Wi-Fi, pueden estar bloqueadas para comunicarse entre sí.
   - Síntoma: `ping` falla aunque ambos tengan IP en la misma subred.

2. **Puertos bloqueados por firewall**: el puerto 22 (SSH) puede estar cerrado.
   - Síntoma: `ping` responde pero `ssh` se queda colgado o muestra "connection refused".

3. **Segmentación por VLAN**: la PC y la Raspberry pueden estar en redes diferentes sin ruta directa.
   - Síntoma: no hay conectividad aunque ambos estén en la red universitaria.

4. **Captive portal o registro obligatorio de dispositivos**: la Raspberry debe registrarse con dirección MAC antes de tener acceso a la red.

### Diagnóstico rápido
- Desde Windows:
  ```powershell
  ping 192.168.1.45
  Test-NetConnection -ComputerName 192.168.1.45 -Port 22
  ```

### Alternativas si la red bloquea SSH
- Solicitar a TI permisos para habilitar el puerto 22 entre tu PC y la Raspberry.
- Usar **Tailscale** o **ZeroTier** para crear una red privada virtual entre tus dispositivos.
- Configurar túneles reversos hacia un servidor externo (VPS).

---

## 8. Recomendación final

Antes de depender del acceso remoto en un laboratorio universitario:
- Verifica conectividad con `ping` y `Test-NetConnection`.
- Si falla, contacta al área de TI con la IP y MAC de la Raspberry, explicando que es para uso académico.
- Considera usar Tailscale si necesitas evitar solicitudes de apertura de puertos.
