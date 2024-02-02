# Foil

## Z-Shell Configuration

<<<<<<< HEAD
<!-- Tutorial link -->
- [Tutorial](https://vitux.com/ubuntu-zsh-shell/)
- Theme: [jonathan](https://github.com/ohmyzsh/ohmyzsh/wiki/Themes#jonathan)
- Plugin:

  - [git](https://github.com/ohmyzsh/ohmyzsh/tree/master/plugins/git)
  - [zsh-navigation-tools](https://github.com/ohmyzsh/ohmyzsh/tree/master/plugins/zsh-navigation-tools)
  - [zsh-interative-cd](https://github.com/ohmyzsh/ohmyzsh/tree/master/plugins/zsh-interactive-cd)
=======
## Getting started
>>>>>>> ac5391e (old readme)

## Lidar

VL-P16
<!-- Tutorial ROS2? -->
[Tutorial VLP-16](http://wiki.ros.org/velodyne/Tutorials/Getting%20Started%20with%20the%20Velodyne%20VLP16)

## SBG

**Attention** : La connection au SBG se fait uniquement avec un port USB 3.0.

## TODOLIST

- [ ] Plan d'expérimentation
- [ ] 2 Antennes Ubiquiti
- [ ] 2 Antennes GNSS
- [ ] SBG IMU avec deux connecteurs
- [ ] Schéma des différents systèmes
- [ ] Plan d'alimentation
- [ ] Alimentation du NUC
- [ ] Driver ROS2 IMU
- [ ] Driver ROS2 GNSS
- [ ] Driver ROS2 Sonar/Lidar
- [ ] Driver ROS2 Sonde Accoustique
- [ ] Driver ROS2 Servomoteurs
- [ ] RC Controller
- [ ] Carte Arduino pour les PWM
- [ ] Algorithme de vol
- [ ] Algorithmes de détection
- [ ] Algorithme d'évitement
- [ ] interface ncurses terminal

### Arduino Setup

```bash
sudo apt install arduino-mk
sudo apt install seyon
sudo apt remove brltty # If no ttyUSB0
make clean all
make upload
seyon -modem /dev/ttyUSB0
# Or
minicom -D /dev/ttyUSB0 -b 115200
```

### Calibration servomotors

Use microcontroller to calibrate servomotors: *Micro Maestro 6-Channel* from *Pololu* :smile:

### Antenna setup

video link: [Antenna Setup](https://www.youtube.com/watch?v=jPwG0O03uEA)

Black Antenna is the Access Point and the White Antenna is the Station.
Username is ubnt
Password is foil

#### Access Point (NUC)

- Reset Ubiquiti by pushing button until it stops blinking
- Change Ubuntu Wired config with Manual IPv4:
  - 192.168.1.45
  - 255.255.255.0
  - 192.168.1.20
- Connect to https://192.168.1.20
- Change IP adress to 192.168.20.101
- Change Netmask to 255.255.255.0
- Change Gateway IP to 192.168.20.1
- Change Primary DNS IP to 8.8.8.8
- Change Secondary DNS IP to 8.8.4.4
- Disable IPv6
- Apply changes
- Change Ubuntu Wired config with Manual IPv4:
  - 192.168.20.45
  - 255.255.255.0
  - 192.168.20.101
- Deactivate Wireless connection
- Reactivate Wireless connection
- Check with `ip a` if new adress is ok
- Connect to https://192.168.20.101
- Change Wireless Mode to Access Point
- Change SSID to WIFI-FOIL
- Change Channel Width to 20 MHz
- Disable Check for Updates
- Change Device Name to Foil Access Point
- Change Time Zone to GMT+01:00
- Logout
- deactivate airMax
- Frequency, MHz: 2437
- Advanced -> Tick Installer EIRP Control
- Wireless -> Calculate EIRP Limit
- Increase output power
  
#### Station (PC)

- Reset Ubiquiti by pushing button until it stops blinking
- Change Ubuntu Wired config with Manual IPv4:
- Change Ubuntu Wired config with Manual IPv4:
  - 192.168.1.45
  - 255.255.255.0
  - 192.168.1.20
- Connect to https://192.168.1.20
- Change IP adress to 192.168.20.102
- Change Netmask to 255.255.255.0
- Change Gateway IP to 192.168.20.1
- Change Primary DNS IP to 8.8.8.8
- Change Secondary DNS IP to 8.8.4.4
- Disable IPv6
- Apply changes
- Change Ubuntu Wired config with Manual IPv4:
  - 192.168.20.45
  - 255.255.255.0
  - 192.168.20.102
- Disable Check for Updates
- Change Device Name to Foil Station
- Change Time Zone to GMT+01:00
- Set airMAX Priority to High
- Change Wireless Mode to Station
- Choose Select and find WIFI-FOIL and Lock to AP

## KillList

- Arduino ATMega 2560 : Morte le 05/10 au Labo ROB. Cause du décès: surtension, prend feu instantanément.
- HS-5646WP : Mort au combat le 09/10 à Guerlédan. Cause du décès: blocage mécanique du servo.
- HS-5646WP : Mort au combat le 11/10 à Guerlédan. Cause du décès: non remise à zéro après le test, décède de ses blessures durant la nuit.
- Torqeedo motor : Mort le 11/10 dans le lac de Guerlédan. Cause du décès: inconnue. Hypothèse retennue : surchauffe.
- HS-646WP : Mort le 12/10 à Guerlédan. Cause du suicide: inconnue.
- Servo Tester Pro-Tronik : Mort le 12/10 à Guerlédan. Cause du décès : grillé à cause d'un court-circuit.

## NUC

Adresse IP avec antennes: 192.168.20.47

## Camera

gst-launch-1.0 -e -v udpsrc port=5600 close-socket=false multicast-iface=false auto-multicast=true ! application/x-rtp, payload=96 ! rtpjitterbuffer ! rtph264depay ! avdec_h264 ! queue ! autovideosink

## Servo Binding

PCB order:

1. Foil Arrière
2. Gouvernail
3. Servo avant ????
4. Servo avant ????

## Servo Transport / Remise à zéro

- Foil arrière: Dévisser la petite vis entre la pièce blanche et la pièce noire. NE PAS RETIRER LA PIECE NOIRE

## UART

Renommer dans le node la sortie USB0 (sur le NUC)
