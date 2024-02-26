<div align="center">
  <center><h1> Foil </h1></center>
</div>

<br/>

<div align="center">

<!-- tag line -->
<h3> Projet d'autonomisation d'un kayak sur foil </h3>

<!-- primary badges -------------------------------------->
<p>
  <img src=https://img.shields.io/badge/Arduino_IDE-00979D?style=for-the-badge&logo=arduino&logoColor=white>
  <img src=https://img.shields.io/badge/Ubuntu-E95420?style=for-the-badge&logo=ubuntu&logoColor=white>
  <img src=https://img.shields.io/badge/ROS2-22314E?style=for-the-badge&logo=ros&logoColor=white>
  <img src=https://img.shields.io/badge/Python-3776AB?style=for-the-badge&logo=python&logoColor=white>
  <img src=https://img.shields.io/badge/C++-00599C?style=for-the-badge&logo=c%2B%2B&logoColor=white>
  <img src=https://img.shields.io/badge/espressif-E7352C?style=for-the-badge&logo=espressif&logoColor=white>
</p>

<p>
  <img src=https://forthebadge.com/images/badges/built-with-love.svg>
</p>

_Ludovic Mustière_ - _Apolline de Vaulchier_ - _Gwendal Crecquer_ - _Louis-Nam Gros_ - _Kevin Beaupuy_

<p>
  <img src=images/Logo_ENSTA_Bretagne.png width="256" height="83">
  <img src=images/Logo_Lab_STICC.png width="256" height="83">
</p>

</div>

## Introduction

Ce projet contient l'ensemble des codes et des configurations pour le projet de foil de l'ENSTA Bretagne. Il emporte de nombreux capteurs qu'il faut configurer et interconnecter. Le projet est basé sur ROS2 et utilise un NUC pour le traitement des données.

## Installation

### ROS2

Il suffit de suivre le tutoriel officiel de ROS2 pour installer ROS2 sur Ubuntu 22.04 disponible [ici](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html).

### Z-Shell Installation

Voici la liste de commande pour installer Z-Shell sur Ubuntu 22.04.

```bash
# Update and upgrade
sudo apt update && sudo apt dist-upgrade -y
sudo apt install build-essential curl file git

# Install Zsh
sudo apt install zsh
zsh --version

# Set Zsh as default shell
chsh -s $(which zsh)

# Install Oh My Zsh
sudo apt install git-core curl fonts-powerline
sh -c "$(curl -fsSL https://raw.github.com/robbyrussell/oh-my-zsh/master/tools/install.sh)"
```

### Z-Shell Configuration

Pour plus d'informations sur la configuration de Z-Shell, veuillez consulter le [tutoriel](https://vitux.com/ubuntu-zsh-shell/). La configuration actuelle est la suivante :

- Theme: [jonathan](https://github.com/ohmyzsh/ohmyzsh/wiki/Themes#jonathan)
- Plugins:

  - [git](https://github.com/ohmyzsh/ohmyzsh/tree/master/plugins/git)
  - [zsh-navigation-tools](https://github.com/ohmyzsh/ohmyzsh/tree/master/plugins/zsh-navigation-tools)
  - [zsh-interative-cd](https://github.com/ohmyzsh/ohmyzsh/tree/master/plugins/zsh-interactive-cd)

## Lidar

Voir [tuto_velodyne.md](./tuto_velodyne.md) pour la configuration du Velodyne.
**Adresse IP du Velodyne : 192.168.10.91**

## SBG

Ellipse D
** Attention ** : La connection au SBG se fait uniquement avec un port USB 3.0.

Récupération de la branche devel du git de sbg_systems (https://github.com/SBG-Systems/sbg_ros2_driver/tree/devel) le 14/02/2024
Modification du fichier sbg_device_uart_default.yaml pour autoriser le RTK et calibrer la sbg.

Modification de la config de la sbg pour passer la correction RTCM sur le PORT A sur sbgCenter. cf README sbg_driver
Modification sbg_driver pour passer les messages RTCM en mavros_msgs/RTCM.msg.

## RTK

ros2 launch ntrip_client ntrip_client_launch.py host:='147.100.179.214' mountpoint:='IUEM' username:='centipede' password:='centipede'

## Modifier les paramètres de foil_consigne_node :

Liste des paramètres :

- kz
- kroll
- kpitch
- kz_proportional
- kroll_proportional
- kpitch_proportional
- kyaw_proportional

```bash
ros2 param set /foil_consigne_node nom_du_parametre valeur
```

** ATTENTION ** : Mettre la valeur en double même pour les entiers (exemple : 0.0).

## Mettre une nouveau nom pour un port :

Obtenir les information sur le port :

```bash
udevadm info -a -p $(udevadm info -q path -n adresse_du_port)
```

En général les informations à retenir sont :

- idVendor
- idProduct
- kernel
- subsystem

Ensuite il faut modifier notre fichier de règle udev :

```bash
nano /etc/udev/rules.d/myrule.rules
```

Et ajouter le port en prenant exemple sur les lignes suivantes.
On suppose dans cet exemple que le port est un port ttyACM\* et que le idVendor est 1546 et le idProduct est 01a8.
On souhaite le renomer en ttyGPS et que les droits d'accès soient donnés à l'utilisateur.

```
KERNEL=="ttyACM*", SUBSYSTEM=="tty", ATTRS{idVendor}=="1546", ATTRS{idProduct}=="01a8", SYMLINK="ttyGPS", MODE="0777"
```

Puis on recharge les règles udev :

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

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

Use microcontroller to calibrate servomotors: _Micro Maestro 6-Channel_ from _Pololu_ :smile:

### Antenna setup

video link: [Antenna Setup](https://www.youtube.com/watch?v=jPwG0O03uEA)

Black Antenna is the Access Point and the White Antenna is the Station.
Username is ubnt
Password is foil

### Webcam setup

Commande gstreamer pour publier le flux vidéo de la webcam sur le réseau:

```bash
gst-launch-1.0 -v v4l2src device=/dev/video2 do-timestamp=true ! video/x-h264, width=1920, height=1080, framerate=30/1 ! h264parse ! queue ! rtph264pay config-interval=10 pt=96 ! udpsink host=adresse_ip_station port=5600
```

Commande gstreamer pour lire le flux vidéo de la webcam sur le réseau:

```bash
gst-launch-1.0 -e -v udpsrc port=5600 close-socket=false multicast-iface=false auto-multicast=true ! application/x-rtp, payload=96 ! rtpjitterbuffer ! rtph264depay ! avdec_h264 ! queue ! autovideosink
```

## Servo Binding

PCB order:

1. Foil Arrière
2. Gouvernail
3. Servo avant ????
4. Servo avant ????

## Servo Transport / Remise à zéro

- Foil arrière: Dévisser la petite vis entre la pièce blanche et la pièce noire. NE PAS RETIRER LA PIECE NOIRE

## UART

Renommer dans le node la sortie USB0 (sur le NUC)

#### Access Point

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

#### Station

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

## Clé 4G SETUP sur UBUNTU SERVEUR (ici 22.04)

- Brancher et configurer la clé 4G sur un pc possédant une version d'ubuntu similaire mais avec une interface graphique (L'outil graphique est en haut à droite et en dessous du wifi).
- Récupérer les paramètres config à l'aides des commandes suivantes :

-Installer ModemManager :

```bash
sudo apt install modemmanager
```

-Recherche du modem (il devrait apparaitre comme gsmmodem)

```bash
ls /dev
```

-Récupération des infos du modem (ici 0 parfois à changer voir avec la commande mmcli -L)

```bash
mmcli -L
mmcli -m 0
mmcli -i 3
```

-Installer NetworkManager :

```bash
sudo apt install networkmanager
```

-Récupération de la config réseau

```bash
nmcli con show
```

- Sur le pc embarqué on branche la clé.

-Installer NetworkManager et ModemManager:

```bash
sudo apt install networkmanager
sudo apt install modemmanager
```

-Recherche du modem (il devrait apparaitre comme gsmmodem)

```bash
ls /dev
mmcli -L
```

-Récupération des infos du modem (ici 0 parfois à changer voir avec la commande mmcli -L)

```bash
mmcli -m 0
```

-Configuration de l'APN (Basé sur les informations récupéré, l'APN est b2bouygtel.com) :

```bash
sudo mmcli -m 2 --create-bearer="apn='b2bouygtel.com',ip-type=ipv4v6"
```

-Établir la connexion avec les paramètres APN spécifiés.

```bash
sudo mmcli -m 0 --simple-connect="apn='b2bouygtel.com',ip-type=ipv4v6"
```

-Vérifier l'état du modem et du porteur:

```bash
mmcli -m 2
mmcli -b 2
```

-Créer et activer une connexion mobile broadband en utilisant nmcli.

```bash
sudo nmcli con add type gsm ifname "*" con-name "Bouygues4G" apn "b2bouygtel.com"
sudo nmcli con up "Bouygues4G"
```

-Vérifier l'état de la connexion

```bash
nmcli con show
```

-Tester la connectivité Internet

```bash
ping -c 4 google.com
```

## ROSBAG

sudo apt-get install ros-humble-rosbag2-storage-mcap

cf https://mcap.dev/guides/getting-started/ros-2

## KillList

- Arduino ATMega 2560 : Morte le 05/10 au Labo ROB. Cause du décès: surtension, prend feu instantanément.
- HS-5646WP : Mort au combat le 09/10 à Guerlédan. Cause du décès: blocage mécanique du servo.
- HS-5646WP : Mort au combat le 11/10 à Guerlédan. Cause du décès: non remise à zéro après le test, décède de ses blessures durant la nuit.
- Torqeedo motor : Mort le 11/10 dans le lac de Guerlédan. Cause du décès: inconnue. Hypothèse retennue : surchauffe.
- HS-646WP : Mort le 12/10 à Guerlédan. Cause du suicide: inconnue.
- Servo Tester Pro-Tronik : Mort le 12/10 à Guerlédan. Cause du décès : grillé à cause d'un court-circuit.
