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

## Table des matières

- [Introduction](#introduction)
- [Table des matières](#table-des-matières)
- [Installation](#installation)
  - [ROS2](#ros2)
  - [Z-Shell Installation](#z-shell-installation)
  - [Z-Shell Configuration](#z-shell-configuration)
  - [Lidar](#lidar)
  - [SBG](#sbg)
  - [RTK](#rtk)
  - [Arduino Setup](#arduino-setup)
  - [Rosbag Setup](#rosbag-setup)
  - [NUC Setup](#nuc-setup)
  - [NUC USB Port Configuration](#nuc-usb-port-configuration)
  - [NUC Ethernet and WiFi Configuration](#nuc-ethernet-and-wifi-configuration)
  - [Antennas Setup](#antennas-setup)
    - [Access Point](#access-point)
    - [Station](#station)
  - [Clé 4G SETUP sur UBUNTU SERVEUR (ici 22.04)](#clé-4g-setup-sur-ubuntu-serveur-ici-2204)
  - [Webcam setup](#webcam-setup)
  - [Positionnement des Servomoteurs sur le PCB](#positionnement-des-servomoteurs-sur-le-pcb)
  - [Capteurs Ultrason](#capteurs-ultrason)
- [Modifier les paramètres de foil\_consigne\_node](#modifier-les-paramètres-de-foil_consigne_node)
- [KillList](#killlist)

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

### Lidar

Voir [tuto_velodyne.md](./tuto_velodyne.md) pour la configuration du Velodyne.
**Adresse IP du Velodyne : 192.168.10.91**

### SBG

La centrale inertielle utilisée est une SBG Ellipse D. Pour la configurer, il faut utiliser le logiciel Inertial SDK disponible [ici](https://github.com/SBG-Systems/sbgECom) pour **Linux** et **MacOS** ou [ici](https://files.sbg-systems.com/s/ANQxBt55aiQjRNx/download) pour **Windows** .

Le driver ROS2 utilisé est sbg_ros2_driver. **Attention**, il faut utiliser la branche `devel` pour avoir accès à la dernière version du driver et notamment pour avoir le **RTK**. Elle est disponible sur ce [lien](https://github.com/SBG-Systems/sbg_ros2_driver/tree/devel) et peut être téléchargé avec la commande suivante. Cette opération avait été réalisée le 14/02/2024:

```bash
git clone https://github.com/SBG-Systems/sbg_ros2_driver -b devel
```

**Attention** : La connection au SBG se fait uniquement avec un port USB 3.0.

**Attention** : Pour activer le RTK, il faut modifier la configuration de la centrale inertielle sur le logiciel Inertial SDK. Il faut connecter la SBG à l'ordinateur et aller dans l'onglet `Assignment panel`, puis passer la valeur de `RTCM` à `Port A`. Pour plus de détail, se reporter aux informations disponibles [ici](./ros2_ws/src/sbg_ros2_driver/README.md).

Le fichier de configuration utilisé est le fichier `sbg_device_uart_default.yaml` disponible [ici](./ros2_ws/src/sbg_ros2_driver/config/sbg_device_uart_default.yaml).

Les différentes modification effectuées sont les suivantes :

```yaml
# Port Name
portName: "/dev/ttySBG"


# IMU_ALIGNMENT_LEVER_ARM
imuAlignementLeverArm:
  # IMU X axis direction in vehicle frame
  # 0 ALIGNMENT_FORWARD   IMU Axis is turned in vehicle's forward direction
  # 1 ALIGNMENT_BACKWARD  IMU Axis is turned in vehicle's backward direction
  # 2 ALIGNMENT_LEFT      IMU Axis is turned in vehicle's left direction
  # 3 ALIGNMENT_RIGHT     IMU Axis is turned in vehicle's right direction
  # 4 ALIGNMENT_UP        IMU Axis is turned in vehicle's up direction
  # 5 ALIGNMENT_DOWN      IMU Axis is turned in vehicle's down direction
  axisDirectionX: 0
  # IMU Y axis direction in vehicle frame
  # 0 ALIGNMENT_FORWARD   IMU Axis is turned in vehicle's forward direction
  # 1 ALIGNMENT_BACKWARD  IMU Axis is turned in vehicle's backward direction
  # 2 ALIGNMENT_LEFT      IMU Axis is turned in vehicle's left direction
  # 3 ALIGNMENT_RIGHT     IMU Axis is turned in vehicle's right direction
  # 4 ALIGNMENT_UP        IMU Axis is turned in vehicle's up direction
  # 5 ALIGNMENT_DOWN      IMU Axis is turned in vehicle's down direction
  axisDirectionY: 3
  # Residual roll error after axis alignment rad
  misRoll: 0.0
  # Residual pitch error after axis alignment rad
  misPitch: 0.0
  # Residual yaw error after axis alignment rad
  misYaw: 0.0
  # X Primary lever arm in IMU X axis (once IMU alignment is applied) m
  leverArmX: -0.25
  # Y Primary lever arm in IMU Y axis (once IMU alignment is applied) m
  leverArmY: 0.
  # Z Primary lever arm in IMU Z axis (once IMU alignment is applied) m
  leverArmZ: 0.01


# AIDING_ASSIGNMENT
# Note: GNSS1 module configuration can only be set to an external port on Ellipse-E version.
# Ellipse-N users must set this module to MODULE_INTERNAL. On the other hand, rtcmModule is only
# available for Ellipse-N users. This module must be set to MODULE_DISABLED for other users.
aidingAssignment:
  # GNSS module port assignment:
  # 255 Module is disabled
  # 1 Module connected on PORT_B
  # 2 Module connected on PORT_C
  # 3 Module connected on PORT_D
  # 5 Module is connected internally
  gnss1ModulePortAssignment: 5
  # GNSS module sync assignment:
  # 0 Module is disabled
  # 1 Synchronization is done using SYNC_IN_A pin
  # 2 Synchronization is done using SYNC_IN_B pin
  # 3 Synchronization is done using SYNC_IN_C pin
  # 4 Synchronization is done using SYNC_IN_D pin
  # 5 Synchronization is internal
  # 6 Synchronization is done using SYNC_OUT_A pin
  # 7 Synchronization is done using SYNC_OUT_B pin
  gnss1ModuleSyncAssignment: 5
  # RTCM input port assignment for Ellipse-N DGPS (see gnss1ModulePortAssignment for values)
  rtcmPortAssignment: 5


# GNSS configuration
# Note: Secondary level arms should only be considered in case of dual antenna GNSS receiver. It can be left to 0 otherwise.
gnss:
  # Gnss Model Id
  # 101 Used on Ellipse-N to setup the internal GNSS in GPS+GLONASS
  # 102 Default mode for Ellipse-E connection to external GNSS
  # 103 Used on Ellipse-N to setup the internal GNSS in GPS+BEIDOU
  # 104 Used on Ellipse-E to setup a connection to ublox in read only mode.
  # 106 Used on Ellipse-E to setup a connection to Novatel receiver in read only mode.
  # 107 Used on Ellipse-D by default
  gnss_model_id: 107

  #GNSS primary antenna lever arm in IMU X axis (m)
  primaryLeverArmX: 0.71
  #GNSS primary antenna lever arm in IMU Y axis (m)
  primaryLeverArmY: 0.
  #GNSS primary antenna lever arm in IMU Z axis (m)
  primaryLeverArmZ: -0.05
  #GNSS primary antenna precise. Set to true if the primary lever arm has been accurately entered and doesn't need online re-estimation.
  primaryLeverPrecise: true

  #GNSS secondary antenna lever arm in IMU X axis (m)
  secondaryLeverArmX: -1.25
  #GNSS secondary antenna lever arm in IMU Y axis (m)
  secondaryLeverArmY: 0.
  #GNSS secondary antenna lever arm in IMU Z axis (m)
  secondaryLeverArmZ: -0.05

  # Secondary antenna operating mode.
  # 1 The GNSS will be used in single antenna mode only and the secondary lever arm is not used.
  # 2 [Reserved] The GNSS dual antenna information will be used but the secondary lever arm is not known.
  # 3 The GNSS dual antenna information will be used and we have a rough guess for the secondary lever arm.
  # 4 The GNSS dual antenna information will be used and the secondary lever arm is accurately entered and doesn't need online re-estimation.
  secondaryLeverMode: 3


rtcm:
  # Should ros driver subscribe to RTCM topic
  subscribe: true
  # Topic on which RTCM is published
  topic_name: rtcm
  # Namespace where topic is published
  namespace: ntrip_client

nmea:
  # Should ros driver publish NMEA string
  publish: true
  # Topic on which to publish nmea data
  topic_name: nmea
  # Namespace where to publish topic
  namespace: ntrip_client
```

Enfin, il faut modifier le driver ROS2 pour qu'il prenne en compte les messages RTCM. Nous avons choisi d'utiliser le package `mavros_msgs` pour les messages RTCM. Pour cela, il faut modifier les fichiers `sbg_device` disponible [ici](./ros2_ws/src/sbg_ros2_driver/include/sbg_driver/sbg_device.h) et [ici](./ros2_ws/src/sbg_ros2_driver/include/sbg_driver/sbg_device.cpp). Les modifications sont les suivantes :

- Fichier `sbg_device.h` :

  - Ajout de l'include `#include <mavros_msgs/RTCM.h>` dans le fichier `sbg_device.h` à la ligne 43.
  - Modification du subscriber en `rclcpp::Subscription<mavros_msgs::msg::RTCM>::SharedPtr  rtcm_sub_;` à la ligne 86.
  - Modification de la fonction `void writeRtcmMessageToDevice(const mavros_msgs::msg::RTCM::SharedPtr msg);` à la ligne 213.
- Fichier `sbg_device.cpp` :

  - Modification du message à l'initialisation du subscriber à la ligne 211
  - Modification de la fonction `void SbgDevice::writeRtcmMessageToDevice(const mavros_msgs::msg::RTCM::SharedPtr msg)` à la ligne 467.

```c++
void SbgDevice::initSubscribers()
{
  if (config_store_.shouldSubscribeToRtcm())
  {
    auto rtcm_cb = [&](const mavros_msgs::msg::RTCM::SharedPtr msg) -> void {
        this->writeRtcmMessageToDevice(msg);
    };
    
    rtcm_sub_ = ref_node_.create_subscription<mavros_msgs::msg::RTCM>(config_store_.getRtcmFullTopic(), 10, rtcm_cb);
  }
}
```

Enfin, il faut modifier le fichier `CMakeLists.txt` disponible [ici](./ros2_ws/src/sbg_ros2_driver/CMakeLists.txt) pour ajouter le package `mavros_msgs` à la liste des dépendances.

### RTK

Le RTK utilisé est celui du réseau **centipede**. Pour s'y connecter, il faut utiliser le package `ntrip_client` disponible [ici](./ros2_ws/src/ntrip_client). C'est un **submodule** issu de la branche **ROS2** du git [suivant](https://github.com/LORD-MicroStrain/ntrip_client). Toute la configuration s'effectue au lancement du node. Voici un exemple de lancement du node à l'ENSTA Bretagne:

```bash
ros2 launch ntrip_client ntrip_client_launch.py host:='147.100.179.214' mountpoint:='IUEM' username:='centipede' password:='centipede'
```

Pour plus d'informations sur le réseaux centipède, veuillez consulter le [site](https://docs.centipede.fr/). Pour la position des balises, veuillez consulter la [carte intéractive](https://centipede.fr/index.php/view/map/?repository=cent&project=centipede).

Un autre réseau est disponible en cas de problème sur le premier. Il s'agit de celui de RTG2GO. Voici un exemple de lancement :
```bash
ros2 launch ntrip_client ntrip_client_launch.py host:='3.143.243.81' mountpoint:='ENSTABRE' username:='adresse_mail_valide' password:='none'
```
Pour plus d'information sur le réseau RTK2GO, veuillez consulter le [site](http://rtk2go.com/).

### Arduino Setup

Il est possible de flasher le microcontroleur avec les codes disponibles dans le submodule `kayak_foil_io`. Pour cela, il faut installer les packages suivants :

```bash
sudo apt install arduino-mk
sudo apt remove brltty # If no ttyUSB0
```

Ensuite, il suffit de se rendre dans le dossier `kayak_foil_io` et de lancer la commande `make upload` pour flasher le microcontroleur.

```bash
make clean all
make upload
```

Le paramétrage des ports se fait dans le fichier `Makefile` disponible [ici](./kayak_foil_io/Makefile). Il faut modifier la ligne `MONITOR_PORT = /dev/ttyArduino` et la ligne `̀DEVICE_PATH = /dev/ttyArduino` pour le port de l'Arduino.
Il faut aussi installer la librairie `Servo` dans le dossier `libraries` de l'Arduino IDE disponible [ici](https://github.com/arduino-libraries/Servo).

### Rosbag Setup

Pour enregistrer les données, nous utilisons un format spécial de rosbag nommé **mcap**. Pour l'installer, il suffit de suivre les instructions disponibles [ici](https://mcap.dev/guides/getting-started/ros-2).

```bash
sudo apt-get install ros-humble-rosbag2-storage-mcap
```

La sauvegarde des rosbags se fait dans deux fichier simultanément. Le premier contient toutes les informations pour le contrôle du foil et le second les informations du Lidar pour une reconstruction de l'environnement en 3D.

Pour tracer les données, le format **mcap** est compatible avec **Plotjuggler**. Plus d'informations disponibles sur leur [github](https://github.com/facontidavide/PlotJuggler).

### NUC Setup

Pour configurer le NUC, il faut installer Ubuntu Server 22.04. Pour cela, il suffit de suivre le tutoriel officiel disponible [ici](https://ubuntu.com/download/server).

### NUC USB Port Configuration

La configuration du NUC consiste à affecter les différents ports USB au bon device. Pour cela, il faut utiliser le fichier `udev` disponible [ici](./rules/my.rules). Il faut le copier dans le dossier `/etc/udev/rules.d/` et redémarrer le NUC.

```bash
#lien symbolique vers la sonde acoustique
KERNEL=="ttyUSB*", SUBSYSTEM=="tty", ATTRS{idVendor}=="0557", ATTRS{idProduct}=="2008", SYMLINK="ttyAcous", MODE="0777"

#lien symbolique vers l'INS SBG
KERNEL=="ttyUSB*", SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", SYMLINK="ttySBG", MODE="0777"

#lien symbolique vers Arduino
KERNEL=="ttyUSB*", SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", ATTRS{bcdDevice}=="8132", SYMLINK="ttyArduino", MODE="0777"

#lien symbolique vers la caméra
KERNEL=="video*", MODE="0777", OWNER="foil"

#lien symbolique vers l'ESP
KERNEL=="ttyUSB*", SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", ATTRS{bcdDevice}=="0254", SYMLINK="ttyESP", MODE="0777"
```

Si vous voulez ajouter un **nouvel élément** sur le NUC, il suffit de suivre la procédure suivante:

1. Obtenir les information sur le port :

```bash
udevadm info -a -p $(udevadm info -q path -n adresse_du_port)
```

En général les informations à retenir sont :

- idVendor
- idProduct
- kernel
- subsystem

2. Modifier notre fichier de règle udev :

```bash
nano /etc/udev/rules.d/myrule.rules
```

3. Ajouter le port en prenant exemple sur les lignes suivantes.
On suppose dans cet exemple que le port est un port ttyACM\* et que le idVendor est 1546 et le idProduct est 01a8.
On souhaite le renomer en ttyGPS et que les droits d'accès soient donnés à l'utilisateur.

```bash
KERNEL=="ttyACM*", SUBSYSTEM=="tty", ATTRS{idVendor}=="1546", ATTRS{idProduct}=="01a8", SYMLINK="ttyGPS", MODE="0777"
```

4. Recharger les règles udev :

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

### NUC Ethernet and WiFi Configuration

Il faut configurer les différentes connections Ethernet et WiFi pour que le NUC puisse se connecter à internet. Pour cela, nous vous conseillons d'utiliser le logiciel **nmtui** disponible sur Ubuntu Server. Vous pouvez l'installer avec la commande suivante :

```bash
sudo apt install network-manager
```

Il suffit ensuite de lancer l'utilitaire et d'appeler **Fabrice** ou de consulter le fichier [Ubuntu.txt](https://www.ensta-bretagne.fr/lebars/Share/Ubuntu.txt).

**ATTENTION**: **NE TOUCHER A CELA QU'EN DERNIER RECOURS!!!**

<div align="center">
  <p>
    <img src=images/meme_reseaux.jpg width="250" height="830">
  </p>
</div>


### Antennas Setup

Deux Ubiquiti sont utilisées pour la communication entre votre PC et le NUC. Actuellement, l'antenne noire est configurée en **Access Point** et l'antenne blanche en **Station**. Le nom du réseau est **WIFI-FOIL**. Le nom d'utilisateur est **ubnt** et le mot de passe est **foil**.

Les adresses IP sont les suivantes :

- Access Point : 192.168.20.101
- Station : 192.168.20.102

Pour les configurer, il faut suivre les tutoriels suivants ou sur la vidéo [suivante](https://www.youtube.com/watch?v=jPwG0O03uEA) :

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

### Clé 4G SETUP sur UBUNTU SERVEUR (ici 22.04)

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

### Webcam setup

Commande gstreamer pour publier le flux vidéo de la webcam sur le réseau:

```bash
gst-launch-1.0 -v v4l2src device=/dev/video2 do-timestamp=true ! video/x-h264, width=1920, height=1080, framerate=30/1 ! h264parse ! queue ! rtph264pay config-interval=10 pt=96 ! udpsink host=adresse_ip_station port=5600
```

Commande gstreamer pour lire le flux vidéo de la webcam sur le réseau:

```bash
gst-launch-1.0 -e -v udpsrc port=5600 close-socket=false multicast-iface=false auto-multicast=true ! application/x-rtp, payload=96 ! rtpjitterbuffer ! rtph264depay ! avdec_h264 ! queue ! autovideosink
```
Commande gstreamer pour enregistrer en .mp4 le flux vidéo de la webcam sur un ordinateur:

```bash
gst-launch-1.0 -e -v udpsrc port=5600 close-socket=false multicast-iface=false auto-multicast=true ! application/x-rtp, payload=96 ! rtpjitterbuffer ! rtph264depay ! h264parse ! mp4mux ! filesink location=guerledan.mp4
```

### Positionnement des Servomoteurs sur le PCB

PCB order:

1. RF
2. Servo avant gauche
3. Servo avant droit
4. Gouvernail
5. Foil Arrière

**Attention** : Lors du transport et du flash de la carte ATMega, il faut débrancher mechaniquement les servomoteurs pour éviter tout blocage mechanique.

### Capteurs Ultrason

Se référer au schéma suivant pour le branchement des capteurs ultrason :

<div align="center">
  <p>
    <img src=images/schema_capteurs.png width="1000" height="1000">
  </p>
</div>

**Attention** : En cas de problèmes avec les capteurs ultrason, adressez-vous à notre meilleur <a href="mailto:gwendal.crecquer@ensta-bretagne.org?subject=Les Capteurs Ultrasons ne marchent pas">élément</a>

## Modifier les paramètres de foil_consigne_node

Liste des paramètres :

- kpitch_
- kspeed_
- kroll_

```bash
ros2 param set /foil_consigne_node nom_du_parametre valeur
```

**Attention** : Mettre la valeur en double même pour les entiers (exemple : 0.0).

## KillList

- Arduino ATMega 2560 : Morte le 05/10 au Labo ROB. Cause du décès: surtension, prend feu instantanément.
- HS-5646WP : Mort au combat le 09/10 à Guerlédan. Cause du décès: blocage mécanique du servo.
- HS-5646WP : Mort au combat le 11/10 à Guerlédan. Cause du décès: non remise à zéro après le test, décède de ses blessures durant la nuit.
- Torqeedo motor : Mort le 11/10 dans le lac de Guerlédan. Cause du décès: inconnue. Hypothèse retennue : surchauffe.
- HS-646WP : Mort le 12/10 à Guerlédan. Cause du suicide: inconnue.
- Servo Tester Pro-Tronik : Mort le 12/10 à Guerlédan. Cause du décès: grillé à cause d'un court-circuit.
- ESP32 : Mort lors de la préparation pour Guerlédan. Cause du décès: Grillé à cause d'un court-circuit.
- ESP32 : Mort lors de la préparation pour Guerlédan. Cause du décès: Grillé à cause d'un court-circuit.
- Capteur Ultrason droit : Mort lors de la première mise à l'eau le 06/02 à Guerlédan. Cause du décès: Marsouinage.
- Capteur Ultrason gauche : Mort lors du premier virage le 06/02 à Guerlédan. Cause du décès: Marsouinage.
- Arduino ATMega 2560 : Morte le 09/10 à Guerlédan. Cause du décès: Arrachement du port série.
- Cable d'antenne pour SBG : Mort lors des tests avant le Grand Retour. Cause du décès: Décapité par Grorobotix.
