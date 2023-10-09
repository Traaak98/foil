# Foil

## Z-Shell Configuration

<!-- Tutorial link -->
- [Tutorial](https://vitux.com/ubuntu-zsh-shell/)
- Theme: [jonathan](https://github.com/ohmyzsh/ohmyzsh/wiki/Themes#jonathan)
- Plugin:

  - [git](https://github.com/ohmyzsh/ohmyzsh/tree/master/plugins/git)
  - [zsh-navigation-tools](https://github.com/ohmyzsh/ohmyzsh/tree/master/plugins/zsh-navigation-tools)
  - [zsh-interative-cd](https://github.com/ohmyzsh/ohmyzsh/tree/master/plugins/zsh-interactive-cd)

## Lidar

VL-P16
<!-- Tutorial ROS2? -->
[Tutorial VLP-16](http://wiki.ros.org/velodyne/Tutorials/Getting%20Started%20with%20the%20Velodyne%20VLP16)

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
sudo apt remove brltty
make clean all
make upload
seyon -modem /dev/ttyUSB0
```

### Calibration servomotors

Use microcontroller to calibrate servomotors: *Micro Maestro 6-Channel* from *Pololu* :smile:
