# Comment utiliser le LiDAR Velodyne PUCK Lite !

## I. Se connecter

Pour le connecter il faut brancher le LiDAR avec son seul câble d’alimentation, brancher le câble Ethernet au PC et créer manuellement (dans les paramètres de connexion de l’ordinateur) une adresse IPv4 similaire mais non identique à celle du LiDAR (ex : 192.168.10.91 devient 192.168.10.92). On peut vérifier sur WireShark s’il voit bien l’adresse du LiDAR en Ethernet. Une fois cela fait nous éteignons la WiFi et allons sur un moteur de recherche puis rentrons `192.168.10.91` dans la barre de recherche nous amenant sur une page pour gérer le LiDAR. Ici on change les numéros du destinataire pour bien lier les 2.
Ensuite on ouvre VeloView et on fait `Open > Sensor stream > VLP-16` et on voit s’afficher l’ensemble des points.

## II. ROS2

### II.1. Installation

Il n’y a quasi pas de documentation ROS2 pour velodyne.

Ne pas chercher de packages à installer

En effet, les fonctions velodyne sont toutes déjà incluses dans ROS2. Il est donc inutile de perdre son temps à chercher à les télécharger sur internet.

Cependant, les fonctions etc sont déjà présentes dans le ROS2 mais il faut encore les activer ! Pour cela faites :
```sudo apt install ros-humble-velodyne…```
Sur les … faites des tabs jusqu'à afficher les différents choix et complétez votre commande par celui que vous voulez. Par exemple :
```sudo apt install ros-humble-velodyne```
```sudo apt install ros-humble-velodyne-driver```
etc.

Gardez bien en tête les multiples tabs pour compléter vos commandes ! Ainsi lorsque vous écrivez ros2… cela vous mets les différentes commandes possibles avec ros2.

### II.2. Utilisation

Ensuite, en choisissant ```ros2 launch velodyne…``` vous avez pleins de choix de suite de commande. Ainsi :
```
ros2 launch velodyne velodyne-all-nodes-VLP16-launch.py
```
lance un code pour récupérer des valeurs du LiDAR.

Cependant, il est écrit qu’il le fait pour l’adresse IP `192.168.1.201` cad celle par défaut. Or notre LiDAR à pour adresse IP `192.168.10.91` donc il faut aller modifier un fichier. Celui qu’il faut modifier ce trouve ici :
`/opt/ros/humble/share/velodyne_driver/config/`
et se nomme :
`VLP16-velodyne_driver_node-params.yaml`
Il faut alors l’ouvrir dans le terminal avec :
```
sudo gedit /opt/ros/humble/share/velodyne_driver/config/VLP16-velodyne_driver_node-params.yaml
```
Cela ouvre le document dans une fenêtre où vous pourrez modifier l’adresse IP.
ATTENTION : CRÉEZ BIEN UNE COPIE AVEC `save as` AVANT DE LE MODIFIER POUR ÊTRE SÛR DE GARDER UNE VERSION QUI MARCHE !!!
Une fois l’adresse modifiée il sera possible de lancer les commandes avec votre terminal en étant bien connecté au LiDAR.
Maintenant, en lançant :
```
ros2 launch velodyne velodyne-all-nodes-VLP16-launch.py
```
Vous observez qu’il ne passe toujours pas grand chose mais en fait il faut ouvrir un nouveau terminal où lancer des commandes pour récupérer les données qui avec la commande précédente sont rendues accessibles.
Ouvrez un autre terminal et faites :
```
ros2 topic echo /velodyne_points
```
Avec cette commande vous allez voir l’affichage de valeurs en boucle.
Il y a aussi d’autres commandes commençant par ```ros2 topic echo``` / donc faites des tabs pour voir ce que ça propose.

PS : pour une raison que j’ignore mon second terminal ne connaissait pas ros2 donc j’ai du d’abord écrire ceci :
```
source /opt/ros/humble/setup.bash
```

Voilà tout ce que j’ai pour l’instant.
Si vous avez découvert des choses à rajouter, écrivez en dessous svp.

### II.3. Récupérer les données en Linux

Pour récupérer les données du LiDAR il faut d’abord lancer le LiDAR avec la commande :
```
ros2 launch velodyne velodyne-all-nodes-VLP16-launch.py
```
Puis dans un autre terminal il faut faire :
```
ros2 topic echo /velodyne_points
```
Cela affiche les données en boucle.
Pour les récupérer il faut faire :
```
ros2 topic echo /velodyne_points > nom_du_fichier.txt
```
Cela va créer un fichier texte avec les données du LiDAR.
Pour l’arrêter il faut faire Ctrl + C.

### II.4. Visualiser les données en Linux

Pour visualiser les données du LiDAR il faut d’abord lancer le LiDAR avec la commande :
```
ros2 launch velodyne velodyne-all-nodes-VLP16-launch.py
```
Puis dans un autre terminal il faut faire :
```
rviz2
```
Cela ouvre une fenêtre avec un fond noir et des options sur la gauche.
Il faut alors cliquer sur `Add` puis `By topic` et choisir `PointCloud2`.
Ensuite il faut remplacer `map` par `velodyne` et vous devriez voir les données du LiDAR s’afficher.

Il est alors possible de changer la taille des points, leur couleur, etc.

### II.5. Traiter les topics

Pour regarder le type de message renvoyé par le LiDAR il faut faire :
```
rqt
```
Cela ouvre une fenêtre avec des options sur la gauche.
Il faut alors cliquer sur `Plugins` puis `Topics` et choisir `Topic Monitor`.
Ensuite, vous pouvez sélectionner le topic qui vous intéresse et avoir toutes les informations sur le type de message renvoyé.
Ainsi, en appuyant sur `velodyne_points` vous pouvez voir que le type de message est `sensor_msgs/msg/PointCloud2`.
Vous observez aussi les types de données renvoyées par le LiDAR, leur nom, etc. Cela peut être utile pour la suite. (Voir image ci-dessous)

![rqt](rqt.png)


#############################################################################################################################################################
#############################################################################################################################################################
#############################################################################################################################################################

## III. ROS2 en Python

### III.1. Récupérer les données en Python

Pour récupérer les données du LiDAR il faut d’abord lancer le LiDAR avec la commande :
```
ros2 launch velodyne velodyne-all-nodes-VLP16-launch.py
```
Puis dans un autre terminal il faut faire :
```
ros2 run velodyne_driver velodyne_driver_node
```
Cela affiche les données en boucle.
Pour les récupérer dans un fichier texte il faut faire un fichier python avec le code suivant :
```
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import numpy as np

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/velodyne_points',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        gen = pc2.read_points(msg, skip_nans=True)
        int_data = np.array(list(gen))
        np.savetxt('nom_du_fichier.txt', int_data)

def main(args=None):
        
            rclpy.init(args=args)
        
            minimal_subscriber = MinimalSubscriber()
        
            rclpy.spin(minimal_subscriber)
        
            minimal_subscriber.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
```
Cela va créer un fichier texte avec les données du LiDAR.
Pour l’arrêter il faut faire Ctrl + C.

Pour les récupérer il faut faire :
```
python3 nom_du_fichier.py > nom_du_fichier.txt
```
Cela va créer un fichier texte avec les données du LiDAR.
Pour l’arrêter il faut faire Ctrl + C.

### III.2. Visualiser les données en Python

Pour visualiser en 3D les données du LiDAR il faut d’abord lancer le LiDAR avec la commande :
```
ros2 launch velodyne velodyne-all-nodes-VLP16-launch.py
```
Puis dans un autre terminal il faut faire :
```
ros2 run velodyne_driver velodyne_driver_node
```
Cela affiche les données en boucle.
Pour les visualiser il faut faire un fichier python avec le code suivant :
```
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/velodyne_points',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        gen = pc2.read_points(msg, skip_nans=True)
        int_data = np.array(list(gen))
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.scatter(int_data[:,0], int_data[:,1], int_data[:,2], c=int_data[:,3], cmap='viridis', s=0.1)
        plt.show()

def main(args=None):
    
        rclpy.init(args=args)
    
        minimal_subscriber = MinimalSubscriber()
    
        rclpy.spin(minimal_subscriber)
    
        minimal_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```
Cela va afficher les données du LiDAR en 3D.
Pour l’arrêter il faut faire Ctrl + C.

### IV. ROS2 en C++

### IV.1. Récupérer les données en C++

Pour récupérer les données du LiDAR il faut d’abord lancer le LiDAR avec la commande :
```
ros2 launch velodyne velodyne-all-nodes-VLP16-launch.py
```
Puis dans un autre terminal il faut faire :
```
ros2 run velodyne_driver velodyne_driver_node
```
Cela affiche les données en boucle.
Pour les récupérer dans un fichier texte il faut faire un fichier cpp avec le code suivant :
```
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

using namespace std::chrono_literals;

class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("minimal_subscriber")
    {
      subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "velodyne_points", 10, std::bind(&MinimalSubscriber::topic_callback, this, std::placeholders::_1));
    }

  private:
    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
      sensor_msgs::PointCloud2Iterator<float> iter_x(*msg, "x");
      sensor_msgs::PointCloud2Iterator<float> iter_y(*msg, "y");
      sensor_msgs::PointCloud2Iterator<float> iter_z(*msg, "z");
      sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(*msg, "r");
      sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(*msg, "g");
      sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(*msg, "b");
      for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_r, ++iter_g, ++iter_b)
      {
        RCLCPP_INFO(this->get_logger(), "    %f, %f, %f, %d, %d, %d", *iter_x, *iter_y, *iter_z, *iter_r, *iter_g, *iter_b);
      }
    }
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
```
Cela va créer un fichier texte avec les données du LiDAR.
Pour l’arrêter il faut faire Ctrl + C.

### IV.2. Visualiser les données en C++

Pour visualiser en 3D les données du LiDAR il faut d’abord lancer le LiDAR avec la commande :
```
ros2 launch velodyne velodyne-all-nodes-VLP16-launch.py
```
Puis dans un autre terminal il faut faire :
```
ros2 run velodyne_driver velodyne_driver_node
```
Cela affiche les données en boucle.
Pour les visualiser en 3D dans des graphiques il faut faire un fichier cpp avec le code suivant :
```

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

using namespace std::chrono_literals;

class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("minimal_subscriber")
    {
      subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "velodyne_points", 10, std::bind(&MinimalSubscriber::topic_callback, this, std::placeholders::_1));
    }

  private:
    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
      sensor_msgs::PointCloud2Iterator<float> iter_x(*msg, "x");
      sensor_msgs::PointCloud2Iterator<float> iter_y(*msg, "y");
      sensor_msgs::PointCloud2Iterator<float> iter_z(*msg, "z");
      sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(*msg, "r");
      sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(*msg, "g");
      sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(*msg, "b");
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
      for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_r, ++iter_g, ++iter_b)
      {
        pcl::PointXYZRGB point;
        point.x = *iter_x;
        point.y = *iter_y;
        point.z = *iter_z;
        point.r = *iter_r;
        point.g = *iter_g;
        point.b = *iter_b;
        cloud->points.push_back(point);
      }
      cloud->width = (int)cloud->points.size();
      cloud->height = 1;
      pcl::visualization::CloudViewer viewer("Cloud Viewer");
      viewer.showCloud(cloud);
      while (!viewer.wasStopped())
      {
      }
    }
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
```
Cela va afficher les données du LiDAR en 3D.
Pour l’arrêter il faut faire Ctrl + C.