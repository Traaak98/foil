#include <Arduino.h>

// Définir la structure pour stocker les commandes
struct CommandData {
    float command1;
    float command2;
    float command3;
    float command4;
    float command5;
};


void setup() {
    // Initialiser le port série à 9600 bauds
    Serial.begin(115200);
}

void loop() {
    // Envoyer un message toutes les 10 secondes
    while (Serial.available() < sizeof(CommandData)) {
        delay(10);
    }

    // Lire les données du port série
    CommandData command_data;
    command_data.command1 = 0.1;
    command_data.command2 = 0.2;
    command_data.command3 = 0.3;
    command_data.command4 = 0.4;
    command_data.command5 = 0.5;
    Serial.readBytes(reinterpret_cast<char*>(&command_data), sizeof(CommandData));
    
    // Créer une instance de la structure avec des commandes fictives
    CommandData send_data = command_data;

    // Envoyer les données sur le port série
    Serial.write(reinterpret_cast<const uint8_t*>(&send_data), sizeof(CommandData));

    // Attendre secondes avant d'envoyer le prochain message
    delay(100);
}
