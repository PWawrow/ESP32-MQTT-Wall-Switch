#ifndef MQTT_API_H // include guard
#define MQTT_API_H
#define MQTT_WWB_TOPIC "wawrov/sideLight/WWB"
#define MQTT_CWB_TOPIC "wawrov/sideLight/CWB"

void mqtt_app_start(void);
int mqtt_publish(char *topic, char *data);
#endif