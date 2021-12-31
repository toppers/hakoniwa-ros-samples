#include "camera_mqtt.hpp"
#include <cstring>
#include <mosquitto.h>
#include <stdio.h>
#include <signal.h>
#include <thread>

#define CAMERA_MQTT_ID      "mqtt/pub"
#define CAMERA_MQTT_HOST    "192.168.11.36"
#define CAMERA_MQTT_PORT    54011
#define CAMERA_MQTT_TOPIC   "mqtt_test"
#define KEPPALIBE   600
#define CLEAN_SESSION       true
#define IS_DEBUG            true
struct mosquitto *camera_mosq;
static bool is_debug;

/**
 * Brokerとの接続成功時に実行されるcallback関数
 */
static void mqtt_on_connect(struct mosquitto *mosq, void *obj, int result)
{
    if (is_debug) {
        printf("%s(%d): mosq=%p obj=%p result=%d\n", __FUNCTION__, __LINE__, (void*)mosq, obj, result);
    }
}

/**
 * Brokerとの接続を切断した時に実行されるcallback関数
 */
static void mqtt_on_disconnect(struct mosquitto *mosq, void *obj, int rc)
{
    if (is_debug) {
        printf("%s(%d): mosq=%p obj=%p result=%d\n", __FUNCTION__, __LINE__, (void*)mosq, obj, rc);
    }
}
#if 0
/**
 * BrokerにMQTTメッセージ送信後に実行されるcallback関数
 */
static void on_publish(struct mosquitto *mosq, void *userdata, int mid)
{
    mosquitto_disconnect(mosq);
}
#endif
static void endCatch(int);
static void camera_mqtt_run(void)
{
    printf("INFO: camera_mqtt_run: START\n");
    int ret;
    do {
        ret = mosquitto_loop_forever(camera_mosq, 100, 1);
    } while (ret == MOSQ_ERR_SUCCESS);
    return;
}
void camera_mqtt_init(void)
{
    is_debug = IS_DEBUG;
    signal(SIGINT , endCatch );
    mosquitto_lib_init();
    camera_mosq = mosquitto_new(CAMERA_MQTT_ID, CLEAN_SESSION, NULL);
    if (!camera_mosq) {
        fprintf(stderr, "Cannot create mosquitto object\n");
        mosquitto_lib_cleanup();
        return;
    }

    mosquitto_connect_callback_set(camera_mosq, mqtt_on_connect);
    mosquitto_disconnect_callback_set(camera_mosq, mqtt_on_disconnect);
    //mosquitto_publish_callback_set(camera_mosq, on_publish);

    if (mosquitto_connect_bind(camera_mosq, CAMERA_MQTT_HOST, CAMERA_MQTT_PORT, KEPPALIBE, NULL)) {
        fprintf(stderr, "failed to connect broker.\n");
        mosquitto_lib_cleanup();
        return;
    }
   	std::thread thr(camera_mqtt_run);
	thr.detach();
    return;
}

void camera_mqtt_publish(const char* camera_mqtt_topic_name, const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
    int size_bytes = (std::end(msg->data) - std::begin(msg->data));
    if (size_bytes < 1024) {
        return;
    }
    //printf("camera_mqtt_publish: pub %s\n", camera_mqtt_topic_name);

    mosquitto_publish(camera_mosq, NULL, camera_mqtt_topic_name, size_bytes, &msg->data[0], 0, false);
    return;
}
void hakoenv_mqtt_publish(const char* camera_mqtt_topic_name, const rule_msgs::msg::HakoEnv::SharedPtr msg)
{
    char buf[1024];
    int len = sprintf(buf, "%lu.%lu", msg->simtime / 1000000, msg->simtime % 1000000);
    //printf("simtime=%s\n", buf);
    mosquitto_publish(camera_mosq, NULL, camera_mqtt_topic_name, len, buf, 0, false);
}
static void endCatch(int sig)
{
    mosquitto_destroy(camera_mosq);
    mosquitto_lib_cleanup();
    printf("Finish: sig=%d\n", sig);
    exit(0);
}
