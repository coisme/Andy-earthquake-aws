#define MQTTCLIENT_QOS1 0
#define MQTTCLIENT_QOS2 0

#include "NTPClient.h"
#include "MQTTNetwork.h"
#include "MQTTmbed.h"
#include "MQTTClient.h"
#include "MQTT_server_setting.h"
#include "mbed-trace/mbed_trace.h"
#include "mbed_events.h"
#include "mbedtls/error.h"

#include "SingletonFXOS8700CQ.h"

#define LED_ON 0
#define LED_OFF 1

/* Flag to publish a message to the server. */
static volatile bool isPublish = false;

/* Flag to be set when received a message from the server. */
static volatile bool isMessageArrived = false;
/* Buffer size for a receiving message. */
const int MESSAGE_BUFFER_SIZE = 1024;
/* Buffer for a receiving message. */
char messageBuffer[MESSAGE_BUFFER_SIZE];

// An event queue is a very useful structure to debounce information between contexts (e.g. ISR and normal threads)
// This is great because things such as network operations are illegal in ISR, so updating a resource in a button's fall() function is not allowed
EventQueue eventQueue;
Thread thread1;

/*
 * Callback function called when a message arrived from server.
 */
void messageArrived(MQTT::MessageData& md)
{
    // Copy payload to the buffer.
    MQTT::Message &message = md.message;
    memcpy(messageBuffer, message.payload, message.payloadlen);
    messageBuffer[message.payloadlen] = '\0';

    isMessageArrived = true;
}

/***
 * ACC data
 */
int16_t mean_ax = 0;  // to be calibrated
int16_t mean_ay = 0;  // to be calibrated
int16_t stdev = 0;    // to be calibrated

uint32_t report_vector_size = 0;
/**
 * Measures acceleration and calculates holizontal size of acc vector. Called every 20 msec.
 */
int measure() {
    const int REPORT_COUNT = 50;

    static int count = 0;
    static uint32_t max_vector_size = 0;

    READING reading;
    SingletonFXOS8700CQ &sfxos = SingletonFXOS8700CQ::getInstance();
    
    if (sfxos.getInt2Triggered()){
        sfxos.setInt2Triggered(false);
        sfxos.getData(reading);
        int16_t dx = reading.accelerometer.x - mean_ax;
        int16_t dy = reading.accelerometer.y - mean_ay;
        int32_t sqsize = dx * dx + dy * dy;
        if(sqsize > max_vector_size) {
            max_vector_size = sqsize;
        }
        count++;
        if(count >= REPORT_COUNT) {
            count = 0;
            report_vector_size = max_vector_size;
            max_vector_size = 0;
            isPublish = true;
        }
    }
    else {
        printf("WARNING: Sensor was not ready in time.\r\n");
        return -1;
    }
    return 0;
}

int main(int argc, char* argv[])
{
    mbed_trace_init();
    
    const float version = 0.8;
    bool isSubscribed = false;

    NetworkInterface* network = NULL;
    MQTTNetwork* mqttNetwork = NULL;
    MQTT::Client<MQTTNetwork, Countdown>* mqttClient = NULL;

    DigitalOut led_red(LED_RED, LED_OFF);
    DigitalOut led_green(LED_GREEN, LED_ON);
    DigitalOut led_blue(LED_BLUE, LED_OFF);

    printf("HelloMQTT: version is %.2f\r\n", version);
    printf("\r\n");

    printf("Opening network interface...\r\n");
    {
        network = NetworkInterface::get_default_instance();
        if (!network) {
            printf("Unable to open network interface.\r\n");
            return -1;
        }
    }
    printf("Network interface opened successfully.\r\n");
    printf("\r\n");

    printf("Connecting to network\n");
    nsapi_size_or_error_t ret = network->connect();
    if (ret) {
        printf("Unable to connect! returned %d\n", ret);
        return -1;
    }    

    // sync the real time clock (RTC)
    NTPClient ntp(network);
    ntp.set_server("time.google.com", 123);
    time_t now = ntp.get_timestamp();
    set_time(now);
    printf("Time is now %s", ctime(&now));
  
    printf("Connecting to host %s:%d ...\r\n", MQTT_SERVER_HOST_NAME, MQTT_SERVER_PORT);
    {
        mqttNetwork = new MQTTNetwork(network);
        int rc = mqttNetwork->connect(MQTT_SERVER_HOST_NAME, MQTT_SERVER_PORT, SSL_CA_PEM,
                SSL_CLIENT_CERT_PEM, SSL_CLIENT_PRIVATE_KEY_PEM);
        if (rc != MQTT::SUCCESS){
            const int MAX_TLS_ERROR_CODE = -0x1000;
            // Network error
            if((MAX_TLS_ERROR_CODE < rc) && (rc < 0)) {
                // TODO: implement converting an error code into message.
                printf("ERROR from MQTTNetwork connect is %d.", rc);
            }
            // TLS error - mbedTLS error codes starts from -0x1000 to -0x8000.
            if(rc <= MAX_TLS_ERROR_CODE) {
                const int buf_size = 256;
                char *buf = new char[buf_size];
                mbedtls_strerror(rc, buf, buf_size);
                printf("TLS ERROR (%d) : %s\r\n", rc, buf);
            }            return -1;
        }
    }
    printf("Connection established.\r\n");
    printf("\r\n");

    printf("MQTT client is trying to connect the server ...\r\n");
    {
        MQTTPacket_connectData data = MQTTPacket_connectData_initializer;
        data.MQTTVersion = 3;
        data.clientID.cstring = (char *)MQTT_CLIENT_ID;
        data.username.cstring = (char *)MQTT_USERNAME;
        data.password.cstring = (char *)MQTT_PASSWORD;

        mqttClient = new MQTT::Client<MQTTNetwork, Countdown>(*mqttNetwork);
        int rc = mqttClient->connect(data);
        if (rc != MQTT::SUCCESS) {
            printf("ERROR: rc from MQTT connect is %d\r\n", rc);
            return -1;
        }
    }
    printf("Client connected.\r\n");
    printf("\r\n");

    // Turn off the green LED
    led_green = LED_OFF;

    printf("Client is trying to subscribe a topic \"%s\".\r\n", MQTT_TOPIC_SUB);
    {
        int rc = mqttClient->subscribe(MQTT_TOPIC_SUB, MQTT::QOS0, messageArrived);
        if (rc != MQTT::SUCCESS) {
            printf("ERROR: rc from MQTT subscribe is %d\r\n", rc);
            return -1;
        }
        isSubscribed = true;
    }
    printf("Client has subscribed a topic \"%s\".\r\n", MQTT_TOPIC_SUB);
    printf("\r\n");

    //
    // Accelerometer
    //
    SingletonFXOS8700CQ &sfxos = SingletonFXOS8700CQ::getInstance();
    // Check the connection to the accelerometer
    printf("\n\n\rFXOS8700CQ identity = %X\r\n", sfxos.getWhoAmI());
    // Enable the accelerometer
    sfxos.enable();

    // Call measure() every 20 milli-seconds.
    Ticker ticker;
    ticker.attach(eventQueue.event(&measure), 0.02);

    // Start the event queue in a separate thread so the main thread continues
    thread1.start(callback(&eventQueue, &EventQueue::dispatch_forever));


    //
    // Main loop
    //
    while(1) {
        if(!mqttClient->isConnected()){
            break;
        }
        if(mqttClient->yield(100) != MQTT::SUCCESS) {
            break;
        }
        /* Received a control message. */
        if(isMessageArrived) {
            isMessageArrived = false;
            // Now just print it.
            printf("\r\nMessage arrived:\r\n%s\r\n", messageBuffer);
        }
        /* Ready to publish accelerometer data */
        if(isPublish) {
            isPublish = false;
            static unsigned int id = 0;

            // When sending a message, LED lights blue.
            led_blue = LED_ON;

            MQTT::Message message;
            message.retained = false;
            message.dup = false;

            const size_t buf_size = 100;
            char *buf = new char[buf_size];
            message.payload = (void*)buf;

            message.qos = MQTT::QOS0;
            message.id = id++;
            sprintf(buf, "%ld,%ld", ntp.get_timestamp(), report_vector_size);
            message.payloadlen = strlen(buf)+1;
            // Publish a message.
            printf("Publishing message.\r\n");
            int rc = mqttClient->publish(MQTT_TOPIC_PUB, message);
            if(rc != MQTT::SUCCESS) {
                printf("ERROR: rc from MQTT publish is %d\r\n", rc);
            }
            printf("Message published.\r\n");
            delete[] buf;    

            led_blue = LED_OFF;
        }
    }

    printf("The client has disconnected.\r\n");

    if(mqttClient) {
        if(isSubscribed) {
            mqttClient->unsubscribe(MQTT_TOPIC_SUB);
            mqttClient->setMessageHandler(MQTT_TOPIC_SUB, 0);
        }
        if(mqttClient->isConnected()) 
            mqttClient->disconnect();
        delete mqttClient;
    }
    if(mqttNetwork) {
        mqttNetwork->disconnect();
        delete mqttNetwork;
    }
    if(network) {
        network->disconnect();
        // network is not created by new.
    }

//    exit(0);
}
