#define MQTTCLIENT_QOS1 0
#define MQTTCLIENT_QOS2 0

#include "NTPClient.h"
#include "MQTTNetwork.h"
#include "MQTTmbed.h"
#include "MQTTClient.h"
#include "MQTT_server_setting.h"
#include "mbed-trace/mbed_trace.h"
#include "mbed_events.h"

#include "SingletonFXOS8700CQ.h"

#define LED_ON 0
#define LED_OFF 1

static volatile bool isPublish = false;

/*
 * Callback function called when a message arrived from server.
 */
void messageArrived(MQTT::MessageData& md)
{
    /* TODO: Move printf to outside interrupt context. */
    MQTT::Message &message = md.message;
    printf("\r\n");
    printf("! Message arrived: qos %d, retained %d, dup %d, packetid %d\r\n",
            message.qos, message.retained, message.dup, message.id);
    printf("! Payload %.*s\r\n", message.payloadlen, (char*)message.payload);
    printf("\r\n");
}

void btn1_rise_handler() {
    isPublish = true;
}

void calibrate(double *pmean, double *pdeviation)
{
    READING reading;
    const int calibrationPeriod = 10;    // in seconds
    SingletonFXOS8700CQ &sfxos = SingletonFXOS8700CQ::getInstance();
    
    double *data = new double[calibrationPeriod * 50];
    int i;
    double sum = 0.0;
    double mean = 0.0;
    double temp;
    
    printf("Calibrating...\r\n");
    
    i = calibrationPeriod * 50;
    
    while (i > 0)
    {
        if (sfxos.getInt2Triggered())
        {
            sfxos.setInt2Triggered(false);
            sfxos.getData(reading);
            data[i] = sqrt((double)(reading.accelerometer.x * reading.accelerometer.x) + (reading.accelerometer.y * reading.accelerometer.y));
//            data[i] = reading.accelerometer.x + reading.accelerometer.y;
            printf("x=%d\t\ty=%d\t\tsum=%f\r\n", reading.accelerometer.x, reading.accelerometer.y, data[i]);
            sum += data[i];
            --i;
        }
        else
        {
            printf("WARNING: Sensor was not ready in time during calibration\r\n");
        }
        wait_ms(20);
    }
    
    mean = (double)sum / (double)(calibrationPeriod * 50);
    
    for (i = 0; i < calibrationPeriod * 50; i++)
    {
        temp += ((float)data[i] - mean) * ((float)data[i] - mean);
    }
    
    temp /= (double)(calibrationPeriod * 50);
    
    delete [] data;
    
    *pmean = mean;
    *pdeviation = sqrt(temp);
    
    printf("Calibration complete - mean=%f; devation=%f\r\n", *pmean, *pdeviation);
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
//        network = easy_connect(true);    // If true, prints out connection details.
        network = NetworkInterface::get_default_instance();
        if (!network) {
            printf("Unable to open network interface.\r\n");
            return -1;
        }
    }
    printf("Network interface opened successfully.\r\n");
    printf("\r\n");

    // sync the real time clock (RTC)
    NTPClient ntp(network);
    ntp.set_server("time.google.com", 123);
    time_t now = ntp.get_timestamp();
    set_time(now);
    printf("Time is now %s", ctime(&now));

    //test acceleromter
    READING reading;
    SingletonFXOS8700CQ &sfxos = SingletonFXOS8700CQ::getInstance();
    double mean;
    double deviation;
    printf("\n\n\rFXOS8700CQ identity = %X\r\n", sfxos.getWhoAmI());
    //msgLock = Lock_Init();  // TODO: Check error code    
    sfxos.enable();
    sfxos.getData(reading);
    calibrate(&mean, &deviation);
  
    printf("Connecting to host %s:%d ...\r\n", MQTT_SERVER_HOST_NAME, MQTT_SERVER_PORT);
    {
        mqttNetwork = new MQTTNetwork(network);
        int rc = mqttNetwork->connect(MQTT_SERVER_HOST_NAME, MQTT_SERVER_PORT, SSL_CA_PEM,
                SSL_CLIENT_CERT_PEM, SSL_CLIENT_PRIVATE_KEY_PEM);
        if (rc != MQTT::SUCCESS){
            printf("ERROR: rc from TCP connect is %d\r\n", rc);
            return -1;
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

    // Enable button 1
    InterruptIn btn1 = InterruptIn(BUTTON1);
    btn1.rise(btn1_rise_handler);
    
    printf("To send a packet, push the button 1 on your board.\r\n");


    while(1) {
        if(!mqttClient->isConnected()){
            break;
        }
        if(mqttClient->yield(100) != MQTT::SUCCESS) {
            break;
        }
        if(isPublish) {
            isPublish = false;
            static unsigned int id = 0;
            static unsigned int count = 0;

            count++;

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
            sprintf(buf, "%d", count);
            message.payloadlen = strlen(buf)+1;
            // Publish a message.
            printf("Publishing message.\r\n");
            int rc = mqttClient->publish(MQTT_TOPIC_SUB, message);
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
