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


/**
 * Converts raw data value in LSB to gal.
 * @param raw_val Raw data value in LSB unit.
 * @return The value in gal.
 */
float acc_conv_raw_to_gal(int16_t raw_val) {
    static float sensitivity = 0;
    if(sensitivity == 0) {
        SingletonFXOS8700CQ &sfxos = SingletonFXOS8700CQ::getInstance();
        uint8_t full_scale = sfxos.get_accel_scale(); // G
        int16_t resolution = 16384;    // 2^14, accelerometer resolution
        sensitivity = full_scale / (float)resolution;   // G per LSB
    }
    const float CONV_G_TO_GAL = 980.665;  // 1G = 980.665 gal

    return (raw_val*CONV_G_TO_GAL*sensitivity);
}

/**
 * Calibrates the accelerometer.
 * @return 0 on success, other values on error.
 */
int acc_calibrate() {
    int ret = 0;
    const int SAMPLING_FREQ = 50; // Hz
    const int SAMPLE_TIME = 10;   // seconds
    const int NUM_SAMPLES = SAMPLING_FREQ * SAMPLE_TIME;

    // Get memory in heap ~ 2kB, don't forget to delete.
    int16_t *ax = new int16_t[NUM_SAMPLES];
    int16_t *ay = new int16_t[NUM_SAMPLES];

    int i = NUM_SAMPLES;

    READING reading;
    SingletonFXOS8700CQ &sfxos = SingletonFXOS8700CQ::getInstance();

    int32_t buf_mean_ax = 0;
    int32_t buf_mean_ay = 0;

    while(i > 0) {
        if (sfxos.getInt2Triggered()){
            sfxos.setInt2Triggered(false);
            if(sfxos.getData(reading) == 0){
                i--;
                ax[i] = reading.accelerometer.x;
                ay[i] = reading.accelerometer.y;
                buf_mean_ax += ax[i];
                buf_mean_ay += ay[i];
            }
        }
        // wait 20 milli-seconds, roughly 50 Hz -- not accurate
        wait_ms(20);
    }

    buf_mean_ax = buf_mean_ax / NUM_SAMPLES;
    buf_mean_ay = buf_mean_ay / NUM_SAMPLES;

    // Calculates the square of standard deviation
    int32_t sq_stdev = 0;
    for(i=0; i < NUM_SAMPLES; i++) {
        int32_t dx = ax[i] - buf_mean_ax;
        int32_t dy = ay[i] - buf_mean_ay;
        sq_stdev += dx * dx + dy * dy;
    }
    sq_stdev = sq_stdev / NUM_SAMPLES;

    const float THRESHOLD = 1.5;   // GAL (tentative) - 0.3 gal might be too tight for FXOS8700CQ?
    float val = acc_conv_raw_to_gal(sqrt(((float)sq_stdev))); // GAL
    if(val < THRESHOLD){
        ret = 0;
        mean_ax = (int16_t)buf_mean_ax;
        mean_ay = (int16_t)buf_mean_ay;
        printf("OFFSET AX=%d LSB (%f gal), AY=%d LSB (%f gal)\r\n", 
            mean_ax, acc_conv_raw_to_gal(mean_ax), 
            mean_ay, acc_conv_raw_to_gal(mean_ay));
    } else {
        ret = -1;
        printf("Deviation %f exceeds the threshold %f gal. Try calibration again.\r\n", val, THRESHOLD);
    }

    delete[] ax;
    delete[] ay;

    return ret;
}

// The square of the size of acceleration vector. Published to the server.
uint32_t report_sq_vector_size = 0; 

/**
 * Measures acceleration and calculates holizontal size of acc vector. Called every 20 msec.
 */
int acc_measure() {
    // Once count reaches this value, data is sent to the server.
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
            report_sq_vector_size = max_vector_size;
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

    ///////////////////////////////////////////////////
    //
    // Network setting
    //
    ///////////////////////////////////////////////////
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

    ///////////////////////////////////////////////////
    //
    // Accelerometer setting
    //
    ///////////////////////////////////////////////////
    SingletonFXOS8700CQ &sfxos = SingletonFXOS8700CQ::getInstance();
    // Check the connection to the accelerometer
    printf("\n\n\rFXOS8700CQ identity = %X\r\n", sfxos.getWhoAmI());
    // Enable the accelerometer
    sfxos.enable();

    // Calibration, only init time at the moment.
    printf("Calibrating...\r\n");
    while(acc_calibrate() != 0);
    printf("Calibration done.\r\n");

    // Call measure() every 20 milli-seconds.
    Ticker ticker;
    ticker.attach(eventQueue.event(&acc_measure), 0.02);

    // Start the event queue in a separate thread so the main thread continues
    thread1.start(callback(&eventQueue, &EventQueue::dispatch_forever));

    ///////////////////////////////////////////////////
    //
    // Main loop
    //
    ///////////////////////////////////////////////////
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
            sprintf(buf, "%ld,%f", ntp.get_timestamp(), acc_conv_raw_to_gal(sqrt((float)report_sq_vector_size)));
            message.payloadlen = strlen(buf)+1;
            // Publish a message.
            printf("Publishing message.\r\n");
            int rc = mqttClient->publish(MQTT_TOPIC_PUB, message);
            if(rc != MQTT::SUCCESS) {
                printf("ERROR: rc from MQTT publish is %d\r\n", rc);
            } else {
                printf("%s\r\n", buf);
            }
            printf("Message published.\r\n");
            delete[] buf;    

            led_blue = LED_OFF;
        }
    }

    ///////////////////////////////////////////////////
    //
    // Terminate
    //
    ///////////////////////////////////////////////////
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
