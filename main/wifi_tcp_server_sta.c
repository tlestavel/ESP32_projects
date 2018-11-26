#include "wifi_tcp_server_sta.h"

/* GLOBAL VARIABLES */
const int CONNECTED_BIT = BIT0; //Just a bit for passing information through the event
static const char *TAG="sta_mode_tcp_server";


/* EVENT_HANDLER

    Handle event from the WIFI connectivity (connection, disconnection, IP allocation...)

*/
esp_err_t wifi_event_handler(void *ctx, system_event_t *event)
{
    switch(event->event_id) 
    {
    case SYSTEM_EVENT_STA_START:
        // Once the WiFi STATION started, connect to the wifi network
        wifi_network_connect();
        break;
    case SYSTEM_EVENT_STA_GOT_IP:
        // If IP received, set the "CONNECTED BIT"
        xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        // Try connecting back????
        esp_wifi_connect();

        // If STA disconnected, clean "CONNECTED BIT"
        xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
        break;
    default:
        break;
    }
    return ESP_OK;
}

/* wifi_network_connect

    Connect to the given WiFi network

*/
void wifi_network_connect()
{
    // Wifi network to connect to informations
    wifi_config_t cfg = 
    {
        .sta = 
        {
            .ssid = SSID,
            .password = PASSPHARSE,
        },
    };


    // Disconnect from any prior AP
    ESP_ERROR_CHECK(esp_wifi_disconnect());

    // Set the wifi network configuration
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &cfg));

    // Connect to the wifi network
    ESP_ERROR_CHECK(esp_wifi_connect());
}


/* WIFI_INIT 

Initialise and start the ESP32 WiFi in STATION mode

*/
void wifi_initialise()
{
    // Disable WIFI driver logging
    esp_log_level_set("wifi", ESP_LOG_NONE); 

    // Initialise TCP/IP stack
    tcpip_adapter_init();

    // Wifi stack configuration -- should always be set to default
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

    // WiFi init resource allocation
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Set WiFi mode to station
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

    // Start WiFi
    ESP_ERROR_CHECK(esp_wifi_start());

    // Init de socket_command buffer
    socket_command_received = SOCKET_COMMAND_NONE;
}

/* wifi_print_ip_task 

    Print the allocated IP address

*/
void wifi_print_ip_task(void *pvParam)
{
    printf("Task %s started \n", __func__);

    // Wait for the event "CONNECTED BIT" to be set from wifi_event_group
    xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT, false, true, portMAX_DELAY);

    // Gather IP_INFO from the TCPIP Adapter (ip, mask and gateway)
    tcpip_adapter_ip_info_t ip_info;
	ESP_ERROR_CHECK(tcpip_adapter_get_ip_info(TCPIP_ADAPTER_IF_STA, &ip_info));
    printf("IP :  %s\n", ip4addr_ntoa(&ip_info.ip));

    // Delete task to avoid freeRTOS assert
    vTaskDelete( NULL );
}

/* TCP_SERVER_TASK 

TCP server thread

*/
void tcp_server_task(void *pvParam)
{
    printf("Task %s started \n", __func__);

    // Socket address setting ()
    struct sockaddr_in tcpServerAddr;
    tcpServerAddr.sin_addr.s_addr = htonl(INADDR_ANY); // takes any address
    tcpServerAddr.sin_family = AF_INET; //IPv4
    tcpServerAddr.sin_port = htons( 3000 ); // port used on the socket
    
    UBaseType_t uxHighWaterMark;
    int s, r;
    char recv_buf[64];
    char writeBuffer[320] = {0};
    static struct sockaddr_in remote_addr;
    static unsigned int socklen;
    socklen = sizeof(remote_addr);

    int cs; //client socket

    int disconnectionCounter = 0;
    
    // Wait to be connected (=be granted an IP address in that case)
    xEventGroupWaitBits(wifi_event_group,CONNECTED_BIT,false,true,portMAX_DELAY);

    while(1)
    {
        // Determine if the socket has been allocated
        s = socket(AF_INET, SOCK_STREAM, 0);
        if(s < 0) 
        {
            ESP_LOGE(TAG, "... Failed to allocate socket.\n");
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            continue;
        }

        ESP_LOGI(TAG, "... allocated socket\n");

        // Bind the address of the TCP server
        if(bind(s, (struct sockaddr *)&tcpServerAddr, sizeof(tcpServerAddr)) != 0) 
        {
            ESP_LOGE(TAG, "... socket bind failed errno=%d \n", errno);
            close(s); // close the socket in case of failure
            vTaskDelay(4000 / portTICK_PERIOD_MS);
            continue;
        }

        ESP_LOGI(TAG, "... socket bind done \n");
        
        // Open the listening of the socket
        if(listen (s, LISTENQ) != 0) 
        {
            ESP_LOGE(TAG, "... socket listen failed errno=%d \n", errno);
            close(s);// close the socket in case of failure
            vTaskDelay(4000 / portTICK_PERIOD_MS);
            continue;
        }

        while(1)
        {
            cs = accept(s,(struct sockaddr *)&remote_addr, &socklen);
            ESP_LOGI(TAG,"New connection request,Request data:");
            //set O_NONBLOCK so that recv will return, otherwise we need to implement message end 
            //detection logic. If know the client message format you should instead impliment logic
            //detect the end of message
            // Set the file status flags to the value O_NONBLOCK
            fcntl(cs, F_SETFL, O_NONBLOCK);

            
            // Counter to exit the while loop once connected
            disconnectionCounter = 0;

            // Read from the socket
            while(1) 
            {
                /* ** Read ** */
                // Fill recv_buf with zeros
                bzero(recv_buf, sizeof(recv_buf));

                // Fill recv_buf from the socket message
                r = recv(cs, recv_buf, sizeof(recv_buf)-1,0);

                if(r > 0)
                {
                    disconnectionCounter = 0;
                    printf("Read! %s\n", recv_buf);

                    socket_command_received = recv_buf[0] - 48;                   
                }
                else if(r == 0)
                {
                    break;
                }

                /* ** Write ** */
                

                vTaskDelay(100/portTICK_PERIOD_MS);

                // 4s disconnection counter
                if(++disconnectionCounter == 40)
                {
                    break;
                }

                // uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
                // printf("Stack usage: %d", uxHighWaterMark);
            } //while(r > 0);
            
            ESP_LOGI(TAG, "... done reading from socket. Last read return=%d errno=%d\r\n", r, errno);
            
            for(int i=0; i<320; i++)
            {
                writeBuffer[i] = i*2;
            }

            // Write back to the acception
            if(send(cs, writeBuffer, strlen(writeBuffer), 0) < 0)
            {
                ESP_LOGE(TAG, "... Send failed \n");
                close(s);
                vTaskDelay(4000 / portTICK_PERIOD_MS);
                continue;
            }
            ESP_LOGI(TAG, "... socket send success");

            // Closing the accepted connection
            close(cs);
        }
        ESP_LOGI(TAG, "... server will be opened in 5 seconds");
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
    ESP_LOGI(TAG, "...tcp_client task closed\n");

    // Careful to delete the task if getting there
}

/* WIFI_PRINT_IP_ADDRESS()

    Only print the current ip address;

*/
void wifi_print_ip_address()
{
    // Gather IP_INFO from the TCPIP Adapter (ip, mask and gateway)
    tcpip_adapter_ip_info_t ip_info;
	ESP_ERROR_CHECK(tcpip_adapter_get_ip_info(TCPIP_ADAPTER_IF_STA, &ip_info));
    printf("IP :  %s\n", ip4addr_ntoa(&ip_info.ip));
}


/* WIFI_PRINT_IP_ADDRESS()

    Provide the latest socket command received

*/
int get_socket_command()
{
    int ret = socket_command_received;
    socket_command_received = SOCKET_COMMAND_NONE;
    return ret;
}