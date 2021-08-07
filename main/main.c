/* Physical button to start cycle
 *
 * ValveController_ESP32
 *
 * This controls an ESP32 Thing from Sparkfun running on a custom board
 * with four relays
 *
 */
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include <nvs_flash.h>
#include <nvs.h>
#include "esp_log.h"
#include "mqtt_client.h"

#include <esp_netif.h>
#include <esp_eth.h>
#include <esp_http_server.h>


#include "driver/uart.h"
#include "driver/gpio.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/dns.h"

/* This uses configurations that you can set via
   'make menuconfig'.

   If you'd rather not, just change the below entries to strings with
   the config you want - ie #define WIFI_SSID "mywifissid"
*/

// Constants that aren't configurable (yet)
//
#define RANGE_CHECK_TIME 100
#define WIFI_SSID     "valve"
#define WIFI_PASS     ""
#define MAX_STA_CONN  4
#define DEFAULT_SCAN_LIST_SIZE 10

#define D_TITLE       "Valve Controller"
#define D_DEVICE      "ESP32 Valve Controller"
#define D_NAMESPACE   "valveconfig"

// Board-specific constants
//
#define RFID_TXD  (GPIO_NUM_17)
#define RFID_RXD  (GPIO_NUM_16)
#define RFID_RTS  (UART_PIN_NO_CHANGE)
#define RFID_CTS  (UART_PIN_NO_CHANGE)

#define GPIO_INPUT_GPIO0          0
#define GPIO_INPUT_PRESSURE       16
#define GPIO_INPUT_PIN_SEL ((1ULL<<GPIO_INPUT_GPIO0) | (1ULL<<GPIO_INPUT_PRESSURE))

#define GPIO_OUTPUT_RELAY0        2
#define GPIO_OUTPUT_RELAY1        18
#define GPIO_OUTPUT_RELAY2        4
#define GPIO_OUTPUT_RELAY3        17
#define GPIO_OUTPUT_LED           5
#define GPIO_OUTPUT_PIN_SEL ((1ULL<<GPIO_OUTPUT_RELAY0) | (1ULL<<GPIO_OUTPUT_RELAY1) | (1ULL<<GPIO_OUTPUT_RELAY2) | (1ULL<<GPIO_OUTPUT_RELAY3) | (1ULL<<GPIO_OUTPUT_LED))


// Configurable parameters
//
#define DEFAULT_VALVE_OPEN 10
#define DEFAULT_VALVE_WAIT 4
#define DEFAULT_VALVE_PERIOD (30 * 60)

uint16_t config_open;
uint16_t config_wait;
uint16_t config_period;


// State
//
bool relay0_state;
bool relay1_state;
bool relay2_state;
bool relay3_state;
bool pressure_state;
uint16_t time_left;

static EventGroupHandle_t valve_event_group;

const int CYCLE_START_BIT = BIT0;

const char *TAG = "ValveController";

const char *THIS_HTTP_STYLE =
    "<style>"
    "div,fieldset,input,select{padding:5px;font-size:1em;}"
    "fieldset{background:#4f4f4f;}p{margin:0.5em 0;}"
    "input{width:40%;box-sizing:border-box;-webkit-box-sizing:border-box;-moz-box-sizing:border-box;background:#dddddd;color:#000000;}"
    "input[type=checkbox],input[type=radio]{width:1em;margin-right:6px;vertical-align:-1px;}"
    "input[type=range]{width:99%;}"
    "select{width:100%;background:#dddddd;color:#000000;}"
    "textarea{resize:none;width:98%;height:318px;padding:5px;overflow:auto;background:#1f1f1f;color:#65c115;}"
    "body{text-align:center;font-family:verdana,sans-serif;background:#252525;}"
    "td{padding:0px;}"
    "button{border:0;border-radius:0.3rem;background:#1fa3ec;color:#faffff;line-height:2.4rem;font-size:1.2rem;width:30%;-webkit-transition-duration:0.4s;transition-duration:0.4s;cursor:pointer;}"
    "button:hover{background:#0e70a4;}"
    ".full{width:100%;}"
    ".bred{background:#d43535;}"
    ".bred:hover{background:#931f1f;}"
    ".bgrn{background:#47c266;}"
    ".bgrn:hover{background:#5aaf6f;}"
    "a{color:#1fa3ec;text-decoration:none;}"
    ".p{float:left;text-align:left;}"
    ".q{float:right;text-align:right;}"
    ".r{border-radius:0.3em;padding:2px;margin:6px 2px;}"
    "</style>"
    ;

const char *THIS_HTTP_HEAD_START =
    "<!DOCTYPE html>"
    "<html class=\"\">"
    "<head>"
    "<meta charset='utf-8'>"
    "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1,user-scalable=no\"/>"
    "<title>" D_TITLE "</title>"
    ;

const char *THIS_HTTP_HEAD_END =
    "</head>"
    ;

const char *THIS_HTTP_REDIRECT =
    "<!DOCTYPE html>"
    "<html class=\"\">"
    "<head>"
    "<meta charset='utf-8'>"
    "<meta http-equiv=\"refresh\" content=\"0; url='/'\">"
    "</head>"
    "<body>"
    "</body>"
    "</html>"
    ;


const char *THIS_HTTP_BODY_START =
    "<body>"
    "<div style=\"text-align:left;display:inline-block;color:#eaeaea;min-width:340px;\">"
    ;

const char *THIS_HTTP_BODY_HOME_0 = 
    "<div style=\"text-align:center;color:#eaeaea;\">"
    "<h2>" D_DEVICE "</h2>"
    "</div>"
    "<p>Pressure: "
    ;

const char *THIS_HTTP_BODY_HOME_1 = "</p><p>Relay 0: " ;
const char *THIS_HTTP_BODY_HOME_2 = " Relay 1: " ;
const char *THIS_HTTP_BODY_HOME_3 = "</p><p>Relay 2: " ;
const char *THIS_HTTP_BODY_HOME_4 = " Relay 3: " ;
const char *THIS_HTTP_BODY_HOME_5 = 
    "</p><p>Time Left: ";
    ;

const char *THIS_HTTP_BODY_HOME_6 = 
    "</p><p><form method=\"get\" action=\"config\">"
    "<b>Wait:</b><input id=\"wait\" value=\""
    ;

const char *THIS_HTTP_BODY_HOME_7 = 
    "\" name=\"wait\">"
    " <button name>Update</button></form></p>"
    "</p><p><form method=\"get\" action=\"config\">"
    "<b>Open:</b><input id=\"open\" value=\""
    ;

const char *THIS_HTTP_BODY_HOME_8 = 
    "\" name=\"open\">"
    " <button name>Update</button></form></p>"
    "</p><p><form method=\"get\" action=\"config\">"
    "<b>Period:</b><input id=\"period\" value=\""
    ;

const char *THIS_HTTP_BODY_HOME_9 = 
    "\" name=\"period\">"
    " <button name>Update</button></form></p>"
    ;

const char *THIS_HTTP_BODY_HOME_10 = 
    "<p></p><form action=\"cycle\" method=\"get\">"
    "<button class=\"full\" name>Cycle Valves</button></form>"
    "<p></p><form action=\"save\" method=\"get\">"
    "<button class=\"full\" name>Save Configuration</button></form><p></p>"
    ;

const char *THIS_HTTP_BODY_SAVE = 
    "<div style=\"text-align:center;color:#eaeaea;\">"
    "<h3>Save Configuration</h3>"
    "<h2>" D_DEVICE "</h2>"
    "</div>"
    "Configuration Saved."
    "<p></p>"
    "<form action=\"/\" method=\"get\"><button name>Home</button></form>"
    ;

const char *THIS_HTTP_BODY_END =
    "</div>"
    "</body>"
    "</html>"
    ;

static void send_status(httpd_req_t *req, bool val) {
  if (val)
    httpd_resp_send_chunk (req, "ON ", strlen("ON "));
  else
    httpd_resp_send_chunk (req, "OFF", strlen("OFF"));
}

static void send_home_page(httpd_req_t *req)
{
  char val_str[10];

  httpd_resp_send_chunk (req, THIS_HTTP_HEAD_START, strlen(THIS_HTTP_HEAD_START));
  httpd_resp_send_chunk (req, THIS_HTTP_STYLE, strlen(THIS_HTTP_STYLE));
  httpd_resp_send_chunk (req, THIS_HTTP_HEAD_END, strlen(THIS_HTTP_HEAD_END));
  httpd_resp_send_chunk (req, THIS_HTTP_BODY_START, strlen(THIS_HTTP_BODY_START));

  httpd_resp_send_chunk (req, THIS_HTTP_BODY_HOME_0, strlen(THIS_HTTP_BODY_HOME_0));
  send_status(req, pressure_state);

  httpd_resp_send_chunk (req, THIS_HTTP_BODY_HOME_1, strlen(THIS_HTTP_BODY_HOME_1));
  send_status(req, relay0_state);

  httpd_resp_send_chunk (req, THIS_HTTP_BODY_HOME_2, strlen(THIS_HTTP_BODY_HOME_2));
  send_status(req, relay1_state);

  httpd_resp_send_chunk (req, THIS_HTTP_BODY_HOME_3, strlen(THIS_HTTP_BODY_HOME_3));
  send_status(req, relay2_state);

  httpd_resp_send_chunk (req, THIS_HTTP_BODY_HOME_4, strlen(THIS_HTTP_BODY_HOME_4));
  send_status(req, relay3_state);

  httpd_resp_send_chunk (req, THIS_HTTP_BODY_HOME_5, strlen(THIS_HTTP_BODY_HOME_5));
  sprintf (val_str, "%d", time_left / 10);
  httpd_resp_send_chunk (req, val_str, strlen(val_str));

  httpd_resp_send_chunk (req, THIS_HTTP_BODY_HOME_6, strlen(THIS_HTTP_BODY_HOME_6));
  sprintf (val_str, "%d", config_wait);
  httpd_resp_send_chunk (req, val_str, strlen(val_str));

  httpd_resp_send_chunk (req, THIS_HTTP_BODY_HOME_7, strlen(THIS_HTTP_BODY_HOME_7));
  sprintf (val_str, "%d", config_open);
  httpd_resp_send_chunk (req, val_str, strlen(val_str));

  httpd_resp_send_chunk (req, THIS_HTTP_BODY_HOME_8, strlen(THIS_HTTP_BODY_HOME_8));
  sprintf (val_str, "%d", config_period);
  httpd_resp_send_chunk (req, val_str, strlen(val_str));

  httpd_resp_send_chunk (req, THIS_HTTP_BODY_HOME_9, strlen(THIS_HTTP_BODY_HOME_9));
  httpd_resp_send_chunk (req, THIS_HTTP_BODY_HOME_10, strlen(THIS_HTTP_BODY_HOME_10));
  httpd_resp_send_chunk (req, THIS_HTTP_BODY_END, strlen(THIS_HTTP_BODY_END));
  httpd_resp_send_chunk (req, NULL, 0);
}


void validate_u16 (char *val, unsigned int min, unsigned int max, uint16_t *number) {
  unsigned int sp;
  if (sscanf (val, "%d", &sp) == 1) {
    if ((sp >= min) && (sp <= max)) {
      *number = (uint16_t) sp;
    }
  }
}

static esp_err_t config_get_handler(httpd_req_t *req)
{
  char val[20];
  size_t buf_len;
  char* buf;

  ESP_LOGI(TAG, "in config handler");
  buf_len = httpd_req_get_url_query_len(req) + 1;
  if (buf_len > 1) {
    buf = malloc(buf_len);
    if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
      if (httpd_query_key_value(buf, "wait", val, sizeof(val)) == ESP_OK) {
        validate_u16 (val, 1, 65535, &config_wait);
        ESP_LOGI(TAG, "Updated wait to %u", config_wait);
      }
      if (httpd_query_key_value(buf, "open", val, sizeof(val)) == ESP_OK) {
        validate_u16 (val, 1, 65535, &config_open);
        ESP_LOGI(TAG, "Updated open to %u", config_open);
      }
      if (httpd_query_key_value(buf, "period", val, sizeof(val)) == ESP_OK) {
        validate_u16 (val, 1, 65535, &config_period);
        time_left = config_period * 10;
        ESP_LOGI(TAG, "Updated period to %u", config_period);
      }
    }
    free(buf);
  }

  // back to home page
  httpd_resp_send_chunk (req, THIS_HTTP_REDIRECT, strlen(THIS_HTTP_REDIRECT));
  httpd_resp_send_chunk (req, NULL, 0);
  return (ESP_OK);
}

static const httpd_uri_t config_h = {
  .uri       = "/config",
  .method    = HTTP_GET,
  .handler   = config_get_handler,
  .user_ctx  = NULL
};



static esp_err_t cycle_get_handler(httpd_req_t *req)
{
  ESP_LOGI(TAG, "in cycle handler");
  if (!(xEventGroupGetBits (valve_event_group) & CYCLE_START_BIT)) {
    ESP_LOGI(TAG, "Cycle trigger via http");
    xEventGroupSetBits(valve_event_group, CYCLE_START_BIT);
  }

  // back to home page
  httpd_resp_send_chunk (req, THIS_HTTP_REDIRECT, strlen(THIS_HTTP_REDIRECT));
  httpd_resp_send_chunk (req, NULL, 0);
  return (ESP_OK);
}

static const httpd_uri_t cycle_h = {
  .uri       = "/cycle",
  .method    = HTTP_GET,
  .handler   = cycle_get_handler,
  .user_ctx  = NULL
};



static esp_err_t save_get_handler(httpd_req_t *req)
{
  ESP_LOGI(TAG, "in save config handler");

  esp_err_t err;

  // try to save values to NVS
  nvs_handle_t my_handle;
  err = nvs_open(D_NAMESPACE, NVS_READWRITE, &my_handle);
  if (err != ESP_OK) {
    char *resp_str = "Error opening NVS handle!";
    httpd_resp_send(req, resp_str, strlen(resp_str));
    return (err);
  } else {
    int8_t nvs_valid = true;
    err = nvs_set_i8 (my_handle, "valid_flag", nvs_valid);
    if (err != ESP_OK) {
      char *resp_str = "Error setting valid_flag in NVS!";
      httpd_resp_send(req, resp_str, strlen(resp_str));
      nvs_close (my_handle);
      return (err);
    }

    // save valve configuration to NVS
    if (((err = nvs_set_u16 (my_handle, "config_wait", config_wait)) != ESP_OK) ||
        ((err = nvs_set_u16 (my_handle, "config_open", config_open)) != ESP_OK) ||
        ((err = nvs_set_u16 (my_handle, "config_period", config_period)) != ESP_OK)) {

      char *resp_str = "Error setting config values in NVS!";
      httpd_resp_send(req, resp_str, strlen(resp_str));
      nvs_close (my_handle);
      return (err);
    }

    err = nvs_commit (my_handle);
    if (err != ESP_OK) {
      char *resp_str = "Error committing NVS!";
      httpd_resp_send(req, resp_str, strlen(resp_str));
      nvs_close (my_handle);
      return (err);
    }

    nvs_close (my_handle);
  }

  httpd_resp_send_chunk (req, THIS_HTTP_HEAD_START, strlen(THIS_HTTP_HEAD_START));
  httpd_resp_send_chunk (req, THIS_HTTP_STYLE, strlen(THIS_HTTP_STYLE));
  httpd_resp_send_chunk (req, THIS_HTTP_HEAD_END, strlen(THIS_HTTP_HEAD_END));
  httpd_resp_send_chunk (req, THIS_HTTP_BODY_START, strlen(THIS_HTTP_BODY_START));
  httpd_resp_send_chunk (req, THIS_HTTP_BODY_SAVE, strlen(THIS_HTTP_BODY_SAVE));
  httpd_resp_send_chunk (req, THIS_HTTP_BODY_END, strlen(THIS_HTTP_BODY_END));
  httpd_resp_send_chunk (req, NULL, 0);
  return (ESP_OK);
}

static const httpd_uri_t save_h = {
  .uri       = "/save",
  .method    = HTTP_GET,
  .handler   = save_get_handler,
  .user_ctx  = NULL
};

static esp_err_t home_get_handler(httpd_req_t *req)
{
  ESP_LOGI(TAG, "in home page handler");
  send_home_page(req);
  return ESP_OK;
}

static const httpd_uri_t home = {
  .uri       = "/",
  .method    = HTTP_GET,
  .handler   = home_get_handler,
  .user_ctx  = NULL
};

esp_err_t http_404_error_handler(httpd_req_t *req, httpd_err_code_t err)
{
  if (strcmp("/hello", req->uri) == 0) {
    httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "/hello URI is not available");
    /* Return ESP_OK to keep underlying socket open */
    return ESP_OK;
  } else if (strcmp("/echo", req->uri) == 0) {
    httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "/echo URI is not available");
    /* Return ESP_FAIL to close underlying socket */
    return ESP_FAIL;
  }
  /* For any other URI send 404 and close socket */
  httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "Some 404 error message");
  return ESP_FAIL;
}

static httpd_handle_t start_webserver(void)
{
  httpd_handle_t server = NULL;
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();

  // Start the httpd server
  ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
  if (httpd_start(&server, &config) == ESP_OK) {
    // Set URI handlers
    ESP_LOGI(TAG, "Registering URI handlers");
    httpd_register_uri_handler(server, &home);
    httpd_register_uri_handler(server, &config_h);
    httpd_register_uri_handler(server, &cycle_h);
    httpd_register_uri_handler(server, &save_h);
    return server;
  }

  ESP_LOGI(TAG, "Error starting server!");
  return NULL;
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
    int32_t event_id, void* event_data)
{
  if (event_id == WIFI_EVENT_AP_START) {
    ESP_LOGI(TAG, "ESP32 is started in AP mode");
  } else if (event_id == WIFI_EVENT_AP_STACONNECTED) {
    wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
    ESP_LOGI(TAG, "station "MACSTR" join, AID=%d",
        MAC2STR(event->mac), event->aid);
  } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
    wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
    ESP_LOGI(TAG, "station "MACSTR" leave, AID=%d",
        MAC2STR(event->mac), event->aid);
  } else {
    ESP_LOGI(TAG, "Other event_id: %d", event_id);
  }
}

static uint16_t number = DEFAULT_SCAN_LIST_SIZE;
static wifi_ap_record_t ap_info[DEFAULT_SCAN_LIST_SIZE];
static uint16_t ap_count = 0;

void wificonfig_scan(void) {
    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    assert(sta_netif);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    memset(ap_info, 0, sizeof(ap_info));

    ap_count = 0;
    ESP_LOGI(TAG, "Starting AP scan");
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_scan_start(NULL, true));
    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&number, ap_info));
    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_num(&ap_count));
    ESP_ERROR_CHECK(esp_wifi_stop());
}

void wifi_init_ap(void)
{
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  esp_netif_create_default_wifi_ap();

  wificonfig_scan();

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));

  wifi_config_t wifi_config = {
    .ap = {
      .ssid = WIFI_SSID,
      .ssid_len = strlen(WIFI_SSID),
      .password = WIFI_PASS,
      .max_connection = MAX_STA_CONN,
      .authmode = WIFI_AUTH_WPA_WPA2_PSK
    },
  };
  if (strlen(WIFI_PASS) == 0) {
    wifi_config.ap.authmode = WIFI_AUTH_OPEN;
  }

  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
  ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config));
  ESP_ERROR_CHECK(esp_wifi_start());

  ESP_LOGI(TAG, "wifi_init_softap finished. SSID:%s password:%s",
      WIFI_SSID, WIFI_PASS);
}

void dump_config() {
  ESP_LOGI(TAG, "config_wait = %u", config_wait);
  ESP_LOGI(TAG, "config_open = %u", config_open);
  ESP_LOGI(TAG, "config_period = %u", config_period);
}
// read configuration values from NVS
//
esp_err_t get_nvs_config(nvs_handle_t my_handle) {
  esp_err_t err;
  esp_err_t last_err = ESP_OK;

  // valve parameters
  if ((err = nvs_get_u16(my_handle, "valve_wait", &config_wait)) != ESP_OK) last_err = err;
  if ((err = nvs_get_u16(my_handle, "valve_open", &config_open)) != ESP_OK) last_err = err;
  if ((err = nvs_get_u16(my_handle, "valve_period", &config_period)) != ESP_OK) last_err = err;

  return (last_err);
}

static void wifi_server (void *pvParameters)
{
  // Start the access-point and server
  wifi_init_ap();
  start_webserver();

  // wait forever
  while (1) {
    vTaskDelay(1000 / portTICK_RATE_MS);
  }
}

void blinkit_high(int pin, int count, int wait){
  for (int i=count; i>0; i--) {
    gpio_set_level (pin, 1);
    vTaskDelay(wait / portTICK_RATE_MS);
    gpio_set_level (pin, 0);
    if (i > 1)
      vTaskDelay(wait / portTICK_RATE_MS);
  }
}

void blinkit_low(int pin, int count, int wait){
  for (int i=count; i>0; i--) {
    gpio_set_level (pin, 0);
    vTaskDelay(wait / portTICK_RATE_MS);
    gpio_set_level (pin, 1);
    if (i > 1)
      vTaskDelay(wait / portTICK_RATE_MS);
  }
}

void initialize_pins (void) {
  gpio_config_t io_conf;

  // Configure Inputs

  //disable interrupt
  io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
  //set as output mode
  io_conf.mode = GPIO_MODE_INPUT;
  //bit mask of the pins that you want to set,e.g.GPIO21/32
  io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
  //disable pull-down mode
  io_conf.pull_down_en = 0;
  //disable pull-up mode
  io_conf.pull_up_en = 0;
  //configure GPIO with the given settings
  gpio_config(&io_conf);


  // Configure Outputs

  //disable interrupt
  io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
  //set as output mode
  io_conf.mode = GPIO_MODE_OUTPUT;
  //bit mask of the pins that you want to set,e.g.GPIO21/32
  io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
  //disable pull-down mode
  io_conf.pull_down_en = 0;
  //disable pull-up mode
  io_conf.pull_up_en = 0;
  //configure GPIO with the given settings
  gpio_config(&io_conf);

  gpio_set_level(GPIO_OUTPUT_RELAY0, 0);
  gpio_set_level(GPIO_OUTPUT_RELAY1, 0);
  gpio_set_level(GPIO_OUTPUT_RELAY2, 0);
  gpio_set_level(GPIO_OUTPUT_RELAY3, 0);
  gpio_set_level(GPIO_OUTPUT_LED, 0);

  relay0_state = false;
  relay1_state = false;
  relay2_state = false;
  relay3_state = false;

  // Flash LED on for 500ms
  //
  gpio_set_level(GPIO_OUTPUT_LED, 1);
  vTaskDelay(500 / portTICK_RATE_MS);
  gpio_set_level(GPIO_OUTPUT_LED, 0);
}

// set configuration values to defaults
//
void init_config(void) {
  uint8_t mac[6];
  esp_base_mac_addr_get(mac);
  uint16_t id = mac[5] + (mac[4]<<8);

  // dump default id
  ESP_LOGI(TAG, "init_config: id=0x%x", id);

  // valve parameters
  config_wait = DEFAULT_VALVE_WAIT;
  config_open = DEFAULT_VALVE_OPEN;
  config_period = DEFAULT_VALVE_PERIOD;
}

static void read_config ()
{
  // Initialize NVS
  esp_err_t err = nvs_flash_init ();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    // NVS partition was truncated and needs to be erased
    // Retry nvs_flash_init
    ESP_LOGI(TAG, "Erasing NVS flash");
    ESP_ERROR_CHECK(nvs_flash_erase ());
    err = nvs_flash_init ();
  }
  ESP_ERROR_CHECK (err);
  ESP_LOGI(TAG, "Done with NVS init");

  init_config ();

  // try to read values from NVS
  nvs_handle_t my_handle;
  int8_t nvs_valid = false;
  err = nvs_open(D_NAMESPACE, NVS_READWRITE, &my_handle);
  if (err != ESP_OK) {
    printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    ESP_LOGE(TAG, "Error (%s) opening NVS handle!\n", esp_err_to_name(err));
  } else {
    ESP_LOGI(TAG, "Opened NVS");
    err = nvs_get_i8 (my_handle, "valid_flag", &nvs_valid);
    if (err == ESP_OK) {
      ESP_LOGI(TAG, "Read NVS valid_flag = %d", nvs_valid);
      if ((get_nvs_config(my_handle) == ESP_OK) && nvs_valid) {
        ESP_LOGI(TAG, "Have valid config data");
        nvs_close (my_handle);
      }
    } else {
      nvs_valid = false;
      ESP_LOGI(TAG, "invalid config data");
    }

    nvs_close (my_handle);
  }

  dump_config();
}


static void valve_main_loop (void *pvParameters)
{
  int last_level = 1;
  time_left = config_period * 10;
  while(1) {
    pressure_state = !gpio_get_level(GPIO_INPUT_PRESSURE);
    gpio_set_level(GPIO_OUTPUT_LED, pressure_state);

    // Physical button to start cycle
    int new_level = gpio_get_level(GPIO_INPUT_GPIO0);
    if (new_level != last_level) {
      if ((new_level == 1) && !(xEventGroupGetBits (valve_event_group) & CYCLE_START_BIT)) {
        ESP_LOGI(TAG, "Cycle trigger via GPIO0");
        xEventGroupSetBits(valve_event_group, CYCLE_START_BIT);
      }
    }
    last_level = new_level;

    if (pressure_state)
      time_left -= 1;
    if (time_left == 0) {
      if (!(xEventGroupGetBits (valve_event_group) & CYCLE_START_BIT)) {
        ESP_LOGI(TAG, "Cycle trigger via time");
        xEventGroupSetBits(valve_event_group, CYCLE_START_BIT);
      }
      time_left = config_period * 10;
    }

    vTaskDelay(100 / portTICK_RATE_MS);
  }
}

static void cycle_loop (void *pvParameters)
{
  while(1) {
    xEventGroupWaitBits(valve_event_group, CYCLE_START_BIT, false, false, portMAX_DELAY);

    // Cycle starts!
    //
    ESP_LOGI(TAG, "cycle_loop starting");

    // turn on power to valve A
    //
    ESP_LOGI(TAG, "Relay 1 -> on");
    gpio_set_level(GPIO_OUTPUT_RELAY0, 0);
    gpio_set_level(GPIO_OUTPUT_RELAY1, 1);
    gpio_set_level(GPIO_OUTPUT_RELAY2, 0);
    gpio_set_level(GPIO_OUTPUT_RELAY3, 0);
    relay1_state = true;

    // delay - wait time (to make sure A is closed)
    vTaskDelay(1000 * config_wait / portTICK_RATE_MS);

    ESP_LOGI(TAG, "Relay 0 -> on");
    relay0_state = true;
    gpio_set_level(GPIO_OUTPUT_RELAY0, 1);

    // delay - wait time (to move to open position)
    vTaskDelay(1000 * config_wait / portTICK_RATE_MS);

    // delay - open time
    vTaskDelay(1000 * config_open / portTICK_RATE_MS);

    // close valve A
    ESP_LOGI(TAG, "Relay 0 -> off");
    relay0_state = false;
    gpio_set_level(GPIO_OUTPUT_RELAY0, 0);

    // delay - wait time (to move to closed position)
    vTaskDelay(1000 * config_wait / portTICK_RATE_MS);

    // turn off power to valve A
    ESP_LOGI(TAG, "Relay 1 -> off");
    relay1_state = false;
    gpio_set_level(GPIO_OUTPUT_RELAY1, 0);

    // turn on power to valve B
    ESP_LOGI(TAG, "Relay 3 -> on");
    relay3_state = true;
    gpio_set_level(GPIO_OUTPUT_RELAY3, 1);

    // delay - wait time (to make sure B is closed)
    vTaskDelay(1000 * config_wait / portTICK_RATE_MS);

    // open valve B
    ESP_LOGI(TAG, "Relay 2 -> on");
    relay2_state = true;
    gpio_set_level(GPIO_OUTPUT_RELAY2, 1);

    // delay - wait time (to move to open position)
    vTaskDelay(1000 * config_wait / portTICK_RATE_MS);

    // delay - open time
    vTaskDelay(1000 * config_open / portTICK_RATE_MS);

    // close valve B
    ESP_LOGI(TAG, "Relay 2 -> off");
    relay2_state = false;
    gpio_set_level(GPIO_OUTPUT_RELAY2, 0);

    // delay - wait time (to move to closed position)
    vTaskDelay(1000 * config_wait / portTICK_RATE_MS);

    // turn off power to valve B
    ESP_LOGI(TAG, "Relay 3 -> off");
    relay3_state = false;
    gpio_set_level(GPIO_OUTPUT_RELAY3, 0);

    // Done!
    ESP_LOGI(TAG, "cycle_loop ending");
    xEventGroupClearBits(valve_event_group, CYCLE_START_BIT);
    vTaskDelay(100 / portTICK_RATE_MS);
  }
}

void app_main()
{
  valve_event_group = xEventGroupCreate();
  xEventGroupClearBits(valve_event_group, CYCLE_START_BIT);
  initialize_pins();
  read_config();

  xTaskCreate(&wifi_server, "wifi_server", 4096, NULL, 5, NULL);
  xTaskCreate(&valve_main_loop, "valve_main_loop", 4096, NULL, 5, NULL);
  xTaskCreate(&cycle_loop, "cycle_loop", 4096, NULL, 5, NULL);
}
