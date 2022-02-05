// Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include "esp_http_server.h"
#include "esp_timer.h"
#include "esp_camera.h"
#include "img_converters.h"
#include "camera_index.h"
#include "Arduino.h"
#include "ArduinoJson.h"
#include <HTTPClient.h>

#include "fb_gfx.h"
#include "fd_forward.h"
#include "fr_forward.h"


extern char* server_token;
extern char* client_token;
extern portMUX_TYPE timerMux;
extern volatile int sendLogCounter;
extern int64_t duration;

typedef struct {
        size_t size; //number of values used for filtering
        size_t index; //current value index
        size_t count; //value count
        int sum;
        int * values; //array to be filled with values
} ra_filter_t;

typedef struct {
        httpd_req_t *req;
        size_t len;
} jpg_chunking_t;

#define PART_BOUNDARY "123456789000000000000987654321"
static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

static ra_filter_t ra_filter;
httpd_handle_t stream_httpd = NULL;
httpd_handle_t camera_httpd = NULL;

static mtmn_config_t mtmn_config = {0};
static int8_t detection_enabled = 0;

static ra_filter_t * ra_filter_init(ra_filter_t * filter, size_t sample_size){
    memset(filter, 0, sizeof(ra_filter_t));

    filter->values = (int *)malloc(sample_size * sizeof(int));
    if(!filter->values){
        return NULL;
    }
    memset(filter->values, 0, sample_size * sizeof(int));

    filter->size = sample_size;
    return filter;
}

static int ra_filter_run(ra_filter_t * filter, int value){
    if(!filter->values){
        return value;
    }
    filter->sum -= filter->values[filter->index];
    filter->values[filter->index] = value;
    filter->sum += filter->values[filter->index];
    filter->index++;
    filter->index = filter->index % filter->size;
    if (filter->count < filter->size) {
        filter->count++;
    }
    return filter->sum / filter->count;
}


// Ici qu'est envoyé le stream vidéo
static esp_err_t stream_handler(httpd_req_t *req){
    size_t token_size = httpd_req_get_hdr_value_len(req,  "Authorization")+1;
    char token[token_size];

    /* // Lie le header Authorization
    if(httpd_req_get_hdr_value_str(req,  "Authorization", token, token_size) != ESP_OK){
        httpd_resp_set_type(req, "application/json");
        httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
        Serial.println("Client Token Replacement Failed: Error during reading Authorization header");
        char json_resp[] = "{\"status\": 401, \"message\": \"Error during reading Authorization header\"}";
        httpd_resp_send(req, json_resp, strlen(json_resp));
        return ESP_OK;
    };

    Serial.println(token);
    //Vérifie le token Authorization
    if(strcmp(token, client_token) != 0){
        httpd_resp_set_type(req, "application/json");
        httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
        Serial.println("Client Token Replacement Failed: Invalid token");
        char json_resp[] = "{\"status\": 401, \"message\": \"Invalid token\"}";
        httpd_resp_send(req, json_resp, strlen(json_resp));
        return ESP_OK;
    } */ 

    camera_fb_t * fb = NULL;
    esp_err_t res = ESP_OK;
    size_t _jpg_buf_len = 0;
    uint8_t * _jpg_buf = NULL;
    char * part_buf[64];
    dl_matrix3du_t *image_matrix = NULL;
    bool detected = false;
    int face_id = 0;
    int64_t fr_start = 0;
    int64_t fr_ready = 0;
    int64_t fr_face = 0;
    int64_t fr_recognize = 0;
    int64_t fr_encode = 0;

    static int64_t last_frame = 0;
    if(!last_frame) {
        last_frame = esp_timer_get_time();
    }

    res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
    if(res != ESP_OK){
        return res;
    }

    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    int64_t starting_time = esp_timer_get_time();
    
    while(true){
        detected = false;
        face_id = 0;
        fb = esp_camera_fb_get();
        if (!fb) {
            Serial.println("Camera capture failed");
            res = ESP_FAIL;
        } else {
            fr_start = esp_timer_get_time();
            fr_ready = fr_start;
            fr_face = fr_start;
            fr_encode = fr_start;
            fr_recognize = fr_start;
            if(!detection_enabled || fb->width > 400){
                if(fb->format != PIXFORMAT_JPEG){
                    bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
                    esp_camera_fb_return(fb);
                    fb = NULL;
                    if(!jpeg_converted){
                        Serial.println("JPEG compression failed");
                        res = ESP_FAIL;
                    }
                } else {
                    _jpg_buf_len = fb->len;
                    _jpg_buf = fb->buf;
                }
            } else {

                image_matrix = dl_matrix3du_alloc(1, fb->width, fb->height, 3);

                if (!image_matrix) {
                    Serial.println("dl_matrix3du_alloc failed");
                    res = ESP_FAIL;
                } else {
                    if(!fmt2rgb888(fb->buf, fb->len, fb->format, image_matrix->item)){
                        Serial.println("fmt2rgb888 failed");
                        res = ESP_FAIL;
                    } else {
                        fr_ready = esp_timer_get_time();
                        box_array_t *net_boxes = NULL;
                        if(detection_enabled){
                            net_boxes = face_detect(image_matrix, &mtmn_config);
                        }
                        fr_face = esp_timer_get_time();
                        fr_recognize = fr_face;
                        if (net_boxes || fb->format != PIXFORMAT_JPEG){
                            if(!fmt2jpg(image_matrix->item, fb->width*fb->height*3, fb->width, fb->height, PIXFORMAT_RGB888, 90, &_jpg_buf, &_jpg_buf_len)){
                                Serial.println("fmt2jpg failed");
                                res = ESP_FAIL;
                            }
                            esp_camera_fb_return(fb);
                            fb = NULL;
                        } else {
                            _jpg_buf = fb->buf;
                            _jpg_buf_len = fb->len;
                        }
                        fr_encode = esp_timer_get_time();
                    }
                    dl_matrix3du_free(image_matrix);
                }
            }
        }
        if(res == ESP_OK){
            res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
        }
        if(res == ESP_OK){
            size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART, _jpg_buf_len);
            res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
        }
        if(res == ESP_OK){
            res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
        }
        if(fb){
            esp_camera_fb_return(fb);
            fb = NULL;
            _jpg_buf = NULL;
        } else if(_jpg_buf){
            free(_jpg_buf);
            _jpg_buf = NULL;
        }
        if(res != ESP_OK){
            break;
        }
        int64_t fr_end = esp_timer_get_time();

        int64_t ready_time = (fr_ready - fr_start)/1000;
        int64_t face_time = (fr_face - fr_ready)/1000;
        int64_t recognize_time = (fr_recognize - fr_face)/1000;
        int64_t encode_time = (fr_encode - fr_recognize)/1000;
        int64_t process_time = (fr_encode - fr_start)/1000;
        
        int64_t frame_time = fr_end - last_frame;
        last_frame = fr_end;
        frame_time /= 1000;
        uint32_t avg_frame_time = ra_filter_run(&ra_filter, frame_time);
        Serial.printf("MJPG: %uB %ums (%.1ffps), AVG: %ums (%.1ffps), %u+%u+%u+%u=%u %s%d\n",
            (uint32_t)(_jpg_buf_len),
            (uint32_t)frame_time, 1000.0 / (uint32_t)frame_time,
            avg_frame_time, 1000.0 / avg_frame_time,
            (uint32_t)ready_time, (uint32_t)face_time, (uint32_t)recognize_time, (uint32_t)encode_time, (uint32_t)process_time,
            (detected)?"DETECTED ":"", face_id
        );
    }
    int64_t ending_time = esp_timer_get_time();
    duration = ending_time - starting_time;
	portENTER_CRITICAL(&timerMux);
    sendLogCounter++;
    portEXIT_CRITICAL(&timerMux);
    last_frame = 0;
    return res;
}


static esp_err_t client_change_handler(httpd_req_t *req){
  
  char content[300];

  int ret = httpd_req_recv(req, content, 300);
  if (ret <= 0) {  /* 0 return value indicates connection closed */
      if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
          httpd_resp_send_408(req);
      }
      return ESP_FAIL;
  }
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

  size_t token_size = httpd_req_get_hdr_value_len(req,  "Authorization")+1;
  char token[token_size];

  if(httpd_req_get_hdr_value_str(req,  "Authorization", token, token_size) != ESP_OK){
    Serial.println("Client Token Replacement Failed: Error during reading Authorization header");
    char json_resp[] = "{\"status\": 401, \"message\": \"Error during reading Authorization header\"}";
    httpd_resp_send(req, json_resp, strlen(json_resp));
    return ESP_OK;
  };

  //Vérifie le token
  if(strcmp(token, client_token) != 0){
      Serial.println("Client Token Replacement Failed: Invalid token");
      char json_resp[] = "{\"status\": 401, \"message\": \"Invalid token\"}";
	    httpd_resp_send(req, json_resp, strlen(json_resp));
      return ESP_OK;
  }

  //Change le token client
  DynamicJsonDocument doc(500);
  deserializeJson(doc, content);
const char* c_token = doc["client_token"];
strcpy(client_token, c_token);
Serial.println("Client token changed to :");
Serial.println(client_token);

char json_resp[] = "{\"status\":200,\"message\":\"Client Token successfully changed !\"}";
return httpd_resp_send(req, json_resp, strlen(json_resp));
}


//Initialisation du server
void startCameraServer(){
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

   httpd_uri_t stream_uri = {
        .uri       = "/stream",
        .method    = HTTP_GET,
        .handler   = stream_handler,
        .user_ctx  = NULL
    };

   httpd_uri_t change_client_uri = {
      .uri = "/client",
      .method = HTTP_POST,
      .handler = client_change_handler,
      .user_ctx = NULL
   };


    ra_filter_init(&ra_filter, 20);
    
    mtmn_config.type = FAST;
    mtmn_config.min_face = 80;
    mtmn_config.pyramid = 0.707;
    mtmn_config.pyramid_times = 4;
    mtmn_config.p_threshold.score = 0.6;
    mtmn_config.p_threshold.nms = 0.7;
    mtmn_config.p_threshold.candidate_number = 20;
    mtmn_config.r_threshold.score = 0.7;
    mtmn_config.r_threshold.nms = 0.7;
    mtmn_config.r_threshold.candidate_number = 10;
    mtmn_config.o_threshold.score = 0.7;
    mtmn_config.o_threshold.nms = 0.7;
    mtmn_config.o_threshold.candidate_number = 1;
    
    
    Serial.printf("Starting web server on port: '%d'\n", config.server_port);
    if (httpd_start(&camera_httpd, &config) == ESP_OK) {
		httpd_register_uri_handler(camera_httpd, &change_client_uri);
    }

    config.server_port += 1;
    config.ctrl_port += 1;
    Serial.printf("Starting stream server on port: '%d'\n", config.server_port);
    if (httpd_start(&stream_httpd, &config) == ESP_OK) {
        httpd_register_uri_handler(stream_httpd, &stream_uri);
    }
}