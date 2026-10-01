#pragma once
#include "esp_http_server.h"
// microSD file API handlers: GET /api/file, POST /api/file (write body), /api/upload (stream body),
// /api/mkdir, /api/mv, /api/rm, /api/mkimg (blank MSX 2DD image).
void url_decode(char *s);
esp_err_t h_file_get(httpd_req_t *r);
esp_err_t h_file_put(httpd_req_t *r);
esp_err_t h_mkdir(httpd_req_t *r);
esp_err_t h_mv(httpd_req_t *r);
esp_err_t h_rm(httpd_req_t *r);
esp_err_t h_mkimg(httpd_req_t *r);
esp_err_t h_imgresize(httpd_req_t *r);
esp_err_t h_romhead(httpd_req_t *r);
