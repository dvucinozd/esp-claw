/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "http_server_priv.h"

#include <stdlib.h>
#include <string.h>

esp_err_t http_server_send_embedded_file(httpd_req_t *req,
                                         const uint8_t *start,
                                         const uint8_t *end,
                                         const char *content_type)
{
    static const size_t k_chunk_size = 1024;
    size_t content_len = (size_t)(end - start);
    if (content_len > 0 && start[content_len - 1] == '\0') {
        content_len--;
    }
    httpd_resp_set_type(req, content_type);
    httpd_resp_set_hdr(req, "Cache-Control", "no-store, max-age=0");
    httpd_resp_set_hdr(req, "Pragma", "no-cache");

    size_t sent = 0;
    while (sent < content_len) {
        size_t chunk_len = content_len - sent;
        if (chunk_len > k_chunk_size) {
            chunk_len = k_chunk_size;
        }

        esp_err_t err = httpd_resp_send_chunk(req, (const char *)(start + sent), chunk_len);
        if (err != ESP_OK) {
            return err;
        }
        sent += chunk_len;
    }

    return httpd_resp_send_chunk(req, NULL, 0);
}

void http_server_json_add_string(cJSON *root, const char *key, const char *value)
{
    cJSON_AddStringToObject(root, key, value ? value : "");
}

esp_err_t http_server_send_json_response(httpd_req_t *req, cJSON *root)
{
    char *payload = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (!payload) {
        httpd_resp_send_500(req);
        return ESP_ERR_NO_MEM;
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store, max-age=0");
    esp_err_t err = httpd_resp_sendstr(req, payload);
    free(payload);
    return err;
}

esp_err_t http_server_parse_json_body(httpd_req_t *req, cJSON **out_root)
{
    if (!out_root || req->content_len <= 0 || req->content_len > 8192) {
        return ESP_ERR_INVALID_ARG;
    }

    char *body = calloc(1, req->content_len + 1);
    if (!body) {
        return ESP_ERR_NO_MEM;
    }

    int received = 0;
    while (received < req->content_len) {
        int ret = httpd_req_recv(req, body + received, req->content_len - received);
        if (ret <= 0) {
            free(body);
            return (ret == HTTPD_SOCK_ERR_TIMEOUT) ? ESP_ERR_TIMEOUT : ESP_FAIL;
        }
        received += ret;
    }

    cJSON *root = cJSON_Parse(body);
    free(body);
    if (!root) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    *out_root = root;
    return ESP_OK;
}

void http_server_json_read_string(cJSON *root, const char *key, char *buffer, size_t buffer_size)
{
    cJSON *item = cJSON_GetObjectItemCaseSensitive(root, key);
    if (cJSON_IsString(item)) {
        strlcpy(buffer, item->valuestring, buffer_size);
    }
}
