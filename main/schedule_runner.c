/*
 * schedule_runner.c
 *
 * Polls Firebase for the owner's schedules and fires the feed workflow
 * when the current time matches a schedule entry.
 *
 * Schedule format in Firebase (userSchedules/<uid>/<scheduleId>):
 * {
 *   "time":    "07:30",          // HH:mm (24-hour)
 *   "days":    [1,2,3,4,5,6,7], // 1=Mon … 7=Sun  (ISO 8601 weekday)
 *   "grams":   60,
 *   "enabled": true,
 *   "devices": { "<deviceId>": true, ... }
 * }
 *
 * The runner wakes every 30 seconds.  It fires a schedule at most once
 * per minute by remembering the last HH:mm string it triggered on.
 */

#include "schedule_runner.h"
#include "firebase_client.h"

#include <string.h>
#include <stdio.h>
#include <time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "cJSON.h"

#define TAG              "sched"
#define REFRESH_INTERVAL_S  300    /* re-fetch schedules from Firebase every 5 min */
#define CHECK_INTERVAL_MS   30000  /* check time every 30 s */
#define MAX_SCHEDULES       32

#define UID_MAX_LEN         64
#define DEVICE_ID_MAX_LEN   64

/* ── Internal schedule representation ───────────────────────────────────── */

typedef struct {
    char  id[32];
    char  time_hhmm[6];   /* "HH:mm\0" */
    uint8_t days[7];      /* days[0]=Mon … days[6]=Sun; 1 if active */
    int   grams;
    bool  enabled;
    bool  assigned;       /* true if this device is in the schedule's devices map */
} local_schedule_t;

/* ── Module state ────────────────────────────────────────────────────────── */

static char             s_uid[UID_MAX_LEN]             = {0};
static char             s_device_id[DEVICE_ID_MAX_LEN] = {0};
static feed_callback_t  s_on_feed                      = NULL;

static local_schedule_t s_schedules[MAX_SCHEDULES];
static int              s_schedule_count               = 0;
static SemaphoreHandle_t s_mutex                       = NULL;

static char  s_last_fired_hhmm[6]  = {0};   /* HH:mm we last triggered */
static int   s_last_fired_wday     = -1;     /* weekday we last triggered */

static bool  s_refresh_requested   = false;

/* ── Schedule parsing ────────────────────────────────────────────────────── */

/**
 * Parse the Firebase userSchedules/<uid> snapshot into s_schedules[].
 * Expects json to be a cJSON object whose keys are schedule IDs.
 */
static void parse_schedules(cJSON *root)
{
    if (!root || !cJSON_IsObject(root)) {
        s_schedule_count = 0;
        return;
    }

    int count = 0;
    cJSON *entry = NULL;

    cJSON_ArrayForEach(entry, root) {
        if (count >= MAX_SCHEDULES) break;

        local_schedule_t *s = &s_schedules[count];
        memset(s, 0, sizeof(*s));

        /* ID */
        strncpy(s->id, entry->string, sizeof(s->id) - 1);

        /* enabled */
        cJSON *enabled = cJSON_GetObjectItem(entry, "enabled");
        s->enabled = (!enabled || cJSON_IsTrue(enabled));

        if (!s->enabled) continue;  /* skip disabled schedules */

        /* time */
        cJSON *t = cJSON_GetObjectItem(entry, "time");
        if (!t || !cJSON_IsString(t)) continue;
        strncpy(s->time_hhmm, t->valuestring, sizeof(s->time_hhmm) - 1);

        /* grams */
        cJSON *g = cJSON_GetObjectItem(entry, "grams");
        s->grams = (g && cJSON_IsNumber(g)) ? (int)g->valuedouble : 0;
        if (s->grams <= 0) continue;

        /* days: array of ints 1-7 (1=Mon, 7=Sun) */
        cJSON *days = cJSON_GetObjectItem(entry, "days");
        if (days && cJSON_IsArray(days)) {
            cJSON *d = NULL;
            cJSON_ArrayForEach(d, days) {
                if (cJSON_IsNumber(d)) {
                    int day = (int)d->valuedouble;  /* 1-7 */
                    if (day >= 1 && day <= 7) {
                        s->days[day - 1] = 1;
                    }
                }
            }
        }

        /* devices: check if our device is in the map */
        cJSON *devices = cJSON_GetObjectItem(entry, "devices");
        if (devices && cJSON_IsObject(devices)) {
            cJSON *mine = cJSON_GetObjectItem(devices, s_device_id);
            s->assigned = (mine && cJSON_IsTrue(mine));
        }

        if (!s->assigned) continue;  /* this device not assigned to schedule */

        count++;
    }

    s_schedule_count = count;
    ESP_LOGI(TAG, "Loaded %d active schedule(s) for this device", count);
}

/* ── Firebase fetch ──────────────────────────────────────────────────────── */

static void fetch_schedules(void)
{
    char path[128];
    snprintf(path, sizeof(path), "userSchedules/%s", s_uid);

    cJSON *json = NULL;
    esp_err_t err = firebase_get(path, &json);

    if (err == ESP_OK) {
        xSemaphoreTake(s_mutex, portMAX_DELAY);
        parse_schedules(json);
        xSemaphoreGive(s_mutex);
    } else {
        ESP_LOGW(TAG, "Failed to fetch schedules: %s", esp_err_to_name(err));
    }

    if (json) cJSON_Delete(json);
}

/* ── Time check ──────────────────────────────────────────────────────────── */

/**
 * tm_wday: 0=Sun, 1=Mon … 6=Sat
 * Our days[] array: index 0=Mon … 6=Sun (ISO 8601)
 * Conversion: iso_day = (tm_wday == 0) ? 6 : tm_wday - 1
 */
static int tm_wday_to_iso_index(int tm_wday)
{
    return (tm_wday == 0) ? 6 : tm_wday - 1;
}

static void check_and_fire(void)
{
    time_t now = time(NULL);
    struct tm local_tm;
    localtime_r(&now, &local_tm);

    char current_hhmm[6];
    snprintf(current_hhmm, sizeof(current_hhmm), "%02d:%02d",
             local_tm.tm_hour, local_tm.tm_min);

    int iso_day_idx = tm_wday_to_iso_index(local_tm.tm_wday);

    /* Already fired this minute on this weekday? Skip. */
    if (strcmp(current_hhmm, s_last_fired_hhmm) == 0 &&
        iso_day_idx == s_last_fired_wday) {
        return;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);

    for (int i = 0; i < s_schedule_count; i++) {
        local_schedule_t *s = &s_schedules[i];

        if (!s->enabled || !s->assigned) continue;
        if (strcmp(s->time_hhmm, current_hhmm) != 0) continue;
        if (!s->days[iso_day_idx]) continue;

        ESP_LOGI(TAG, "Schedule '%s' fired at %s — dispensing %d g",
                 s->id, current_hhmm, s->grams);

        /* Record so we don't re-fire this minute */
        strncpy(s_last_fired_hhmm, current_hhmm, sizeof(s_last_fired_hhmm));
        s_last_fired_wday = iso_day_idx;

        xSemaphoreGive(s_mutex);

        if (s_on_feed) {
            s_on_feed(s->grams, "schedule");
        }

        return;  /* only one schedule fires per minute */
    }

    xSemaphoreGive(s_mutex);
}

/* ── Background task ─────────────────────────────────────────────────────── */

static void schedule_task(void *arg)
{
    TickType_t last_refresh = 0;

    /* Initial fetch */
    fetch_schedules();
    last_refresh = xTaskGetTickCount();

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(CHECK_INTERVAL_MS));

        /* Periodic re-fetch */
        TickType_t now_ticks = xTaskGetTickCount();
        bool time_to_refresh = (now_ticks - last_refresh) >=
                               pdMS_TO_TICKS((uint32_t)REFRESH_INTERVAL_S * 1000);

        if (time_to_refresh || s_refresh_requested) {
            s_refresh_requested = false;
            fetch_schedules();
            last_refresh = xTaskGetTickCount();
        }

        check_and_fire();
    }
}

/* ── Public API ──────────────────────────────────────────────────────────── */

void schedule_runner_start(const char *device_id,
                           const char *uid,
                           feed_callback_t on_feed)
{
    strncpy(s_device_id, device_id, sizeof(s_device_id) - 1);
    strncpy(s_uid,       uid,       sizeof(s_uid)       - 1);
    s_on_feed = on_feed;

    s_mutex = xSemaphoreCreateMutex();
    configASSERT(s_mutex);

    xTaskCreate(schedule_task, "sched_runner", 8192, NULL, 4, NULL);
    ESP_LOGI(TAG, "Schedule runner started (device=%s uid=%s)", device_id, uid);
}

void schedule_runner_refresh(void)
{
    s_refresh_requested = true;
}
