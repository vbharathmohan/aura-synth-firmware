/**
 * audio_block.c — Block pool allocator.
 *
 * Pre-allocates POOL_SIZE blocks in a static array (DRAM).
 * Uses a FreeRTOS queue as a lock-free freelist.
 *
 * Memory budget: 32 blocks × 2048 bytes = 64KB DRAM.
 * This is the largest single allocation — tune POOL_SIZE in
 * audio_block.h if you need to trade pool depth for other RAM uses.
 */

#include "audio_block.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_log.h"

static const char *TAG = "audio_block";

/* ------------------------------------------------------------------ */
/* Static pool storage                                                 */
/* ------------------------------------------------------------------ */

/** The actual blocks — statically allocated in DRAM */
static audio_block_t s_pool[POOL_SIZE];

/** Freelist queue — holds pointers to available blocks */
static QueueHandle_t s_free_queue = NULL;

/* ------------------------------------------------------------------ */
/* Init                                                                */
/* ------------------------------------------------------------------ */

bool audio_pool_init(void)
{
    s_free_queue = xQueueCreate(POOL_SIZE, sizeof(audio_block_t *));
    if (s_free_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create block pool queue");
        return false;
    }

    /* Seed the freelist with pointers to every block */
    for (int i = 0; i < POOL_SIZE; i++) {
        audio_block_t *ptr = &s_pool[i];
        xQueueSend(s_free_queue, &ptr, 0);
    }

    ESP_LOGI(TAG, "Block pool initialized: %d blocks × %d bytes = %d KB",
             POOL_SIZE, (int)sizeof(audio_block_t),
             (int)(POOL_SIZE * sizeof(audio_block_t) / 1024));

    return true;
}

/* ------------------------------------------------------------------ */
/* Alloc / free                                                        */
/* ------------------------------------------------------------------ */

audio_block_t *audio_alloc(void)
{
    audio_block_t *blk = NULL;
    if (xQueueReceive(s_free_queue, &blk, pdMS_TO_TICKS(5)) != pdTRUE) {
        ESP_LOGW(TAG, "Pool exhausted — increase POOL_SIZE or check for leaks");
        return NULL;
    }
    return blk;
}

void audio_free(audio_block_t *blk)
{
    if (blk == NULL) return;

    /* Sanity check: is this pointer actually inside our pool? */
    if (blk < &s_pool[0] || blk > &s_pool[POOL_SIZE - 1]) {
        ESP_LOGE(TAG, "Tried to free a block not from the pool!");
        return;
    }

    xQueueSend(s_free_queue, &blk, 0);
}

int audio_pool_available(void)
{
    if (s_free_queue == NULL) return 0;
    return (int)uxQueueMessagesWaiting(s_free_queue);
}
