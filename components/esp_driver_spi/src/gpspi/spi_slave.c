/*
 * SPDX-FileCopyrightText: 2015-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <stdatomic.h>
#include "esp_types.h"
#include "esp_attr.h"
#include "esp_check.h"
#include "esp_intr_alloc.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_pm.h"
#include "esp_cache.h"
#include "esp_heap_caps.h"
#include "esp_rom_sys.h"
#include "soc/lldesc.h"
#include "soc/soc_caps.h"
#include "soc/spi_periph.h"
#include "soc/soc_memory_layout.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#include "driver/gpio.h"
#include "driver/spi_slave.h"
#include "hal/gpio_hal.h"
#include "esp_private/sleep_retention.h"
#include "esp_private/spi_slave_internal.h"
#include "esp_private/spi_common_internal.h"
// for &GPIO struct
#include "soc/gpio_struct.h"
// gpio_matrix_in, etc:
#include "rom/gpio.h"
#include "esp_private/esp_cache_private.h"
#include "esp_private/spi_share_hw_ctrl.h"

static const char *SPI_TAG = "spi_slave";

#define SPI_CHECK(a, str, ret_val) ESP_RETURN_ON_FALSE(a, ret_val, SPI_TAG, str)

#ifdef CONFIG_SPI_SLAVE_ISR_IN_IRAM
#define SPI_SLAVE_ISR_ATTR IRAM_ATTR
#else
#define SPI_SLAVE_ISR_ATTR
#endif

#ifdef CONFIG_SPI_SLAVE_IN_IRAM
#define SPI_SLAVE_ATTR IRAM_ATTR
#else
#define SPI_SLAVE_ATTR
#endif

// [MCPGC-63]
// Wasn't able to get GDMA to trigger per-byte with SPI2_HOST
// but let's leave in the refactor since it clarifies things
// and might be useful in the future if that changes.
#define HOST_MAIN SPI3_HOST
#define HOST_PEEK SPI2_HOST

/// struct to hold private transaction data (like tx and rx buffer for DMA).
typedef struct
{
    spi_slave_transaction_t *trans; // original trans
    void *tx_buffer;                // actually tx buffer (re-malloced if needed)
    void *rx_buffer;                // actually rx buffer (re-malloced if needed)
} spi_slave_trans_priv_t;

typedef struct
{
    int id;
    _Atomic spi_bus_fsm_t fsm;
    spi_bus_config_t bus_config;
    spi_dma_ctx_t *dma_ctx;
    spi_slave_interface_config_t cfg;
    intr_handle_t intr;
    spi_slave_hal_context_t hal;
    spi_slave_trans_priv_t cur_trans;
    uint32_t flags;
    uint32_t intr_flags;
    int max_transfer_sz;
    QueueHandle_t trans_queue;
    QueueHandle_t ret_queue;
    bool dma_enabled;
    bool cs_iomux;
    uint8_t cs_in_signal;
    uint16_t internal_mem_align_size;
#ifdef CONFIG_PM_ENABLE
    esp_pm_lock_handle_t pm_lock;
#endif
} spi_slave_t;

static spi_slave_t *spihost[SOC_SPI_PERIPH_NUM];

static void spi_intr(void *arg);

__attribute__((always_inline)) static inline bool is_valid_host(spi_host_device_t host)
{
// SPI1 can be used as GPSPI only on ESP32
#if CONFIG_IDF_TARGET_ESP32
    return host >= SPI1_HOST && host <= SPI3_HOST;
#elif (SOC_SPI_PERIPH_NUM == 2)
    return host == SPI2_HOST;
#elif (SOC_SPI_PERIPH_NUM == 3)
    return host >= SPI2_HOST && host <= SPI3_HOST;
#endif
}

static inline bool SPI_SLAVE_ISR_ATTR bus_is_iomux(spi_slave_t *host)
{
    return host->flags & SPICOMMON_BUSFLAG_IOMUX_PINS;
}

static inline void SPI_SLAVE_ISR_ATTR freeze_cs(spi_slave_t *host)
{
#if SPI_LL_SLAVE_NEEDS_CS_WORKAROUND
    // This workaround only for ESP32 due to old hardware design, see MR !3207
    esp_rom_gpio_connect_in_signal(GPIO_MATRIX_CONST_ONE_INPUT, host->cs_in_signal, false);
#endif
}

// Use this function instead of cs_initial to avoid overwrite the output config
// This is used in test by internal gpio matrix connections
static inline void SPI_SLAVE_ISR_ATTR restore_cs(spi_slave_t *host)
{
#if SPI_LL_SLAVE_NEEDS_CS_WORKAROUND
    // This workaround only for ESP32 due to old hardware design, see MR !3207
    if (host->cs_iomux)
    {
        gpio_ll_set_input_signal_from(GPIO_HAL_GET_HW(GPIO_PORT_0), host->cs_in_signal, false);
    }
    else
    {
        esp_rom_gpio_connect_in_signal(host->cfg.spics_io_num, host->cs_in_signal, false);
    }
#endif
}

#if (SOC_CPU_CORES_NUM > 1) && (!CONFIG_FREERTOS_UNICORE)
typedef struct
{
    spi_slave_t *host;
    esp_err_t *err;
} spi_ipc_param_t;

static void ipc_isr_reg_to_core(void *args)
{
    spi_slave_t *host = ((spi_ipc_param_t *)args)->host;
    *((spi_ipc_param_t *)args)->err = esp_intr_alloc(spicommon_irqsource_for_host(host->id), host->intr_flags | ESP_INTR_FLAG_INTRDISABLED, spi_intr, (void *)host, &host->intr);
}
#endif

#if SOC_SPI_SUPPORT_SLEEP_RETENTION && CONFIG_PM_POWER_DOWN_PERIPHERAL_IN_LIGHT_SLEEP
static esp_err_t s_spi_create_sleep_retention_cb(void *arg)
{
    spi_slave_t *context = arg;
    return sleep_retention_entries_create(spi_reg_retention_info[context->id - 1].entry_array,
                                          spi_reg_retention_info[context->id - 1].array_size,
                                          REGDMA_LINK_PRI_GPSPI,
                                          spi_reg_retention_info[context->id - 1].module_id);
}
#endif // SOC_SPI_SUPPORT_SLEEP_RETENTION

esp_err_t spi_slave_initialize(spi_host_device_t host, const spi_bus_config_t *bus_config, const spi_slave_interface_config_t *slave_config, spi_dma_chan_t dma_chan)
{
    bool spi_chan_claimed;
    esp_err_t ret = ESP_OK;
    esp_err_t err;
    SPI_CHECK(is_valid_host(host), "invalid host", ESP_ERR_INVALID_ARG);
#ifdef CONFIG_IDF_TARGET_ESP32
    SPI_CHECK(dma_chan >= SPI_DMA_DISABLED && dma_chan <= SPI_DMA_CH_AUTO, "invalid dma channel", ESP_ERR_INVALID_ARG);
#elif CONFIG_IDF_TARGET_ESP32S2
    SPI_CHECK(dma_chan == SPI_DMA_DISABLED || dma_chan == (int)host || dma_chan == SPI_DMA_CH_AUTO, "invalid dma channel", ESP_ERR_INVALID_ARG);
#elif SOC_GDMA_SUPPORTED
    SPI_CHECK(dma_chan == SPI_DMA_DISABLED || dma_chan == SPI_DMA_CH_AUTO, "invalid dma channel, chip only support spi dma channel auto-alloc", ESP_ERR_INVALID_ARG);
#endif
    SPI_CHECK((bus_config->intr_flags & (ESP_INTR_FLAG_HIGH | ESP_INTR_FLAG_EDGE | ESP_INTR_FLAG_SHARED | ESP_INTR_FLAG_INTRDISABLED)) == 0, "intr flag not allowed", ESP_ERR_INVALID_ARG);
#ifndef CONFIG_SPI_SLAVE_ISR_IN_IRAM
    SPI_CHECK((bus_config->intr_flags & ESP_INTR_FLAG_IRAM) == 0, "ESP_INTR_FLAG_IRAM should be disabled when CONFIG_SPI_SLAVE_ISR_IN_IRAM is not set.", ESP_ERR_INVALID_ARG);
#endif
    SPI_CHECK(slave_config->spics_io_num < 0 || GPIO_IS_VALID_GPIO(slave_config->spics_io_num), "spics pin invalid", ESP_ERR_INVALID_ARG);

    // Check post_trans_cb status when `SPI_SLAVE_NO_RETURN_RESULT` flag is set.
    if (slave_config->flags & SPI_SLAVE_NO_RETURN_RESULT)
    {
        SPI_CHECK(slave_config->post_trans_cb != NULL, "use feature flag 'SPI_SLAVE_NO_RETURN_RESULT' but no post_trans_cb function sets", ESP_ERR_INVALID_ARG);
    }

    spi_chan_claimed = spicommon_periph_claim(host, "spi slave");
    SPI_CHECK(spi_chan_claimed, "host already in use", ESP_ERR_INVALID_STATE);

    // spi_slave_t contains atomic variable, memory must be allocated from internal memory
    spihost[host] = heap_caps_malloc(sizeof(spi_slave_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (spihost[host] == NULL)
    {
        ret = ESP_ERR_NO_MEM;
        goto cleanup;
    }
    memset(spihost[host], 0, sizeof(spi_slave_t));
    memcpy(&spihost[host]->cfg, slave_config, sizeof(spi_slave_interface_config_t));
    memcpy(&spihost[host]->bus_config, bus_config, sizeof(spi_bus_config_t));
    spihost[host]->id = host;
    atomic_store(&spihost[host]->fsm, SPI_BUS_FSM_ENABLED);
    spi_slave_hal_context_t *hal = &spihost[host]->hal;

    spihost[host]->dma_enabled = (dma_chan != SPI_DMA_DISABLED);
    if (spihost[host]->dma_enabled)
    {
        ret = spicommon_dma_chan_alloc(host, dma_chan, &spihost[host]->dma_ctx);
        if (ret != ESP_OK)
        {
            goto cleanup;
        }
        ret = spicommon_dma_desc_alloc(spihost[host]->dma_ctx, bus_config->max_transfer_sz, &spihost[host]->max_transfer_sz);
        if (ret != ESP_OK)
        {
            goto cleanup;
        }

        hal->dmadesc_tx = spihost[host]->dma_ctx->dmadesc_tx;
        hal->dmadesc_rx = spihost[host]->dma_ctx->dmadesc_rx;
        hal->dmadesc_n = spihost[host]->dma_ctx->dma_desc_num;

#if SOC_CACHE_INTERNAL_MEM_VIA_L1CACHE
        size_t alignment;
        esp_cache_get_alignment(MALLOC_CAP_DMA, &alignment);
        spihost[host]->internal_mem_align_size = alignment;
#else
        spihost[host]->internal_mem_align_size = 4;
#endif
    }
    else
    {
        // We're limited to non-DMA transfers: the SPI work registers can hold 64 bytes at most.
        spihost[host]->max_transfer_sz = SOC_SPI_MAXIMUM_BUFFER_SIZE;
    }

    err = spicommon_bus_initialize_io(host, bus_config, SPICOMMON_BUSFLAG_SLAVE | bus_config->flags, &spihost[host]->flags);
    if (err != ESP_OK)
    {
        ret = err;
        goto cleanup;
    }
    if (slave_config->spics_io_num >= 0)
    {
        spicommon_cs_initialize(host, slave_config->spics_io_num, 0, !bus_is_iomux(spihost[host]));
        // check and save where cs line really route through
        spihost[host]->cs_iomux = (slave_config->spics_io_num == spi_periph_signal[host].spics0_iomux_pin) && bus_is_iomux(spihost[host]);
        spihost[host]->cs_in_signal = spi_periph_signal[host].spics_in;
    }

    // The slave DMA suffers from unexpected transactions. Forbid reading if DMA is enabled by disabling the CS line.
    if (spihost[host]->dma_enabled)
    {
        freeze_cs(spihost[host]);
    }

#ifdef CONFIG_PM_ENABLE
#if CONFIG_IDF_TARGET_ESP32P4
    // use CPU_MAX lock to ensure PSRAM bandwidth and usability during DFS
    err = esp_pm_lock_create(ESP_PM_CPU_FREQ_MAX, 0, "spi_slave", &spihost[host]->pm_lock);
#else
    err = esp_pm_lock_create(ESP_PM_APB_FREQ_MAX, 0, "spi_slave", &spihost[host]->pm_lock);
#endif
    if (err != ESP_OK)
    {
        ret = err;
        goto cleanup;
    }
    // Lock APB frequency while SPI slave driver is in use
    esp_pm_lock_acquire(spihost[host]->pm_lock);
#endif // CONFIG_PM_ENABLE

#if SOC_SPI_SUPPORT_SLEEP_RETENTION && CONFIG_PM_POWER_DOWN_PERIPHERAL_IN_LIGHT_SLEEP
    sleep_retention_module_init_param_t init_param = {
        .cbs = {
            .create = {
                .handle = s_spi_create_sleep_retention_cb,
                .arg = spihost[host],
            },
        },
        .depends = RETENTION_MODULE_BITMAP_INIT(CLOCK_SYSTEM),
    };

    if (ESP_OK == sleep_retention_module_init(spi_reg_retention_info[host - 1].module_id, &init_param))
    {
        if ((bus_config->flags & SPICOMMON_BUSFLAG_SLP_ALLOW_PD) && (sleep_retention_module_allocate(spi_reg_retention_info[host - 1].module_id) != ESP_OK))
        {
            // even though the sleep retention create failed, SPI driver should still work, so just warning here
            ESP_LOGW(SPI_TAG, "Alloc sleep recover failed, spi may hold power on");
        }
    }
    else
    {
        // even the sleep retention init failed, SPI driver should still work, so just warning here
        ESP_LOGW(SPI_TAG, "Init sleep recover failed, spi may offline after sleep");
    }
#else
    if (bus_config->flags & SPICOMMON_BUSFLAG_SLP_ALLOW_PD)
    {
        ESP_LOGE(SPI_TAG, "power down peripheral in sleep is not enabled or not supported on your target");
    }
#endif // SOC_SPI_SUPPORT_SLEEP_RETENTION

    // Create queues
    spihost[host]->trans_queue = xQueueCreate(slave_config->queue_size, sizeof(spi_slave_trans_priv_t));
    if (!spihost[host]->trans_queue)
    {
        ret = ESP_ERR_NO_MEM;
        goto cleanup;
    }
    if (!(slave_config->flags & SPI_SLAVE_NO_RETURN_RESULT))
    {
        spihost[host]->ret_queue = xQueueCreate(slave_config->queue_size, sizeof(spi_slave_trans_priv_t));
        if (!spihost[host]->ret_queue)
        {
            ret = ESP_ERR_NO_MEM;
            goto cleanup;
        }
    }

#if (SOC_CPU_CORES_NUM > 1) && (!CONFIG_FREERTOS_UNICORE)
    if (bus_config->isr_cpu_id > ESP_INTR_CPU_AFFINITY_AUTO)
    {
        spihost[host]->intr_flags = bus_config->intr_flags;
        SPI_CHECK(bus_config->isr_cpu_id <= ESP_INTR_CPU_AFFINITY_1, "invalid core id", ESP_ERR_INVALID_ARG);
        spi_ipc_param_t ipc_args = {
            .host = spihost[host],
            .err = &err,
        };
        esp_ipc_call_blocking(ESP_INTR_CPU_AFFINITY_TO_CORE_ID(bus_config->isr_cpu_id), ipc_isr_reg_to_core, (void *)&ipc_args);
    }
    else
#endif
    {
        err = esp_intr_alloc(spicommon_irqsource_for_host(host), bus_config->intr_flags | ESP_INTR_FLAG_INTRDISABLED, spi_intr, (void *)spihost[host], &spihost[host]->intr);
    }
    if (err != ESP_OK)
    {
        ret = err;
        goto cleanup;
    }

    // assign the SPI, RX DMA and TX DMA peripheral registers beginning address
    spi_slave_hal_config_t hal_config = {
        .host_id = host,
    };
    spi_slave_hal_init(hal, &hal_config);

    hal->rx_lsbfirst = (slave_config->flags & SPI_SLAVE_RXBIT_LSBFIRST) ? 1 : 0;
    hal->tx_lsbfirst = (slave_config->flags & SPI_SLAVE_TXBIT_LSBFIRST) ? 1 : 0;
    hal->mode = slave_config->mode;
    hal->use_dma = spihost[host]->dma_enabled;
    spi_slave_hal_setup_device(hal);
    return ESP_OK;

cleanup:
    spi_slave_free(host);
    return ret;
}

esp_err_t spi_slave_free(spi_host_device_t host)
{
    SPI_CHECK(is_valid_host(host), "invalid host", ESP_ERR_INVALID_ARG);
    SPI_CHECK(spihost[host], "host not slave", ESP_ERR_INVALID_ARG);
    if (spihost[host]->trans_queue)
    {
        vQueueDelete(spihost[host]->trans_queue);
    }
    if (spihost[host]->ret_queue)
    {
        vQueueDelete(spihost[host]->ret_queue);
    }
    if (spihost[host]->dma_enabled)
    {
        free(spihost[host]->dma_ctx->dmadesc_tx);
        free(spihost[host]->dma_ctx->dmadesc_rx);
        spicommon_dma_chan_free(spihost[host]->dma_ctx);
    }
    spicommon_bus_free_io_cfg(&spihost[host]->bus_config);
    esp_intr_free(spihost[host]->intr);

#if SOC_SPI_SUPPORT_SLEEP_RETENTION && CONFIG_PM_POWER_DOWN_PERIPHERAL_IN_LIGHT_SLEEP
    const periph_retention_module_t retention_id = spi_reg_retention_info[spihost[host]->id - 1].module_id;
    if (sleep_retention_is_module_created(retention_id))
    {
        assert(sleep_retention_is_module_inited(retention_id));
        sleep_retention_module_free(retention_id);
    }
    if (sleep_retention_is_module_inited(retention_id))
    {
        sleep_retention_module_deinit(retention_id);
    }
#endif
#ifdef CONFIG_PM_ENABLE
    if (spihost[host]->pm_lock)
    {
        esp_pm_lock_release(spihost[host]->pm_lock);
        esp_pm_lock_delete(spihost[host]->pm_lock);
    }
#endif // CONFIG_PM_ENABLE
    free(spihost[host]);
    spihost[host] = NULL;
    spicommon_periph_free(host);
    return ESP_OK;
}

esp_err_t spi_slave_enable(spi_host_device_t host)
{
    SPI_CHECK(is_valid_host(host), "invalid host", ESP_ERR_INVALID_ARG);
    SPI_CHECK(spihost[host], "host not slave or not initialized", ESP_ERR_INVALID_ARG);
    spi_bus_fsm_t curr_sta = SPI_BUS_FSM_DISABLED;
    SPI_CHECK(atomic_compare_exchange_strong(&spihost[host]->fsm, &curr_sta, SPI_BUS_FSM_ENABLED), "host already enabled", ESP_ERR_INVALID_STATE);

#ifdef CONFIG_PM_ENABLE
    esp_pm_lock_acquire(spihost[host]->pm_lock);
#endif // CONFIG_PM_ENABLE

// If going to TOP_PD power down, the bus_clock is required during reg_dma, and will be disabled by sleep flow then
#if !CONFIG_PM_POWER_DOWN_PERIPHERAL_IN_LIGHT_SLEEP
    SPI_COMMON_RCC_CLOCK_ATOMIC()
    {
        spi_ll_enable_bus_clock(host, true);
    }
#endif
    return ESP_OK;
}

esp_err_t spi_slave_disable(spi_host_device_t host)
{
    SPI_CHECK(is_valid_host(host), "invalid host", ESP_ERR_INVALID_ARG);
    SPI_CHECK(spihost[host], "host not slave or not initialized", ESP_ERR_INVALID_ARG);
    spi_bus_fsm_t curr_sta = SPI_BUS_FSM_ENABLED;
    SPI_CHECK(atomic_compare_exchange_strong(&spihost[host]->fsm, &curr_sta, SPI_BUS_FSM_DISABLED), "host already disabled", ESP_ERR_INVALID_STATE);

#ifdef CONFIG_PM_ENABLE
    esp_pm_lock_release(spihost[host]->pm_lock);
#endif // CONFIG_PM_ENABLE

// same as above
#if !CONFIG_PM_POWER_DOWN_PERIPHERAL_IN_LIGHT_SLEEP
    SPI_COMMON_RCC_CLOCK_ATOMIC()
    {
        spi_ll_enable_bus_clock(host, false);
    }
#endif
    return ESP_OK;
}

static void SPI_SLAVE_ISR_ATTR spi_slave_uninstall_priv_trans(spi_host_device_t host, spi_slave_trans_priv_t *priv_trans)
{
#if SOC_CACHE_INTERNAL_MEM_VIA_L1CACHE
    spi_slave_transaction_t *trans = (spi_slave_transaction_t *)priv_trans->trans;

    if (spihost[host]->dma_enabled)
    {
        if (trans->tx_buffer && (trans->tx_buffer != priv_trans->tx_buffer))
        {
            free(priv_trans->tx_buffer);
        }
        if (trans->rx_buffer && (trans->rx_buffer != priv_trans->rx_buffer))
        {
            memcpy(trans->rx_buffer, priv_trans->rx_buffer, (trans->length + 7) / 8);
            free(priv_trans->rx_buffer);
        }
    }
#endif // SOC_CACHE_INTERNAL_MEM_VIA_L1CACHE
}

static esp_err_t SPI_SLAVE_ISR_ATTR spi_slave_setup_priv_trans(spi_host_device_t host, spi_slave_trans_priv_t *priv_trans)
{
    spi_slave_transaction_t *trans = (spi_slave_transaction_t *)priv_trans->trans;

    priv_trans->tx_buffer = (void *)trans->tx_buffer;
    priv_trans->rx_buffer = trans->rx_buffer;

#if SOC_CACHE_INTERNAL_MEM_VIA_L1CACHE
    uint16_t alignment = spihost[host]->internal_mem_align_size;
    uint32_t buffer_byte_len = (trans->length + 7) / 8;

    if (spihost[host]->dma_enabled && trans->tx_buffer)
    {
        if ((!esp_ptr_dma_capable(trans->tx_buffer) || ((((uint32_t)trans->tx_buffer) | buffer_byte_len) & (alignment - 1))))
        {
            ESP_RETURN_ON_FALSE_ISR(trans->flags & SPI_SLAVE_TRANS_DMA_BUFFER_ALIGN_AUTO, ESP_ERR_INVALID_ARG, SPI_TAG, "TX buffer addr&len not align to %d byte, or not dma_capable", alignment);
            // if txbuf in the desc not DMA-capable, or not align to "alignment", malloc a new one
            ESP_EARLY_LOGD(SPI_TAG, "Allocate TX buffer for DMA");
            buffer_byte_len = (buffer_byte_len + alignment - 1) & (~(alignment - 1)); // up align to "alignment"
            uint32_t *temp = heap_caps_aligned_alloc(alignment, buffer_byte_len, MALLOC_CAP_DMA);
            if (temp == NULL)
            {
                return ESP_ERR_NO_MEM;
            }

            memcpy(temp, trans->tx_buffer, (trans->length + 7) / 8);
            priv_trans->tx_buffer = temp;
        }
        esp_err_t ret = esp_cache_msync((void *)priv_trans->tx_buffer, buffer_byte_len, ESP_CACHE_MSYNC_FLAG_DIR_C2M);
        ESP_RETURN_ON_FALSE_ISR(ESP_OK == ret, ESP_ERR_INVALID_STATE, SPI_TAG, "mem sync c2m(writeback) fail");
    }
    if (spihost[host]->dma_enabled && trans->rx_buffer)
    {
        if ((!esp_ptr_dma_capable(trans->rx_buffer) || ((((uint32_t)trans->rx_buffer) | (trans->length + 7) / 8) & (alignment - 1))))
        {
            ESP_RETURN_ON_FALSE_ISR(trans->flags & SPI_SLAVE_TRANS_DMA_BUFFER_ALIGN_AUTO, ESP_ERR_INVALID_ARG, SPI_TAG, "RX buffer addr&len not align to %d byte, or not dma_capable", alignment);
            // if rxbuf in the desc not DMA-capable, or not align to "alignment", malloc a new one
            ESP_EARLY_LOGD(SPI_TAG, "Allocate RX buffer for DMA");
            buffer_byte_len = (buffer_byte_len + alignment - 1) & (~(alignment - 1)); // up align to "alignment"
            priv_trans->rx_buffer = heap_caps_aligned_alloc(alignment, buffer_byte_len, MALLOC_CAP_DMA);
            if (priv_trans->rx_buffer == NULL)
            {
                free(priv_trans->tx_buffer);
                return ESP_ERR_NO_MEM;
            }
        }
        esp_err_t ret = esp_cache_msync((void *)priv_trans->rx_buffer, buffer_byte_len, ESP_CACHE_MSYNC_FLAG_DIR_M2C);
        ESP_RETURN_ON_FALSE_ISR(ESP_OK == ret, ESP_ERR_INVALID_STATE, SPI_TAG, "mem sync m2c(invalid) fail");
    }
#endif // SOC_CACHE_INTERNAL_MEM_VIA_L1CACHE
    return ESP_OK;
}

esp_err_t SPI_SLAVE_ATTR spi_slave_queue_trans(spi_host_device_t host, const spi_slave_transaction_t *trans_desc, TickType_t ticks_to_wait)
{
    BaseType_t r;
    SPI_CHECK(is_valid_host(host), "invalid host", ESP_ERR_INVALID_ARG);
    SPI_CHECK(spihost[host], "host not slave", ESP_ERR_INVALID_ARG);
    SPI_CHECK(spihost[host]->dma_enabled == 0 || trans_desc->tx_buffer == NULL || esp_ptr_dma_capable(trans_desc->tx_buffer),
              "txdata not in DMA-capable memory", ESP_ERR_INVALID_ARG);

    // We don't check length WORD alignment for rx when using DMA, seems break DMA requirement,
    // however peripheral can also stop DMA from over writing memory even if it not aligned (except esp32).
    // ATTENTION!: On esp32, peripheral can NOT stop DMA, if length not WORD aligned,
    // remain bytes in last word domain will overwritten by DMA HW, which may cause unexpected issues!
    // But driver already used for long time, to avoid breaking changes, we still don't add alignment limit.
    SPI_CHECK(spihost[host]->dma_enabled == 0 || trans_desc->rx_buffer == NULL ||
                  (esp_ptr_dma_capable(trans_desc->rx_buffer) && esp_ptr_word_aligned(trans_desc->rx_buffer) &&
                   (trans_desc->length % 8 == 0)),
              "rxdata not in DMA-capable memory or not BYTE aligned", ESP_ERR_INVALID_ARG);

    SPI_CHECK(trans_desc->length <= spihost[host]->max_transfer_sz * 8, "data transfer > host maximum", ESP_ERR_INVALID_ARG);

    spi_slave_trans_priv_t priv_trans = {.trans = (spi_slave_transaction_t *)trans_desc};
    SPI_CHECK(ESP_OK == spi_slave_setup_priv_trans(host, &priv_trans), "slave setup priv_trans failed", ESP_ERR_NO_MEM);

    r = xQueueSend(spihost[host]->trans_queue, (void *)&priv_trans, ticks_to_wait);
    if (!r)
    {
        return ESP_ERR_TIMEOUT;
    }
    esp_intr_enable(spihost[host]->intr);
    return ESP_OK;
}

/**
 * @note
 * This API is used to reset SPI Slave transaction queue. After calling this function:
 * - The SPI Slave transaction queue will be reset.
 * - The transaction which already mount on hardware will NOT be reset, and can be overwritten by next `trans_queue`
 *
 * Therefore, this API shouldn't be called when the corresponding SPI Master is doing an SPI transaction.
 *
 * @note
 * We don't actually need to enter a critical section here.
 * SPI Slave ISR will only get triggered when its corresponding SPI Master's transaction is done.
 * As we don't expect this function to be called when its corresponding SPI Master is doing an SPI transaction,
 * so concurrent call to these registers won't happen
 *
 */
esp_err_t SPI_SLAVE_ATTR spi_slave_queue_reset(spi_host_device_t host)
{
    SPI_CHECK(is_valid_host(host), "invalid host", ESP_ERR_INVALID_ARG);
    SPI_CHECK(spihost[host], "host not slave", ESP_ERR_INVALID_ARG);

    esp_intr_disable(spihost[host]->intr);
    spi_ll_set_int_stat(spihost[host]->hal.hw);

    spi_slave_trans_priv_t trans;
    while (uxQueueMessagesWaiting(spihost[host]->trans_queue))
    {
        SPI_CHECK(pdTRUE == xQueueReceive(spihost[host]->trans_queue, &trans, 0), "can't reset queue", ESP_ERR_INVALID_STATE);
        spi_slave_uninstall_priv_trans(host, &trans);
    }
    spihost[host]->cur_trans.trans = NULL;

    return ESP_OK;
}

esp_err_t SPI_SLAVE_ISR_ATTR spi_slave_queue_trans_isr(spi_host_device_t host, const spi_slave_transaction_t *trans_desc)
{
    BaseType_t r;
    BaseType_t do_yield = pdFALSE;
    ESP_RETURN_ON_FALSE_ISR(is_valid_host(host), ESP_ERR_INVALID_ARG, SPI_TAG, "invalid host");
    ESP_RETURN_ON_FALSE_ISR(spihost[host], ESP_ERR_INVALID_ARG, SPI_TAG, "host not slave");
    ESP_RETURN_ON_FALSE_ISR(trans_desc->length <= spihost[host]->max_transfer_sz * 8, ESP_ERR_INVALID_ARG, SPI_TAG, "data transfer > host maximum");
    if (spihost[host]->dma_enabled)
    {
        uint16_t alignment = spihost[host]->internal_mem_align_size;
        (void)alignment;

#if SOC_CACHE_INTERNAL_MEM_VIA_L1CACHE
        // For those targets length and addr alignment is still required from Cache side
        uint32_t buffer_byte_len = (trans_desc->length + 7) / 8;
        bool tx_aligned = (trans_desc->tx_buffer == NULL) || (esp_ptr_dma_capable(trans_desc->tx_buffer) && ((((uint32_t)trans_desc->tx_buffer | buffer_byte_len) & (alignment - 1)) == 0));
        bool rx_aligned = (trans_desc->rx_buffer == NULL) || (esp_ptr_dma_capable(trans_desc->rx_buffer) && ((((uint32_t)trans_desc->rx_buffer | buffer_byte_len) & (alignment - 1)) == 0));
#else
        bool tx_aligned = (trans_desc->tx_buffer == NULL) || esp_ptr_dma_capable(trans_desc->tx_buffer);
        bool rx_aligned = (trans_desc->rx_buffer == NULL) || (esp_ptr_dma_capable(trans_desc->rx_buffer) && esp_ptr_word_aligned(trans_desc->rx_buffer) && (trans_desc->length % 8 == 0));
#endif

        ESP_RETURN_ON_FALSE_ISR(tx_aligned, ESP_ERR_INVALID_ARG, SPI_TAG, "txdata addr & len not align to %d bytes or not dma_capable", alignment);
        ESP_RETURN_ON_FALSE_ISR(rx_aligned, ESP_ERR_INVALID_ARG, SPI_TAG, "rxdata addr & len not align to %d bytes or not dma_capable", alignment);
    }

    spi_slave_trans_priv_t priv_trans = {
        .trans = (spi_slave_transaction_t *)trans_desc,
        .tx_buffer = (void *)trans_desc->tx_buffer,
        .rx_buffer = trans_desc->rx_buffer,
    };
    r = xQueueSendFromISR(spihost[host]->trans_queue, (void *)&priv_trans, &do_yield);
    if (!r)
    {
        return ESP_ERR_NO_MEM;
    }
    if (do_yield)
    {
        portYIELD_FROM_ISR();
    }
    return ESP_OK;
}

esp_err_t SPI_SLAVE_ISR_ATTR spi_slave_queue_reset_isr(spi_host_device_t host)
{
    ESP_RETURN_ON_FALSE_ISR(is_valid_host(host), ESP_ERR_INVALID_ARG, SPI_TAG, "invalid host");
    ESP_RETURN_ON_FALSE_ISR(spihost[host], ESP_ERR_INVALID_ARG, SPI_TAG, "host not slave");

    esp_err_t err = ESP_OK;
    spi_slave_trans_priv_t trans;
    BaseType_t do_yield = pdFALSE;
    while (pdFALSE == xQueueIsQueueEmptyFromISR(spihost[host]->trans_queue))
    {
        if (pdTRUE != xQueueReceiveFromISR(spihost[host]->trans_queue, &trans, &do_yield))
        {
            err = ESP_ERR_INVALID_STATE;
            break;
        }
        spi_slave_uninstall_priv_trans(host, &trans);
    }
    if (do_yield)
    {
        portYIELD_FROM_ISR();
    }
    ESP_RETURN_ON_ERROR_ISR(err, SPI_TAG, "can't reset queue");

    spihost[host]->cur_trans.trans = NULL;
    return ESP_OK;
}

esp_err_t SPI_SLAVE_ATTR spi_slave_get_trans_result(spi_host_device_t host, spi_slave_transaction_t **trans_desc, TickType_t ticks_to_wait)
{
    BaseType_t r;
    SPI_CHECK(is_valid_host(host), "invalid host", ESP_ERR_INVALID_ARG);
    SPI_CHECK(spihost[host], "host not slave", ESP_ERR_INVALID_ARG);
    // if SPI_SLAVE_NO_RETURN_RESULT is set, ret_queue will always be empty
    SPI_CHECK(!(spihost[host]->cfg.flags & SPI_SLAVE_NO_RETURN_RESULT), "API not Supported!", ESP_ERR_NOT_SUPPORTED);

    spi_slave_trans_priv_t priv_trans;
    r = xQueueReceive(spihost[host]->ret_queue, (void *)&priv_trans, ticks_to_wait);
    if (!r)
    {
        return ESP_ERR_TIMEOUT;
    }

    spi_slave_uninstall_priv_trans(host, &priv_trans);
    *trans_desc = priv_trans.trans;
    return ESP_OK;
}

esp_err_t SPI_SLAVE_ATTR spi_slave_transmit(spi_host_device_t host, spi_slave_transaction_t *trans_desc, TickType_t ticks_to_wait)
{
    esp_err_t ret;
    spi_slave_transaction_t *ret_trans;
    // ToDo: check if any spi transfers in flight
    ret = spi_slave_queue_trans(host, trans_desc, ticks_to_wait);
    if (ret != ESP_OK)
    {
        return ret;
    }
    ret = spi_slave_get_trans_result(host, &ret_trans, ticks_to_wait);
    if (ret != ESP_OK)
    {
        return ret;
    }
    assert(ret_trans == trans_desc);
    return ESP_OK;
}

#if SOC_GDMA_SUPPORTED // AHB_DMA_V1 and AXI_DMA
// dma is provided by gdma driver on these targets
#define spi_dma_reset gdma_reset
#define spi_dma_start(chan, addr) gdma_start(chan, (intptr_t)(addr))
#endif

static void SPI_SLAVE_ISR_ATTR s_spi_slave_dma_prepare_data(spi_dma_ctx_t *dma_ctx, spi_slave_hal_context_t *hal)
{
    if (hal->rx_buffer)
    {
        spicommon_dma_desc_setup_link(dma_ctx->dmadesc_rx, hal->rx_buffer, ((hal->bitlen + 7) / 8), true);

        spi_dma_reset(dma_ctx->rx_dma_chan);
        spi_slave_hal_hw_prepare_rx(hal->hw);
        spi_dma_start(dma_ctx->rx_dma_chan, dma_ctx->dmadesc_rx);
    }
    if (hal->tx_buffer)
    {
        spicommon_dma_desc_setup_link(dma_ctx->dmadesc_tx, hal->tx_buffer, (hal->bitlen + 7) / 8, false);

        spi_dma_reset(dma_ctx->tx_dma_chan);
        spi_slave_hal_hw_prepare_tx(hal->hw);
        spi_dma_start(dma_ctx->tx_dma_chan, dma_ctx->dmadesc_tx);
    }
}

static void SPI_SLAVE_ISR_ATTR s_spi_slave_prepare_data(spi_slave_t *host)
{
    spi_slave_hal_context_t *hal = &host->hal;

    if (host->dma_enabled)
    {
        s_spi_slave_dma_prepare_data(host->dma_ctx, &host->hal);
    }
    else
    {
        // No DMA. Copy data to transmit buffers.
        spi_slave_hal_push_tx_buffer(hal);
        spi_slave_hal_hw_fifo_reset(hal, true, false);
    }
    spi_slave_hal_set_trans_bitlen(hal);

#ifdef CONFIG_IDF_TARGET_ESP32
    // SPI Slave mode on ESP32 requires MOSI/MISO enable
    spi_slave_hal_enable_data_line(hal);
#endif
}

#if CONFIG_IDF_TARGET_ESP32
static void SPI_SLAVE_ISR_ATTR spi_slave_restart_after_dmareset(void *arg)
{
    spi_slave_t *host = (spi_slave_t *)arg;
    esp_intr_enable(host->intr);
}
#endif // #if CONFIG_IDF_TARGET_ESP32

// This is run in interrupt context and apart from initialization and destruction, this is the only code
// touching the host (=spihost[x]) variable. The rest of the data arrives in queues. That is why there are
// no muxes in this code.
static void SPI_SLAVE_ISR_ATTR spi_intr(void *arg)
{
    BaseType_t r;
    BaseType_t do_yield = pdFALSE;
    spi_slave_t *host = (spi_slave_t *)arg;
    spi_slave_hal_context_t *hal = &host->hal;

    assert(spi_slave_hal_usr_is_done(hal));

    bool use_dma = host->dma_enabled;
    if (host->cur_trans.trans)
    {
        // When DMA is enabled, the slave rx dma suffers from unexpected transactions. Forbid reading until transaction ready.
        if (use_dma)
        {
            freeze_cs(host);
        }

        spi_slave_hal_store_result(hal);
        host->cur_trans.trans->trans_len = spi_slave_hal_get_rcv_bitlen(hal);

#if CONFIG_IDF_TARGET_ESP32
        // This workaround is only for esp32
        if (spi_slave_hal_dma_need_reset(hal))
        {
            // On ESP32, actual_tx_dma_chan and actual_rx_dma_chan are always same
            spicommon_dmaworkaround_req_reset(host->dma_ctx->tx_dma_chan.chan_id, spi_slave_restart_after_dmareset, host);
        }
#endif // #if CONFIG_IDF_TARGET_ESP32

#if SOC_CACHE_INTERNAL_MEM_VIA_L1CACHE // invalidate here to let user access rx data in post_cb if possible
        if (use_dma && host->cur_trans.rx_buffer)
        {
            uint16_t alignment = host->internal_mem_align_size;
            uint32_t buffer_byte_len = (host->cur_trans.trans->length + 7) / 8;
            buffer_byte_len = (buffer_byte_len + alignment - 1) & (~(alignment - 1));
            // invalidate priv_trans.buffer_to_rcv anyway, only user provide aligned buffer can rcv correct data in post_cb
            esp_err_t ret = esp_cache_msync((void *)host->cur_trans.rx_buffer, buffer_byte_len, ESP_CACHE_MSYNC_FLAG_DIR_M2C);
            assert(ret == ESP_OK);
        }
#endif
        if (host->cfg.post_trans_cb)
        {
            host->cfg.post_trans_cb(host->cur_trans.trans);
        }

        if (!(host->cfg.flags & SPI_SLAVE_NO_RETURN_RESULT))
        {
            xQueueSendFromISR(host->ret_queue, &host->cur_trans, &do_yield);
        }
        host->cur_trans.trans = NULL;
    }

#if CONFIG_IDF_TARGET_ESP32
    // This workaround is only for esp32
    if (use_dma)
    {
        // On ESP32, actual_tx_dma_chan and actual_rx_dma_chan are always same
        spicommon_dmaworkaround_idle(host->dma_ctx->tx_dma_chan.chan_id);
        if (spicommon_dmaworkaround_reset_in_progress())
        {
            // We need to wait for the reset to complete. Disable int (will be re-enabled on reset callback) and exit isr.
            esp_intr_disable(host->intr);
            if (do_yield)
            {
                portYIELD_FROM_ISR();
            }
            return;
        }
    }
#endif // #if CONFIG_IDF_TARGET_ESP32

    // Disable interrupt before checking to avoid concurrency issue.
    esp_intr_disable(host->intr);
    spi_slave_trans_priv_t priv_trans;
    // Grab next transaction
    r = xQueueReceiveFromISR(host->trans_queue, &priv_trans, &do_yield);
    if (r)
    {
        // sanity check
        assert(priv_trans.trans);

        // enable the interrupt again if there is packet to send
        esp_intr_enable(host->intr);

        // We have a transaction. Send it.
        host->cur_trans = priv_trans;

        hal->bitlen = priv_trans.trans->length;
        hal->rx_buffer = priv_trans.rx_buffer;
        hal->tx_buffer = priv_trans.tx_buffer;

#if CONFIG_IDF_TARGET_ESP32
        if (use_dma)
        {
            // This workaround is only for esp32
            // On ESP32, actual_tx_dma_chan and actual_rx_dma_chan are always same
            spicommon_dmaworkaround_transfer_active(host->dma_ctx->tx_dma_chan.chan_id);
        }
#endif // #if CONFIG_IDF_TARGET_ESP32

        spi_slave_hal_hw_reset(hal);
        s_spi_slave_prepare_data(host);

        // The slave rx dma get disturbed by unexpected transaction. Only connect the CS and start DMA when slave is ready.
        if (use_dma)
        {
            restore_cs(host);
        }

        // Kick off transfer
        spi_slave_hal_user_start(hal);
        if (host->cfg.post_setup_cb)
        {
            host->cfg.post_setup_cb(priv_trans.trans);
        }
    }
    if (do_yield)
    {
        portYIELD_FROM_ISR();
    }
}

//
// Begin MCPro Mods
//

// For GDMA
#include "soc/gdma_struct.h"
#include "hal/gdma_ll.h"

#pragma GCC push_options
#pragma GCC optimize("-O3")

// TODO: can I remove this now?
static uint32_t *localIntrFlags = 0;

static void SPI_SLAVE_ISR_ATTR dummyInterruptCallback(void *arg)
{
    return;
}

static spi_slave_hal_context_t *activeHal = NULL;
spi_dma_ctx_t *dmaCtx = NULL;
static uint32_t activeRxChan = 0;
static uint32_t activeTxChan = 0;

// formerly spi_slave_init_lite
esp_err_t SpiSlaveInitLite(spi_host_device_t host, const spi_bus_config_t *bus_config, const spi_slave_interface_config_t *slave_config, spi_dma_chan_t dma_chan, uint32_t *inIntrFlags)
{

    // uint32_t CHUNK_SIZE = SPI_MAX_DMA_LEN
    //  we wan 256 byte dma
    uint32_t CHUNK_SIZE = 256;

    localIntrFlags = inIntrFlags;

    bool spi_chan_claimed;
    uint32_t actual_tx_dma_chan = 0;
    uint32_t actual_rx_dma_chan = 0;

    esp_err_t ret = ESP_OK;
    esp_err_t err;
    SPI_CHECK(is_valid_host(host), "invalid host", ESP_ERR_INVALID_ARG);

    SPI_CHECK((bus_config->intr_flags & (ESP_INTR_FLAG_HIGH | ESP_INTR_FLAG_EDGE | ESP_INTR_FLAG_INTRDISABLED)) == 0, "intr flag not allowed", ESP_ERR_INVALID_ARG);

#ifndef CONFIG_SPI_SLAVE_ISR_IN_IRAM
    SPI_CHECK((bus_config->intr_flags & ESP_INTR_FLAG_IRAM) == 0, "ESP_INTR_FLAG_IRAM should be disabled when CONFIG_SPI_SLAVE_ISR_IN_IRAM is not set.", ESP_ERR_INVALID_ARG);
#endif
    SPI_CHECK(slave_config->spics_io_num < 0 || GPIO_IS_VALID_GPIO(slave_config->spics_io_num), "spics pin invalid", ESP_ERR_INVALID_ARG);

    spi_chan_claimed = spicommon_periph_claim(host, "spi slave");
    SPI_CHECK(spi_chan_claimed, "host already in use", ESP_ERR_INVALID_STATE);

    spihost[host] = malloc(sizeof(spi_slave_t));
    if (spihost[host] == NULL)
    {
        ret = ESP_ERR_NO_MEM;
        goto cleanup;
    }
    memset(spihost[host], 0, sizeof(spi_slave_t));
    memcpy(&spihost[host]->cfg, slave_config, sizeof(spi_slave_interface_config_t));
    spihost[host]->id = host;

    bool use_dma = (dma_chan != SPI_DMA_DISABLED);
    spihost[host]->dma_enabled = use_dma;
    if (use_dma)
    {
        ret = spicommon_dma_chan_alloc(host, dma_chan, &dmaCtx);
        if (ret != ESP_OK)
        {
            goto cleanup;
        }
    }

    err = spicommon_bus_initialize_io(host, bus_config, SPICOMMON_BUSFLAG_SLAVE | bus_config->flags, &spihost[host]->flags);
    if (err != ESP_OK)
    {
        ret = err;
        goto cleanup;
    }
    if (slave_config->spics_io_num >= 0)
    {
        spicommon_cs_initialize(host, slave_config->spics_io_num, 0, !bus_is_iomux(spihost[host]));
    }

    // The slave DMA suffers from unexpected transactions. Forbid reading if DMA is enabled by disabling the CS line.
    // if (use_dma) freeze_cs(spihost[host]);

    int dma_desc_ct = 0;
    spihost[host]->dma_ctx = dmaCtx;
    if (use_dma)
    {
        // See how many dma descriptors we need and allocate them
        // dma_desc_ct = (bus_config->max_transfer_sz + CHUNK_SIZE - 1) / CHUNK_SIZE;
        dma_desc_ct = 2;
        if (dma_desc_ct == 0)
            dma_desc_ct = 1; // default to 4k when max is not given
        spihost[host]->max_transfer_sz = dma_desc_ct * CHUNK_SIZE;
    }
    else
    {
        // We're limited to non-DMA transfers: the SPI work registers can hold 64 bytes at most.
        spihost[host]->max_transfer_sz = SOC_SPI_MAXIMUM_BUFFER_SIZE;
    }

    printf("Num descs = %d\n", dma_desc_ct);

#ifdef CONFIG_PM_ENABLE
    err = esp_pm_lock_create(ESP_PM_APB_FREQ_MAX, 0, "spi_slave",
                             &spihost[host]->pm_lock);
    if (err != ESP_OK)
    {
        ret = err;
        goto cleanup;
    }
    // Lock APB frequency while SPI slave driver is in use
    esp_pm_lock_acquire(spihost[host]->pm_lock);
#endif // CONFIG_PM_ENABLE

    /*
    //Create queues
    spihost[host]->trans_queue = xQueueCreate(slave_config->queue_size, sizeof(spi_slave_transaction_t *));
    spihost[host]->ret_queue = xQueueCreate(slave_config->queue_size, sizeof(spi_slave_transaction_t *));
    if (!spihost[host]->trans_queue || !spihost[host]->ret_queue) {
        ret = ESP_ERR_NO_MEM;
        goto cleanup;
    }
    */

    int flags = bus_config->intr_flags | ESP_INTR_FLAG_INTRDISABLED;

    // this is in the original source
    // you can still use it with DMA off, if you set DMA OFF, and the dma_rx_ena, etc are disabled.

    uint32_t irqSource = spicommon_irqsource_for_host(host);
    printf("assigning irq source %lx\n", irqSource);
    err = esp_intr_alloc(irqSource, flags, dummyInterruptCallback, (void *)spihost[host], &spihost[host]->intr);
    if (err != ESP_OK)
    {
        ret = err;
        goto cleanup;
    }

    spi_slave_hal_context_t *hal = &spihost[host]->hal;
    // assign the SPI, RX DMA and TX DMA peripheral registers beginning address
    spi_slave_hal_config_t hal_config = {
        .host_id = host,
    };
    spi_slave_hal_init(hal, &hal_config);

    if (dma_desc_ct)
    {
        hal->dmadesc_tx = heap_caps_malloc(sizeof(lldesc_t) * dma_desc_ct, MALLOC_CAP_DMA);
        hal->dmadesc_rx = heap_caps_malloc(sizeof(lldesc_t) * dma_desc_ct, MALLOC_CAP_DMA);
        if (!hal->dmadesc_tx || !hal->dmadesc_rx)
        {
            ret = ESP_ERR_NO_MEM;
            goto cleanup;
        }
    }
    hal->dmadesc_n = dma_desc_ct;
    hal->rx_lsbfirst = (slave_config->flags & SPI_SLAVE_RXBIT_LSBFIRST) ? 1 : 0;
    hal->tx_lsbfirst = (slave_config->flags & SPI_SLAVE_TXBIT_LSBFIRST) ? 1 : 0;
    hal->mode = slave_config->mode;
    hal->use_dma = use_dma;
    // hal->tx_dma_chan = actual_tx_dma_chan;
    // hal->rx_dma_chan = actual_rx_dma_chan;

    spi_slave_hal_setup_device(hal);

    // GDMA.channel[0].in.wight.rx_weight = 0xFFFFFFFF;
    GDMA.channel[0].out.weight.tx_weight = 0xFFFFFFFF;
    GDMA.channel[0].in.pri.rx_pri = 1;
    GDMA.channel[0].out.pri.tx_pri = 9;
    // GDMA.channel[0].in.conf1.dma_infifo_full_thrs = 4;
    // GDMA.channel[0].out.conf1.reserved0 = 4;

    // access to all ints
    GDMA.channel[0].out.int_ena.val = 0xFFFFFFFF;
    GDMA.channel[0].in.int_ena.val = 0xFFFFFFFF;

    return ESP_OK;

cleanup:
    if (spihost[host])
    {
        if (spihost[host]->trans_queue)
            vQueueDelete(spihost[host]->trans_queue);
        if (spihost[host]->ret_queue)
            vQueueDelete(spihost[host]->ret_queue);
        free(spihost[host]->hal.dmadesc_tx);
        free(spihost[host]->hal.dmadesc_rx);
#ifdef CONFIG_PM_ENABLE
        if (spihost[host]->pm_lock)
        {
            esp_pm_lock_release(spihost[host]->pm_lock);
            esp_pm_lock_delete(spihost[host]->pm_lock);
        }
#endif
    }
    spi_slave_hal_deinit(&spihost[host]->hal);
    if (spihost[host]->dma_enabled)
    {
        spicommon_dma_chan_free(dmaCtx);
    }

    free(spihost[host]);
    spihost[host] = NULL;
    spicommon_periph_free(host);

    return ret;

} // SpiSlaveInitLite

static IRAM_ATTR void QuickLink_SingleDescriptor(dma_descriptor_t *dmadesc, const void *data, int len, bool isrx)
{

    // only using a single chunk to make this
    // function quicker, but it means we're limtied to like 4096 bytes
    assert(len <= DMA_DESCRIPTOR_BUFFER_MAX_SIZE_4B_ALIGNED);

    int dmachunklen = len;

    if (isrx)
    {
        // Receive needs DMA length rounded to next 32-bit boundary
        dmadesc[0].dw0.size = (dmachunklen + 3) & (~3);
        dmadesc[0].dw0.length = (dmachunklen + 3) & (~3);
    }
    else
    {
        dmadesc[0].dw0.size = dmachunklen;
        dmadesc[0].dw0.length = dmachunklen;
    }
    dmadesc[0].buffer = (uint8_t *)data;
    dmadesc[0].dw0.suc_eof = 1; // we're only using the one chunk
    dmadesc[0].dw0.reserved29 = 0;
    dmadesc[0].dw0.owner = 1;
    dmadesc[0].next = NULL;

} // QuickLink

// lldesc_setup_link_constrained from lldesc.c
// Make sure spi_slave_init_lite sets up enough descriptors. Probs does.
// Tbh I know for sure it's setting up way too many. Yolo.
static IRAM_ATTR void QuickLink_Chunked(dma_descriptor_t *dmadesc, const void *data, int len, bool isrx)
{

    int dmachunklen = len;

    int n = 0;
    while (len)
    {

        // int dmachunklen = 128 + 4 + 2 + 1;
        //  it won't be, we're picking 16
        //  if (dmachunklen > max_desc_size) {
        //      dmachunklen = max_desc_size;
        //  }

        if (isrx)
        {
            // Receive needs DMA length rounded to next 32-bit boundary
            dmadesc[n].dw0.size = (dmachunklen + 3) & (~3);
            dmadesc[n].dw0.length = (dmachunklen + 3) & (~3);
        }
        else
        {

            // dmachunklen, len = can't inject [1]
            // len, len = can't inject [1], doubt it's working

            // dmachunklen, dmachunklen = does inject chunktest to [1], but fails with repeats on [2]
            // (without the full reset you also appear to be able to change the buffer via setsecondchunk, on the correct boundary)

            // len, dmachunklen = does inject chunktest to [1], but fails with repeats on [2]
            // (without full reset, you're able to change the buffer via setsecondchunk, on the correct boundary )

            dmadesc[n].dw0.size = dmachunklen;
            dmadesc[n].dw0.length = dmachunklen;
        }

        dmadesc[n].buffer = (uint8_t *)data;
        dmadesc[n].dw0.suc_eof = 0;
        dmadesc[n].dw0.reserved29 = 0;
        dmadesc[n].dw0.owner = 1;
        dmadesc[n].next = &dmadesc[n + 1];
        len -= dmachunklen;
        data += dmachunklen;
        n++;
    }
    dmadesc[n - 1].dw0.suc_eof = 1; // Mark last DMA desc as end of stream.
    dmadesc[n - 1].next = NULL;
    printf("set up %d descriptors\n", n);

} // QuickLink_Chunked

// cached inlink and outlink values
static uint32_t outLink = 0;
static uint32_t inLink = 0;
static uint32_t addrOnly = 0;

static uint32_t inlink_hostMAIN = 0;
static uint32_t outlink_hostMAIN = 0;

// Setup sequence derived from "prepare_data"
// but without setting up the same unnecessary bits every time
// e.g. need to get us below about 2.6us
// formerly InitStuff

void SpiSlaveInitBuffersLite(uint32_t whichHost, uint8_t *txBuffer, uint8_t *rxBuffer, uint32_t inLength, uint32_t flashDebug, int whichDMAChannel)
{
    // add a transaction so we have one to write into
    // TODO: is this transaction bs actually necessary? or is it all HAL
    spi_slave_transaction_t transaction;
    transaction.length = inLength;
    transaction.rx_buffer = rxBuffer;
    transaction.tx_buffer = txBuffer;

    spi_slave_trans_priv_t priv_transaction;
    priv_transaction.rx_buffer = rxBuffer;
    priv_transaction.tx_buffer = txBuffer;
    priv_transaction.trans = &transaction;

    spihost[whichHost]->cur_trans = priv_transaction;

    // spi_slave_hal.h
    spi_slave_hal_context_t *hal = &spihost[whichHost]->hal;
    uint32_t rxChan = whichDMAChannel;
    uint32_t txChan = whichDMAChannel;

    // Doens't really work without this
    hal->bitlen = inLength * 8;

    // lldesc_setup_link(hal->dmadesc_rx, hal->rx_buffer, ((hal->bitlen  7) / 8), true);
    QuickLink_SingleDescriptor(hal->dmadesc_rx, hal->rx_buffer, inLength, true);

    hal->hw->dma_conf.dma_rx_ena = 1;
    GDMA.channel[rxChan].in.link.addr = (uint32_t)&hal->dmadesc_rx[0];
    GDMA.channel[rxChan].in.link.start = 1;
    printf("initial inlink %lx\n", GDMA.channel[rxChan].in.link.val);
    inLink = GDMA.channel[rxChan].in.link.val | (1 << 22);

    // lldesc_setup_link(hal->dmadesc_tx, hal->tx_buffer, (hal->bitlen  7) / 8, false);
    QuickLink_Chunked(hal->dmadesc_tx, hal->tx_buffer, inLength, false);

    // Note: out link starts with value 0x800000 ('park')
    // typically, the outlink value will be addr + start + park:
    // (&hal->dmadesc_tx[0] & 0xFFFFF) | (1<<21) | (1<<23);
    hal->hw->dma_conf.dma_tx_ena = 1;
    GDMA.channel[txChan].out.link.addr = (uint32_t)(&hal->dmadesc_tx[0]);
    GDMA.channel[txChan].out.link.start = 1;
    printf("initial outlink %lx\n", GDMA.channel[txChan].out.link.val);
    outLink = GDMA.channel[txChan].out.link.val | (1 << 21);

    // first 20 bytes
    addrOnly = GDMA.channel[txChan].out.link.val & 0xFFFFF;

    // on the hal side of things
    hal->rx_buffer = rxBuffer;
    hal->tx_buffer = txBuffer;

    // and on the transaction side of things
    // note: will fail if SetTrans hasn't happened
    // spihost[whichHost]->cur_trans->rxBuffer;
    // spihost[whichHost]->cur_trans->txBuffer;

    if (whichHost == HOST_MAIN)
    {
        CacheValues_HostMAIN(whichDMAChannel);
    }
}

// Conditional MISO timing: when true, output MISO half a clock earlier
// to compensate for FPGA passthrough propagation delay.
// Only enabled during MMCE block transfers on the FPGA version.
static volatile bool rsck_data_out_enabled = false;

void SetRsckDataOut(bool enabled)
{
    rsck_data_out_enabled = enabled;
    GPSPI3.slave.rsck_data_out = enabled;
}

// This is largely taken from spi_slave_hal_iram.c's prepare_data() function
// formerly SendStuff
IRAM_ATTR void SpiSlaveSendLite(uint32_t whichHost)
{

    // E.g.
    // spi_slave_hal_context_t * hal = &spihost[whichHost]->hal;
    activeHal = &spihost[whichHost]->hal;
    activeRxChan = 0; // activeHal->rx_dma_chan;
    activeTxChan = 0; // activeHal->tx_dma_chan;

    // Fill DMA descriptors
    // if (hal->rx_buffer) {

    // Moved to InitStuff
    // lldesc_setup_link(hal->dmadesc_rx, hal->rx_buffer, ((hal->bitlen + 7) / 8), true);
    // QuickLink(hal->dmadesc_rx, hal->rx_buffer, hal->bitlen, true);
    activeHal->dmadesc_rx[0].buffer = (uint8_t *)activeHal->rx_buffer;

    // reset dma inlink, this should be reset before spi related reset
    //  325ns vs 285ns
    // gdma_ll_rx_reset_channel(&GDMA, hal->rx_dma_chan);
    // GDMA.channel[rxChan].in.conf0.in_rst = 1;
    // GDMA.channel[rxChan].in.conf0.in_rst = 0;
    GDMA.channel[activeRxChan].in.conf0.val = 0b1;
    GDMA.channel[activeRxChan].in.conf0.val = 0b0;

    // Moved below to shared handler
    // 325ns vs 300ns
    // original // spi_ll_dma_rx_fifo_reset(hal->dma_in);
    // hal->dma_in->dma_conf.rx_afifo_rst = 1;
    // original // hal->dma_in->dma_conf.rx_afifo_rst = 0; // doesn't seem necessary

    // 204ns vs 295ns (hardly worth it?)
    // Moved below (shared between tx and rx)
    // original // spi_ll_slave_reset(hal->hw);
    // hal->dma_in->slave.soft_reset = 1;
    // hal->dma_in->slave.soft_reset = 0;

    // 145ns vs 140ns
    // original // spi_ll_infifo_full_clr(hal->hw);
    // hal->dma_in->dma_int_clr.infifo_full_err = 1;
    // this is a risky one since it clears all the fuckin ints
    // but it saves tons of time
    activeHal->hw->dma_int_clr.val = 0xFFFFFFFF;

    // 140ns vs 145ns
    // Moved to one-time init
    // spi_ll_dma_rx_enable(hal->hw, 1);
    // hal->dma_in->dma_conf.dma_rx_ena = 1;

    // Moved to one-time init
    // spi_dma_ll_rx_start(hal->dma_in, hal->rx_dma_chan, &hal->dmadesc_rx[0]);

    // 175ns vs 145ns
    // original // gdma_ll_rx_set_desc_addr(&GDMA, hal->rx_dma_chan, &hal->dmadesc_rx[0]);
    // GDMA.channel[rxChan].in.link.addr = &hal->dmadesc_rx[0];

    // 165ns vs 140ns
    //  Moved to one-time init
    //  original // gdma_ll_rx_start(&GDMA, hal->rx_dma_chan);
    // GDMA.channel[rxChan].in.link.start = 1;
    // ets_printf( "inlink %x\n", GDMA.channel[rxChan].in.link.val );
    GDMA.channel[activeRxChan].in.link.val = inLink;

    //}

    // if (hal->tx_buffer) {

    // Moved to InitStuff
    // lldesc_setup_link(hal->dmadesc_tx, hal->tx_buffer, (hal->bitlen  7) / 8, false);
    // QuickLink(hal->dmadesc_tx, hal->tx_buffer, hal->bitlen, false);
    activeHal->dmadesc_tx[0].buffer = (uint8_t *)activeHal->tx_buffer;

    // reset dma outlink, this should be reset before spi related reset
    // gdma_ll_tx_reset_channel(&GDMA, hal->tx_dma_chan);
    // GDMA.channel[txChan].out.conf0.out_rst = 1;
    // GDMA.channel[txChan].out.conf0.out_rst = 0;
    GDMA.channel[activeTxChan].out.conf0.val = 0b111001; // reset
    GDMA.channel[activeTxChan].out.conf0.val = 0b111000; // unreset

    // moved below, shared between tx and rx
    // spi_ll_dma_tx_fifo_reset(hal->dma_out);
    // hal->dma_out->dma_conf.dma_afifo_rst = 1;
    // original // hal->dma_out->dma_conf.dma_afifo_rst = 0; // doesn't seem necessary

    // Moved below (shared between tx and rx)
    // original // spi_ll_slave_reset(hal->hw);
    // hal->hw->slave.soft_reset = 1;
    // hal->hw->slave.soft_reset = 0;

    // spi_ll_outfifo_empty_clr(hal->hw);
    // hal->hw->dma_int_clr.outfifo_empty_err = 1;
    //  this is a risky one since it clears all the fuckin ints
    //  but it saves tons of time
    activeHal->hw->dma_int_clr.val = 0xFFFFFFFF;

    // Movet to one-time init
    // original // spi_ll_dma_tx_enable(hal->hw, 1);
    // hal->hw->dma_conf.dma_tx_ena = 1;

    // Moved to one-time init
    // spi_dma_ll_tx_start(hal->dma_out, hal->tx_dma_chan, (&hal->dmadesc_tx[0]));

    // Moved to one-time init
    // original // gdma_ll_tx_set_desc_addr(&GDMA, hal->tx_dma_chan, (&hal->dmadesc_tx[0]));
    // GDMA.channel[txChan].out.link.addr = (&hal->dmadesc_tx[0]);

    // Moved to one-time init
    // original // gdma_ll_tx_start(&GDMA, hal->tx_dma_chan);
    // ets_printf( "outlink %x\n", GDMA.channel[txChan].out.link.val );
    // GDMA.channel[txChan].out.link.start = 1;
    GDMA.channel[activeTxChan].out.link.val = outLink;

    //}

    // shared
    // hal->hw->slave.soft_reset = 1;
    // skip this // hal->hw->slave.soft_reset = 0; // skip this
    activeHal->hw->slave.val = 0b00001110100000000000001000000000;
    // skip this // hal->hw->slave.val = 0b0000'0110'1000'0000'0000'0000'0000'0000;

    // This points at the <dma>.dma_conf.* union
    // e.g. the thing in spi_struct.h
    // bits 0 & 1 (full/empty), bits (idk, they're already set, let's re-set them)
    // bits 27 & 28 (dma_rx_ena  & dma_tx_ena)
    // bits 31 & 29 (dma_afifo_rst & rx_afifo_rst) <-- this is correct, idk about the third one
    activeHal->hw->dma_conf.val = 0b10111000000000000000000000000011;

    // NOTE: rsck_data_out is now managed exclusively by SetRsckDataOut()
    // from the MMCE access-mode handler. Defensive re-writes were removed
    // because they added RMW time to the fast-path reset, pushing the
    // reset past one SCK period at 27.5 MHz and causing bit-shift
    // corruption on the first data-phase edge. See
    // project_4bit_shift_investigation in memory.

    // shared

    // not used in the esp32s3
    // spi_ll_slave_set_rx_bitlen(hal->hw, hal->bitlen);
    // spi_ll_slave_set_tx_bitlen(hal->hw, hal->bitlen);

    // works without
    // spicommon_dmaworkaround_transfer_active(spihost[whichHost]->tx_dma_chan);
    // works without
    // 2023 note: no it doesn't.
    // spi_slave_hal_user_start(hal);
}

static spi_slave_hal_context_t *hal_hostMAIN = NULL;
static volatile uint32_t rxChan_hostMAIN = 0;
static volatile uint32_t txChan_hostMAIN = 0;
static volatile uint32_t *gdma_channel_rxChan_hostMAIN_in_conf0_val = NULL;
static volatile uint32_t *hal_hostMAIN_dma_in_dma_int_clr_val = NULL;
static volatile uint32_t *gdma_channel_rxChan_hostMAIN_in_link_val = NULL;
static volatile uint32_t *gdma_channel_txChan_hostMAIN_out_conf0_val = NULL;
static volatile uint32_t *gdma_channel_txChan_hostMAIN_out_link_val = NULL;
static volatile uint32_t *hal_hostMAIN_dma_out_dma_conf_val = NULL;
static spi_dma_desc_t *dmadesc_tx_hostMAIN = NULL;
static spi_dma_desc_t *dmadesc_rx_hostMAIN = NULL;
// Pre-computed first-word value for RX descriptor re-arm.
// Layout: size[11:0] | length[23:12] | offset[28:24] | sosf[29] | eof[30] | owner[31]
// Values: size=<preserved>, length=0, offset=0, sosf=0, eof=1, owner=1 = 0xC0000000 | size
static volatile uint32_t *dmadesc_rx_hostMAIN_word0 = NULL;
static uint32_t rx_desc_rearm_word0 = 0;

void CacheValues_HostMAIN(int whichDmaChannel)
{
    hal_hostMAIN = &spihost[HOST_MAIN]->hal;

    rxChan_hostMAIN = whichDmaChannel;
    txChan_hostMAIN = whichDmaChannel;

    inlink_hostMAIN = inLink;
    outlink_hostMAIN = outLink;

    // for quickreset

    gdma_channel_rxChan_hostMAIN_in_conf0_val = &GDMA.channel[rxChan_hostMAIN].in.conf0.val;
    gdma_channel_rxChan_hostMAIN_in_link_val = &GDMA.channel[rxChan_hostMAIN].in.link.val;

    hal_hostMAIN_dma_in_dma_int_clr_val = &hal_hostMAIN->hw->dma_int_clr.val;

    gdma_channel_txChan_hostMAIN_out_conf0_val = &GDMA.channel[txChan_hostMAIN].out.conf0.val;

    gdma_channel_txChan_hostMAIN_out_link_val = &GDMA.channel[txChan_hostMAIN].out.link.val;
    hal_hostMAIN_dma_out_dma_conf_val = &hal_hostMAIN->hw->dma_conf.val;
    dmadesc_tx_hostMAIN = hal_hostMAIN->dmadesc_tx;
    dmadesc_rx_hostMAIN = hal_hostMAIN->dmadesc_rx;

    // Pre-compute the RX descriptor re-arm word once so QuickReset is a single store.
    dmadesc_rx_hostMAIN_word0 = (volatile uint32_t *)dmadesc_rx_hostMAIN;
    rx_desc_rearm_word0 = 0xC0000000u | (dmadesc_rx_hostMAIN->dw0.size & 0xFFFu);
}

// overkill but it avoids us having to do a bunch of array lookups
// See QuickReset for a list of changes from the original version
// what it does:
// resets the trans-done flag
// updates the contents of the next output buffer
IRAM_ATTR inline void QuickReset_HostMAIN()
{
    *hal_hostMAIN_dma_in_dma_int_clr_val = 0xFFFFFFFF;

    // Single 32-bit store to re-arm the RX descriptor: owner=1, length=0,
    // eof=1, size preserved. Avoids the two read-modify-write cycles the
    // bit-field form would generate — every nanosecond in this path pushes
    // the reset closer to the next CS edge.
    *dmadesc_rx_hostMAIN_word0 = rx_desc_rearm_word0;

    // GDMA.channel[rxChan_hostMAIN].in.link.val = inlink_hostMAIN;//[HOST_MAIN];
    *gdma_channel_rxChan_hostMAIN_in_link_val = inlink_hostMAIN; // inLink[HOST_MAIN];

    // GDMA.channel[txChan_hostMAIN].out.conf0.val = 0b111001; // reset
    *gdma_channel_txChan_hostMAIN_out_conf0_val = 0b111001; // reset

    // GDMA.channel[txChan_hostMAIN].out.conf0.val = 0b111000; // unreset
    *gdma_channel_txChan_hostMAIN_out_conf0_val = 0b111000; // unreset

    // GDMA.channel[txChan_hostMAIN].out.link.val = outlink_hostMAIN;//outLink[HOST_MAIN];
    *gdma_channel_txChan_hostMAIN_out_link_val = outlink_hostMAIN; // outLink[HOST_MAIN];

    // hal_hostMAIN->dma_out->dma_conf.val = 0b10111000000000000000000000000011;
    // Pulse `dma_afifo_rst` (bit 31): assert, then release.
    // Matches v5.5 upstream `spi_slave_hal_hw_prepare_tx`, which toggles this bit
    // as a level-triggered reset. The single-write form (set, no clear) leaves the
    // SPI TX async FIFO held in reset between packets — tolerated when there's idle
    // time between transactions but causes a 1-byte MISO shift on tightly packed
    // sequences (0x83 → 790 ns → 0x89 → 0xF2). One extra 32-bit store, ~5 ns.
    *hal_hostMAIN_dma_out_dma_conf_val = 0b10111000000000000000000000000011;
    *hal_hostMAIN_dma_out_dma_conf_val = 0b00111000000000000000000000000011;

    // rsck_data_out now managed exclusively by SetRsckDataOut()
}

// A qucker version of quickreset
// if you don't need to reset the input buffer
// but just update the outgoing bytes a bit
// then this is your guy
IRAM_ATTR inline void QuickerReset_HostMAIN()
{

    // 1) and 2) required to reset the trans_done flag (if u care)
    // if not, just wait it out

    // 1) this alone does not reset trans_done
    // GDMA.channel[rxChan_hostMAIN].in.conf0.val = 0b0;
    //*gdma_channel_rxChan_hostMAIN_in_conf0_val = 0b0;

    // 2) this alone does not reset trans_done
    // hal_hostMAIN->dma_in->dma_int_clr.val = 0xFFFFFFFF;
    //*hal_hostMAIN_dma_in_dma_int_clr_val = 0xFFFFFFFF;

    // this alone doees not reset trans_done
    // GDMA.channel[rxChan_hostMAIN].in.link.val = inlink_hostMAIN;//[HOST_MAIN];
    //*gdma_channel_rxChan_hostMAIN_in_link_val = inlink_hostMAIN;//inLink[HOST_MAIN];

    // GDMA.channel[txChan_hostMAIN].out.conf0.val = 0b111001; // reset
    *gdma_channel_txChan_hostMAIN_out_conf0_val = 0b111001; // reset

    // GDMA.channel[txChan_hostMAIN].out.conf0.val = 0b111000; // unreset
    *gdma_channel_txChan_hostMAIN_out_conf0_val = 0b111000; // unreset

    // Reset TX descriptor for next transaction
    dmadesc_tx_hostMAIN->dw0.owner = 1;
    // dmadesc_tx_hostMAIN->offset = 0;

    // GDMA.channel[txChan_hostMAIN].out.link.val = outlink_hostMAIN;//outLink[HOST_MAIN];
    *gdma_channel_txChan_hostMAIN_out_link_val = outlink_hostMAIN; // outLink[HOST_MAIN];

    // hal_hostMAIN->dma_out->dma_conf.val = 0b10111000000000000000000000000011;
    *hal_hostMAIN_dma_out_dma_conf_val = 0b10111000000000000000000000000011;

    // rsck_data_out now managed exclusively by SetRsckDataOut()
}

// Split version of QuickerReset for deferred outlink selection
// Call Prepare first, then Finalize with the appropriate outlink after detecting byte 2
IRAM_ATTR inline void QuickerReset_HostMAIN_Prepare()
{
    // Reset/unreset the TX DMA channel
    *gdma_channel_txChan_hostMAIN_out_conf0_val = 0b111001; // reset
    *gdma_channel_txChan_hostMAIN_out_conf0_val = 0b111000; // unreset
    dmadesc_tx_hostMAIN->dw0.owner = 1;
    // dmadesc_tx_hostMAIN->offset = 0;
}

IRAM_ATTR inline void QuickerReset_HostMAIN_Finalize(uint32_t outlink)
{
    *gdma_channel_txChan_hostMAIN_out_link_val = outlink;
    // Configure DMA
    *hal_hostMAIN_dma_out_dma_conf_val = 0b10111000000000000000000000000011;

    // rsck_data_out now managed exclusively by SetRsckDataOut()
}

// Get the default outlink value (for ping response etc)
uint32_t GetOutlink_HostMAIN()
{
    return outlink_hostMAIN;
}

// Get TX DMA descriptor pointer for manual reset of offset/length fields
spi_dma_desc_t *GetTxDescriptor_HostMAIN()
{
    return hal_hostMAIN->dmadesc_tx;
}

// Create an outlink value from a DMA descriptor address
// The descriptor should be set up with buffer, size, length, eof=1, owner=1
uint32_t CreateOutlinkFromDescriptor(void *dmadesc_addr)
{
    // outlink register format:
    // bits 0-19: 20 LSBs of descriptor address
    // bit 20: stop
    // bit 21: start
    // bit 22: restart
    // Note: descriptor must be in DMA-capable memory (internal SRAM)
    return (((uint32_t)dmadesc_addr) & 0xFFFFF) | (1 << 21);
}

// Get pointers for direct register writes (maximum speed, bypass function call overhead)
volatile uint32_t *Get_OutlinkRegPtr_HostMAIN()
{
    return gdma_channel_txChan_hostMAIN_out_link_val;
}

volatile uint32_t *Get_DmaConfRegPtr_HostMAIN()
{
    return hal_hostMAIN_dma_out_dma_conf_val;
}

volatile uint32_t *Get_Conf0RegPtr_HostMAIN()
{
    return gdma_channel_txChan_hostMAIN_out_conf0_val;
}

// The magic DMA conf value for reference
#define DMA_CONF_VALUE_HOSTMAIN 0b10111000000000000000000000000011

IRAM_ATTR void QuickReset(uint32_t whichHost)
{

    activeHal = &spihost[whichHost]->hal;

    // the comment "//X" marks bits which have been removed from SpiSlaveSendLite
    // Note: if you remove too much, it doesn't clear the trans_done flag

    // use a cached version instead to save some time
    // spi_slave_hal_context_t * hal = &spihost[whichHost]->hal;
    // uint32_t rxChan = hal->rx_dma_chan;
    // uint32_t txChan = hal->tx_dma_chan;

    // Fill DMA descriptors
    // if (hal->rx_buffer) {

    // Moved to InitStuff
    // lldesc_setup_link(hal->dmadesc_rx, hal->rx_buffer, ((hal->bitlen  7) / 8), true);
    // QuickLink(hal->dmadesc_rx, hal->rx_buffer, hal->bitlen, true);
    // X
    // Y hal->dmadesc_rx[0].buf = (uint8_t *)hal->rx_buffer;

    // reset dma inlink, this should be reset before spi related reset
    //  325ns vs 285ns
    // gdma_ll_rx_reset_channel(&GDMA, hal->rx_dma_chan);
    // GDMA.channel[rxChan].in.conf0.in_rst = 1;
    // GDMA.channel[rxChan].in.conf0.in_rst = 0;
    // X
    // Y GDMA.channel[rxChan].in.conf0.val = 0b1;
    *gdma_channel_rxChan_hostMAIN_in_conf0_val = 0b0;

    // Moved below to shared handler
    // 325ns vs 300ns
    // original // spi_ll_dma_rx_fifo_reset(hal->dma_in);
    // hal->dma_in->dma_conf.rx_afifo_rst = 1;
    // original // hal->dma_in->dma_conf.rx_afifo_rst = 0; // doesn't seem necessary

    // 204ns vs 295ns (hardly worth it?)
    // Moved below (shared between tx and rx)
    // original // spi_ll_slave_reset(hal->hw);
    // hal->dma_in->slave.soft_reset = 1;
    // hal->dma_in->slave.soft_reset = 0;

    // 145ns vs 140ns
    // original // spi_ll_infifo_full_clr(hal->hw);
    // hal->dma_in->dma_int_clr.infifo_full_err = 1;
    // this is a risky one since it clears all the fuckin ints
    // but it saves tons of time
    // X works without, but will not clear the trans_done flag
    *hal_hostMAIN_dma_in_dma_int_clr_val = 0xFFFFFFFF;

    // Reset RX descriptor so DMA re-owns it for the next transaction.
    // Symmetric to the TX fix; without this the descriptor stays owner=0
    // after an EOF and new bytes are silently dropped.
    activeHal->dmadesc_rx[0].dw0.owner = 1;
    activeHal->dmadesc_rx[0].dw0.length = 0;

    // 140ns vs 145ns
    // Moved to one-time init
    // spi_ll_dma_rx_enable(hal->hw, 1);
    // hal->dma_in->dma_conf.dma_rx_ena = 1;

    // Moved to one-time init
    // spi_dma_ll_rx_start(hal->dma_in, hal->rx_dma_chan, &hal->dmadesc_rx[0]);

    // 175ns vs 145ns
    // original // gdma_ll_rx_set_desc_addr(&GDMA, hal->rx_dma_chan, &hal->dmadesc_rx[0]);
    // GDMA.channel[rxChan].in.link.addr = &hal->dmadesc_rx[0];

    // 165ns vs 140ns
    //  Moved to one-time init
    //  original // gdma_ll_rx_start(&GDMA, hal->rx_dma_chan);
    // GDMA.channel[rxChan].in.link.start = 1;
    // ets_printf( "inlink %x\n", GDMA.channel[rxChan].in.link.val );
    *gdma_channel_rxChan_hostMAIN_in_link_val = inLink;

    //}

    // if (hal->tx_buffer) {

    // Moved to InitStuff
    // lldesc_setup_link(hal->dmadesc_tx, hal->tx_buffer, (hal->bitlen  7) / 8, false);
    // QuickLink(hal->dmadesc_tx, hal->tx_buffer, hal->bitlen, false);
    // X
    // Yhal->dmadesc_tx[0].buf = (uint8_t*)hal->tx_buffer;

    // reset dma outlink, this should be reset before spi related reset
    // gdma_ll_tx_reset_channel(&GDMA, hal->tx_dma_chan);
    // GDMA.channel[txChan].out.conf0.out_rst = 1;
    // GDMA.channel[txChan].out.conf0.out_rst = 0;
    *gdma_channel_txChan_hostMAIN_out_conf0_val = 0b111001; // reset
    *gdma_channel_txChan_hostMAIN_out_conf0_val = 0b111000; // unreset

    // moved below, shared between tx and rx
    // spi_ll_dma_tx_fifo_reset(hal->dma_out);
    // hal->dma_out->dma_conf.dma_afifo_rst = 1;
    // original // hal->dma_out->dma_conf.dma_afifo_rst = 0; // doesn't seem necessary

    // Moved below (shared between tx and rx)
    // original // spi_ll_slave_reset(hal->hw);
    // hal->hw->slave.soft_reset = 1;
    // hal->hw->slave.soft_reset = 0;

    // spi_ll_outfifo_empty_clr(hal->hw);
    // hal->hw->dma_int_clr.outfifo_empty_err = 1;
    //  this is a risky one since it clears all the fuckin ints
    //  but it saves tons of time
    // X
    // Yhal->hw->dma_int_clr.val = 0xFFFFFFFF;

    // Movet to one-time init
    // original // spi_ll_dma_tx_enable(hal->hw, 1);
    // hal->hw->dma_conf.dma_tx_ena = 1;

    // Moved to one-time init
    // spi_dma_ll_tx_start(hal->dma_out, hal->tx_dma_chan, (&hal->dmadesc_tx[0]));

    // Moved to one-time init
    // original // gdma_ll_tx_set_desc_addr(&GDMA, hal->tx_dma_chan, (&hal->dmadesc_tx[0]));
    // GDMA.channel[txChan].out.link.addr = (&hal->dmadesc_tx[0]);

    // Moved to one-time init
    // original // gdma_ll_tx_start(&GDMA, hal->tx_dma_chan);
    // ets_printf( "outlink %x\n", GDMA.channel[txChan].out.link.val );
    // GDMA.channel[txChan].out.link.start = 1;
    *gdma_channel_txChan_hostMAIN_out_link_val = outLink;

    //}

    // shared
    // hal->hw->slave.soft_reset = 1;
    // skip this // hal->hw->slave.soft_reset = 0; // skip this
    // X
    // Yhal->hw->slave.val = 0b00001110100000000000000000000000;
    // skip this // hal->hw->slave.val = 0b0000'0110'1000'0000'0000'0000'0000'0000;

    // This points at the <dma>.dma_conf.* union
    // e.g. the thing in spi_struct.h
    // bits 0 & 1 (full/empty), bits (idk, they're already set, let's re-set them)
    // bits 27 & 28 (dma_rx_ena  & dma_tx_ena)
    // bits 31 & 29 (dma_afifo_rst & rx_afifo_rst) <-- this is correct, idk about the third one
    *hal_hostMAIN_dma_out_dma_conf_val = 0b10111000000000000000000000000011;

    // shared

    // not used in the esp32s3
    // spi_ll_slave_set_rx_bitlen(hal->hw, hal->bitlen);
    // spi_ll_slave_set_tx_bitlen(hal->hw, hal->bitlen);

    // works without
    // spicommon_dmaworkaround_transfer_active(spihost[whichHost]->tx_dma_chan);
    // works without
    // 2023 note: no it doesn't.
    // spi_slave_hal_user_start(hal);

    // rsck_data_out now managed exclusively by SetRsckDataOut()
}

uint32_t GetHalRXBufferPtr(uint32_t whichHost)
{
    return (uint32_t)&spihost[whichHost]->hal.rx_buffer;
}

uint32_t GetHalTXBufferPtr(uint32_t whichHost)
{
    return (uint32_t)&spihost[whichHost]->hal.tx_buffer;
}

#pragma GCC pop_options

//
// End MCPro Mods
//
