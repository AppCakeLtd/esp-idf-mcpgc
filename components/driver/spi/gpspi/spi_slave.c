/*
 * SPDX-FileCopyrightText: 2015-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include "esp_types.h"
#include "esp_attr.h"
#include "esp_check.h"
#include "esp_intr_alloc.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_pm.h"
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
#include "hal/spi_slave_hal.h"
#include "esp_private/spi_slave_internal.h"
#include "esp_private/spi_common_internal.h"

// for &GPIO struct
#include "soc/gpio_struct.h"
// gpio_matrix_in, etc:
#include "rom/gpio.h"

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

typedef struct
{
    int id;
    spi_bus_config_t bus_config;
    spi_slave_interface_config_t cfg;
    intr_handle_t intr;
    spi_slave_hal_context_t hal;
    spi_slave_transaction_t *cur_trans;
    uint32_t flags;
    uint32_t intr_flags;
    int max_transfer_sz;
    QueueHandle_t trans_queue;
    QueueHandle_t ret_queue;
    bool dma_enabled;
    bool cs_iomux;
    uint8_t cs_in_signal;
    uint32_t tx_dma_chan;
    uint32_t rx_dma_chan;
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

static void SPI_SLAVE_ISR_ATTR freeze_cs(spi_slave_t *host)
{
    esp_rom_gpio_connect_in_signal(GPIO_MATRIX_CONST_ONE_INPUT, host->cs_in_signal, false);
}

// Use this function instead of cs_initial to avoid overwrite the output config
// This is used in test by internal gpio matrix connections
static inline void SPI_SLAVE_ISR_ATTR restore_cs(spi_slave_t *host)
{
    if (host->cs_iomux)
    {
        gpio_ll_iomux_in(GPIO_HAL_GET_HW(GPIO_PORT_0), host->cfg.spics_io_num, host->cs_in_signal);
    }
    else
    {
        esp_rom_gpio_connect_in_signal(host->cfg.spics_io_num, host->cs_in_signal, false);
    }
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

esp_err_t spi_slave_initialize(spi_host_device_t host, const spi_bus_config_t *bus_config, const spi_slave_interface_config_t *slave_config, spi_dma_chan_t dma_chan)
{
    bool spi_chan_claimed;
    uint32_t actual_tx_dma_chan = 0;
    uint32_t actual_rx_dma_chan = 0;
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
    SPI_CHECK((bus_config->intr_flags & (ESP_INTR_FLAG_HIGH | ESP_INTR_FLAG_EDGE | ESP_INTR_FLAG_INTRDISABLED)) == 0, "intr flag not allowed", ESP_ERR_INVALID_ARG);
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

    spihost[host] = malloc(sizeof(spi_slave_t));
    if (spihost[host] == NULL)
    {
        ret = ESP_ERR_NO_MEM;
        goto cleanup;
    }
    memset(spihost[host], 0, sizeof(spi_slave_t));
    memcpy(&spihost[host]->cfg, slave_config, sizeof(spi_slave_interface_config_t));
    memcpy(&spihost[host]->bus_config, bus_config, sizeof(spi_bus_config_t));
    spihost[host]->id = host;

    bool use_dma = (dma_chan != SPI_DMA_DISABLED);
    spihost[host]->dma_enabled = use_dma;
    if (use_dma)
    {
        ret = spicommon_dma_chan_alloc(host, dma_chan, &actual_tx_dma_chan, &actual_rx_dma_chan);
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
        // check and save where cs line really route through
        spihost[host]->cs_iomux = (slave_config->spics_io_num == spi_periph_signal[host].spics0_iomux_pin) && bus_is_iomux(spihost[host]);
        spihost[host]->cs_in_signal = spi_periph_signal[host].spics_in;
    }

    // The slave DMA suffers from unexpected transactions. Forbid reading if DMA is enabled by disabling the CS line.
    if (use_dma)
        freeze_cs(spihost[host]);

    int dma_desc_ct = 0;
    spihost[host]->tx_dma_chan = actual_tx_dma_chan;
    spihost[host]->rx_dma_chan = actual_rx_dma_chan;
    if (use_dma)
    {
        // See how many dma descriptors we need and allocate them
        dma_desc_ct = (bus_config->max_transfer_sz + SPI_MAX_DMA_LEN - 1) / SPI_MAX_DMA_LEN;
        if (dma_desc_ct == 0)
            dma_desc_ct = 1; // default to 4k when max is not given
        spihost[host]->max_transfer_sz = dma_desc_ct * SPI_MAX_DMA_LEN;
    }
    else
    {
        // We're limited to non-DMA transfers: the SPI work registers can hold 64 bytes at most.
        spihost[host]->max_transfer_sz = SOC_SPI_MAXIMUM_BUFFER_SIZE;
    }
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

    // Create queues
    spihost[host]->trans_queue = xQueueCreate(slave_config->queue_size, sizeof(spi_slave_transaction_t *));
    if (!spihost[host]->trans_queue)
    {
        ret = ESP_ERR_NO_MEM;
        goto cleanup;
    }
    if (!(slave_config->flags & SPI_SLAVE_NO_RETURN_RESULT))
    {
        spihost[host]->ret_queue = xQueueCreate(slave_config->queue_size, sizeof(spi_slave_transaction_t *));
        if (!spihost[host]->ret_queue)
        {
            ret = ESP_ERR_NO_MEM;
            goto cleanup;
        }
    }

#if (SOC_CPU_CORES_NUM > 1) && (!CONFIG_FREERTOS_UNICORE)
    if (bus_config->isr_cpu_id > INTR_CPU_ID_AUTO)
    {
        spihost[host]->intr_flags = bus_config->intr_flags;
        SPI_CHECK(bus_config->isr_cpu_id <= INTR_CPU_ID_1, "invalid core id", ESP_ERR_INVALID_ARG);
        spi_ipc_param_t ipc_args = {
            .host = spihost[host],
            .err = &err,
        };
        esp_ipc_call_blocking(INTR_CPU_CONVERT_ID(bus_config->isr_cpu_id), ipc_isr_reg_to_core, (void *)&ipc_args);
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

    spi_slave_hal_context_t *hal = &spihost[host]->hal;
    // assign the SPI, RX DMA and TX DMA peripheral registers beginning address
    spi_slave_hal_config_t hal_config = {
        .host_id = host,
        .dma_in = SPI_LL_GET_HW(host),
        .dma_out = SPI_LL_GET_HW(host)};
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
    hal->tx_dma_chan = actual_tx_dma_chan;
    hal->rx_dma_chan = actual_rx_dma_chan;

    spi_slave_hal_setup_device(hal);

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
        spicommon_dma_chan_free(host);
    }

    free(spihost[host]);
    spihost[host] = NULL;
    spicommon_periph_free(host);

    return ret;
}

esp_err_t spi_slave_free(spi_host_device_t host)
{
    SPI_CHECK(is_valid_host(host), "invalid host", ESP_ERR_INVALID_ARG);
    SPI_CHECK(spihost[host], "host not slave", ESP_ERR_INVALID_ARG);
    if (spihost[host]->trans_queue)
        vQueueDelete(spihost[host]->trans_queue);
    if (spihost[host]->ret_queue)
        vQueueDelete(spihost[host]->ret_queue);
    if (spihost[host]->dma_enabled)
    {
        spicommon_dma_chan_free(host);
    }
    spicommon_bus_free_io_cfg(&spihost[host]->bus_config);
    free(spihost[host]->hal.dmadesc_tx);
    free(spihost[host]->hal.dmadesc_rx);
    esp_intr_free(spihost[host]->intr);
#ifdef CONFIG_PM_ENABLE
    esp_pm_lock_release(spihost[host]->pm_lock);
    esp_pm_lock_delete(spihost[host]->pm_lock);
#endif // CONFIG_PM_ENABLE
    free(spihost[host]);
    spihost[host] = NULL;
    spicommon_periph_free(host);
    return ESP_OK;
}

/**
 * @note
 * This API is used to reset SPI Slave transaction queue. After calling this function:
 * - The SPI Slave transaction queue will be reset.
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

    spihost[host]->cur_trans = NULL;
    xQueueReset(spihost[host]->trans_queue);

    return ESP_OK;
}

esp_err_t SPI_SLAVE_ISR_ATTR spi_slave_queue_reset_isr(spi_host_device_t host)
{
    ESP_RETURN_ON_FALSE_ISR(is_valid_host(host), ESP_ERR_INVALID_ARG, SPI_TAG, "invalid host");
    ESP_RETURN_ON_FALSE_ISR(spihost[host], ESP_ERR_INVALID_ARG, SPI_TAG, "host not slave");

    spi_slave_transaction_t *trans = NULL;
    BaseType_t do_yield = pdFALSE;
    while (pdFALSE == xQueueIsQueueEmptyFromISR(spihost[host]->trans_queue))
    {
        xQueueReceiveFromISR(spihost[host]->trans_queue, &trans, &do_yield);
    }
    if (do_yield)
    {
        portYIELD_FROM_ISR();
    }

    spihost[host]->cur_trans = NULL;
    return ESP_OK;
}

esp_err_t SPI_SLAVE_ATTR spi_slave_queue_trans(spi_host_device_t host, const spi_slave_transaction_t *trans_desc, TickType_t ticks_to_wait)
{
    BaseType_t r;
    SPI_CHECK(is_valid_host(host), "invalid host", ESP_ERR_INVALID_ARG);
    SPI_CHECK(spihost[host], "host not slave", ESP_ERR_INVALID_ARG);
    SPI_CHECK(spihost[host]->dma_enabled == 0 || trans_desc->tx_buffer == NULL || esp_ptr_dma_capable(trans_desc->tx_buffer),
              "txdata not in DMA-capable memory", ESP_ERR_INVALID_ARG);
    SPI_CHECK(spihost[host]->dma_enabled == 0 || trans_desc->rx_buffer == NULL ||
                  (esp_ptr_dma_capable(trans_desc->rx_buffer) && esp_ptr_word_aligned(trans_desc->rx_buffer) &&
                   (trans_desc->length % 4 == 0)),
              "rxdata not in DMA-capable memory or not WORD aligned", ESP_ERR_INVALID_ARG);

    SPI_CHECK(trans_desc->length <= spihost[host]->max_transfer_sz * 8, "data transfer > host maximum", ESP_ERR_INVALID_ARG);
    r = xQueueSend(spihost[host]->trans_queue, (void *)&trans_desc, ticks_to_wait);
    if (!r)
        return ESP_ERR_TIMEOUT;
    esp_intr_enable(spihost[host]->intr);
    return ESP_OK;
}

esp_err_t SPI_SLAVE_ISR_ATTR spi_slave_queue_trans_isr(spi_host_device_t host, const spi_slave_transaction_t *trans_desc)
{
    BaseType_t r;
    BaseType_t do_yield = pdFALSE;
    ESP_RETURN_ON_FALSE_ISR(is_valid_host(host), ESP_ERR_INVALID_ARG, SPI_TAG, "invalid host");
    ESP_RETURN_ON_FALSE_ISR(spihost[host], ESP_ERR_INVALID_ARG, SPI_TAG, "host not slave");
    ESP_RETURN_ON_FALSE_ISR(spihost[host]->dma_enabled == 0 || trans_desc->tx_buffer == NULL || esp_ptr_dma_capable(trans_desc->tx_buffer),
                            ESP_ERR_INVALID_ARG, SPI_TAG, "txdata not in DMA-capable memory");
    ESP_RETURN_ON_FALSE_ISR(spihost[host]->dma_enabled == 0 || trans_desc->rx_buffer == NULL ||
                                (esp_ptr_dma_capable(trans_desc->rx_buffer) && esp_ptr_word_aligned(trans_desc->rx_buffer) &&
                                 (trans_desc->length % 4 == 0)),
                            ESP_ERR_INVALID_ARG, SPI_TAG, "rxdata not in DMA-capable memory or not WORD aligned");
    ESP_RETURN_ON_FALSE_ISR(trans_desc->length <= spihost[host]->max_transfer_sz * 8, ESP_ERR_INVALID_ARG, SPI_TAG, "data transfer > host maximum");

    r = xQueueSendFromISR(spihost[host]->trans_queue, (void *)&trans_desc, &do_yield);
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

esp_err_t SPI_SLAVE_ATTR spi_slave_get_trans_result(spi_host_device_t host, spi_slave_transaction_t **trans_desc, TickType_t ticks_to_wait)
{
    BaseType_t r;
    SPI_CHECK(is_valid_host(host), "invalid host", ESP_ERR_INVALID_ARG);
    SPI_CHECK(spihost[host], "host not slave", ESP_ERR_INVALID_ARG);
    // if SPI_SLAVE_NO_RETURN_RESULT is set, ret_queue will always be empty
    SPI_CHECK(!(spihost[host]->cfg.flags & SPI_SLAVE_NO_RETURN_RESULT), "API not Supported!", ESP_ERR_NOT_SUPPORTED);

    r = xQueueReceive(spihost[host]->ret_queue, (void *)trans_desc, ticks_to_wait);
    if (!r)
        return ESP_ERR_TIMEOUT;
    return ESP_OK;
}

esp_err_t SPI_SLAVE_ATTR spi_slave_transmit(spi_host_device_t host, spi_slave_transaction_t *trans_desc, TickType_t ticks_to_wait)
{
    esp_err_t ret;
    spi_slave_transaction_t *ret_trans;
    // ToDo: check if any spi transfers in flight
    ret = spi_slave_queue_trans(host, trans_desc, ticks_to_wait);
    if (ret != ESP_OK)
        return ret;
    ret = spi_slave_get_trans_result(host, &ret_trans, ticks_to_wait);
    if (ret != ESP_OK)
        return ret;
    assert(ret_trans == trans_desc);
    return ESP_OK;
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
    spi_slave_transaction_t *trans = NULL;
    spi_slave_t *host = (spi_slave_t *)arg;
    spi_slave_hal_context_t *hal = &host->hal;

    assert(spi_slave_hal_usr_is_done(hal));

    bool use_dma = host->dma_enabled;
    if (host->cur_trans)
    {
        // When DMA is enabled, the slave rx dma suffers from unexpected transactions. Forbid reading until transaction ready.
        if (use_dma)
            freeze_cs(host);

        spi_slave_hal_store_result(hal);
        host->cur_trans->trans_len = spi_slave_hal_get_rcv_bitlen(hal);

#if CONFIG_IDF_TARGET_ESP32
        // This workaround is only for esp32
        if (spi_slave_hal_dma_need_reset(hal))
        {
            // On ESP32, actual_tx_dma_chan and actual_rx_dma_chan are always same
            spicommon_dmaworkaround_req_reset(host->tx_dma_chan, spi_slave_restart_after_dmareset, host);
        }
#endif // #if CONFIG_IDF_TARGET_ESP32

        if (host->cfg.post_trans_cb)
            host->cfg.post_trans_cb(host->cur_trans);

        if (!(host->cfg.flags & SPI_SLAVE_NO_RETURN_RESULT))
        {
            xQueueSendFromISR(host->ret_queue, &host->cur_trans, &do_yield);
        }
        host->cur_trans = NULL;
    }

#if CONFIG_IDF_TARGET_ESP32
    // This workaround is only for esp32
    if (use_dma)
    {
        // On ESP32, actual_tx_dma_chan and actual_rx_dma_chan are always same
        spicommon_dmaworkaround_idle(host->tx_dma_chan);
        if (spicommon_dmaworkaround_reset_in_progress())
        {
            // We need to wait for the reset to complete. Disable int (will be re-enabled on reset callback) and exit isr.
            esp_intr_disable(host->intr);
            if (do_yield)
                portYIELD_FROM_ISR();
            return;
        }
    }
#endif // #if CONFIG_IDF_TARGET_ESP32

    // Disable interrupt before checking to avoid concurrency issue.
    esp_intr_disable(host->intr);
    // Grab next transaction
    r = xQueueReceiveFromISR(host->trans_queue, &trans, &do_yield);
    if (r)
    {
        // sanity check
        assert(trans);

        // enable the interrupt again if there is packet to send
        esp_intr_enable(host->intr);

        // We have a transaction. Send it.
        host->cur_trans = trans;

        hal->bitlen = trans->length;
        hal->rx_buffer = trans->rx_buffer;
        hal->tx_buffer = trans->tx_buffer;

#if CONFIG_IDF_TARGET_ESP32
        if (use_dma)
        {
            // This workaround is only for esp32
            // On ESP32, actual_tx_dma_chan and actual_rx_dma_chan are always same
            spicommon_dmaworkaround_transfer_active(host->tx_dma_chan);
        }
#endif // #if CONFIG_IDF_TARGET_ESP32

        spi_slave_hal_prepare_data(hal);

        // The slave rx dma get disturbed by unexpected transaction. Only connect the CS when slave is ready.
        if (use_dma)
        {
            restore_cs(host);
        }

        // Kick off transfer
        spi_slave_hal_user_start(hal);
        if (host->cfg.post_setup_cb)
            host->cfg.post_setup_cb(trans);
    }
    if (do_yield)
        portYIELD_FROM_ISR();
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
        ret = spicommon_dma_chan_alloc(host, dma_chan, &actual_tx_dma_chan, &actual_rx_dma_chan);
        if (ret != ESP_OK)
        {
            goto cleanup;
        }
    }
    printf("SPI Slave %ld using DMA channelels tx=%ld and rx=%lx\n", (uint32_t)host, actual_tx_dma_chan, actual_rx_dma_chan);

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
    spihost[host]->tx_dma_chan = actual_tx_dma_chan;
    spihost[host]->rx_dma_chan = actual_rx_dma_chan;
    if (use_dma)
    {
        // See how many dma descriptors we need and allocate them
        dma_desc_ct = (bus_config->max_transfer_sz + CHUNK_SIZE - 1) / CHUNK_SIZE;
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
        .dma_in = SPI_LL_GET_HW(host),
        .dma_out = SPI_LL_GET_HW(host)};
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
    hal->tx_dma_chan = actual_tx_dma_chan;
    hal->rx_dma_chan = actual_rx_dma_chan;

    spi_slave_hal_setup_device(hal);

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
        spicommon_dma_chan_free(host);
    }

    free(spihost[host]);
    spihost[host] = NULL;
    spicommon_periph_free(host);

    return ret;

} // SpiSlaveInitLite

static IRAM_ATTR void QuickLink_SingleDescriptor(lldesc_t *dmadesc, const void *data, int len, bool isrx)
{

// from lldesc.h
#define LLDESC_MAX_NUM_PER_DESC (4096 - 4)

    // only using a single chunk to make this
    // function quicker, but it means we're limtied to like 4096 bytes
    assert(len <= LLDESC_MAX_NUM_PER_DESC);

    int dmachunklen = len;

    if (isrx)
    {
        // Receive needs DMA length rounded to next 32-bit boundary
        dmadesc[0].size = (dmachunklen + 3) & (~3);
        dmadesc[0].length = (dmachunklen + 3) & (~3);
    }
    else
    {
        dmadesc[0].size = dmachunklen;
        dmadesc[0].length = dmachunklen;
    }
    dmadesc[0].buf = (uint8_t *)data;
    dmadesc[0].eof = 1; // we're only using the one chunk
    dmadesc[0].sosf = 0;
    dmadesc[0].owner = 1;
    dmadesc[0].qe.stqe_next = NULL; // we're only using one chunk

} // QuickLink

// lldesc_setup_link_constrained from lldesc.c
// Make sure spi_slave_init_lite sets up enough descriptors. Probs does.
// Tbh I know for sure it's setting up way too many. Yolo.
static IRAM_ATTR void QuickLink_Chunked(lldesc_t *dmadesc, const void *data, int len, bool isrx)
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
            dmadesc[n].size = (dmachunklen + 3) & (~3);
            dmadesc[n].length = (dmachunklen + 3) & (~3);
        }
        else
        {

            // dmachunklen, len = can't inject [1]
            // len, len = can't inject [1], doubt it's working

            // dmachunklen, dmachunklen = does inject chunktest to [1], but fails with repeats on [2]
            // (without the full reset you also appear to be able to change the buffer via setsecondchunk, on the correct boundary)

            // len, dmachunklen = does inject chunktest to [1], but fails with repeats on [2]
            // (without full reset, you're able to change the buffer via setsecondchunk, on the correct boundary )

            dmadesc[n].size = dmachunklen;
            dmadesc[n].length = dmachunklen;
        }

        dmadesc[n].buf = (uint8_t *)data;
        dmadesc[n].eof = 0;
        dmadesc[n].sosf = 0;
        dmadesc[n].owner = 1;
        dmadesc[n].qe.stqe_next = &dmadesc[n + 1];
        len -= dmachunklen;
        data += dmachunklen;
        n++;
    }
    dmadesc[n - 1].eof = 1; // Mark last DMA desc as end of stream.
    dmadesc[n - 1].qe.stqe_next = NULL;
    printf("set up %d descriptors\n", n);

} // QuickLink_Chunked

// cached inlink and outlink values
static uint32_t outLink[3] = {0};
static uint32_t inLink[3] = {0};
// static uint32_t pickle = 0;

// Setup sequence derived from "prepare_data"
// but without setting up the same unnecessary bits every time
// e.g. need to get us below about 2.6us
// formerly InitStuff

spi_slave_transaction_t transactions[3];

void SpiSlaveInitBuffersLite(uint32_t whichHost, uint8_t *txBuffer, uint8_t *rxBuffer, uint32_t inLength, uint32_t flashDebug)
{

    // add a transaction so we have one to write into
    // TODO: is this transaction bs actually necessary? or is it all HAL
    spi_slave_transaction_t *transaction = &transactions[whichHost];
    transaction->length = inLength;
    transaction->rx_buffer = rxBuffer;
    transaction->tx_buffer = txBuffer;
    spihost[whichHost]->cur_trans = &transaction;

    // spi_slave_hal.h
    spi_slave_hal_context_t *hal = &spihost[whichHost]->hal;
    uint32_t rxChan = hal->rx_dma_chan;
    uint32_t txChan = hal->tx_dma_chan;

    // Doens't really work without this
    hal->bitlen = inLength * 8;

    // lldesc_setup_link(hal->dmadesc_rx, hal->rx_buffer, ((hal->bitlen  7) / 8), true);
    QuickLink_SingleDescriptor(hal->dmadesc_rx, hal->rx_buffer, inLength, true);

    hal->dma_in->dma_conf.dma_rx_ena = 1;
    GDMA.channel[rxChan].in.link.addr = &hal->dmadesc_rx[0];
    GDMA.channel[rxChan].in.link.start = 1;
    printf("initial inlink %lx\n", GDMA.channel[rxChan].in.link.val);
    inLink[whichHost] = GDMA.channel[rxChan].in.link.val | (1 << 22);

    // lldesc_setup_link(hal->dmadesc_tx, hal->tx_buffer, (hal->bitlen  7) / 8, false);
    QuickLink_Chunked(hal->dmadesc_tx, hal->tx_buffer, inLength, false);

    // pickle = (uint32_t)&hal->dmadesc_tx[0].buf;
    hal->hw->dma_conf.dma_tx_ena = 1;
    GDMA.channel[txChan].out.link.addr = (&hal->dmadesc_tx[0]);
    GDMA.channel[txChan].out.link.start = 1;
    printf("initial outlink %lx\n", GDMA.channel[txChan].out.link.val);
    outLink[whichHost] = GDMA.channel[txChan].out.link.val | (1 << 21);

    // on the hal side of things
    hal->rx_buffer = rxBuffer;
    hal->tx_buffer = txBuffer;

    // and on the transaction side of things
    // note: will fail if SetTrans hasn't happened
    // spihost[whichHost]->cur_trans->rxBuffer;
    // spihost[whichHost]->cur_trans->txBuffer;

    // Might have to go in the buffers
    if ( whichHost == SPI2_HOST ){
        CacheValues_HOST2();
    }

}

// This is largely taken from spi_slave_hal_iram.c's prepare_data() function
// formerly SendStuff
IRAM_ATTR void SpiSlaveSendLite(uint32_t whichHost)
{

    // E.g.
    // spi_slave_hal_context_t * hal = &spihost[whichHost]->hal;
    activeHal = &spihost[whichHost]->hal;
    activeRxChan = activeHal->rx_dma_chan;
    activeTxChan = activeHal->tx_dma_chan;

    // Fill DMA descriptors
    // if (hal->rx_buffer) {

    // Moved to InitStuff
    // lldesc_setup_link(hal->dmadesc_rx, hal->rx_buffer, ((hal->bitlen + 7) / 8), true);
    // QuickLink(hal->dmadesc_rx, hal->rx_buffer, hal->bitlen, true);
    activeHal->dmadesc_rx[0].buf = (uint8_t *)activeHal->rx_buffer;

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
    activeHal->dma_in->dma_int_clr.val = 0xFFFFFFFF;

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
    GDMA.channel[activeRxChan].in.link.val = inLink[whichHost];

    //}

    // if (hal->tx_buffer) {

    // Moved to InitStuff
    // lldesc_setup_link(hal->dmadesc_tx, hal->tx_buffer, (hal->bitlen + 7) / 8, false);
    // QuickLink(hal->dmadesc_tx, hal->tx_buffer, hal->bitlen, false);
    activeHal->dmadesc_tx[0].buf = (uint8_t *)activeHal->tx_buffer;

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
    GDMA.channel[activeTxChan].out.link.val = outLink[whichHost];

    //}

    // shared
    // hal->hw->slave.soft_reset = 1;
    // skip this // hal->hw->slave.soft_reset = 0; // skip this
    activeHal->hw->slave.val = 0b00001110100000000000000000000000;
    // skip this // hal->hw->slave.val = 0b0000'0110'1000'0000'0000'0000'0000'0000;

    // This points at the <dma>.dma_conf.* union
    // e.g. the thing in spi_struct.h
    // bits 0 & 1 (full/empty), bits (idk, they're already set, let's re-set them)
    // bits 27 & 28 (dma_rx_ena  & dma_tx_ena)
    // bits 31 & 29 (dma_afifo_rst & rx_afifo_rst) <-- this is correct, idk about the third one
    activeHal->dma_out->dma_conf.val = 0b10111000000000000000000000000011;

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

//
// Resets only the RX buffer
// e.g. just reset the "trans_done" flag
// the tx buffer can stay unchanged and just
// send garbage bytes till the next interruption.
//

//
// Note: requires SpiSlaveSendLite to be called at least once
//       to set up the cachedHal variable
//

IRAM_ATTR void SetCSConnected(uint32_t whichHost, uint32_t inState)
{

    // 0x3fcc589c
    // 0x6000430c


    // 60 did have an affect, but the wrong pins
    // 70-72 card must be formatted - messes with MISO, slightly affects /CS2
    // 80 kills the screen
    // 90 breaks some fings
    // 95-100 is fine

    activeHal = &spihost[whichHost]->hal;
    int start = 71;
    int end = 72;

    // gpio_matrix_in(GPIO_MATRIX_CONST_ONE_INPUT, spi_periph_signal[whichHost].spics_in, false);
    // uint32_t oldValue = GPIO.func_in_sel_cfg[start].sig_in_sel;
    // printf("potatoes 0x%lx\n" + oldValue);

    // gpio_matrix_in(9, spi_periph_signal[whichHost].spics_in, false);
    // uint32_t newValue = GPIO.func_in_sel_cfg[start].sig_in_sel;
    // printf("potatoes 0x%lx\n" + newValue);

    if (inState)
    {

        

        // 0b0000'0110'1000'0000'0000'0000'0000'0000
        // activeHal->hw->slave.val = 0b00001110100000000000000000000000;
        // QuickReset(whichHost);
        // GDMA.channel[activeRxChan].in.conf0.val = 0b0;
        // activeHal->dma_in->dma_int_clr.val = 0xFFFFFFFF;
        // activeHal->dma_in->misc.slave_cs_pol = 0;
        // activeHal->dma_in->misc.cs_keep_active = 0;
        // uint32_t read = READ_PERI_REG(GPIO_FUNC9_IN_SEL_CFG_REG);
        // WRITE_PERI_REG(GPIO_FUNC9_IN_SEL_CFG_REG, read | GPIO_FUNC9_IN_INV_SEL);

        for (int i = start; i < end; i++)
        {
            GPIO.func_in_sel_cfg[i].sig_in_inv = 0;
            // REG_WRITE( &GPIO.func_in_sel_cfg[i].val, 0x01010101);
            // GPIO.func_in_sel_cfg[i].val = 0x01010101;
            //GPIO.func_in_sel_cfg[i].sig_in_sel = !GPIO.func_in_sel_cfg[i].sig_in_sel;
            //GPIO.func_in_sel_cfg[i].func_sel = !GPIO.func_in_sel_cfg[i].func_sel;
        }

        // uint32_t pickles = &GPIO.func_in_sel_cfg[9].val;
    }
    else
    {
        // 0b0000'0110'1000'0000'0000'0000'0000'0000
        // activeHal->hw->slave.val = 0b00001110100000000000000000000000;
        // QuickReset(whichHost);
        // GDMA.channel[activeRxChan].in.conf0.val = 0b0;
        // activeHal->dma_in->dma_int_clr.val = 0xFFFFFFFF;
        // activeHal->dma_in->misc.slave_cs_pol = 0;
        // activeHal->dma_in->misc.cs_keep_active = 0;
        // uint32_t read = READ_PERI_REG(GPIO_FUNC9_IN_SEL_CFG_REG);
        // WRITE_PERI_REG(GPIO_FUNC9_IN_SEL_CFG_REG, read | GPIO_FUNC9_IN_INV_SEL);

        for (int i = start; i < end; i++)
        {
            // REG_WRITE( &GPIO.func_in_sel_cfg[i].val, 0x01010101);
            // GPIO.func_in_sel_cfg[i].val = 0x01010101;
            //GPIO.func_in_sel_cfg[i].sig_in_inv = 1;
            
            //GPIO.func_in_sel_cfg[i].sig_in_sel = !GPIO.func_in_sel_cfg[i].sig_in_sel;
            //GPIO.func_in_sel_cfg[i].func_sel = !GPIO.func_in_sel_cfg[i].func_sel;
            //gpio_hold_en((gpio_num_t)9);
        }

        // uint32_t pickles = &GPIO.func_in_sel_cfg[9].val;

    }
}


// SPI2_HOST = HOST_PEEK
// SPI3_HOST = HOST_MAINs
static spi_slave_hal_context_t * hal_host2 = NULL;
static uint32_t rxChan_host2 = 0;
static uint32_t txChan_host2 = 0;


void CacheValues_HOST2(){
    printf("__TEST__Caching values for SPI2 host\n");
    hal_host2 = &spihost[SPI2_HOST]->hal;
    rxChan_host2 = hal_host2->rx_dma_chan;
    txChan_host2 = hal_host2->tx_dma_chan;
}

// overkill but it avoids us having to do a bunch of array lookups
// See QuickReset for a list of changes from the original version
IRAM_ATTR void QuickReset_HOST2(){

    // QuickReset(SPI2_HOST);
    // return;

    GDMA.channel[rxChan_host2].in.conf0.val = 0b0;

    hal_host2->dma_in->dma_int_clr.val = 0xFFFFFFFF;
    GDMA.channel[rxChan_host2].in.link.val = inLink[SPI2_HOST];


    GDMA.channel[txChan_host2].out.conf0.val = 0b111001; // reset
    GDMA.channel[txChan_host2].out.conf0.val = 0b111000; // unreset

    GDMA.channel[txChan_host2].out.link.val = outLink[SPI2_HOST];
    hal_host2->dma_out->dma_conf.val = 0b10111000000000000000000000000011;


}

IRAM_ATTR void QuickReset(uint32_t whichHost)
{

    activeHal = &spihost[whichHost]->hal;
    activeRxChan = activeHal->rx_dma_chan;
    activeTxChan = activeHal->tx_dma_chan;

    // the comment "//X" marks bits which have been removed from SpiSlaveSendLite
    // Note: if you remove too much, it doesn't clear the trans_done flag

    // use a cached version instead to save some time
    // spi_slave_hal_context_t * hal = &spihost[whichHost]->hal;
    // uint32_t rxChan = hal->rx_dma_chan;
    // uint32_t txChan = hal->tx_dma_chan;

    // Fill DMA descriptors
    // if (hal->rx_buffer) {

    // Moved to InitStuff
    // lldesc_setup_link(hal->dmadesc_rx, hal->rx_buffer, ((hal->bitlen + 7) / 8), true);
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
    // X works without, but will not clear the trans_done flag
    activeHal->dma_in->dma_int_clr.val = 0xFFFFFFFF;

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
    GDMA.channel[activeRxChan].in.link.val = inLink[whichHost];

    //}

    // if (hal->tx_buffer) {

    // Moved to InitStuff
    // lldesc_setup_link(hal->dmadesc_tx, hal->tx_buffer, (hal->bitlen + 7) / 8, false);
    // QuickLink(hal->dmadesc_tx, hal->tx_buffer, hal->bitlen, false);
    // X
    // Yhal->dmadesc_tx[0].buf = (uint8_t*)hal->tx_buffer;

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
    GDMA.channel[activeTxChan].out.link.val = outLink[whichHost];

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
    activeHal->dma_out->dma_conf.val = 0b10111000000000000000000000000011;

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
