#include "link_diag.h"
#include "robot_config.h"
#include "stm32f4xx_hal.h"
#include <string.h>

#ifndef FAULT_LOG_CAPACITY
#define FAULT_LOG_CAPACITY 64u
#endif

typedef struct {
    link_diag_snapshot_t snap;
} link_diag_ctx_t;

static link_diag_ctx_t s_diag[LINK_ID_MAX];
static fault_log_record_t s_fault_log[FAULT_LOG_CAPACITY];
static uint16_t s_fault_head = 0;
static uint16_t s_fault_count = 0;
static uint32_t s_fault_dropped = 0;

static uint8_t valid_link(link_id_t link_id)
{
    return (link_id >= 0 && link_id < LINK_ID_MAX) ? 1u : 0u;
}

void LinkDiag_Init(void)
{
    memset(s_diag, 0, sizeof(s_diag));
    memset(s_fault_log, 0, sizeof(s_fault_log));
    s_fault_head = 0;
    s_fault_count = 0;
    s_fault_dropped = 0;
}

void LinkDiag_ResetLink(link_id_t link_id)
{
    if (!valid_link(link_id)) return;
    memset(&s_diag[link_id], 0, sizeof(s_diag[link_id]));
}

void LinkDiag_Count(link_id_t link_id, uint16_t err_code)
{
    if (!valid_link(link_id)) return;
    s_diag[link_id].snap.last_err = err_code;
}

void LinkDiag_IncRxTotal(link_id_t link_id) { if (valid_link(link_id)) s_diag[link_id].snap.rx_total++; }
void LinkDiag_IncRxOk(link_id_t link_id) { if (valid_link(link_id)) s_diag[link_id].snap.rx_ok++; }
void LinkDiag_IncRxDrop(link_id_t link_id) { if (valid_link(link_id)) s_diag[link_id].snap.rx_drop++; }
void LinkDiag_IncCobsErr(link_id_t link_id) { if (valid_link(link_id)) s_diag[link_id].snap.cobs_err++; }
void LinkDiag_IncCrcErr(link_id_t link_id) { if (valid_link(link_id)) s_diag[link_id].snap.crc_err++; }
void LinkDiag_IncLenErr(link_id_t link_id) { if (valid_link(link_id)) s_diag[link_id].snap.len_err++; }
void LinkDiag_IncSofErr(link_id_t link_id) { if (valid_link(link_id)) s_diag[link_id].snap.sof_err++; }
void LinkDiag_IncEofErr(link_id_t link_id) { if (valid_link(link_id)) s_diag[link_id].snap.eof_err++; }
void LinkDiag_IncSeqDup(link_id_t link_id) { if (valid_link(link_id)) s_diag[link_id].snap.seq_dup++; }
void LinkDiag_IncAckTimeout(link_id_t link_id) { if (valid_link(link_id)) s_diag[link_id].snap.ack_timeout++; }
void LinkDiag_IncRetransmit(link_id_t link_id) { if (valid_link(link_id)) s_diag[link_id].snap.retransmit++; }
void LinkDiag_IncUartErr(link_id_t link_id) { if (valid_link(link_id)) s_diag[link_id].snap.uart_err++; }
void LinkDiag_IncTxFail(link_id_t link_id) { if (valid_link(link_id)) s_diag[link_id].snap.tx_fail++; }

void LinkDiag_PushFault(link_id_t link_id,
                        uint8_t severity,
                        uint16_t err_code,
                        uint8_t seq,
                        uint8_t msg_type,
                        uint16_t aux0,
                        uint16_t aux1)
{
    if (!valid_link(link_id)) return;

    fault_log_record_t rec;
    rec.ts_ms = HAL_GetTick();
    rec.link_id = (uint8_t)link_id;
    rec.severity = severity;
    rec.err_code = err_code;
    rec.seq = seq;
    rec.msg_type = msg_type;
    rec.aux0 = aux0;
    rec.aux1 = aux1;

    if (s_fault_count >= FAULT_LOG_CAPACITY) {
        s_fault_dropped++;
    } else {
        s_fault_count++;
    }
    s_fault_log[s_fault_head] = rec;
    s_fault_head = (uint16_t)((s_fault_head + 1u) % FAULT_LOG_CAPACITY);

    s_diag[link_id].snap.last_err = err_code;
}

void LinkDiag_GetSnapshot(link_id_t link_id, link_diag_snapshot_t *out)
{
    if (!out) return;
    if (!valid_link(link_id)) {
        memset(out, 0, sizeof(*out));
        return;
    }
    *out = s_diag[link_id].snap;
}

void LinkDiag_ClearSnapshot(link_id_t link_id)
{
    if (!valid_link(link_id)) return;
    memset(&s_diag[link_id].snap, 0, sizeof(s_diag[link_id].snap));
}

uint16_t LinkDiag_GetFaultLog(uint8_t cursor,
                              fault_log_record_t *out,
                              uint8_t max_records,
                              uint8_t *next_cursor)
{
    uint16_t i;
    uint16_t count = s_fault_count;
    uint16_t oldest;
    uint16_t copied = 0;

    if (next_cursor) *next_cursor = cursor;
    if (!out || max_records == 0u || count == 0u) return 0u;

    if (cursor >= count) cursor = 0u;

    oldest = (uint16_t)((s_fault_head + FAULT_LOG_CAPACITY - count) % FAULT_LOG_CAPACITY);
    for (i = cursor; i < count && copied < max_records; ++i) {
        uint16_t idx = (uint16_t)((oldest + i) % FAULT_LOG_CAPACITY);
        out[copied++] = s_fault_log[idx];
    }

    if (next_cursor) {
        uint16_t n = (uint16_t)(cursor + copied);
        *next_cursor = (n >= count) ? 0u : (uint8_t)n;
    }

    return copied;
}

void LinkDiag_ClearFaultLog(void)
{
    memset(s_fault_log, 0, sizeof(s_fault_log));
    s_fault_head = 0u;
    s_fault_count = 0u;
    s_fault_dropped = 0u;
}

uint32_t LinkDiag_GetDroppedFaultCount(void)
{
    return s_fault_dropped;
}
