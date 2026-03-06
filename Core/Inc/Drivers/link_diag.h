#ifndef LINK_DIAG_H
#define LINK_DIAG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef enum {
    LINK_ID_ESP = 0,
    LINK_ID_PC = 1,
    LINK_ID_MAX
} link_id_t;

typedef struct {
    uint32_t rx_total;
    uint32_t rx_ok;
    uint32_t rx_drop;
    uint32_t cobs_err;
    uint32_t crc_err;
    uint32_t len_err;
    uint32_t sof_err;
    uint32_t eof_err;
    uint32_t seq_dup;
    uint32_t ack_timeout;
    uint32_t retransmit;
    uint32_t uart_err;
    uint32_t tx_fail;
    uint16_t last_err;
} link_diag_snapshot_t;

typedef struct {
    uint32_t ts_ms;
    uint8_t link_id;
    uint8_t severity;
    uint16_t err_code;
    uint8_t seq;
    uint8_t msg_type;
    uint16_t aux0;
    uint16_t aux1;
} fault_log_record_t;

enum {
    FAULT_SEV_INFO = 0,
    FAULT_SEV_WARN = 1,
    FAULT_SEV_ERROR = 2,
    FAULT_SEV_FATAL = 3,
};

void LinkDiag_Init(void);
void LinkDiag_ResetLink(link_id_t link_id);
void LinkDiag_Count(link_id_t link_id, uint16_t err_code);
void LinkDiag_IncRxTotal(link_id_t link_id);
void LinkDiag_IncRxOk(link_id_t link_id);
void LinkDiag_IncRxDrop(link_id_t link_id);
void LinkDiag_IncCobsErr(link_id_t link_id);
void LinkDiag_IncCrcErr(link_id_t link_id);
void LinkDiag_IncLenErr(link_id_t link_id);
void LinkDiag_IncSofErr(link_id_t link_id);
void LinkDiag_IncEofErr(link_id_t link_id);
void LinkDiag_IncSeqDup(link_id_t link_id);
void LinkDiag_IncAckTimeout(link_id_t link_id);
void LinkDiag_IncRetransmit(link_id_t link_id);
void LinkDiag_IncUartErr(link_id_t link_id);
void LinkDiag_IncTxFail(link_id_t link_id);

void LinkDiag_PushFault(link_id_t link_id,
                        uint8_t severity,
                        uint16_t err_code,
                        uint8_t seq,
                        uint8_t msg_type,
                        uint16_t aux0,
                        uint16_t aux1);

void LinkDiag_GetSnapshot(link_id_t link_id, link_diag_snapshot_t *out);
void LinkDiag_ClearSnapshot(link_id_t link_id);

uint16_t LinkDiag_GetFaultLog(uint8_t cursor,
                              fault_log_record_t *out,
                              uint8_t max_records,
                              uint8_t *next_cursor);
void LinkDiag_ClearFaultLog(void);
uint32_t LinkDiag_GetDroppedFaultCount(void);

#ifdef __cplusplus
}
#endif

#endif
