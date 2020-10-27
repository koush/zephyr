/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 * Copyright (c) 2020 Demant
 *
 * SPDX-License-Identifier: Apache-2.0
 */

void test_print_conn(struct ull_cp_conn *conn);

void test_set_role(struct ull_cp_conn *conn, uint8_t role);
void test_setup(struct ull_cp_conn *conn);
void event_prepare(struct ull_cp_conn *conn);
void event_tx_ack(struct ull_cp_conn *conn, struct node_tx *tx);
void event_done(struct ull_cp_conn *conn);
uint16_t event_counter(struct ull_cp_conn *conn);



#define lt_tx(_opcode, _conn, _param) lt_tx_real(__FILE__, __LINE__, _opcode, _conn, _param)
#define lt_rx(_opcode, _conn, _tx_ref, _param) lt_rx_real(__FILE__, __LINE__, _opcode, _conn, _tx_ref, _param)
#define lt_rx_q_is_empty(_conn) lt_rx_q_is_empty_real(__FILE__, __LINE__, _conn)

#define ut_rx_pdu(_opcode, _ntf_ref, _param) ut_rx_pdu_real(__FILE__, __LINE__, _opcode, _ntf_ref, _param)
#define ut_rx_node(_opcode, _ntf_ref, _param) ut_rx_node_real(__FILE__, __LINE__, _opcode, _ntf_ref, _param)
#define ut_rx_q_is_empty() ut_rx_q_is_empty_real(__FILE__, __LINE__)

void lt_tx_real(const char *file, uint32_t line, helper_pdu_opcode_t opcode, struct ull_cp_conn *conn, void *param);
void lt_rx_real(const char *file, uint32_t line, helper_pdu_opcode_t opcode, struct ull_cp_conn *conn, struct node_tx **tx_ref, void *param);
void lt_rx_q_is_empty_real(const char *file, uint32_t line, struct ull_cp_conn *conn);
void ut_rx_pdu_real(const char *file, uint32_t line, helper_pdu_opcode_t opcode, struct node_rx_pdu **ntf_ref, void *param);
void ut_rx_node_real(const char *file, uint32_t line, helper_node_opcode_t opcode, struct node_rx_pdu **ntf_ref, void *param);
void ut_rx_q_is_empty_real(const char *file, uint32_t line);
