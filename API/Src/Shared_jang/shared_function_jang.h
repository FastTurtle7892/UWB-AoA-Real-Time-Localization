#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdio.h>


int getPreambleLength(int txPreambLength);

int getPacLength(int rxPAC);

void getDelayTime_initiator(dwt_config_t *config, uint32_t* res_rx_final_tx, uint32_t* poll_tx_res_rx, uint32_t* resp_timeout);
void getDelayTime_responder(dwt_config_t *config, uint32_t* poll_rx_res_tx, uint32_t* res_tx_final_Rx, uint32_t* final_timeout);
void saveDistancesToTxt(const double* distances, const char* filename, int size);

void timer_init_uss(void);
void timer4_irqhandler(void);
float TIMER4_OFTime(void);
uint32_t TIMER4_Check(uint32_t starttime, float clk_time);
uint64_t TIMER4_Check2(uint32_t starttime, float clk_time);

uint64_t get_10_pkt_time(uint8_t preamble_len);


typedef enum {
    DB_N30 = (0x00<<2)+3,
    DB_N29 = (0x01<<2)+1,
    DB_N28 = (0x02<<2)+0,
    DB_N27 = (0x01<<2)+3,
    DB_N26 = (0x02<<2)+1,
    DB_N25 = (0x03<<2)+0,
    DB_N24 = (0x02<<2)+3,
    DB_N23 = (0x03<<2)+1,
    DB_N22 = (0x05<<2)+0,
    DB_N21 = (0x03<<2)+3,
    DB_N20 = (0x07<<2)+0,
    DB_N19 = (0x04<<2)+3,
    DB_N18 = (0x06<<2)+1,
    DB_N17 = (0x05<<2)+3,
    DB_N16 = (0x06<<2)+3,
    DB_N15 = (0x07<<2)+3,
    DB_N14 = (0x08<<2)+3,
    DB_N13 = (0x09<<2)+3,
    DB_N12 = (0x0A<<2)+3,
    DB_N11 = (0x0F<<2)+1,
    DB_N10 = (0x0D<<2)+3,
    DB_N9 = (0x0F<<2)+3,
    DB_N8 = (0x11<<2)+3,
    DB_N7 = (0x14<<2)+3,
    DB_N6 = (0x17<<2)+3,
    DB_N5 = (0x1A<<2)+3,
    DB_N4 = (0x1F<<2)+3,
    DB_N3 = (0x24<<2)+3,
    DB_N2 = (0x2A<<2)+3,
    DB_N1 = (0x34<<2)+3,
    Db_N0 = (0x3F<<2)+3
} db_gain_e;






#ifdef __cplusplus
}
#endif