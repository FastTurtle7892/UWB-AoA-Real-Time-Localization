#include <boards.h>
#include <deca_spi.h>
#include <examples_defines.h>
#include <port.h>
#include <sdk_config.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <deca_device_api.h>
#include "shared_function_jang.h"






int getPreambleLength(int txPreambLength) {
    switch (txPreambLength) {
    case DWT_PLEN_32: return 32;
    case DWT_PLEN_64: return 64;
    case DWT_PLEN_72: return 72;
    case DWT_PLEN_128: return 128;
    case DWT_PLEN_256: return 256;
    case DWT_PLEN_512: return 512;
    case DWT_PLEN_1024: return 1024;
    case DWT_PLEN_1536: return 1536;
    case DWT_PLEN_2048: return 2048;
    case DWT_PLEN_4096: return 4096;
    default: return 256; // 기본 �
    }
}

int getPacLength(int rxPAC) {
    switch (rxPAC) {
    case DWT_PAC8: return 8;
    case DWT_PAC16: return 16;
    case DWT_PAC32: return 32;
    default: return 4;
    }
}

void getDelayTime_initiator(dwt_config_t *config, uint32_t* res_rx_final_tx, uint32_t* poll_tx_res_rx, uint32_t* resp_timeout) {
    int preamble_len = getPreambleLength(config->txPreambLength);
    int sts_len = GET_STS_REG_SET_VALUE((uint16_t)(config->stsLength)) * 8;
    int pac_len = getPacLength(config->rxPAC);
    int sfd_len = 8;
    //float Tsymbol = 4 * 128 * 0.002003; // [usec]
    float Tsymbol = 4 * 127 * 0.002003; // [usec]
    int Tbit = ((19 + 17 * 8) / 0.85); // [usec]

    if (config->stsMode != DWT_STS_MODE_OFF) {
        // STS mode 관추� �업 �요 �기추�
        if ((config->pdoaMode == DWT_PDOA_M1) || (config->pdoaMode == DWT_PDOA_M0)) {
            // In PDOA mode 1, number of accumulated symbols is the whole length of the STS
        } else {
            // In PDOA mode 3 number of accumulated symbols is half of the length of STS symbols
        }
    }

    config->sfdTO = (preamble_len + 1 + sfd_len - pac_len); // SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only.

    int Tframe = Tsymbol * (preamble_len + sfd_len) + Tbit + 26;
    *res_rx_final_tx = (1000 + 7 + Tframe);
    *poll_tx_res_rx = (1000);
    *resp_timeout = (uint32_t)Tframe + 100;

}

void getDelayTime_responder(dwt_config_t *config, uint32_t* poll_rx_res_tx, uint32_t* res_tx_final_Rx, uint32_t* final_timeout) {
    int preamble_len = getPreambleLength(config->txPreambLength);
    int sts_len = GET_STS_REG_SET_VALUE((uint16_t)(config->stsLength)) * 8;
    int pac_len = getPacLength(config->rxPAC);
    int sfd_len = 8;
    //float Tsymbol = 4 * 128 * 0.002003; // [usec]
    float Tsymbol = 4 * 127 * 0.002003; // [usec]
    int Tbit = ((19 + 17 * 8) / 0.85); // [usec]

    if (config->stsMode != DWT_STS_MODE_OFF) {
        // STS mode 관추� �업 �요 �기추�
        if ((config->pdoaMode == DWT_PDOA_M1) || (config->pdoaMode == DWT_PDOA_M0)) {
            // In PDOA mode 1, number of accumulated symbols is the whole length of the STS
        } else {
            // In PDOA mode 3 number of accumulated symbols is half of the length of STS symbols
        }
    }

    config->sfdTO = (preamble_len + 1 + sfd_len - pac_len); // SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only.

    int Tframe = Tsymbol * (preamble_len + sfd_len) + Tbit + 26;
    //int Tframe = Tsymbol * (preamble_len + sfd_len + sts_len) + Tbit;
    *poll_rx_res_tx = (1000 + 5 + Tframe);
    *res_tx_final_Rx = (1000);
    *final_timeout = (uint32_t)Tframe + 100;

}

void getDelayTime_ss_twr_initiator(dwt_config_t *config, uint32_t* poll_tx_res_rx, uint32_t* resp_timeout) {
    int preamble_len = getPreambleLength(config->txPreambLength);
    int sts_len = GET_STS_REG_SET_VALUE((uint16_t)(config->stsLength)) * 8;
    int pac_len = getPacLength(config->rxPAC);
    int sfd_len = 8;
    float Tsymbol = 4 * 127 * 0.002003; // [usec]
    int Tbit = ((19 + 17 * 8) / 0.85); // [usec]

    if (config->stsMode != DWT_STS_MODE_OFF) {
        if ((config->pdoaMode == DWT_PDOA_M1) || (config->pdoaMode == DWT_PDOA_M0)) {
            // In PDOA mode 1, number of accumulated symbols is the whole length of the STS
        } else {
            // In PDOA mode 3 number of accumulated symbols is half of the length of STS symbols
        }
    }

    config->sfdTO = (preamble_len + 1 + sfd_len - pac_len); // SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only.

    int Tframe = Tsymbol * (preamble_len + sfd_len) + Tbit + 26;
    *poll_tx_res_rx = (1000);
    *resp_timeout = (uint32_t)Tframe + 100;

}

void getDelayTime_ss_twr_responder(dwt_config_t *config, uint32_t* poll_rx_res_tx) {
    int preamble_len = getPreambleLength(config->txPreambLength);
    int sts_len = GET_STS_REG_SET_VALUE((uint16_t)(config->stsLength)) * 8;
    int pac_len = getPacLength(config->rxPAC);
    int sfd_len = 8;
    float Tsymbol = 4 * 127 * 0.002003; // [usec]
    int Tbit = ((19 + 17 * 8) / 0.85); // [usec]

    if (config->stsMode != DWT_STS_MODE_OFF) {
        if ((config->pdoaMode == DWT_PDOA_M1) || (config->pdoaMode == DWT_PDOA_M0)) {
            // In PDOA mode 1, number of accumulated symbols is the whole length of the STS
        } else {
            // In PDOA mode 3 number of accumulated symbols is half of the length of STS symbols
        }
    }

    config->sfdTO = (preamble_len + 1 + sfd_len - pac_len); // SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only.

    int Tframe = Tsymbol * (preamble_len + sfd_len) + Tbit + 26;
    //int Tframe = Tsymbol * (preamble_len + sfd_len + sts_len) + Tbit;
    *poll_rx_res_tx = (1000 + 5 + Tframe);

}

void saveDistancesToTxt(const double* distances, const char* filename, int size) 
{
    FILE* file = fopen(filename, "w");
    if (file == NULL) {
        return;
    }
    for (int i = 0; i < size; ++i) {
        fprintf(file, "%f\n", distances[i]);
    }
    fclose(file);
}

void timer_init_uss(void){
    
    NRF_TIMER4->TASKS_STOP  = 1;
    NRF_TIMER4->TASKS_CLEAR = 1;

    NRF_TIMER4->MODE = TIMER_MODE_MODE_Timer;

    NRF_TIMER4->BITMODE = TIMER_BITMODE_BITMODE_32Bit;

    NRF_TIMER4->PRESCALER = 3;
    
    //NRF_TIMER4->CC[0] = (float)2 * ((float)period_us);

    NRF_TIMER4->INTENSET = 4294967295; //65536
    
    NRF_TIMER4->SHORTS = TIMER_SHORTS_COMPARE0_CLEAR_Msk;

    NVIC_EnableIRQ(TIMER4_IRQn);

    NRF_TIMER4->TASKS_START = 1;

}
//static int testCount= 0;
void TIMER4_IRQHandler(void){
    // overflow time = 2 ^32 * 0.25us(16M/4) = 2^10 * 3 = 2M sec... 
    if(NRF_TIMER4->EVENTS_COMPARE[0] == 1){
        
        NRF_TIMER4->EVENTS_COMPARE[0] = 0;
        NRF_TIMER4->TASKS_STOP  = 1;
        NRF_TIMER4->TASKS_CLEAR = 1;
        printf("IRQon");
        
    }
}

float TIMER4_OFTime(){
    int pre = NRF_TIMER4->PRESCALER;
    float clk_time = 1 / ( (16) / pow(2.0, pre));
    
    printf("prescaler = %d, clk time = %f us\n", pre, clk_time);
    printf("timer overflow time = %f second \n",pow(2.0, 32)* clk_time/ 1000000);
    return clk_time;
}
uint32_t TIMER4_Check(uint32_t starttime, float clk_time){
        NRF_TIMER4->TASKS_CAPTURE[1] = 1;
        uint32_t end_time = NRF_TIMER4->CC[1];
        uint32_t ticks = end_time - starttime;
        //printf("clock = %f", clk_time);
        //printf("elapsed_time = %f us, current time = %f us previous time = %f us/", ticks * clk_time, end_time * clk_time, starttime * clk_time);
        printf("\nelapsed_time = %f us\n", ticks * clk_time);
        return end_time;
}

uint64_t TIMER4_Check2(uint32_t starttime, float clk_time){

      NRF_TIMER4->TASKS_CAPTURE[1] = 1;
      uint32_t end_time = NRF_TIMER4->CC[1];
      uint32_t ticks = end_time - starttime;
      uint64_t result = ticks * clk_time;
      return result;
}


uint64_t get_10_pkt_time(uint8_t preamble_len)
{
  switch (preamble_len)
  {
    case DWT_PLEN_64: return 108000 * 11;
    case DWT_PLEN_128: return 108000 * 11;
    case DWT_PLEN_256: return 109000 * 11;
    case DWT_PLEN_512: return 111000 * 11;
    case DWT_PLEN_1024: return 115000 * 11;
    case DWT_PLEN_2048: return 122000 * 11;
    case DWT_PLEN_4096: return 136000 * 11;
    default: return 108000 * 11; 
  }
}
