#ifndef STPM3X_DEF_H
#define STPM3X_DEF_H
#include "stdint.h"
#include "spi_stpm32.h"

/*===========DSP_CR1===================*/

/*DSP_CR1 lsw Addr: 0x00
Bit 7:5   TC1[2:0]
Bit 4     ENVREF1
Bit 3     CLEARSS
Bit 3:0   CLRSS_TO1[3:0]
Bit 15:8  Reserved
*/
#define DSP_CR1_ADDR 0x00

#define DSP_CR_LSB_ADDR 0X00
#define DSP_CR1_TC1_Pos 5U
#define DSP_CR1_ENVREF1_Pos 4U
#define DSP_CR1_CLEARSS_Pos 3U
#define DSP_CR1_CLRSS_TO1_Pos 0U

#define DSP_CR1_TC1_Msk (0x7U << DSP_CR1_TC1_Pos)
#define DSP_CR1_ENVREF1_Msk (1U << DSP_CR1_ENVREF1_Pos)
#define DSP_CR1_CLEARSS_Msk (1U << DSP_CR1_CLEARSS_Pos)
#define DSP_CR1_CLRSS_TO1_Msk (0xFU << DSP_CR1_CLRSS_TO1_Pos)

typedef struct
{
    uint8_t tc1;       /* 0..7 */
    uint8_t envref1;   /* 0 or 1 */
    uint8_t clears;    /* 0 or 1 */
    uint8_t clrss_to1; /* 0..15 */
} dsp_cr1_lsw_cfg_t;

static inline uint16_t dsp_cr1_lsw_pack(const dsp_cr1_lsw_cfg_t *cfg)
{
    uint16_t reg = 0;

    reg |= ((cfg->tc1 & 0x7U) << DSP_CR1_TC1_Pos);
    reg |= ((cfg->envref1 & 0x1U) << DSP_CR1_ENVREF1_Pos);
    reg |= ((cfg->clears & 0x1U) << DSP_CR1_CLEARSS_Pos);
    reg |= ((cfg->clrss_to1 & 0xFU) << DSP_CR1_CLRSS_TO1_Pos);

    return reg;
}

/*dsp msw addr 0x00 DSP_CR1
Bits 31:28  LCS1[1:0]
Bits 27:24  LPS1[1:0]
Bits 23:20  LPW1[3:0]
Bits 19:18  ROC1[1:0]
Bit  17     BHPFC1
Bit  16     BHPFV1
*/

#define DSP_CR1_MSW_ADDR 0x01

#define DSP_CR1_LCS1_Pos 28U
#define DSP_CR1_LPS1_Pos 24U
#define DSP_CR1_LPW1_Pos 20U
#define DSP_CR1_ROC1_Pos 20U
#define DSP_CR1_BHPFC1_Pos 17U
#define DSP_CR1_BHPFV1_Pos 16U

#define DSP_CR1_LCS1_Msk (0x3UL << DSP_CR1_LCS1_Pos)
#define DSP_CR1_LPS1_Msk (0x3UL << DSP_CR1_LPS1_Pos)
#define DSP_CR1_LPW1_Msk (0xFUL << DSP_CR1_LPW1_Pos)
#define DSP_CR1_ROC1_Msk (0x3UL << DSP_CR1_ROC1_Pos)
#define DSP_CR1_BHPFC1_Msk (1UL << DSP_CR1_BHPFC1_Pos)
#define DSP_CR1_BHPFV1_Msk (1UL << DSP_CR1_BHPFV1_Pos)
#define DSP_CR1_APM1_Pos 18U
#define DSP_CR1_APM1_Msk (1UL << DSP_CR1_APM1_Pos)

typedef struct
{
    uint8_t lcs1;   /* 0..3 */
    uint8_t lps1;   /* 0..3 */
    uint8_t lpw1;   /* 0..15 */
    uint8_t roc1;   /* 0..3 */
    uint8_t bhpfc1; /* 0 or 1 */
    uint8_t bhpfv1; /* 0 or 1 */
    uint8_t apm1;   /* 0: fundamental, 1: active */
} dsp_cr1_msw_cfg_t;

static inline uint16_t dsp_cr1_msw_pack(const dsp_cr1_msw_cfg_t *cfg)
{
    uint32_t reg = 0;

    reg |= ((uint32_t)(cfg->lcs1 & 0x3U) << DSP_CR1_LCS1_Pos);
    reg |= ((uint32_t)(cfg->lps1 & 0x3U) << DSP_CR1_LPS1_Pos);
    reg |= ((uint32_t)(cfg->lpw1 & 0xFU) << DSP_CR1_LPW1_Pos);
    reg |= ((uint32_t)(cfg->roc1 & 0x3U) << DSP_CR1_ROC1_Pos);
    reg |= ((uint32_t)(cfg->bhpfc1 & 0x1U) << DSP_CR1_BHPFC1_Pos);
    reg |= ((uint32_t)(cfg->bhpfv1 & 0x1U) << DSP_CR1_BHPFV1_Pos);
    reg |= ((uint32_t)(cfg->apm1 & 0x1U) << DSP_CR1_APM1_Pos);

    /* Return MSW slice */
    return (uint16_t)((reg >> 16) & 0xFFFFU);
}

/*===============================DSP_CR2: LADDR:0X02(LSW) AND MADDRR 0X03(MSW)==========================*/

/* DSP_CR2_MSW [31:16] (address:0x03)
Bits 31:28  LCS2[1:0]
Bits 27:24  LPS2[1:0]
Bits 23:20  LPW2[3:0]
Bits 19:18  ROC2[1:0]
Bit  17     BHPFC2
Bit  16     BHPFV2

DSP_CR2_LSW [15:0] (address 0x02)
Bits 7:5   TC2[2:0]
Bit  4     ENVREF2
Bit  3     CLEARSS
Bits 3:0   CLRSS_TO2[3:0]
Bits 15:8  Reserved

*/

/*DSP_CR2 LSW */
#define DSP_CR2_LSW_ADDR 0x02
#define DSP_CR2_MSW_ADDR 0x03

#define DSP_CR2_TC2_Pos 5U
#define DSP_CR2_ENVREF2_Pos 4U
#define DSP_CR2_CLEARSS_Pos 3U
#define DSP_CR2_CLRSS_TO2_Pos 0U

#define DSP_CR2_TC2_Msk (0x7U << DSP_CR2_TC2_Pos)
#define DSP_CR2_ENVREF2_Msk (1U << DSP_CR2_ENVREF2_Pos)
#define DSP_CR2_CLEARSS_Msk (1U << DSP_CR2_CLEARSS_Pos)
#define DSP_CR2_CLRSS_TO2_Msk (0xFU << DSP_CR2_CLRSS_TO2_Pos)

/*LSW CHANNEL 2 DSP_CR2 */
typedef struct
{
    uint8_t tc2;       /* 0..7 */
    uint8_t envref2;   /* 0 or 1 */
    uint8_t clears;    /* 0 or 1 */
    uint8_t clrss_to2; /* 0..15 */
} dsp_cr2_lsw_cfg_t;

static inline uint16_t dsp_cr2_lsw_pack(const dsp_cr2_lsw_cfg_t *cfg)
{
    uint16_t reg = 0;

    reg |= ((cfg->tc2 & 0x7U) << DSP_CR2_TC2_Pos);
    reg |= ((cfg->envref2 & 0x1U) << DSP_CR2_ENVREF2_Pos);
    reg |= ((cfg->clears & 0x1U) << DSP_CR2_CLEARSS_Pos);
    reg |= ((cfg->clrss_to2 & 0xFU) << DSP_CR2_CLRSS_TO2_Pos);

    return reg;
}

/*DSP_CR2 MSW*/

#define DSP_CR2_LCS2_Pos 28U
#define DSP_CR2_LPS2_Pos 24U
#define DSP_CR2_LPW2_Pos 20U
#define DSP_CR2_ROC2_Pos 18U
#define DSP_CR2_BHPFC2_Pos 17U
#define DSP_CR2_BHPFV2_Pos 16U

#define DSP_CR2_LCS2_Msk (0x3UL << DSP_CR2_LCS2_Pos)
#define DSP_CR2_LPS2_Msk (0x3UL << DSP_CR2_LPS2_Pos)
#define DSP_CR2_LPW2_Msk (0xFUL << DSP_CR2_LPW2_Pos)
#define DSP_CR2_ROC2_Msk (0x3UL << DSP_CR2_ROC2_Pos)
#define DSP_CR2_BHPFC2_Msk (1UL << DSP_CR2_BHPFC2_Pos)
#define DSP_CR2_BHPFV2_Msk (1UL << DSP_CR2_BHPFV2_Pos)

typedef struct
{
    uint8_t lcs2;   /* 0..3 */
    uint8_t lps2;   /* 0..3 */
    uint8_t lpw2;   /* 0..15 */
    uint8_t roc2;   /* 0..3 */
    uint8_t bhpfc2; /* 0 or 1 */
    uint8_t bhpfv2; /* 0 or 1 */
} dsp_cr2_msw_cfg_t;

static inline uint16_t dsp_cr2_msw_pack(const dsp_cr2_msw_cfg_t *cfg)
{
    uint32_t reg = 0;

    reg |= ((uint32_t)(cfg->lcs2 & 0x3U) << DSP_CR2_LCS2_Pos);
    reg |= ((uint32_t)(cfg->lps2 & 0x3U) << DSP_CR2_LPS2_Pos);
    reg |= ((uint32_t)(cfg->lpw2 & 0xFU) << DSP_CR2_LPW2_Pos);
    reg |= ((uint32_t)(cfg->roc2 & 0x3U) << DSP_CR2_ROC2_Pos);
    reg |= ((uint32_t)(cfg->bhpfc2 & 0x1U) << DSP_CR2_BHPFC2_Pos);
    reg |= ((uint32_t)(cfg->bhpfv2 & 0x1U) << DSP_CR2_BHPFV2_Pos);

    return (uint16_t)((reg >> 16) & 0xFFFFU);
}

/*=================================DSP_CR3 Address: 0x04======================================================================*/

/* DSP_CR3 MSW 31:16
  Bit 23     REF_FREQ
Bit 22     EN_CUM
Bit 21     LED2_OFF
Bit 20     LED1_OFF
Bit 19     SW_AUTO_LATCH
Bit 18     SW_LATCH
Bit 17     SW_LATCH
Bit 16     SW_RESET


LSW [15:0] DSP_CR3
Bit 15     TMP_EN
Bits 14:13 TMP_TOL[1:0]
Bit 12     ZCR_EN
Bits 11:10 ZCR_SEL[1:0]
Bits 13:0  SAG_TIME_THR[13:0]

*/
#define DSP_CR3_LSW_ADDR 0x04
#define DSP_CR3_MSW_ADDR 0x05

/*LSW*/
#define DSP_CR3_SAG_TIME_Pos 0U
#define DSP_CR3_ZCR_SEL_Pos 10U
#define DSP_CR3_ZCR_EN_Pos 12U
#define DSP_CR3_TMP_TOL_Pos 13U
#define DSP_CR3_TMP_EN_Pos 15U

/*MSW*/
#define DSP_CR3_SW_RESET_Pos 16U
#define DSP_CR3_SW_LATCH_Pos 17U
#define DSP_CR3_SW_AUTO_Pos 19U
#define DSP_CR3_LED1_OFF_Pos 20U
#define DSP_CR3_LED2_OFF_Pos 21U
#define DSP_CR3_EN_CUM_Pos 22U
#define DSP_CR3_REF_FREQ_Pos 23U

typedef struct
{
    uint16_t sag_time_thr; /* 14 bits */
    uint8_t zcr_sel;       /* 0..3 */
    uint8_t zcr_en;
    uint8_t tmp_tol; /* 0..3 */
    uint8_t tmp_en;
} dsp_cr3_lsw_cfg_t;

typedef struct
{
    uint8_t sw_reset;
    uint8_t sw_latch;
    uint8_t sw_auto_latch;
    uint8_t led1_off;
    uint8_t led2_off;
    uint8_t en_cum;
    uint8_t ref_freq;
} dsp_cr3_msw_cfg_t;

static inline uint16_t dsp_cr3_lsw_pack(const dsp_cr3_lsw_cfg_t *c)
{
    uint16_t r = 0;
    r |= (c->sag_time_thr & 0x3FFFU) << DSP_CR3_SAG_TIME_Pos;
    r |= (c->zcr_sel & 0x3U) << DSP_CR3_ZCR_SEL_Pos;
    r |= (c->zcr_en & 0x1U) << DSP_CR3_ZCR_EN_Pos;
    r |= (c->tmp_tol & 0x3U) << DSP_CR3_TMP_TOL_Pos;
    r |= (c->tmp_en & 0x1U) << DSP_CR3_TMP_EN_Pos;
    return r;
}

static inline uint16_t dsp_cr3_msw_pack(const dsp_cr3_msw_cfg_t *c)
{
    uint32_t r = 0;
    r |= (uint32_t)c->sw_reset << DSP_CR3_SW_RESET_Pos;
    r |= (uint32_t)c->sw_latch << DSP_CR3_SW_LATCH_Pos;
    r |= (uint32_t)c->sw_auto_latch << DSP_CR3_SW_AUTO_Pos;
    r |= (uint32_t)c->led1_off << DSP_CR3_LED1_OFF_Pos;
    r |= (uint32_t)c->led2_off << DSP_CR3_LED2_OFF_Pos;
    r |= (uint32_t)c->en_cum << DSP_CR3_EN_CUM_Pos;
    r |= (uint32_t)c->ref_freq << DSP_CR3_REF_FREQ_Pos;
    return (uint16_t)(r >> 16);
}
/*============================DSP STATUS============================================================*/
#define STPM32_DSP_STATUS 0U
#define DSP_STATUS_ZX_Pos 0U
#define DSP_STATUS_OVF_Pos 1U

#define DSP_STATUS_ZX_Msk (1UL << DSP_STATUS_ZX_Pos)
#define DSP_STATUS_OVF_Msk (1UL << DSP_STATUS_OVF_Pos)
/*==========================================DSP_CR5 ADDR : 0X08=====================================*/

#define DSP_CR5_ADDR 0x08

typedef struct
{
    uint16_t sag_thr1;
    uint16_t swv_thr1;
    uint16_t chv1;
} dsp_cr5_cfg_t;

static inline uint32_t dsp_cr5_pack(const dsp_cr5_cfg_t *c)
{
    uint32_t r = 0;
    r |= (c->sag_thr1 & 0x3FFU) << 0;
    r |= (c->swv_thr1 & 0x3FFU) << 10;
    r |= (c->chv1 & 0xFFFU) << 20;
    return r;
}

/*========================================DSP_CR7  ADDR:0X0C==================================================*/

#define DSP_CR7_ADDR 0x0C

#define DSP_CR7_SAG_THR2_Pos 0U
#define DSP_CR7_SWV_THR2_Pos 10U
#define DSP_CR7_CHV2_Pos 20U

typedef struct
{
    uint16_t sag_thr2; /* 0..1023 */
    uint16_t swv_thr2; /* 0..1023 */
    uint16_t chv2;     /* 0..4095 */
} dsp_cr7_cfg_t;

static inline uint32_t dsp_cr7_pack(const dsp_cr7_cfg_t *c)
{
    uint32_t r = 0;
    r |= (c->sag_thr2 & 0x3FFU) << DSP_CR7_SAG_THR2_Pos;
    r |= (c->swv_thr2 & 0x3FFU) << DSP_CR7_SWV_THR2_Pos;
    r |= (c->chv2 & 0xFFFU) << DSP_CR7_CHV2_Pos;
    return r;
}

/*=================================DSP_CR8 ADDR :0X0E===================================*/
/*Bits  9:0   SWC_THR2[9:0]
Bits 21:10  CHC2[11:0]
Bits 31:22  Reserved
*/
#define DSP_CR8_ADDR 0x0E

#define DSP_CR8_SWC_THR2_Pos 0U
#define DSP_CR8_CHC2_Pos 10U

typedef struct
{
    uint16_t swc_thr2; /* 0..1023 */
    uint16_t chc2;     /* 0..4095 */
} dsp_cr8_cfg_t;

static inline uint32_t dsp_cr8_pack(const dsp_cr8_cfg_t *c)
{
    uint32_t r = 0;
    r |= (c->swc_thr2 & 0x3FFU) << DSP_CR8_SWC_THR2_Pos;
    r |= (c->chc2 & 0xFFFU) << DSP_CR8_CHC2_Pos;
    return r;
}

/*================================DFE_CR1 ADDR :0X18=====================================*/

/*Bits 31:30  Reserved
Bit  29     ENV1
Bits 28:26  Reserved
Bit  25     ENC1
Bits 24:2   Reserved
Bits 1:0    GAIN1[1:0]
*/

#define DFE_CR1_ADDR 0x18

#define DFE_CR1_GAIN1_Pos 0U
#define DFE_CR1_ENC1_Pos 25U
#define DFE_CR1_ENV1_Pos 29U

typedef struct
{
    uint8_t gain1; /* 0..3 */
    uint8_t enc1;  /* 0/1 */
    uint8_t env1;  /* 0/1 */
} dfe_cr1_cfg_t;

static inline uint32_t dfe_cr1_pack(const dfe_cr1_cfg_t *c)
{
    uint32_t r = 0;

    r |= (c->gain1 & 0x3U) << DFE_CR1_GAIN1_Pos;
    r |= (c->enc1 & 0x1U) << DFE_CR1_ENC1_Pos;
    r |= (c->env1 & 0x1U) << DFE_CR1_ENV1_Pos;

    return r;
}

/*=========================================DFE_CR2 ADDR : 0X1A=============================================*/

/*Bits 31:30  Reserved
Bit  29     ENV2
Bits 28:26  Reserved
Bit  25     ENC2
Bits 24:2   Reserved
Bits 1:0    GAIN2[1:0]
*/

#define DFE_CR2_ADDR 0x1A

#define DFE_CR2_GAIN2_Pos 0U
#define DFE_CR2_ENC2_Pos 25U
#define DFE_CR2_ENV2_Pos 29U

typedef struct
{
    uint8_t gain2; /* 0..3 */
    uint8_t enc2;  /* 0/1 */
    uint8_t env2;  /* 0/1 */
} dfe_cr2_cfg_t;

static inline uint32_t dfe_cr2_pack(const dfe_cr2_cfg_t *c)
{
    uint32_t r = 0;

    r |= (c->gain2 & 0x3U) << DFE_CR2_GAIN2_Pos;
    r |= (c->enc2 & 0x1U) << DFE_CR2_ENC2_Pos;
    r |= (c->env2 & 0x1U) << DFE_CR2_ENV2_Pos;

    return r;
}

/*==============================DSP_IRQ1 ADDR : 0X1C================================================*/

/*Bit 0   SAG1
Bit 1   SWV1
Bit 2   SWC1
Bit 3   ZCR1
Bit 4   TEMP1
Bit 5   LED1
Bit 6   ENERGY1
Bit 7   Reserved
Bits 31:8 Reserved
*/

#define DSP_IRQ1_ADDR 0x1C

#define DSP_IRQ1_SAG_Pos 0U
#define DSP_IRQ1_SWV_Pos 1U
#define DSP_IRQ1_SWC_Pos 2U
#define DSP_IRQ1_ZCR_Pos 3U
#define DSP_IRQ1_TEMP_Pos 4U
#define DSP_IRQ1_LED_Pos 5U
#define DSP_IRQ1_ENERGY_Pos 6U

typedef struct
{
    uint8_t sag;
    uint8_t swv;
    uint8_t swc;
    uint8_t zcr;
    uint8_t temp;
    uint8_t led;
    uint8_t energy;
} dsp_irq1_cfg_t;

static inline uint32_t dsp_irq1_pack(const dsp_irq1_cfg_t *c)
{
    uint32_t r = 0;

    r |= (c->sag & 0x1U) << DSP_IRQ1_SAG_Pos;
    r |= (c->swv & 0x1U) << DSP_IRQ1_SWV_Pos;
    r |= (c->swc & 0x1U) << DSP_IRQ1_SWC_Pos;
    r |= (c->zcr & 0x1U) << DSP_IRQ1_ZCR_Pos;
    r |= (c->temp & 0x1U) << DSP_IRQ1_TEMP_Pos;
    r |= (c->led & 0x1U) << DSP_IRQ1_LED_Pos;
    r |= (c->energy & 0x1U) << DSP_IRQ1_ENERGY_Pos;

    return r;
}

/*===============================================DSP_IRQ2 ADDR : 0X1E=======================================*/

#define DSP_IRQ2_ADDR 0x1E

#define DSP_IRQ2_SAG_Pos 0U
#define DSP_IRQ2_SWV_Pos 1U
#define DSP_IRQ2_SWC_Pos 2U
#define DSP_IRQ2_ZCR_Pos 3U
#define DSP_IRQ2_TEMP_Pos 4U
#define DSP_IRQ2_LED_Pos 5U
#define DSP_IRQ2_ENERGY_Pos 6U

typedef struct
{
    uint8_t sag;
    uint8_t swv;
    uint8_t swc;
    uint8_t zcr;
    uint8_t temp;
    uint8_t led;
    uint8_t energy;
} dsp_irq2_cfg_t;

static inline uint32_t dsp_irq2_pack(const dsp_irq2_cfg_t *c)
{
    uint32_t r = 0;

    r |= (c->sag & 0x1U) << DSP_IRQ2_SAG_Pos;
    r |= (c->swv & 0x1U) << DSP_IRQ2_SWV_Pos;
    r |= (c->swc & 0x1U) << DSP_IRQ2_SWC_Pos;
    r |= (c->zcr & 0x1U) << DSP_IRQ2_ZCR_Pos;
    r |= (c->temp & 0x1U) << DSP_IRQ2_TEMP_Pos;
    r |= (c->led & 0x1U) << DSP_IRQ2_LED_Pos;
    r |= (c->energy & 0x1U) << DSP_IRQ2_ENERGY_Pos;

    return r;
}

/*===============================DSP_SR1  ADDR:0X20 [RWL] ==========================================================*/

/*MSW [31:16]
V1 events:
  TAMPER / WRONG / TAMPER_END
  Sag Start / Sag End / Period
  Signal Stuck / Signal End

C1 events:
  Sag Start / Sag End / Period
  Signal Stuck / Signal End


  LSW[15:0]
  PH1:
  Energy Overflow  (S R F A)
  Power Sign       (S R F A)

PH2:
  Energy Overflow  (S R F A)
  Power Sign       (S R F A)

PH1+PH2:
  Energy Overflow  (R A)
  Power Sign       (R A)
S = Set, R = Reset, F = Flag, A = Acknowledge  LETTERS:LATCH FLAG
*/
#define DSP_SR1_PH1_EOVF_S_Pos 12U
#define DSP_SR1_PH1_EOVF_R_Pos 11U
#define DSP_SR1_PH1_EOVF_F_Pos 10U
#define DSP_SR1_PH1_EOVF_A_Pos 9U

#define DSP_SR1_PH1_PSIGN_S_Pos 8U
#define DSP_SR1_PH1_PSIGN_R_Pos 7U
#define DSP_SR1_PH1_PSIGN_F_Pos 6U
#define DSP_SR1_PH1_PSIGN_A_Pos 5U

#define DSP_SR1_ADDR 0x20U

#define DSP_STATUS_BUSY_Pos 1U
#define DSP_STATUS_VALID_Pos 0U

#define DSP_STATUS_BUSY_Msk BIT(DSP_STATUS_BUSY_Pos)

#define DSP_STATUS_VALID_Msk BIT(DSP_STATUS_VALID_Pos)

/*========================  DSP_SR2 Address 0x22==============================================================*/

#define DSP_SR2_ADDR 0x22U

#define DSP_SR2_PH1_EOVF_S_Pos 12U
#define DSP_SR2_PH1_EOVF_R_Pos 13U
#define DSP_SR2_PH1_EOVF_F_Pos 14U
#define DSP_SR2_PH1_EOVF_A_Pos 15U

#define DSP_SR2_PH1_EOVF_Msk (0xFUL << DSP_SR2_PH1_EOVF_S_Pos)

#define DSP_SR2_PH2_EOVF_S_Pos 8U
#define DSP_SR2_PH2_EOVF_R_Pos 9U
#define DSP_SR2_PH2_EOVF_F_Pos 10U
#define DSP_SR2_PH2_EOVF_A_Pos 11U

#define DSP_SR2_PH2_EOVF_Msk (0xFUL << DSP_SR2_PH2_EOVF_S_Pos)

#define DSP_SR2_PH12_EOVF_R_Pos 4U
#define DSP_SR2_PH12_EOVF_A_Pos 5U

#define DSP_SR2_PH12_EOVF_Msk (0x3UL << DSP_SR2_PH12_EOVF_R_Pos)

#define DSP_SR2_EOVF_ALL_Msk \
    (DSP_SR2_PH1_EOVF_Msk |  \
     DSP_SR2_PH2_EOVF_Msk |  \
     DSP_SR2_PH12_EOVF_Msk)

/*Power sign Flags*/
#define DSP_SR2_PH1_PSIGN_Pos 2U
#define DSP_SR2_PH2_PSIGN_Pos 1U
#define DSP_SR2_PH12_PSIGN_Pos 0U

#define DSP_SR2_PSIGN_Msk (0x7UL << DSP_SR2_PH12_PSIGN_Pos)

/*Voltage (V2) status flags (MSB)*/
#define DSP_SR2_V2_SAG_START_Pos 27U
#define DSP_SR2_V2_SAG_END_Pos 26U
#define DSP_SR2_V2_SAG_ERR_Pos 25U
#define DSP_SR2_V2_PERIOD_Pos 24U
#define DSP_SR2_V2_SIGNAL_Pos 23U
#define DSP_SR2_V2_STUCK_Pos 22U
#define DSP_SR2_V2_SW_END_Pos 21U
#define DSP_SR2_V2_SW_START_Pos 20U
#define DSP_SR2_V2_NAH_Pos 19U
#define DSP_SR2_V2_SIGNAL_STUCK_Pos 18U

#define DSP_SR2_V2_ALL_Msk (0x3FFUL << 18U)

/*Current (C2) status flags*/
#define DSP_SR2_C2_SAG_START_Pos 11U
#define DSP_SR2_C2_SAG_END_Pos 10U
#define DSP_SR2_C2_SAG_ERR_Pos 9U
#define DSP_SR2_C2_PERIOD_Pos 8U
#define DSP_SR2_C2_SIGNAL_Pos 7U
#define DSP_SR2_C2_STUCK_Pos 6U
#define DSP_SR2_C2_SW_END_Pos 5U
#define DSP_SR2_C2_SW_START_Pos 4U
#define DSP_SR2_C2_NAH_Pos 3U
#define DSP_SR2_C2_SIGNAL_STUCK_Pos 2U

/* -------- DSP output data -------- */
#define DSP_REG2_ADDR 0x30U /* V1 Data [23:0] */
#define DSP_REG3_ADDR 0x32U /* C1 Data [23:0] */
#define DSP_REG4_ADDR 0x34U /* V2 Data [23:0] */
#define DSP_REG5_ADDR 0x36U /* C2 Data [23:0] */

#define DSP_REG6_ADDR 0x38U /* V1 Fund [23:0] */
#define DSP_REG7_ADDR 0x3AU /* C1 Fund [23:0] */
#define DSP_REG8_ADDR 0x3CU /* V2 Fund [23:0] */
#define DSP_REG9_ADDR 0x3EU /* C2 Fund [23:0] */

#define DSP_REG14_ADDR 0x48U /* RMS */
#define DSP_REG15_ADDR 0x4AU
#define DSP_REG16_ADDR 0x4CU
#define DSP_REG17_ADDR 0x4EU
#define DSP_REG18_ADDR 0x50U
#define DSP_REG19_ADDR 0x52U

/*Vx / Cx instantaneous & fundamental (24-bit signed)*/
#define DSP_DATA24_Msk 0x00FFFFFFUL
#define DSP_DATA24_SIGN BIT(23)

/*==========================vrms irms(dsp)==================================*/
#define PH1_VRMS_ADDR 0x48U
#define PH1_IRMS_ADDR 0X49U

#define PH2_VRMS_ADDR 0x4AU
#define PH2_IRMS_ADDR 0X4BU

static inline int32_t stpm32_sign_extend24(uint32_t v)
{
    v &= DSP_DATA24_Msk;
    if (v & DSP_DATA24_SIGN)
    {
        v |= 0xFF000000UL;
    }
    return (int32_t)v;
}

/*RMS registers*/
/*DSP_REG14 (0x48)*/
/*[31:16] C1 RMS [16:0]
[15:0 ] V1 RMS [14:0]
*/
#define DSP_RMS_V1_Pos 0U
#define DSP_RMS_V1_Msk (0x7FFFUL << DSP_RMS_V1_Pos)

#define DSP_RMS_C1_Pos 16U
#define DSP_RMS_C1_Msk (0x1FFFFUL << DSP_RMS_C1_Pos)

/*DSP_REG15 (0x4A)*/
#define DSP_RMS_V2_Pos 0U
#define DSP_RMS_V2_Msk (0x7FFFUL << DSP_RMS_V2_Pos)

#define DSP_RMS_C2_Pos 16U
#define DSP_RMS_C2_Msk (0x1FFFFUL << DSP_RMS_C2_Pos)

/* ==============================Sag / Swell / Phase======================================*/

/* DSP_REG16: V1 Sag / Swell time */
#define DSP_REG16_ADDR 0x4CU

/* DSP_REG17: C1 Phase angle + C1 Swell time */
#define DSP_REG17_ADDR 0x4EU

/* DSP_REG18: V2 Sag / Swell time */
#define DSP_REG18_ADDR 0x50U

/* DSP_REG19: C2 Phase angle + C2 Swell time */
#define DSP_REG19_ADDR 0x52U

/*Sag / Swell time (15-bit)*/
#define DSP_TIME_Pos 0U
#define DSP_TIME_Msk 0x7FFFUL

/*Phase angle (12-bit signed)*/
#define DSP_PHASE_Pos 0U
#define DSP_PHASE_Msk 0x0FFFUL
#define DSP_PHASE_SIGN 0x0800UL

static inline int16_t stpm32_phase_decode(uint32_t v)
{
    v &= DSP_PHASE_Msk;
    if (v & DSP_PHASE_SIGN)
    {
        v |= 0xF000U;
    }
    return (int16_t)v;
}
/*==========================PH1 Power Registers=========================*/
#define PH1_REG1_ADDR 0x54
#define PH1_REG2_ADDR 0x56
#define PH1_REG3_ADDR 0x58
#define PH1_REG4_ADDR 0x5A
#define PH1_REG5_ADDR 0x5C
#define PH1_REG6_ADDR 0x5E
#define PH1_REG7_ADDR 0x60
#define PH1_REG8_ADDR 0x62
#define PH1_REG9_ADDR 0x64
#define PH1_REG10_ADDR 0x66
#define PH1_REG11_ADDR 0x68
#define PH1_REG12_ADDR 0x6A

/* -------- PH2 Power Registers (28-bit signed) -------- */

#define PH2_REG5_ADDR 0x74U  /* PH2 Active Power [28:0] */
#define PH2_REG6_ADDR 0x76U  /* PH2 Fundamental Power [28:0] */
#define PH2_REG7_ADDR 0x78U  /* PH2 Reactive Power [28:0] */
#define PH2_REG8_ADDR 0x7AU  /* PH2 Apparent RMS Power [28:0] */
#define PH2_REG9_ADDR 0x7CU  /* PH2 Apparent Vectorial Power [28:0] */
#define PH2_REG10_ADDR 0x7EU /* PH2 Momentary Active Power [28:0] */
#define PH2_REG11_ADDR 0x80U /* PH2 Momentary Fundamental Power [28:0] */

#define PH2_REG12_ADDR 0x82U /* PH2 Ah Accumulator */

#define TOT_REG1_ADDR 0x84U /* Total Active Energy */
#define TOT_REG2_ADDR 0x86U /* Total Fundamental Energy */
#define TOT_REG3_ADDR 0x88U /* Total Reactive Energy */
#define TOT_REG4_ADDR 0x8AU /* Total Apparent Energy */

#define DSP_PWR_BITS 29

static inline int32_t stpm32_sign_extend(uint32_t v, uint8_t bits)
{
    uint32_t mask = (1UL << bits) - 1;
    uint32_t sign = 1UL << (bits - 1);

    v &= mask;
    if (v & sign)
    {
        v |= ~mask;
    }
    return (int32_t)v;
}

static inline int32_t stpm32_power_decode(uint32_t raw)
{
    return stpm32_sign_extend(raw, DSP_PWR_BITS);
}

#endif