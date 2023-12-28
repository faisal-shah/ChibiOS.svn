#include <stdlib.h>
#include <string.h>
#include "hal.h"

#if (HAL_USE_ADC == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/**
 * @brief   ADCx driver identifier.
 */
#if (SIMULATOR_ADC_USE_ADC1 == TRUE) || defined(__DOXYGEN__)
ADCDriver ADCD1;
#endif

#if (SIMULATOR_ADC_USE_ADC2 == TRUE) || defined(__DOXYGEN__)
ADCDriver ADCD2;
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

static void adc_lld_handler(ADCDriver * adcp)
{
    osalDbgCheck(adcp->samples != NULL);
    osalDbgCheck(adcp->channel_data != NULL);

    if (adcp->grpp != NULL)
    {
        if (adcp->config->conv_cb != NULL)
        {
            adcp->config->conv_cb(adcp);
        }
        for (size_t i = 0; i < adcp->depth; i++)
        {
            memcpy(&adcp->samples[adcp->nch * i], adcp->channel_data, adcp->nch * sizeof(adcsample_t));
        }
        adcp->isr_rdy = false;
        _adc_isr_full_code(adcp);
    }
}

static void vt_trigger_interrupt_cb(virtual_timer_t *vtp, void * par)
{
    (void)vtp;
    ADCDriver * adcp = (ADCDriver *)par;
    syssts_t sts = osalSysGetStatusAndLockX();
    if (adcp->state == ADC_ACTIVE)
    {
        osalDbgCheck(!adcp->isr_rdy);
        adcp->isr_rdy = true;
        if (adcp->config->continuous)
        {
            sim_adc_lld_trigger_conversion(adcp);
        }
    }
    osalSysRestoreStatusX(sts);
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if SIMULATOR_ADC_USE_ADC1 || defined(__DOXYGEN__)
void adc_lld_int_handler0(void) {

  OSAL_IRQ_PROLOGUE();

  adc_lld_handler(&ADCD1);

  OSAL_IRQ_EPILOGUE();
}

bool adc_lld_interrupt_pending0(void)
{
    bool ret = false;

    osalSysLock();
    ret = ADCD1.isr_rdy;
    osalSysUnlock();

    return ret;
}
#endif

#if SIMULATOR_ADC_USE_ADC2 || defined(__DOXYGEN__)
void adc_lld_int_handler1(void) {

  OSAL_IRQ_PROLOGUE();

  adc_lld_handler(&ADCD2);

  OSAL_IRQ_EPILOGUE();
}

bool adc_lld_interrupt_pending1(void)
{
    bool ret = false;

    osalSysLock();
    ret = ADCD2.isr_rdy;
    osalSysUnlock();

    return ret;
}
#endif

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level ADC driver initialization.
 *
 * @notapi
 */
void adc_lld_init(void) {

#if SIMULATOR_ADC_USE_ADC1 == TRUE
  /* Driver initialization.*/
  adcObjectInit(&ADCD1);
  ADCD1.channel_data = NULL;
#endif

#if SIMULATOR_ADC_USE_ADC2 == TRUE
  /* Driver initialization.*/
  adcObjectInit(&ADCD2);
  ADCD1.channel_data = NULL;
#endif
}

/**
 * @brief   Configures and activates the ADC peripheral.
 *
 * @param[in] adcp      pointer to the @p ADCDriver object
 *
 * @notapi
 */
void adc_lld_start(ADCDriver *adcp) {

  if (adcp->state == ADC_STOP) {
    /* Enables the peripheral.*/
#if SIMULATOR_ADC_USE_ADC1 == TRUE
    if (&ADCD1 == adcp) {

    }
#endif
#if SIMULATOR_ADC_USE_ADC2 == TRUE
    if (&ADCD2 == adcp) {

    }
#endif
  }
  /* Configures the peripheral.*/

}

/**
 * @brief   Deactivates the ADC peripheral.
 *
 * @param[in] adcp      pointer to the @p ADCDriver object
 *
 * @notapi
 */
void adc_lld_stop(ADCDriver *adcp) {

  if (adcp->state == ADC_READY) {
    /* Resets the peripheral.*/

    /* Disables the peripheral.*/
#if SIMULATOR_ADC_USE_ADC1 == TRUE
    if (&ADCD1 == adcp) {
        ADCD1.channel_data = NULL;
    }
#endif
#if SIMULATOR_ADC_USE_ADC2 == TRUE
    if (&ADCD2 == adcp) {
        ADCD2.channel_data = NULL;
    }
#endif
  }
}

/**
 * @brief   Starts an ADC conversion.
 *
 * @param[in] adcp      pointer to the @p ADCDriver object
 *
 * @notapi
 */
void adc_lld_start_conversion(ADCDriver *adcp) {
    adcp->nch = adcp->grpp->num_channels;
    adcp->channel_data = malloc(adcp->grpp->num_channels * sizeof(adcsample_t));
    osalDbgCheck(adcp->channel_data != NULL);

    if (adcp->grpp->channels_default != NULL)
    {
        memcpy(adcp->channel_data, adcp->grpp->channels_default,
                sizeof(adcsample_t) * adcp->grpp->num_channels);
    }
    else
    {
        memset(adcp->channel_data, 0, sizeof(adcsample_t) * adcp->grpp->num_channels);
    }

    if (adcp->config->continuous)
    {
        sim_adc_lld_trigger_conversion(adcp);
    }
}

/**
 * @brief   Stops an ongoing conversion.
 *
 * @param[in] adcp      pointer to the @p ADCDriver object
 *
 * @notapi
 */
void adc_lld_stop_conversion(ADCDriver *adcp) {
    if (adcp->channel_data != NULL)
    {
        free(adcp->channel_data);
        adcp->channel_data = NULL;
    }
    syssts_t sts = osalSysGetStatusAndLockX();
    chVTResetI(&adcp->vt);
    adcp->isr_rdy = false;
    if (!port_is_isr_context())
    {
        chSchRescheduleS();
    }
    osalSysRestoreStatusX(sts);
}

int sim_adc_lld_trigger_conversion (ADCDriver *adcp)
{
  int ret = -1;
  syssts_t sts = osalSysGetStatusAndLockX();
  osalDbgCheck((adcp != NULL) && (adcp->grpp != NULL) && (adcp->samples != NULL) &&
               (adcp->depth > 0U));
  if (adcp->state == ADC_ACTIVE)
  {
      if (chVTIsArmedI(&adcp->vt))
      {
          ret = -1;
      }
      else
      {
          if (adcp->config->start_conv_cb != NULL)
          {
              adcp->config->start_conv_cb(adcp);
          }
          chVTSetI(&adcp->vt, adcp->config->dt, vt_trigger_interrupt_cb, adcp);
          ret = 0;
      }
  }
  if (!port_is_isr_context())
  {
      chSchRescheduleS();
  }
  osalSysRestoreStatusX(sts);
  return ret;
}

#endif /* HAL_USE_ADC == TRUE */
