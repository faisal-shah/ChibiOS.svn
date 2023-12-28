#pragma once

#if HAL_USE_ADC || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @brief   ADC1 driver enable switch.
 * @details If set to @p TRUE the support for ADC1 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(SIMULATOR_ADC_USE_ADC1) || defined(__DOXYGEN__)
#define SIMULATOR_ADC_USE_ADC1                  FALSE
#endif

#if !defined(SIMULATOR_ADC_USE_ADC2) || defined(__DOXYGEN__)
#define SIMULATOR_ADC_USE_ADC2                  FALSE
#endif

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   ADC sample data type.
 */
typedef uint16_t adcsample_t;

/**
 * @brief   Channels number in a conversion group.
 */
typedef uint16_t adc_channels_num_t;

/**
 * @brief   Possible ADC failure causes.
 * @note    Error codes are architecture dependent and should not relied
 *          upon.
 */
typedef enum {
  ADC_ERR_GENERAL = 0,
} adcerror_t;

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/**
 * @brief   Low level fields of the ADC driver structure.
 */
#define adc_lld_driver_fields                                               \
  /* Conversion Ready interrupt */                                          \
  bool isr_rdy;                                                             \
  /* Channel data storage */                                                \
  adcsample_t * channel_data;                                               \
  /* Number of channels */                                                  \
  size_t nch;                                                               \
  /* virtual timer for continuous mode */                                   \
  virtual_timer_t vt


/**
 * @brief   Low level fields of the ADC configuration structure.
 */
#define adc_lld_config_fields                                               \
  /* Continuous mode (internally triggered) */                              \
  bool continuous;                                                          \
  /* conversion time for continuous mode */                                 \
  sysinterval_t dt;                                                         \
  /* start of conversion callback */                                        \
  adccallback_t start_conv_cb;                                              \
  /* end of conversion callback, before data is copied */                   \
  /* from channel_data to user buffer */                                    \
  adccallback_t conv_cb


/**
 * @brief   Low level fields of the ADC configuration structure.
 */
#define adc_lld_configuration_group_fields                                  \
  /* Default channel data */                                                \
  adcsample_t * channels_default


/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if SIMULATOR_ADC_USE_ADC1 && !defined(__DOXYGEN__)
extern ADCDriver ADCD1;
#endif

#if SIMULATOR_ADC_USE_ADC2 && !defined(__DOXYGEN__)
extern ADCDriver ADCD2;
#endif

#ifdef __cplusplus
extern "C" {
#endif
  void adc_lld_init(void);
  void adc_lld_start(ADCDriver *adcp);
  void adc_lld_stop(ADCDriver *adcp);
  void adc_lld_start_conversion(ADCDriver *adcp);
  void adc_lld_stop_conversion(ADCDriver *adcp);
  void adc_lld_int_handler0(void);
  void adc_lld_int_handler1(void);
  bool adc_lld_interrupt_pending0(void);
  bool adc_lld_interrupt_pending1(void);
  int sim_adc_lld_trigger_conversion (ADCDriver *adcp);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_ADC */
