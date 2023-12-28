#include <string.h>
#include <stddef.h>
#include "ch.h"
#include "hal.h"

#define ADC_NCH (10U)

static adcsample_t sAdcDefChData[ADC_NCH];
static adcsample_t sResults[2*ADC_NCH];

static void gpt_cb(GPTDriver * gptp)
{
  (void)gptp;
  int ret;
  ret = sim_adc_lld_trigger_conversion(&ADCD1);
  printf("triggered = %d\n", ret);
}

static void adc_sc_cb(ADCDriver * adcp)
{
  for (size_t i = 0; i < adcp->grpp->num_channels; i++)
  {
    sAdcDefChData[i]++;
  }
}

static void adc_conv_cb(ADCDriver * adcp)
{
  memcpy(adcp->channel_data, sAdcDefChData, sizeof(sAdcDefChData));
}

static void adc_end_cb(ADCDriver * adcp)
{
  printf("Samples: ");
  for (size_t d = 0; d < adcp->depth; d++)
  {
      for (size_t i = 0; i < adcp->grpp->num_channels; i++)
      {
        printf("%d ", adcp->samples[(d*adcp->grpp->num_channels)+i]);
      }
  }
  printf("\n");
}

static GPTConfig sGptConfig = {
    .frequency = 1000000U,
    .callback = NULL,
    .secondary_callback = gpt_cb,
};

static ADCConfig sAdcConfig = {
    .continuous = false, .dt = OSAL_MS2I(10),
    .start_conv_cb = adc_sc_cb,
    .conv_cb = adc_conv_cb
};

static ADCConversionGroup sAdcCg = {
    .circular = true,
    .num_channels = ADC_NCH,
    .end_cb = adc_end_cb,
    .error_cb = NULL,
    .channels_default = sAdcDefChData,
};

int main(int argc, char * argv[])
{
  (void)argc;
  (void)argv;

  halInit();
  chSysInit();

  memset(sResults, 0, sizeof(sResults));
  for (size_t i = 0; i < ADC_NCH; i++)
  {
    sAdcDefChData[i] = i;
  }

  adcStart(&ADCD1, &sAdcConfig);
  adcStartConversion(&ADCD1, &sAdcCg, sResults, (sizeof(sResults) / sizeof(sResults[0]))/ADC_NCH);

  gptStart(&GPTD1, &sGptConfig);
  gptStartContinuous(&GPTD1, 100000U);

  while (1)
  {
    chThdSleepMilliseconds(1000);
  }
}
