#include <stdio.h>
#include <string.h>

#include "senseair.h"
#include "senseair_params.h"
#include "fram.h"
#include "saml21_backup_mode.h"

#include "periph/gpio.h"
#include "periph/rtc.h"
#include "rtc_utils.h"

/* use { .pin=EXTWAKE_NONE } to disable */
#define EXTWAKE { \
    .pin=EXTWAKE_PIN6, \
    .polarity=EXTWAKE_HIGH, \
    .flags=EXTWAKE_IN_PU }
#define SLEEP_TIME 5 /* in seconds; -1 to disable */

static senseair_t dev;
static senseair_abc_data_t abc_data;

#define SENSEAIR_STATE_FRAM_ADDR    0

static saml21_extwake_t extwake = EXTWAKE;

void sensor_read(void)
{
    uint16_t conc_ppm;
    int16_t temp_cC;

    if (gpio_init(ACMEBUS_ENABLE, GPIO_OUT)) {
        puts("ACME Bus enable failed.");
        return;
    }
    gpio_set(ACMEBUS_ENABLE);

    if (senseair_init(&dev, &senseair_params[0]) != SENSEAIR_OK) {
        puts("Senseair init failed.");
        gpio_clear(ACMEBUS_ENABLE);
        return;
    }

    memset(&abc_data, 0, sizeof(abc_data));
    if (fram_read(SENSEAIR_STATE_FRAM_ADDR, &abc_data, sizeof(abc_data))) {
        puts("FRAM read failed.");
    } else {
        if (senseair_write_abc_data(&dev, &abc_data) == SENSEAIR_OK) {
            puts("ABC data restored to sensor.");
        } else {
            puts("ABC data not available.");
        }
    }

    if (senseair_read(&dev, &conc_ppm, &temp_cC) == SENSEAIR_OK) {
        printf("Concentration: %d ppm\n", conc_ppm);
        printf("Temperature: %4.2f °C\n", (temp_cC/100.));
    }

    if (senseair_read_abc_data(&dev, &abc_data) == SENSEAIR_OK) {
        puts("Saving sensor calibration data to FRAM.");
        if (fram_write(SENSEAIR_STATE_FRAM_ADDR, (uint8_t *)&abc_data, sizeof(abc_data))) {
            puts("FRAM write failed.");
        }
    }

    gpio_clear(ACMEBUS_ENABLE);
}

void boot_task(void)
{
    struct tm time;
    puts("Boot task.");
    rtc_localtime(0, &time);
    rtc_set_time(&time);
    fram_erase();
}

int main(void)
{
    fram_init();
    switch(saml21_wakeup_cause()) {
        case BACKUP_EXTWAKE:
            // wakeup_task
            break;
        case BACKUP_RTC:
            // periodic_task
            break;
        default:
            if (extwake.pin != EXTWAKE_NONE) {
                printf("GPIO PA%d will wake the board.\n", extwake.pin);
            }
            if (SLEEP_TIME > -1) {
                printf("Periodic task running every %d seconds.\n", SLEEP_TIME);
            }
            boot_task();
            break;
    }
    sensor_read();
    puts("Entering backup mode.");
    saml21_backup_mode_enter(RADIO_OFF_REQUESTED, extwake, SLEEP_TIME, 0);
    // never reached
    return 0;
}
