#include <stdio.h>
#include <string.h>

#include "fram.h"
#include "saml21_cpu_debug.h"
#include "saml21_backup_mode.h"

#include "periph_conf.h"
#include "periph/gpio.h"
#include "periph/i2c.h"
#include "periph/rtc.h"
#include "rtc_utils.h"
#include "ztimer.h"
#include "od.h"

/* use { .pin=EXTWAKE_NONE } to disable */
#define EXTWAKE { \
    .pin=EXTWAKE_PIN6, \
    .polarity=EXTWAKE_HIGH, \
    .flags=EXTWAKE_IN_PU }
#define SLEEP_TIME 5 /* in seconds; -1 to disable */

#define ACME1_ENABLE_PIN     GPIO_PIN(PA, 28)

#define SENSEAIR_I2C_DEV     I2C_DEV(1)
#define SENSEAIR_I2C_ADDR    0x68
#define SENSEAIR_ENABLE_PIN  GPIO_PIN(PB, 23)

#define SENSEAIR_ERROR_STATUS_REG     0x00
#define SENSEAIR_CONCENTRATION_REG    0x06
#define SENSEAIR_TEMPERATURE_REG      0x08
#define SENSEAIR_MEASURE_COUNT_REG    0x0d
#define SENSEAIR_START_MEASURE_REG    0x93
#define SENSEAIR_MEASURE_MODE_REG     0x95
#define SENSEAIR_SAVED_STATE_REG      0xc0

#define SENSEAIR_STATE_FRAM_ADDR    0

#define BSWAP(data)     (((data & 0xff) << 8) + ((data >> 8) & 0xff))

static saml21_extwake_t extwake = EXTWAKE;

const char *senseair_signature="SUNR";
static struct {
    char signature[5];
    uint32_t last_update;
    uint8_t  data[32];
} senseair_data;

void _print_senseair_data(void)
{
    printf("Signature: %s; last update: 0x%08lx\n", senseair_data.signature, senseair_data.last_update);
    od_hex_dump(&senseair_data.data, sizeof(senseair_data.data), 0);
}

void _print_time(struct tm *time)
{
    printf("%04i-%02i-%02i %02i:%02i:%02i\n",
        time->tm_year + 1900, time->tm_mon + 1, time->tm_mday,
        time->tm_hour, time->tm_min, time->tm_sec
    );
}

void sensor_read(void)
{
    int res;
    uint8_t reg;
    uint16_t data = 0;
    struct tm time;
    uint32_t tstamp;

    puts("Activating sensor.");
    gpio_init(SENSEAIR_ENABLE_PIN, GPIO_OUT);
    gpio_set(SENSEAIR_ENABLE_PIN);
    ztimer_sleep(ZTIMER_MSEC, 20);
    res = i2c_read_regs(SENSEAIR_I2C_DEV, SENSEAIR_I2C_ADDR, SENSEAIR_ERROR_STATUS_REG, &data, 2, 0);
    if (res) {
        printf("ERROR %d reading I2C.\n", res);
        goto out;
    }
    printf("Status: 0x%04x\n", data);
    if (data == 0x8000) {
        puts("Sensor: active.");
    } else {
        puts("Sensor: not found.");
        goto out;
    }
    i2c_read_reg(SENSEAIR_I2C_DEV, SENSEAIR_I2C_ADDR, SENSEAIR_MEASURE_MODE_REG, &reg, 0);
    if (reg == 1) {
        puts("Single measurement mode: active.");
    } else {
        puts("Single measurement mode: set.");
        i2c_write_reg(SENSEAIR_I2C_DEV, SENSEAIR_I2C_ADDR, SENSEAIR_MEASURE_MODE_REG, 1, 0);
    }

    memset(&senseair_data, 0, sizeof(senseair_data));
    if (fram_read(SENSEAIR_STATE_FRAM_ADDR, &senseair_data, sizeof(senseair_data))) {
        puts("FRAM read failed.");
    } else {
        if (strncmp(senseair_data.signature, senseair_signature, sizeof(senseair_data.signature))==0) {
            puts("Found sensor calibration data in FRAM, restoring.");
            _print_senseair_data();
            // bytes 0x04, 0x05 in senseair_data.data represents ABC time (in hours)
            // it must be incremented every hour
            rtc_get_time(&time);
            _print_time(&time);
            tstamp=rtc_mktime(&time);
            printf("tstamp=0x%08lx\n", tstamp);
            if (tstamp - senseair_data.last_update > 3600) {
                printf("ABC time update: 0x%02x%02x -> ", senseair_data.data[4], senseair_data.data[5]);
                data = (senseair_data.data[4] << 8) + senseair_data.data[5] + (tstamp - senseair_data.last_update) / 3600;
                senseair_data.data[4] = (data >> 8) & 0xff;
                senseair_data.data[5] = data & 0xff;
                printf("0x%02x%02x\n", senseair_data.data[4], senseair_data.data[5]);
            }
            if (i2c_write_regs(SENSEAIR_I2C_DEV, SENSEAIR_I2C_ADDR, SENSEAIR_SAVED_STATE_REG, &senseair_data.data, sizeof(senseair_data.data), 0)) {
                puts("ERROR: restore failed");
            }
        }
    }

    puts("Starting measure.");
    i2c_write_reg(SENSEAIR_I2C_DEV, SENSEAIR_I2C_ADDR, SENSEAIR_START_MEASURE_REG, 1, 0);
    reg = 0;
    while (reg == 0) {
        ztimer_sleep(ZTIMER_MSEC, 50);
        i2c_read_reg(SENSEAIR_I2C_DEV, SENSEAIR_I2C_ADDR, SENSEAIR_MEASURE_COUNT_REG, &reg, 0);
    }
    i2c_read_regs(SENSEAIR_I2C_DEV, SENSEAIR_I2C_ADDR, SENSEAIR_CONCENTRATION_REG, &data, 2, 0);
    printf("Concentration: %d ppm\n", BSWAP(data));
    i2c_read_regs(SENSEAIR_I2C_DEV, SENSEAIR_I2C_ADDR, SENSEAIR_TEMPERATURE_REG, &data, 2, 0);
    printf("Temperature: %4.2f °C\n", (BSWAP(data)/100.));

    puts("Saving sensor calibration data to FRAM.");
    memset(&senseair_data, 0, sizeof(senseair_data));
    strncpy(senseair_data.signature, senseair_signature, sizeof(senseair_data.signature));
    rtc_get_time(&time);
    _print_time(&time);
    senseair_data.last_update=rtc_mktime(&time);
    i2c_read_regs(SENSEAIR_I2C_DEV, SENSEAIR_I2C_ADDR, SENSEAIR_SAVED_STATE_REG, &senseair_data.data, sizeof(senseair_data.data), 0);
    _print_senseair_data();
    if (fram_write(SENSEAIR_STATE_FRAM_ADDR, (uint8_t *)&senseair_data, sizeof(senseair_data))) {
        puts("FRAM write failed.");
    }
out:
    gpio_clear(SENSEAIR_ENABLE_PIN);
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
