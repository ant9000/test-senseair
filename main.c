#include <stdio.h>
#include "saml21_cpu_debug.h"
#include "saml21_backup_mode.h"

#include "periph_conf.h"
#include "periph/gpio.h"
#include "periph/i2c.h"
#include "ztimer.h"

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

#define BSWAP(data)     (((data & 0xff) << 8) + ((data >> 8) & 0xff))

static saml21_extwake_t extwake = EXTWAKE;

void sensor_read(void)
{
    int res;
    uint8_t reg;
    uint16_t data = 0;
    puts("Activating sensor.");
    gpio_init(ACME1_ENABLE_PIN, GPIO_OUT);
    gpio_init(SENSEAIR_ENABLE_PIN, GPIO_OUT);
    gpio_set(ACME1_ENABLE_PIN);
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
/*
    if state in FRAM:
    - read 32 bytes of persisted state from FRAM
    - bytes 0x04, 0x05 are ABC time (in hours);
      value should be incremented each hour
    - write state to 32 registers starting at 0xc0
*/
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
/*
    - read 32 bytes of state from registers starting at 0xc0
    - persist state to FRAM

*/
out:
    gpio_clear(SENSEAIR_ENABLE_PIN);
    gpio_clear(ACME1_ENABLE_PIN);
}

void poweroff_devices(void)
{
    size_t i;

    // turn I2C devices off (leave internal bus I2C_DEV(0) alone)
    for(i = 1; i < I2C_NUMOF; i++) {
        i2c_release(I2C_DEV(i));
        i2c_deinit_pins(I2C_DEV(i));
        gpio_init(i2c_config[i].scl_pin, GPIO_IN_PU);
        gpio_init(i2c_config[i].sda_pin, GPIO_IN_PU);
    }
}

int main(void)
{
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
            // boot_task
            break;
    }

    sensor_read();
    puts("Entering backup mode.");
    poweroff_devices();
    saml21_backup_mode_enter(RADIO_OFF_REQUESTED, extwake, SLEEP_TIME, 1);
    // never reached
    return 0;
}
