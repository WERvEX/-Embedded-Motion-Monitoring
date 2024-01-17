#include "mbed.h"
#include <deque>

// The device should be put

// Define Regs & Configurations --> Gyroscope's settings
#define CTRL_REG1 0x20
#define CTRL_REG1_CONFIG 0b01'10'1'1'1'1
#define CTRL_REG4 0x23 // Second configure to set the DPS
#define CTRL_REG4_CONFIG 0b0'0'01'0'00'0

#define CTRL_REG3 0x22
#define CTRL_REG3_CONFIG 0b0'0'0'0'1'000

#define OUT_X_L 0x28

#define SPI_FLAG 1
#define DATA_READY_FLAG 2

#define SCALING_FACTOR (17.5f * 0.017453292519943295769236907684886f / 1000.0f)

#define FILTER_COEFFICIENT 0.075f // according to the observation, the noise normally is less than 0.05

#define LEG_LENGTH 0.9f // assume the people hight is 1.8 meter, Legs as a percentage of height is about 0.5. 1.8*0.5=0.9 m

// EventFlags object declaration
EventFlags flags;

// spi callback function
void spi_cb(int event)
{
    flags.set(SPI_FLAG);
}

// data ready callback function
void data_cb()
{
    flags.set(DATA_READY_FLAG);
}

int main()
{
    // spi initialization
    SPI spi(PF_9, PF_8, PF_7, PC_1, use_gpio_ssel);
    uint8_t write_buf[32], read_buf[32];

    // interrupt initialization
    InterruptIn int2(PA_2, PullDown);
    int2.rise(&data_cb);

    // spi format and frequency
    spi.format(8, 3);
    spi.frequency(1'000'000);

    // Write to control registers --> spi transfer
    write_buf[0] = CTRL_REG1;
    write_buf[1] = CTRL_REG1_CONFIG;
    spi.transfer(write_buf, 2, read_buf, 2, spi_cb);
    flags.wait_all(SPI_FLAG);

    write_buf[0] = CTRL_REG4;
    write_buf[1] = CTRL_REG4_CONFIG;
    spi.transfer(write_buf, 2, read_buf, 2, spi_cb);
    flags.wait_all(SPI_FLAG);

    write_buf[0] = CTRL_REG3;
    write_buf[1] = CTRL_REG3_CONFIG;
    spi.transfer(write_buf, 2, read_buf, 2, spi_cb);
    flags.wait_all(SPI_FLAG);

    write_buf[1] = 0xFF;

    //(polling for\setting) data ready flag
    if (!(flags.get() & DATA_READY_FLAG) && (int2.read() == 1))
    {
        flags.set(DATA_READY_FLAG);
    }

    float filtered_gx = 0.0f, filtered_gy = 0.0f, filtered_gz = 0.0f;

    deque<float> speedRecord;        // record the person velocity
    deque<float> angularSpeedRecord; // record the angular velocity
    float currentSpeed = 0;          // current person velocity
    float totalDistent = 0;          // total distance from beginning
    float totalAngularSpeed = 0;

    while (1)
    {
        int16_t raw_gx, raw_gy, raw_gz;
        float gx, gy, gz;

        flags.wait_all(DATA_READY_FLAG);
        write_buf[0] = OUT_X_L | 0x80 | 0x40;

        spi.transfer(write_buf, 7, read_buf, 7, spi_cb);
        flags.wait_all(SPI_FLAG);

        // Process raw data
        raw_gx = (((uint16_t)read_buf[2]) << 8) | ((uint16_t)read_buf[1]);
        raw_gy = (((uint16_t)read_buf[4]) << 8) | ((uint16_t)read_buf[3]);
        raw_gz = (((uint16_t)read_buf[6]) << 8) | ((uint16_t)read_buf[5]);

        gx = ((float)raw_gx) * SCALING_FACTOR;
        gy = ((float)raw_gy) * SCALING_FACTOR;
        gz = ((float)raw_gz) * SCALING_FACTOR;

        // apply filter (only larger than the FILTER_COEFFICIENT will take account, filter out the noise when not active)
        filtered_gx = gx > FILTER_COEFFICIENT ? gx : 0;
        filtered_gy = gy > FILTER_COEFFICIENT ? gy : 0;
        filtered_gz = gz > FILTER_COEFFICIENT ? gz : 0;

        // calculate the vector sum (sqrt(x^2+y^2+z^2))
        totalAngularSpeed = sqrt(filtered_gx * filtered_gx + filtered_gy * filtered_gy + filtered_gz * filtered_gz);

        angularSpeedRecord.push_back(totalAngularSpeed); // add current angular velocity to the record set

        if (angularSpeedRecord.size() >= 40) // set size = 40 (0.5s * 40 = 20s)
        {
            angularSpeedRecord.pop_front(); // if time larger than 20s, pop the most the earliest elements
        }
        printf(">totalAngularSpeed:%4.5f|g\n", totalAngularSpeed);

        currentSpeed = totalAngularSpeed * LEG_LENGTH; // Linear velocity = angular velocity * radius
        speedRecord.push_back(currentSpeed);           // add current people velocity to the record set
        if (speedRecord.size() >= 40)
        {
            speedRecord.pop_front(); // if time larger than 20s, pop the most the earliest elements
        }
        printf(">currentSpeed:%4.5f|g\n", currentSpeed);
        // calculate the distance travel within the recent 20s
        float distentIn20s = 0;
        for (float i : speedRecord)
        {
            distentIn20s += i * 0.5f; // distance with in 20s
        }
        printf(">distentIn20s:%4.5f|g\n", distentIn20s);
        totalDistent += currentSpeed * 0.5f; // record the total distance
        printf(">totalDistent:%4.5f|g\n", totalDistent);

        thread_sleep_for(500); // subsample 0.5s
    }
}
