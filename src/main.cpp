/**
 * @file main.cpp
 * @author Vivatsathorn Thitasirivit
 * @date 13 May 2024
 * @brief LUNA Firmware
 */

#include <Arduino.h>
#include "Arduino_Extended.h"
#include "File_Utility.h"
#include "vt_linalg"
#include "vt_kalman"
#include <STM32LowPower.h>

#include <Wire.h>
#include <SPI.h>

#include <SdFat.h>
#include <SparkFun_u-blox_GNSS_v3.h>
#include <ICM_20948.h>
#include <ICM42688.h>
#include <MS5611_SPI.h>
#include <INA236.h>

#include "luna_pin_def.h"
#include "luna_state_def.h"
#include "luna_peripheral_def.h"
#include "message.h"

// Device specific
#define THIS_DEVICE_ID      (0)
#define THIS_FILE_PREFIX    "LUNA_LOGGER_0_"
#define THIS_FILE_EXTENSION "CSV"

// #define HW_FORMAT_CARD

// Type aliases

using time_type    = uint32_t;
using smart_delay  = vt::smart_delay<time_type>;
using on_off_timer = vt::on_off_timer<time_type>;
using task_type    = vt::task_t<vt::smart_delay, time_type>;

template<size_t N>
using dispatcher_type         = vt::task_dispatcher<N, vt::smart_delay, time_type>;

using sd_sd_t                 = SdExFat;
using sd_file_t               = ExFile;
using flash_sd_t              = SdFat32;
using flash_file_t            = File32;

constexpr size_t FILTER_ORDER = 4;
constexpr double dt_base      = 0.1;
constexpr double covariance   = 0.01;
constexpr double alpha        = 0.;
constexpr double beta         = 0.;
constexpr double G            = 9.81;

// Type defs

struct peripherals_t {
    union {
        struct {
            uint8_t ina236;
            uint8_t gnss_m9v;
            uint8_t imu_icm42688;
            uint8_t imu_icm20948;

            uint8_t ms1;
            uint8_t ms2;
            uint8_t flash;
            uint8_t sd;
        };

        uint8_t arr[8];
    };

    template<traits::has_ostream OStream>
    OStream &operator>>(OStream &ostream) {
        ostream << "INA236: " << ina236 << stream::crlf;
        ostream << "GNSS M9V: " << gnss_m9v << stream::crlf;
        ostream << "IMU ICM42688: " << imu_icm42688 << stream::crlf;
        ostream << "IMU ICM20948: " << imu_icm20948 << stream::crlf;
        ostream << "BARO MS1: " << ms1 << stream::crlf;
        ostream << "BARO MS2: " << ms2 << stream::crlf;
        ostream << "STOR FLASH: " << flash << stream::crlf;
        ostream << "STOR SD: " << sd << stream::crlf;
        return ostream;
    }
};

struct ms_ref_t {
    MS5611_SPI &ms;
    float &temp;
    float &pres;
    float &alt;
    vt::kf_pos<FILTER_ORDER> &kf;

    ms_ref_t(MS5611_SPI &t_ms, float &t_temp, float &t_pres, float &t_alt, vt::kf_pos<FILTER_ORDER> &t_kf)
        : ms{t_ms}, temp{t_temp}, pres{t_pres}, alt{t_alt}, kf{t_kf} {}
};

// Hardware interfaces

HardwareSerial Serial2(stella::pins::comm::lora::UART_RX, stella::pins::comm::lora::UART_TX);
HardwareSerial Serial4(stella::pins::comm::rpi::UART_RX, stella::pins::comm::rpi::UART_TX);

TwoWire Wire1(to_digital(stella::pins::i2c::ch1::SDA),
              to_digital(stella::pins::i2c::ch1::SCL));

SPIClass SPI_1(to_digital(stella::pins::spi::ch1::MOSI),
               to_digital(stella::pins::spi::ch1::MISO),
               to_digital(stella::pins::spi::ch1::SCK));
SPIClass SPI_3(to_digital(stella::pins::spi::ch3::MOSI),
               to_digital(stella::pins::spi::ch3::MISO),
               to_digital(stella::pins::spi::ch3::SCK));
SPIClass SPI_4(to_digital(stella::pins::spi::ch4::MOSI),
               to_digital(stella::pins::spi::ch4::MISO),
               to_digital(stella::pins::spi::ch4::SCK));

SdSpiConfig flash_config(stella::pins::spi::cs::flash,
                         SHARED_SPI,
                         SD_SCK_MHZ(stella::config::SD_SPI_CLOCK_MHZ),
                         &SPI_3);
SdSpiConfig sd_config(stella::pins::spi::cs::sd,
                      SHARED_SPI,
                      SD_SCK_MHZ(stella::config::SD_SPI_CLOCK_MHZ),
                      &SPI_3);

on_off_timer::interval_params buzzer_intervals(stella::config::BUZZER_ON_INTERVAL,
                                               stella::config::BUZZER_OFF_INTERVAL(stella::config::BUZZER_IDLE_INTERVAL));

// Hardware references

HardwareSerial &USB_DEBUG = Serial4;
HardwareSerial &UART_LORA = Serial2;
HardwareSerial &UART_RPI  = Serial4;

// Software data

stella::sensor_data_t sensor_data;
uint8_t rx_buffer[stella::config::MESSAGE_BUFFER_SIZE];

// Software filters
struct software_filters {
    vt::kf_pos<FILTER_ORDER> ms1_pres{dt_base, covariance, alpha, beta};
    vt::kf_pos<FILTER_ORDER> ms2_pres{dt_base, covariance, alpha, beta};

    struct {
        stella::vec3_u<vt::kf_pos<4>> acc{dt_base, covariance, alpha, beta};
        stella::vec3_u<vt::kf_pos<4>> gyro{dt_base, covariance, alpha, beta};
    } imu_1, imu_2;

    vt::kf_pos<FILTER_ORDER> altitude{dt_base, covariance, alpha, beta};
    vt::kf_acc<FILTER_ORDER> acceleration{dt_base, covariance, alpha, beta};
} filters;

// Communication data

String tx_data;
String sd_data;

time_type tx_interval  = stella::config::TX_IDLE_INTERVAL;
time_type log_interval = stella::config::LOG_IDLE_INTERVAL;

// Peripherals

peripherals_t pvalid;

FsUtil<flash_sd_t, flash_file_t> flash_util;
FsUtil<sd_sd_t, sd_file_t> sd_util;

INA236 ina236(stella::config::INA236_ADDRESS, &Wire1);
SFE_UBLOX_GNSS gnss_m9v;
ICM_20948_SPI imu_icm20948;
ICM42688 imu_icm42688(SPI_1, to_digital(stella::pins::spi::cs::icm42688));
MS5611_SPI ms1(to_digital(stella::pins::spi::cs::ms1), &SPI_1);
MS5611_SPI ms2(to_digital(stella::pins::spi::cs::ms2), &SPI_4);

ms_ref_t ms1_ref = {ms1, sensor_data.ms1_temp, sensor_data.ms1_pres, sensor_data.ms1_alt, filters.ms1_pres};
ms_ref_t ms2_ref = {ms2, sensor_data.ms2_temp, sensor_data.ms2_pres, sensor_data.ms2_alt, filters.ms2_pres};

// Software control
dispatcher_type<32> dispatcher;
bool launch_override = false;

struct {
    uint8_t pressure     : 2 = 0b11;  // Use both sensors (averaging)
    uint8_t acceleration : 2 = 0b11;  // Use both sensors (averaging)
} sensor_mode;

inline const char *sensor_mode_string(const uint8_t mode) {
    switch (mode) {
        case 0b01:
            return "1";
        case 0b10:
            return "2";
        case 0b11:
            return "F";
        default:
            __builtin_unreachable();
    }
}

struct {
    float altitude;
    float altitude_offset;
    stella::vec3_u<double> acc;
} ground_truth;

HardwareTimer timer_led(TIM2);
HardwareTimer timer_buz(TIM3);
stella::pins::PwmLed pwm_led(timer_led, timer_buz);

template<typename SdType, typename FileType>
extern void init_storage(FsUtil<SdType, FileType> &sd_util_instance);

extern void read_continuity();

extern void read_internal();

extern void read_gnss();

extern void read_ms(ms_ref_t *ms);

extern void read_icm20948();

extern void read_icm42688();

extern void calculate_ground_truth();

extern void synchronize_kf();

extern void accept_command(HardwareSerial *istream);

extern void construct_data();

extern void transmit_data(time_type *interval_ms);

extern void save_data(time_type *interval_ms);

extern void fsm_eval();

extern void buzzer_led_control(on_off_timer::interval_params *intervals_ms);

void setup() {
    // GPIO and Digital Pins
    dout_low << stella::pins::gpio::LED_R
             << stella::pins::gpio::LED_G
             << stella::pins::gpio::LED_B
             << stella::pins::pyro::SIG_A
             << stella::pins::pyro::SIG_B
             << stella::pins::pyro::SIG_C
             << stella::pins::power::PIN_24V;

    pwm_led.set_range(16, 255);

    pwm_led.set_color(stella::YELLOW);
    pwm_led.set_frequency(2);
    pwm_led.reset();

    timer_buz.setOverflow(stella::config::BUZZER_ON_INTERVAL * 1000, MICROSEC_FORMAT);
    timer_buz.attachInterrupt([] {
        gpio_write << io_function::pull_low(stella::pins::gpio::BUZZER);
        timer_buz.pause();
    });
    timer_buz.pause();

    // GPIO Configuration

    dout_low << stella::pins::gpio::BUZZER;

    din_config << stella::pins::pyro::SENS_A
               << stella::pins::pyro::SENS_B
               << stella::pins::pyro::SENS_C
               << stella::pins::gpio::USER_1
               << stella::pins::gpio::USER_2;

    // UART Interfaces

    UART_LORA.begin(stella::config::UART_BAUD);
    UART_RPI.begin(stella::config::RFD900X_BAUD);

    // SPI Mode
    SPI_3.setDataMode(SPI_MODE0);

    SPI_1.begin();
    SPI_3.begin();
    SPI_4.begin();

    // I2C (Wire) Interfaces
    Wire1.setClock(400000);
    Wire1.begin();


    // I2C Debug
    i2c_detect(USB_DEBUG, Wire1, 0, 127);

    // SPI Interfaces
    // SPI 1, 2, 3: 64 MHz; SPI 4,5: 120 MHz

    // Storage Initialization
    pvalid.flash = flash_util.sd().begin(flash_config);
    if (pvalid.flash) { init_storage(flash_util); }


    pvalid.sd = sd_util.sd().begin(sd_config);
    if (pvalid.sd) { init_storage(sd_util); }


    // Peripherals Initialization

    // Battery voltage monitor
    pvalid.ina236 = ina236.begin();
    if (pvalid.ina236) {
        ina236.setADCRange(stella::config::INA236ADCRange::RANGE_80MV);
    }


    // Barometer and temperature
    pvalid.ms1 = ms1.begin();
    pvalid.ms2 = ms2.begin();

    float gnd  = 0.f;

    if (pvalid.ms1) {
        ms1.reset();
        ms1.setOversampling(OSR_STANDARD);
        for (size_t i = 0; i < 20; ++i) {
            read_ms(&ms1_ref);
        }
        gnd += sensor_data.ms1_alt;
    }

    if (pvalid.ms2) {
        ms2.reset();
        ms2.setOversampling(OSR_STANDARD);
        for (size_t i = 0; i < 20; ++i) {
            read_ms(&ms2_ref);
        }
        gnd += sensor_data.ms2_alt;
    }

    ground_truth.altitude_offset = gnd * 0.5;

    // IMU
    pvalid.imu_icm42688 = imu_icm42688.begin() == 1;
    if (pvalid.imu_icm42688) {
    }

    pvalid.imu_icm20948 = imu_icm20948.begin(to_digital(stella::pins::spi::cs::icm20948), SPI_4) == ICM_20948_Stat_Ok;
    if (pvalid.imu_icm20948) {
        imu_icm20948.swReset();
        delay(250);
        imu_icm20948.sleep(false);
        imu_icm20948.lowPower(false);
        imu_icm20948.setSampleMode(ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr, ICM_20948_Sample_Mode_Continuous);
        imu_icm20948.setFullScale(ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr, {.a = gpm16, .g = dps2000});
        imu_icm20948.setDLPFcfg(ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr, {.a = acc_d246bw_n265bw, .g = gyr_d196bw6_n229bw8});
        imu_icm20948.enableDLPF(ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr, true);
    }

    // GPS
    pvalid.gnss_m9v = gnss_m9v.begin(Wire1);
    if (pvalid.gnss_m9v) {
        // Basic configuration
        gnss_m9v.setI2COutput(COM_TYPE_UBX, VAL_LAYER_RAM_BBR, stella::config::UBLOX_CUSTOM_MAX_WAIT);
        gnss_m9v.setNavigationFrequency(25, VAL_LAYER_RAM_BBR, stella::config::UBLOX_CUSTOM_MAX_WAIT);
        gnss_m9v.setAutoPVT(true, VAL_LAYER_RAM_BBR, stella::config::UBLOX_CUSTOM_MAX_WAIT);
        gnss_m9v.setDynamicModel(DYN_MODEL_AIRBORNE4g, VAL_LAYER_RAM_BBR, stella::config::UBLOX_CUSTOM_MAX_WAIT);
    }

    // Task initialization
    dispatcher << task_type(read_continuity, 500ul, micros, 0)
               << task_type(buzzer_led_control, &buzzer_intervals, 0)  // Adaptive

               << task_type(calculate_ground_truth, 0)
               << task_type(fsm_eval, 25ul, millis, 0)

               << (task_type(read_icm42688, 25ul, millis, 1), pvalid.imu_icm42688)
               << (task_type(read_icm20948, 25ul, millis, 1), pvalid.imu_icm20948)
               << (task_type(read_ms, &ms1_ref, 25ul, millis, 1), pvalid.ms1)
               << (task_type(read_ms, &ms2_ref, 25ul, millis, 1), pvalid.ms2)
               << task_type(synchronize_kf, 25ul, millis, 1)

               << (task_type(read_gnss, 100ul, millis, 2), pvalid.gnss_m9v)

               << task_type(accept_command, &UART_RPI, 100ul, millis, 252)

               << task_type(construct_data, 25ul, millis, 253)
               << (task_type(save_data, &log_interval, 254), pvalid.sd)  // Adaptive
               << task_type(transmit_data, &tx_interval, 254)            // Adaptive

               << task_type(read_internal, 500ul, millis, 255);

    // Low power mode
    // See details: https://github.com/stm32duino/STM32LowPower/blob/main/README.md
    LowPower.begin();
    LowPower.enableWakeupFrom(&UART_RPI, [] {
        accept_command(&UART_RPI);
    });

    // Peripheral validation
    {
        constexpr auto blink_green = [] {
            stella::pins::SET_LED(stella::BLUE);
            gpio_write << io_function::pull_high(stella::pins::gpio::BUZZER);
            delay(stella::config::BUZZER_ON_INTERVAL);
            stella::pins::SET_LED(stella::BLACK);
            gpio_write << io_function::pull_low(stella::pins::gpio::BUZZER);
            delay(250);
        };

        constexpr auto blink_red = [] {
            stella::pins::SET_LED(stella::RED);
            gpio_write << io_function::pull_high(stella::pins::gpio::BUZZER);
            delay(stella::config::BUZZER_ON_INTERVAL * 5);
            stella::pins::SET_LED(stella::BLACK);
            gpio_write << io_function::pull_low(stella::pins::gpio::BUZZER);
            delay(250);
        };

        pwm_led.disable();

        for (const uint8_t status: pvalid.arr) {
            if (status) {
                blink_green();
            } else {
                blink_red();
            }
        }
    }

    tx_data.reserve(512);
    sd_data.reserve(768);

    pvalid >> USB_DEBUG << stream::crlf;
    USB_DEBUG << "Init successful!" << stream::crlf;

    // Reset timers before starting
    pwm_led.set_frequency(stella::config::INTERVAL_MS_TO_HZ(stella::config::BUZZER_IDLE_INTERVAL));
    pwm_led.reset();

    gpio_write << io_function::pull_low(stella::pins::gpio::BUZZER);
    dispatcher.reset();
}

void loop() { dispatcher(); }

template<typename SdType, typename FileType>
void init_storage(FsUtil<SdType, FileType> &sd_util_instance) {
    sd_util_instance.find_file_name(THIS_FILE_PREFIX, THIS_FILE_EXTENSION);
    sd_util_instance.template open_one<FsMode::WRITE>();
}

void read_continuity() {
    sensor_data.cont_a = gpio_read.sample<128>(stella::pins::pyro::SENS_A);
    sensor_data.cont_b = gpio_read.sample<128>(stella::pins::pyro::SENS_B);
    sensor_data.cont_c = gpio_read.sample<128>(stella::pins::pyro::SENS_C);
}

void read_internal() {
    sensor_data.cpu_temp = internal::read_cpu_temp(internal::read_vref());
    sensor_data.batt_v   = ina236.getBusVoltage();
}

void read_gnss() {
    if (gnss_m9v.getPVT(stella::config::UBLOX_CUSTOM_MAX_WAIT)) {
        sensor_data.timestamp = gnss_m9v.getUnixEpoch(sensor_data.timestamp_us, stella::config::UBLOX_CUSTOM_MAX_WAIT);
        sensor_data.gps_siv   = gnss_m9v.getSIV(stella::config::UBLOX_CUSTOM_MAX_WAIT);
        sensor_data.gps_lat   = static_cast<double>(gnss_m9v.getLatitude(stella::config::UBLOX_CUSTOM_MAX_WAIT)) * 1.e-7;
        sensor_data.gps_lon   = static_cast<double>(gnss_m9v.getLongitude(stella::config::UBLOX_CUSTOM_MAX_WAIT)) * 1.e-7;
        sensor_data.gps_alt   = static_cast<float>(gnss_m9v.getAltitudeMSL(stella::config::UBLOX_CUSTOM_MAX_WAIT)) * 1.e-3f;
    }
}

void read_ms(ms_ref_t *ms) {
    static uint32_t t_prev = millis();

    ms->ms.read();

    if (const float t = ms->ms.getTemperature(); t > 0.) {
        ms->temp = t;
    }

    if (const float p = ms->ms.getPressure(); p > 0.) {
        ms->kf.update_dt(millis() - t_prev);
        ms->kf.kf.predict().update(p);
        ms->pres = ms->kf.kf.state;
    }

    ms->alt = pressure_altitude(ms->pres);

    t_prev  = millis();
}

void read_icm20948() {
    static uint32_t t_prev = millis();

    if (imu_icm20948.dataReady()) {
        imu_icm20948.getAGMT();

        // !!! DIRECTION TUNED !!!
        // Acc: milli-G unit to mss
        sensor_data.imu_2.acc.x  = -imu_icm20948.accY() * (0.001 * G);
        sensor_data.imu_2.acc.y  = -imu_icm20948.accX() * (0.001 * G);
        sensor_data.imu_2.acc.z  = imu_icm20948.accZ() * (0.001 * G);
        sensor_data.imu_2.gyro.x = -imu_icm20948.gyrY();
        sensor_data.imu_2.gyro.y = -imu_icm20948.gyrX();
        sensor_data.imu_2.gyro.z = imu_icm20948.gyrZ();
    }

    for (size_t i = 0; i < 3; ++i) {
        const uint32_t dt = millis() - t_prev;
        filters.imu_2.acc.values[i].update_dt(dt);
        filters.imu_2.gyro.values[i].update_dt(dt);
        filters.imu_2.acc.values[i].kf.predict().update(sensor_data.imu_2.acc.values[i]);
        filters.imu_2.gyro.values[i].kf.predict().update(sensor_data.imu_2.gyro.values[i]);
        sensor_data.imu_2.acc.values[i]  = filters.imu_2.acc.values[i].kf.state;
        sensor_data.imu_2.gyro.values[i] = filters.imu_2.gyro.values[i].kf.state;
    }

    t_prev = millis();
}

void read_icm42688() {
    static uint32_t t_prev = millis();

    if (imu_icm42688.readSensor() > 0) {
        // !!! DIRECTION TUNED !!!
        sensor_data.imu_1.acc.x  = -imu_icm42688.getAccelX_mss();
        sensor_data.imu_1.acc.y  = -imu_icm42688.getAccelY_mss();
        sensor_data.imu_1.acc.z  = -imu_icm42688.getAccelZ_mss();
        sensor_data.imu_1.gyro.x = -imu_icm42688.getGyroX_dps();
        sensor_data.imu_1.gyro.y = -imu_icm42688.getGyroY_dps();
        sensor_data.imu_1.gyro.z = -imu_icm42688.getGyroZ_dps();
    }

    for (size_t i = 0; i < 3; ++i) {
        const uint32_t dt = millis() - t_prev;
        filters.imu_1.acc.values[i].update_dt(dt);
        filters.imu_1.gyro.values[i].update_dt(dt);
        filters.imu_1.acc.values[i].kf.predict().update(sensor_data.imu_1.acc.values[i]);
        filters.imu_1.gyro.values[i].kf.predict().update(sensor_data.imu_1.gyro.values[i]);
        sensor_data.imu_1.acc.values[i]  = filters.imu_1.acc.values[i].kf.state;
        sensor_data.imu_1.gyro.values[i] = filters.imu_1.gyro.values[i].kf.state;
    }

    t_prev = millis();
}

void calculate_ground_truth() {
    switch (sensor_mode.pressure) {
        case 0b01:
            ground_truth.altitude = sensor_data.ms1_alt;
            break;
        case 0b10:
            ground_truth.altitude = sensor_data.ms2_alt;
            break;
        case 0b11:
            ground_truth.altitude = (sensor_data.ms1_alt + sensor_data.ms2_alt) * 0.5;
            break;
        default:
            __builtin_unreachable();
    }

    switch (sensor_mode.acceleration) {
        case 0b01:
            ground_truth.acc.x = sensor_data.imu_1.acc.x;
            ground_truth.acc.y = sensor_data.imu_1.acc.y;
            ground_truth.acc.z = sensor_data.imu_1.acc.z;
            break;
        case 0b10:
            ground_truth.acc.x = sensor_data.imu_2.acc.x;
            ground_truth.acc.y = sensor_data.imu_2.acc.y;
            ground_truth.acc.z = sensor_data.imu_2.acc.z;
            break;
        case 0b11:
            ground_truth.acc.x = (sensor_data.imu_1.acc.x + sensor_data.imu_2.acc.x) * 0.5;
            ground_truth.acc.y = (sensor_data.imu_1.acc.y + sensor_data.imu_2.acc.y) * 0.5;
            ground_truth.acc.z = (sensor_data.imu_1.acc.z + sensor_data.imu_2.acc.z) * 0.5;
            break;
        default:
            __builtin_unreachable();
    }
}

void synchronize_kf() {
    static uint32_t t_prev = millis();

    const double total_acc = algorithm::root_sum_square(ground_truth.acc.x, ground_truth.acc.y, ground_truth.acc.z);

    filters.altitude.update_dt(millis() - t_prev);
    filters.acceleration.update_dt(millis() - t_prev);
    filters.altitude.kf.predict().update(ground_truth.altitude);
    filters.acceleration.kf.predict().update(total_acc - G);

    t_prev = millis();
}

void accept_command(HardwareSerial *istream) {
    Stream &stream = *istream;  // Alias to istream

    if (!stream.available()) return;
    delay(20ul);

    String rx_message = "";
    rx_message.reserve(64);

    while (stream.available()) {
        rx_message += static_cast<char>(stream.read());
    }

    // Repeats for at least 5 times before continuing
    if (rx_message.substring(0, 4) != "cmd ") {
        // Return if cmd header is invalid
        ++sensor_data.last_nack;
        return;
    }

    String command = rx_message.substring(4);
    command.trim();

    ++sensor_data.last_ack;

    if (command == "ping" || command == "wake" || command == "on") {
        // <--- Maybe a wakeup command --->

    } else if (command == "mode-pres-1") {
        sensor_mode.pressure = 0b01;

    } else if (command == "mode-pres-2") {
        sensor_mode.pressure = 0b10;

    } else if (command == "mode-pres-f") {
        sensor_mode.pressure = 0b11;

    } else if (command == "mode-accel-1") {
        sensor_mode.acceleration = 0b01;

    } else if (command == "mode-accel-2") {
        sensor_mode.acceleration = 0b10;

    } else if (command == "mode-accel-f") {
        sensor_mode.acceleration = 0b11;

    } else if (command == "launch-override") {
        launch_override = true;

    } else if (command == "zero") {
        // <--- Zero barometric altitude --->
        read_ms(&ms1_ref);
        read_ms(&ms2_ref);
        switch (sensor_mode.pressure) {
            case 0b01:
                ground_truth.altitude_offset = sensor_data.ms1_alt;
                break;
            case 0b10:
                ground_truth.altitude_offset = sensor_data.ms2_alt;
                break;
            case 0b11:
                ground_truth.altitude_offset = (sensor_data.ms1_alt + sensor_data.ms2_alt) * 0.5;
                break;
            default:
                __builtin_unreachable();
        }

    } else if (command == "sleep") {
        // <--- Put the device into deep sleep mode (power saving) --->
        pwm_led.disable();
        stella::pins::PINS_OFF();
        stella::pins::SET_LED(stella::BLUE);
        LowPower.deepSleep();
        pwm_led.reset();

    } else if (command == "shutdown") {
        // <--- Shutdown the device --->
        pwm_led.disable();
        stella::pins::PINS_OFF();
        stella::pins::SET_LED(stella::RED);

        if (pvalid.sd) {
            sd_util.close_one();
        }
        if (pvalid.flash) {
            flash_util.close_one();
        }

        LowPower.deepSleep();

        __NVIC_SystemReset();

    } else if (command == "reboot" || command == "restart") {
        // <--- Reboot/reset the device --->
        if (pvalid.sd) {
            sd_util.close_one();
        }
        if (pvalid.flash) {
            flash_util.close_one();
        }
        __NVIC_SystemReset();

    } else if (command == "clear") {
        // <--- Clear ack and nack flags --->
        sensor_data.last_ack  = 0;
        sensor_data.last_nack = 0;

    } else {
        // <--- Unknown command: send back nack --->
        ++sensor_data.last_nack;
        --sensor_data.last_ack;
    }
}

void construct_data() {
    tx_data = "";
    csv_stream_crlf(tx_data)
            << "<45>"
            << sensor_data.timestamp
            << sensor_data.timestamp_us
            << millis()
            << sensor_data.tx_pc++
            << stella::state_string(sensor_data.ps)
            << sensor_mode_string(sensor_mode.pressure)
            << sensor_mode_string(sensor_mode.acceleration)

            << String(sensor_data.gps_lat, 6)
            << String(sensor_data.gps_lon, 6)
            << String(ground_truth.altitude, 4)

            << "N"
            << "N"
            << "N"
            << sensor_data.cont_a
            << sensor_data.cont_b
            << sensor_data.cont_c

            << sensor_data.ms1_temp << sensor_data.ms1_pres << sensor_data.ms1_alt
            << sensor_data.ms2_temp << sensor_data.ms2_pres << sensor_data.ms2_alt

            << sensor_data.imu_1.acc.x << sensor_data.imu_1.acc.y << sensor_data.imu_1.acc.z
            << sensor_data.imu_1.gyro.x << sensor_data.imu_1.gyro.y << sensor_data.imu_1.gyro.z

            << sensor_data.imu_2.acc.x << sensor_data.imu_2.acc.y << sensor_data.imu_2.acc.z
            << sensor_data.imu_2.gyro.x << sensor_data.imu_2.gyro.y << sensor_data.imu_2.gyro.z

            << sensor_data.cpu_temp
            << sensor_data.batt_v
            << sensor_data.last_ack
            << sensor_data.last_nack;
}

void transmit_data(time_type *interval_ms) {
    static time_type prev = *interval_ms;
    static smart_delay sd(*interval_ms, millis);

    if (prev != *interval_ms) {
        sd.set_interval(*interval_ms);
        prev = *interval_ms;
    }

    sd([&]() -> void {
        csv_stream_crlf(UART_RPI)
                << "GPS_TIME"
                << sensor_data.timestamp;
    });
}

void save_data(time_type *interval_ms) {
    static time_type prev = *interval_ms;
    static smart_delay sd(*interval_ms, millis);
    static smart_delay sd_save(1000, millis);

    if (prev != *interval_ms) {
        sd.set_interval(*interval_ms);
        prev = *interval_ms;
    }

    sd([&]() -> void {
        sd_util.file() << tx_data;
    });

    sd_save([&]() -> void {
        sd_util.flush_one();
    });
}

void fsm_eval() {
    static bool ok = false;
    static algorithm::Sampler sampler;

    switch (sensor_data.ps) {
        case stella::state_t::STARTUP: {
            sensor_data.ps = stella::state_t::IDLE;
            break;
        }
        case stella::state_t::IDLE: {
            static on_off_timer tim(500, 50, millis);

            buzzer_intervals.t_off = stella::config::BUZZER_OFF_INTERVAL(stella::config::BUZZER_IDLE_INTERVAL);

            if (!ok) {
                sampler.add(gpio_read.sample<8>(stella::pins::gpio::USER_1));

                tim.on_rising([&] {
                    if (sampler.vote<1, 1>()) {
                        ok |= true;
                    }
                    sampler.reset();
                });
            }

            if (ok) {
                sensor_data.ps = stella::state_t::START_RECORD;
            }

            break;
        }
        case stella::state_t::START_RECORD: {
            // <--- Start Recording --->
            csv_stream_crlf(UART_RPI)
                    << "COMMAND"
                    << "START";

            sensor_data.ps = stella::state_t::RECORDING;
            break;
        }
        case stella::state_t::RECORDING: {
            static on_off_timer tim(500, 50, millis);

            if (!ok) {
                sampler.add(gpio_read.sample<8>(stella::pins::gpio::USER_1));

                tim.on_rising([&] {
                    if (sampler.vote<1, 1>()) {
                        ok |= true;
                    }
                    sampler.reset();
                });
            }

            if (ok) {
                sensor_data.ps = stella::state_t::STOP_RECORD;
            }
            break;
        }
        case stella::state_t::STOP_RECORD: {
            // <--- Stop Recording --->
            csv_stream_crlf(UART_RPI)
                    << "COMMAND"
                    << "STOP";

            sensor_data.ps = stella::state_t::IDLE;
            break;
        }
        default:
            __builtin_unreachable();
    }
}

void buzzer_led_control(on_off_timer::interval_params *intervals_ms) {
    // Interval change keeper
    static time_type prev_on          = intervals_ms->t_on;
    static time_type prev_off         = intervals_ms->t_off;
    static stella::state_t prev_state = stella::state_t::STARTUP;

    static struct {
        stella::RGB_MASK value = stella::RGB_MASK::BLACK;
    } onboard_led;

    // On-off timer
    static on_off_timer timer(intervals_ms->t_on, intervals_ms->t_off, millis);

    if (prev_on != intervals_ms->t_on || prev_off != intervals_ms->t_off) {
        timer.set_interval_on(intervals_ms->t_on);
        timer.set_interval_off(intervals_ms->t_off);
        prev_on  = intervals_ms->t_on;
        prev_off = intervals_ms->t_off;

        pwm_led.set_frequency(stella::config::INTERVAL_MS_TO_HZ(intervals_ms->t_on + intervals_ms->t_off));
        pwm_led.reset();
    }

    if (prev_state != sensor_data.ps) {
        prev_state = sensor_data.ps;

        switch (sensor_data.ps) {
            case stella::state_t::IDLE:
                onboard_led.value = stella::RGB_MASK::GREEN;
                break;
            case stella::state_t::RECORDING:
                onboard_led.value = stella::RGB_MASK::RED;
                break;
            default:
                break;
        }

        pwm_led.set_color(onboard_led.value);
        pwm_led.set_buzzer(false);
        pwm_led.reset();
    }
}
