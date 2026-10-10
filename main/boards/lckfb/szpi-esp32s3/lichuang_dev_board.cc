#include "wifi_board.h"
#include "codecs/box_audio_codec.h"
#include "display/lcd_display.h"
#include "display/emote_display.h"
#include "application.h"
#include "button.h"
#include "config.h"
#include "i2c_device.h"
#include "esp32_camera.h"
#include "settings.h"
#include "mcp_server.h"
#include "press_to_talk_mcp_tool.h"

#include <esp_log.h>
#include <esp_lcd_panel_vendor.h>
#include <driver/i2c_master.h>
#include <driver/ledc.h>
#include <driver/spi_common.h>
#include <driver/esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <cstdlib>
#include <esp_lcd_touch_ft5x06.h>
#include <esp_lvgl_port.h>
#include <lvgl.h>

#define TAG "LichuangDevBoard"

class Pca9557 : public I2cDevice {
public:
    Pca9557(i2c_master_bus_handle_t i2c_bus, uint8_t addr) : I2cDevice(i2c_bus, addr) {
        WriteReg(0x01, 0x03);
        WriteReg(0x03, 0xf8);
    }

    void SetOutputState(uint8_t bit, uint8_t level) {
        uint8_t data = ReadReg(0x01);
        data = (data & ~(1 << bit)) | (level << bit);
        WriteReg(0x01, data);
    }
};

class CustomAudioCodec : public BoxAudioCodec {
private:
    Pca9557* pca9557_;

public:
    CustomAudioCodec(i2c_master_bus_handle_t i2c_bus, Pca9557* pca9557)
        : BoxAudioCodec(i2c_bus, AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
                        AUDIO_I2S_GPIO_MCLK, AUDIO_I2S_GPIO_BCLK, AUDIO_I2S_GPIO_WS,
                        AUDIO_I2S_GPIO_DOUT, AUDIO_I2S_GPIO_DIN, GPIO_NUM_NC,
                        AUDIO_CODEC_ES8311_ADDR, AUDIO_CODEC_ES7210_ADDR, AUDIO_INPUT_REFERENCE,
                        28.0f,  // Physical MIC1 gain
                        2,      // Physical MIC3 is the playback reference input
                        0.0f),
          pca9557_(pca9557) {}

    virtual void EnableOutput(bool enable) override {
        BoxAudioCodec::EnableOutput(enable);
        if (enable) {
            pca9557_->SetOutputState(1, 1);
        } else {
            pca9557_->SetOutputState(1, 0);
        }
    }
};

/*
 * Simple LEDC-based hobby servo driver (50 Hz, 13-bit, 500~2500 us pulse).
 * Uses LEDC_TIMER_3 and channels 4/5: the backlight already occupies
 * TIMER_0/CHANNEL_0 and the camera XCLK occupies TIMER_2/CHANNEL_2.
 */
#define SERVO_PWM_FREQ_HZ   50
#define SERVO_PULSE_MIN_US  500
#define SERVO_PULSE_MAX_US  2500
#define SERVO_PWM_PERIOD_US 20000
#define SERVO_PWM_MAX_DUTY  ((1 << 13) - 1)

class Servo {
public:
    Servo(gpio_num_t pin, ledc_channel_t channel) : pin_(pin), channel_(channel) {}

    void Init() {
        ledc_channel_config_t ch_conf = {};
        ch_conf.gpio_num = pin_;
        ch_conf.speed_mode = LEDC_LOW_SPEED_MODE;
        ch_conf.channel = channel_;
        ch_conf.timer_sel = LEDC_TIMER_3;
        ch_conf.duty = 0;
        ch_conf.hpoint = 0;
        ESP_ERROR_CHECK(ledc_channel_config(&ch_conf));
    }

    void SetAngle(int angle) {
        int pulse_us = SERVO_PULSE_MIN_US +
                       (SERVO_PULSE_MAX_US - SERVO_PULSE_MIN_US) * angle / 180;
        uint32_t duty = pulse_us * SERVO_PWM_MAX_DUTY / SERVO_PWM_PERIOD_US;
        ledc_set_duty(LEDC_LOW_SPEED_MODE, channel_, duty);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, channel_);
        ESP_LOGI(TAG, "Servo ch%d angle=%d", channel_, angle);
    }

private:
    gpio_num_t pin_;
    ledc_channel_t channel_;
};

/*
 * PCA9685 16-channel PWM / LED driver on the I2C expansion connector (J1).
 * It has its own 25 MHz oscillator, so unlike the LEDC-based servos above it
 * costs the ESP32 no timers and keeps a rock-steady 50 Hz~1.9 kHz signal.
 * 12-bit duty resolution: a channel is low while counter < on, high from
 * counter >= off, and OFF_H bit 4 forces the line fully high / fully low.
 */
#define PCA9685_REG_MODE1          0x00
#define PCA9685_REG_MODE2          0x01
#define PCA9685_REG_LED0_ON_L      0x06
#define PCA9685_REG_ALL_LED_ON_L   0xfa
#define PCA9685_REG_ALL_LED_OFF_L  0xfc
#define PCA9685_REG_PRE_SCALE      0xfe

#define PCA9685_MODE1_AI           (1 << 4)   // register address auto-increment
#define PCA9685_MODE1_SLEEP        (1 << 5)
#define PCA9685_MODE1_RESTART      (1 << 7)
#define PCA9685_MODE2_OUTDRV       (1 << 2)   // totem-pole outputs
#define PCA9685_MODE2_OUTRE        (1 << 0)   // outputs respond when OE is low
#define PCA9685_MAX_DUTY           4096
#define PCA9685_OSC_FREQ_HZ        25000000
#define PCA9685_CHANNEL_STRIDE     4
#define PCA9685_FULL_ON            (1 << 4)   // bit 4 of OFF_H: output held high all period
#define PCA9685_FULL_OFF           (1 << 4)   // bit 4 of ON_H:  output held low all period

class Pca9685 : public I2cDevice {
public:
    Pca9685(i2c_master_bus_handle_t i2c_bus, uint8_t addr, uint32_t freq_hz)
        : I2cDevice(i2c_bus, addr) {
        // prescale = round(osc / (4096 * freq)) - 1, valid for 24..1526 Hz
        uint32_t prescale = (PCA9685_OSC_FREQ_HZ + freq_hz * 2048) / (freq_hz * 4096) - 1;
        if (prescale < 3) {
            prescale = 3;
        }
        if (prescale > 255) {
            prescale = 255;
        }

        uint8_t mode1 = ReadReg(PCA9685_REG_MODE1);
        WriteReg(PCA9685_REG_MODE1, (mode1 & ~PCA9685_MODE1_AI) | PCA9685_MODE1_SLEEP);
        WriteReg(PCA9685_REG_PRE_SCALE, static_cast<uint8_t>(prescale));
        vTaskDelay(pdMS_TO_TICKS(5));
        mode1 = (mode1 | PCA9685_MODE1_AI) & ~PCA9685_MODE1_SLEEP;
        WriteReg(PCA9685_REG_MODE1, mode1);
        vTaskDelay(pdMS_TO_TICKS(5));
        WriteReg(PCA9685_REG_MODE1, mode1 | PCA9685_MODE1_RESTART);
        WriteReg(PCA9685_REG_MODE2, PCA9685_MODE2_OUTDRV | PCA9685_MODE2_OUTRE);
        SetAllDuty(0);
        ESP_LOGI(TAG, "PCA9685 @0x%02x ready, prescale=%lu", addr, (unsigned long)prescale);
    }

    // duty 0 keeps the line low all period, 4096 and above keeps it high
    void SetDuty(uint8_t channel, uint16_t duty) {
        if (channel > 15) {
            return;
        }
        // register order for one channel: ON_L, ON_H, OFF_L, OFF_H
        uint8_t values[4];
        if (duty == 0) {
            // ON at 4096 (never reached) and OFF at 0 -> held low all period
            values[0] = 0x00;
            values[1] = PCA9685_FULL_OFF;
            values[2] = 0x00;
            values[3] = 0x00;
        } else if (duty >= PCA9685_MAX_DUTY) {
            // ON at 0 and OFF at 4096 (never reached) -> held high all period
            values[0] = 0x00;
            values[1] = 0x00;
            values[2] = 0x00;
            values[3] = PCA9685_FULL_ON;
        } else {
            values[0] = 0x00;
            values[1] = 0x00;
            values[2] = duty & 0xff;
            values[3] = (duty >> 8) & 0x0f;
        }
        WriteRegs(PCA9685_REG_LED0_ON_L + channel * PCA9685_CHANNEL_STRIDE, values, 4);
    }

    void SetAllDuty(uint16_t duty) {
        uint8_t on_values[2] = {0x00, 0x00};
        uint8_t off_values[2] = {static_cast<uint8_t>(duty & 0xff),
                                 static_cast<uint8_t>((duty >> 8) & 0x0f)};
        if (duty == 0) {
            on_values[1] = PCA9685_FULL_OFF;
            off_values[0] = 0x00;
            off_values[1] = 0x00;
        } else if (duty >= PCA9685_MAX_DUTY) {
            off_values[0] = 0x00;
            off_values[1] = PCA9685_FULL_ON;
        }
        WriteRegs(PCA9685_REG_ALL_LED_ON_L, on_values, 2);
        WriteRegs(PCA9685_REG_ALL_LED_OFF_L, off_values, 2);
    }
};

/*
 * One TB6612FNG half-bridge: two direction inputs plus one PWM input.
 * Full brake shorts both direction inputs high with the PWM off, which holds
 * the geared motor against a slope instead of letting it coast.
 */
#define MOTOR_FULL_ON  (PCA9685_MAX_DUTY + 1)

class Tb6612Motor {
public:
    Tb6612Motor(Pca9685* pwm, uint8_t ina_ch, uint8_t inb_ch, uint8_t pwm_ch)
        : pwm_(pwm), ina_ch_(ina_ch), inb_ch_(inb_ch), pwm_ch_(pwm_ch) {}

    // percent: -100 (full speed back) .. 100 (full speed forward), 0 = brake
    void SetSpeed(int percent) {
        if (percent < -100) {
            percent = -100;
        } else if (percent > 100) {
            percent = 100;
        }
        int duty = abs(percent) * PCA9685_MAX_DUTY / 100;
        if (percent == 0) {
            Brake();
            return;
        }
        if (percent > 0) {
            pwm_->SetDuty(ina_ch_, MOTOR_FULL_ON);
            pwm_->SetDuty(inb_ch_, 0);
        } else {
            pwm_->SetDuty(ina_ch_, 0);
            pwm_->SetDuty(inb_ch_, MOTOR_FULL_ON);
        }
        pwm_->SetDuty(pwm_ch_, static_cast<uint16_t>(duty));
        last_percent_ = percent;
    }

    void Brake() {
        pwm_->SetDuty(ina_ch_, MOTOR_FULL_ON);
        pwm_->SetDuty(inb_ch_, MOTOR_FULL_ON);
        pwm_->SetDuty(pwm_ch_, 0);
        last_percent_ = 0;
    }

    void Coast() {
        pwm_->SetDuty(ina_ch_, 0);
        pwm_->SetDuty(inb_ch_, 0);
        pwm_->SetDuty(pwm_ch_, 0);
        last_percent_ = 0;
    }

    int last_percent() const { return last_percent_; }

private:
    Pca9685* pwm_;
    uint8_t ina_ch_;
    uint8_t inb_ch_;
    uint8_t pwm_ch_;
    int last_percent_ = 0;
};

class LichuangDevBoard : public WifiBoard {
private:
    i2c_master_bus_handle_t i2c_bus_;
    i2c_master_dev_handle_t pca9557_handle_;
    Button boot_button_;
    Display* display_;
    Pca9557* pca9557_;
    Esp32Camera* camera_;
    PressToTalkMcpTool* press_to_talk_tool_ = nullptr;
    Servo* pan_servo_ = nullptr;
    Servo* tilt_servo_ = nullptr;
    Pca9685* pca9685_ = nullptr;
    Tb6612Motor* left_motor_ = nullptr;
    Tb6612Motor* right_motor_ = nullptr;
    esp_timer_handle_t drive_stop_timer_ = nullptr;

    void InitializeI2c() {
        // Initialize I2C peripheral
        i2c_master_bus_config_t i2c_bus_cfg = {
            .i2c_port = (i2c_port_t)1,
            .sda_io_num = AUDIO_CODEC_I2C_SDA_PIN,
            .scl_io_num = AUDIO_CODEC_I2C_SCL_PIN,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 0,
            .flags = {
                .enable_internal_pullup = 1,
            },
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_cfg, &i2c_bus_));

        // Initialize PCA9557
        pca9557_ = new Pca9557(i2c_bus_, 0x19);
    }

    void InitializeSpi() {
        spi_bus_config_t buscfg = {};
        buscfg.mosi_io_num = GPIO_NUM_40;
        buscfg.miso_io_num = GPIO_NUM_NC;
        buscfg.sclk_io_num = GPIO_NUM_41;
        buscfg.quadwp_io_num = GPIO_NUM_NC;
        buscfg.quadhd_io_num = GPIO_NUM_NC;
        buscfg.max_transfer_sz = DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t);
        ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO));
    }

    void InitializeButtons() {
        boot_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            // During startup (before connected), pressing BOOT button enters Wi-Fi config mode without reboot
            if (app.GetDeviceState() == kDeviceStateStarting) {
                EnterWifiConfigMode();
                return;
            }
            // In press-to-talk mode the click event is handled by press down/up
            if (!press_to_talk_tool_ || !press_to_talk_tool_->IsPressToTalkEnabled()) {
                app.ToggleChatState();
            }
        });

        boot_button_.OnPressDown([this]() {
            if (press_to_talk_tool_ && press_to_talk_tool_->IsPressToTalkEnabled()) {
                Application::GetInstance().StartListening();
            }
        });
        boot_button_.OnPressUp([this]() {
            if (press_to_talk_tool_ && press_to_talk_tool_->IsPressToTalkEnabled()) {
                Application::GetInstance().StopListening();
            }
        });

#if CONFIG_USE_DEVICE_AEC
        boot_button_.OnDoubleClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateIdle) {
                app.SetAecMode(app.GetAecMode() == kAecOff ? kAecOnDeviceSide : kAecOff);
            }
        });
#endif
    }

    void InitializeSt7789Display() {
        esp_lcd_panel_io_handle_t panel_io = nullptr;
        esp_lcd_panel_handle_t panel = nullptr;
        // 液晶屏控制IO初始化
        ESP_LOGD(TAG, "Install panel IO");
        esp_lcd_panel_io_spi_config_t io_config = {};
        io_config.cs_gpio_num = GPIO_NUM_NC;
        io_config.dc_gpio_num = GPIO_NUM_39;
        io_config.spi_mode = 2;
        io_config.pclk_hz = 80 * 1000 * 1000;
        io_config.trans_queue_depth = 10;
        io_config.lcd_cmd_bits = 8;
        io_config.lcd_param_bits = 8;
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(SPI3_HOST, &io_config, &panel_io));

        // 初始化液晶屏驱动芯片ST7789
        ESP_LOGD(TAG, "Install LCD driver");
        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = GPIO_NUM_NC;
        panel_config.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB;
        panel_config.bits_per_pixel = 16;
        ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(panel_io, &panel_config, &panel));

        esp_lcd_panel_reset(panel);
        pca9557_->SetOutputState(0, 0);

        esp_lcd_panel_init(panel);
        esp_lcd_panel_invert_color(panel, true);
        esp_lcd_panel_swap_xy(panel, DISPLAY_SWAP_XY);
        esp_lcd_panel_mirror(panel, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y);
        esp_lcd_panel_disp_on_off(panel, true);

#if CONFIG_USE_EMOTE_MESSAGE_STYLE
        display_ = new emote::EmoteDisplay(panel, panel_io, DISPLAY_WIDTH, DISPLAY_HEIGHT);
#else
        display_ = new SpiLcdDisplay(panel_io, panel,
            DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY);
#endif
    }

    void InitializeTouch()
    {
        esp_lcd_touch_handle_t tp = nullptr;
        esp_lcd_touch_config_t tp_cfg = {
            .x_max = DISPLAY_HEIGHT,
            .y_max = DISPLAY_WIDTH,
            .rst_gpio_num = GPIO_NUM_NC, // Shared with LCD reset
            .int_gpio_num = GPIO_NUM_NC,
            .levels = {
                .reset = 0,
                .interrupt = 0,
            },
            .flags = {
                .swap_xy = 1,
                .mirror_x = 1,
                .mirror_y = 0,
            },
        };
        esp_lcd_panel_io_handle_t tp_io_handle = NULL;
        esp_lcd_panel_io_i2c_config_t tp_io_config = {
            .dev_addr = ESP_LCD_TOUCH_IO_I2C_FT5x06_ADDRESS,
            .control_phase_bytes = 1,
            .dc_bit_offset = 0,
            .lcd_cmd_bits = 8,
            .flags =
            {
                .disable_control_phase = 1,
            }
        };
        tp_io_config.scl_speed_hz = 400000;

        if (esp_lcd_new_panel_io_i2c(i2c_bus_, &tp_io_config, &tp_io_handle) != ESP_OK) {
            ESP_LOGW(TAG, "Failed to create touch panel IO, continuing without touch");
            return;
        }
        if (esp_lcd_touch_new_i2c_ft5x06(tp_io_handle, &tp_cfg, &tp) != ESP_OK || tp == nullptr) {
            ESP_LOGW(TAG, "FT5x06 touch controller not found, continuing without touch");
            esp_lcd_panel_io_del(tp_io_handle);
            return;
        }

        /* Add touch input (for selected screen) */
        const lvgl_port_touch_cfg_t touch_cfg = {
            .disp = lv_display_get_default(),
            .handle = tp,
        };

        if(touch_cfg.disp) {
            lvgl_port_add_touch(&touch_cfg);
        } else {
            ESP_LOGE(TAG, "Touch display is not initialized");
        }
    }

    void InitializeCamera() {
        // Open camera power
        pca9557_->SetOutputState(2, 0);

        camera_config_t config = {};
        config.ledc_channel = LEDC_CHANNEL_2;
        config.ledc_timer = LEDC_TIMER_2;
        config.pin_d0 = CAMERA_PIN_D0;
        config.pin_d1 = CAMERA_PIN_D1;
        config.pin_d2 = CAMERA_PIN_D2;
        config.pin_d3 = CAMERA_PIN_D3;
        config.pin_d4 = CAMERA_PIN_D4;
        config.pin_d5 = CAMERA_PIN_D5;
        config.pin_d6 = CAMERA_PIN_D6;
        config.pin_d7 = CAMERA_PIN_D7;
        config.pin_xclk = CAMERA_PIN_XCLK;
        config.pin_pclk = CAMERA_PIN_PCLK;
        config.pin_vsync = CAMERA_PIN_VSYNC;
        config.pin_href = CAMERA_PIN_HREF;
        config.pin_sccb_sda = -1;
        config.pin_sccb_scl = CAMERA_PIN_SIOC;
        config.sccb_i2c_port = 1;
        config.pin_pwdn = CAMERA_PIN_PWDN;
        config.pin_reset = CAMERA_PIN_RESET;
        config.xclk_freq_hz = XCLK_FREQ_HZ;
        config.pixel_format = PIXFORMAT_RGB565;
        config.frame_size = FRAMESIZE_QVGA;
        config.jpeg_quality = 12;
        config.fb_count = 1;
        config.fb_location = CAMERA_FB_IN_PSRAM;
        config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;

        camera_ = new Esp32Camera(config);
    }

    void InitializeServos() {
        ledc_timer_config_t timer_conf = {};
        timer_conf.speed_mode = LEDC_LOW_SPEED_MODE;
        timer_conf.duty_resolution = LEDC_TIMER_13_BIT;
        timer_conf.timer_num = LEDC_TIMER_3;
        timer_conf.freq_hz = SERVO_PWM_FREQ_HZ;
        timer_conf.clk_cfg = LEDC_AUTO_CLK;
        ESP_ERROR_CHECK(ledc_timer_config(&timer_conf));

        pan_servo_ = new Servo(SERVO_PAN_PIN, LEDC_CHANNEL_4);
        tilt_servo_ = new Servo(SERVO_TILT_PIN, LEDC_CHANNEL_5);
        pan_servo_->Init();
        tilt_servo_->Init();
    }

    void StopDrive() {
        if (left_motor_ == nullptr || right_motor_ == nullptr) {
            return;
        }
        left_motor_->Brake();
        right_motor_->Brake();
        ESP_LOGI(TAG, "Drive braked");
    }

    void InitializeDrive() {
        // The chip is optional hardware on the I2C expansion connector: probe it
        // first so a robot without wheels still boots cleanly.
        if (i2c_master_probe(i2c_bus_, PCA9685_I2C_ADDR, 100) != ESP_OK) {
            ESP_LOGW(TAG, "PCA9685 @0x%02x not found, drive tools disabled", PCA9685_I2C_ADDR);
            return;
        }

        pca9685_ = new Pca9685(i2c_bus_, PCA9685_I2C_ADDR, PCA9685_PWM_FREQ_HZ);
        left_motor_ = new Tb6612Motor(pca9685_, MOTOR_LEFT_INA_CH, MOTOR_LEFT_INB_CH,
                                      MOTOR_LEFT_PWM_CH);
        right_motor_ = new Tb6612Motor(pca9685_, MOTOR_RIGHT_INA_CH, MOTOR_RIGHT_INB_CH,
                                       MOTOR_RIGHT_PWM_CH);
        left_motor_->Brake();
        right_motor_->Brake();

        const esp_timer_create_args_t stop_timer_args = {
            .callback = [](void* arg) {
                static_cast<LichuangDevBoard*>(arg)->StopDrive();
            },
            .arg = this,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "drive_stop",
            .skip_unhandled_events = true,
        };
        ESP_ERROR_CHECK(esp_timer_create(&stop_timer_args, &drive_stop_timer_));
        ESP_LOGI(TAG, "Drive ready: PCA9685 @0x%02x -> TB6612 -> 2x N20", PCA9685_I2C_ADDR);
    }

    // Same NVS key the provisioning portal writes and ota.cc reads.
    void InitializeOtaUrl() {
        Settings settings("wifi", true);
        std::string url = settings.GetString("ota_url");
        if (url.empty()) {
            settings.SetString("ota_url", LOCAL_OTA_URL);
            ESP_LOGI(TAG, "No OTA URL configured, falling back to the local brain: %s",
                     LOCAL_OTA_URL);
        } else {
            ESP_LOGI(TAG, "OTA URL: %s", url.c_str());
        }
    }

    void InitializeTools() {
        auto &mcp_server = McpServer::GetInstance();
        mcp_server.AddTool("self.system.reconfigure_wifi",
            "End this conversation and enter WiFi configuration mode.\n"
            "**CAUTION** You must ask the user to confirm this action.",
            PropertyList(), [this](const PropertyList& properties) {
                EnterWifiConfigMode();
                return true;
            });

        // Pan-tilt servos on the multi-purpose expansion connector (IO10 / IO11)
        InitializeServos();
        mcp_server.AddTool("self.servo.set_angle",
            "设置桌面机器人摇头舵机的角度。舵机接在板子的外拓接口（多用）上。\n"
            "channel: 1 = 水平转动（pan，左右摇头），2 = 俯仰（tilt，点头）。\n"
            "angle: 0~180 度，90 度为中间位置。",
            PropertyList({
                Property("channel", kPropertyTypeInteger, 1, 1, 2),
                Property("angle", kPropertyTypeInteger, 90, 0, 180)
            }),
            [this](const PropertyList& properties) -> ReturnValue {
                int channel = properties["channel"].value<int>();
                int angle = properties["angle"].value<int>();
                if (channel == 1) {
                    pan_servo_->SetAngle(angle);
                } else {
                    tilt_servo_->SetAngle(angle);
                }
                char message[64];
                snprintf(message, sizeof(message), "舵机 %d 已转到 %d 度", channel, angle);
                return std::string(message);
            });

        // Wheeled drive: PCA9685 on the I2C expansion connector -> TB6612FNG -> 2x N20
        InitializeDrive();
        if (pca9685_ != nullptr) {
            mcp_server.AddTool("self.drive.set_speed",
                "驱动桌面机器人的两个轮子（差速转向）。\n"
                "left / right: -100~100 的速度百分比，正数前进、负数后退、0 为刹停该侧。\n"
                "直行前进 = left=60,right=60；原地左转 = left=-40,right=40；原地右转 = left=40,right=-40。\n"
                "duration_seconds: 大于 0 时到时自动刹停两轮，用于\"往前走 3 秒\"这类动作；填 0 表示持续运动，需要另一次调用才停。",
                PropertyList({
                    Property("left", kPropertyTypeInteger, 0, -100, 100),
                    Property("right", kPropertyTypeInteger, 0, -100, 100),
                    Property("duration_seconds", kPropertyTypeInteger, 0, 0, 30)
                }),
                [this](const PropertyList& properties) -> ReturnValue {
                    int left = properties["left"].value<int>();
                    int right = properties["right"].value<int>();
                    int duration = properties["duration_seconds"].value<int>();
                    left_motor_->SetSpeed(left);
                    right_motor_->SetSpeed(right);
                    if (drive_stop_timer_ != nullptr) {
                        esp_timer_stop(drive_stop_timer_);
                        if (duration > 0) {
                            esp_timer_start_once(drive_stop_timer_,
                                                 static_cast<uint64_t>(duration) * 1000000);
                        }
                    }
                    char message[96];
                    if (duration > 0) {
                        snprintf(message, sizeof(message),
                                 "轮子 左%d%% 右%d%%，%d 秒后自动刹停", left, right, duration);
                    } else {
                        snprintf(message, sizeof(message), "轮子 左%d%% 右%d%%，持续运动", left, right);
                    }
                    return std::string(message);
                });

            mcp_server.AddTool("self.drive.stop",
                "立即刹停桌面机器人的两个轮子（短接刹车，能停住坡道上的车）。",
                PropertyList(), [this](const PropertyList& properties) -> ReturnValue {
                    if (drive_stop_timer_ != nullptr) {
                        esp_timer_stop(drive_stop_timer_);
                    }
                    StopDrive();
                    return std::string("轮子已刹停");
                });
        }

        // Allow switching between press-to-talk (长按说话) and click-to-talk (单击唤醒)
        press_to_talk_tool_ = new PressToTalkMcpTool();
        press_to_talk_tool_->Initialize();
    }

public:
    LichuangDevBoard() : boot_button_(BOOT_BUTTON_GPIO) {
        InitializeOtaUrl();
        InitializeI2c();
        InitializeSpi();
        InitializeSt7789Display();
        InitializeTouch();
        InitializeButtons();
        InitializeCamera();
        InitializeTools();

        GetBacklight()->RestoreBrightness();
    }

    virtual AudioCodec* GetAudioCodec() override {
        static CustomAudioCodec audio_codec(
            i2c_bus_,
            pca9557_);
        return &audio_codec;
    }

    virtual Display* GetDisplay() override {
        return display_;
    }

    virtual Backlight* GetBacklight() override {
        static PwmBacklight backlight(DISPLAY_BACKLIGHT_PIN, DISPLAY_BACKLIGHT_OUTPUT_INVERT);
        return &backlight;
    }

    virtual Camera* GetCamera() override {
        return camera_;
    }
};

DECLARE_BOARD(LichuangDevBoard);
