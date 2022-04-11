#include <Arduino.h>
#include <esp_camera.h>
#include "cam_pins.h"
//#define sdcard
#ifdef sdcard
#include "FS.h"                // SD Card ESP32
#include "SD_MMC.h"            // SD Card ESP32
#endif
#include <HardwareSerial.h>
HardwareSerial mySerial(1);

camera_config_t config;
int sensorPID;

camera_fb_t * fb = NULL;

uint packetsQuantity = 0;
uint currentPacket = 0;

struct frameData
{
    /* data */
    size_t len;                 /*!< Length of the buffer in bytes */
};
const int frameDataSize = sizeof(frameData);

union imgPacket{
    frameData Info;
    uint8_t byteArray[frameDataSize];
};

imgPacket myimagedata;

void StartCamera()
{
    // Populate camera config structure with hardware and other defaults
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sscb_sda = SIOD_GPIO_NUM;
    config.pin_sscb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_JPEG;
    // config.grab_mode = CAMERA_GRAB_LATEST;
    if (psramFound()){
        config.frame_size = FRAMESIZE_SXGA;
        config.jpeg_quality = 10;
        config.fb_count = 2;
    }else{
        config.frame_size = FRAMESIZE_SVGA;
        config.jpeg_quality = 12;
        config.fb_count = 1;
    }

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        delay(100);  // need a delay here or the next serial o/p gets missed
        Serial.printf("\r\n\r\nCRITICAL FAILURE: Camera sensor failed to initialise.\r\n\r\n");
        Serial.printf("A full (hard, power off/on) reboot will probably be needed to recover from this.\r\n");
        Serial.printf("Meanwhile; this unit will reboot in 1 minute since these errors sometime clear automatically\r\n");
        // Reset the I2C bus.. may help when rebooting.
        periph_module_disable(PERIPH_I2C0_MODULE); // try to shut I2C down properly in case that is the problem
        periph_module_disable(PERIPH_I2C1_MODULE);
        periph_module_reset(PERIPH_I2C0_MODULE);
        periph_module_reset(PERIPH_I2C1_MODULE);
    } else {
        Serial.println("Camera init succeeded");

        // Get a reference to the sensor
        sensor_t * s = esp_camera_sensor_get();

        // Dump camera module, warn for unsupported modules.
        sensorPID = s->id.PID;
        switch (sensorPID) {
            case OV9650_PID: Serial.println("WARNING: OV9650 camera module is not properly supported, will fallback to OV2640 operation"); break;
            case OV7725_PID: Serial.println("WARNING: OV7725 camera module is not properly supported, will fallback to OV2640 operation"); break;
            case OV2640_PID: Serial.println("OV2640 camera module detected"); break;
            case OV3660_PID: Serial.println("OV3660 camera module detected"); break;
            default: Serial.println("WARNING: Camera module is unknown and not properly supported, will fallback to OV2640 operation");
        }

        // OV3660 initial sensors are flipped vertically and colors are a bit saturated
        if (sensorPID == OV2640_PID) {
            s->set_brightness(s, 1);  //up the blightness just a bit
            s->set_whitebal(s, 1); //turn on white balance
            s->set_wb_mode(s, 0); //white balance to auto
        }
        // M5 Stack Wide has special needs
        #if defined(CAMERA_MODEL_M5STACK_WIDE)
            s->set_vflip(s, 1);
            s->set_hmirror(s, 1);
        #endif

        // Config can override mirror and flip
        #if defined(H_MIRROR)
            s->set_hmirror(s, H_MIRROR);
        #endif
        #if defined(V_FLIP)
            s->set_vflip(s, V_FLIP);
        #endif

        /*
        * Add any other defaults you want to apply at startup here:
        * uncomment the line and set the value as desired (see the comments)
        *
        * these are defined in the esp headers here:
        * https://github.com/espressif/esp32-camera/blob/master/driver/include/sensor.h#L149
        */

        //s->set_framesize(s, FRAMESIZE_SVGA); // FRAMESIZE_[QQVGA|HQVGA|QVGA|CIF|VGA|SVGA|XGA|SXGA|UXGA|QXGA(ov3660)]);
        //s->set_quality(s, val);       // 10 to 63
        //s->set_brightness(s, 0);      // -2 to 2
        //s->set_contrast(s, 0);        // -2 to 2
        //s->set_saturation(s, 0);      // -2 to 2
        //s->set_special_effect(s, 0);  // 0 to 6 (0 - No Effect, 1 - Negative, 2 - Grayscale, 3 - Red Tint, 4 - Green Tint, 5 - Blue Tint, 6 - Sepia)
        //s->set_whitebal(s, 1);        // aka 'awb' in the UI; 0 = disable , 1 = enable
        //s->set_awb_gain(s, 1);        // 0 = disable , 1 = enable
        //s->set_wb_mode(s, 0);         // 0 to 4 - if awb_gain enabled (0 - Auto, 1 - Sunny, 2 - Cloudy, 3 - Office, 4 - Home)
        //s->set_exposure_ctrl(s, 1);   // 0 = disable , 1 = enable
        //s->set_aec2(s, 0);            // 0 = disable , 1 = enable
        //s->set_ae_level(s, 0);        // -2 to 2
        //s->set_aec_value(s, 300);     // 0 to 1200
        //s->set_gain_ctrl(s, 1);       // 0 = disable , 1 = enable
        //s->set_agc_gain(s, 0);        // 0 to 30
        //s->set_gainceiling(s, (gainceiling_t)0);  // 0 to 6
        //s->set_bpc(s, 0);             // 0 = disable , 1 = enable
        //s->set_wpc(s, 1);             // 0 = disable , 1 = enable
        //s->set_raw_gma(s, 1);         // 0 = disable , 1 = enable
        //s->set_lenc(s, 1);            // 0 = disable , 1 = enable
        //s->set_hmirror(s, 0);         // 0 = disable , 1 = enable
        //s->set_vflip(s, 0);           // 0 = disable , 1 = enable
        //s->set_dcw(s, 1);             // 0 = disable , 1 = enable
        //s->set_colorbar(s, 0);        // 0 = disable , 1 = enable
    }
    // We now have camera with default init
}

void setup()
{
    //set pins
    pinMode(LED_PIN, OUTPUT);
    pinMode(LAMP_PIN, OUTPUT);
    //turn on config LED
    digitalWrite(LED_PIN, LED_ON);
    // Warn if no PSRAM is detected (typically user error with board selection in the IDE)
    if(!psramFound()){
        Serial.println("\r\nFatal Error; Halting");
        while (true) {
            Serial.println("No PSRAM found; camera cannot be initialised: Please check the board config for your module.");
            delay(1000);
        }
    }
    StartCamera(); //initialize cam
    Serial.begin(115200);// BAUD rate
    mySerial.begin(115200, SERIAL_8N1,15,13);//rx, tx
    mySerial.flush();
    digitalWrite(LED_PIN, LED_OFF);

    #ifdef sdcard
    //digitalWrite(LAMP_PIN, HIGH);
    Serial.println("Starting SD Card");
    if(!SD_MMC.begin("/sdcard", true)){
        Serial.println("SD Card Mount Failed");
        return;
    }
    
    uint8_t cardType = SD_MMC.cardType();
    if(cardType == CARD_NONE){
        Serial.println("No SD Card attached");
        return;
    }
    
    fb = esp_camera_fb_get();
    //digitalWrite(LAMP_PIN, LOW);
    String path = "/picture.jpg";

    fs::FS &fs = SD_MMC; 
    Serial.printf("Picture file name: %s\n", path.c_str());
    
    File file = fs.open(path.c_str(), FILE_WRITE);
    if(!file){
        Serial.println("Failed to open file in writing mode");
    } 
    else {
        file.write(fb->buf, fb->len); // payload (image), payload length
        Serial.printf("Saved file to path: %s\n", path.c_str());
    }
    file.close();
    
    #endif
    //mySerial.println(fb->len, DEC);
}

void loop()
{
    // put your main code here, to run repeatedly:
    delay(1);
    if(mySerial.available()){
        byte command = mySerial.read();
        switch (command)
        {
        case 0x10/* take picture */:
            digitalWrite(LAMP_PIN, HIGH);
            delay(100);
            fb = esp_camera_fb_get();
            //esp_camera_deinit();
            digitalWrite(LAMP_PIN, LOW);
            break;
        case 0x11/* send picture */:
            digitalWrite(LED_PIN, LED_ON);
            myimagedata.Info.len = fb->len;
            mySerial.write(myimagedata.byteArray, sizeof(myimagedata.byteArray));
            delay(1);
            mySerial.write(fb->buf, fb->len);
            esp_camera_fb_return(fb);
            digitalWrite(LED_PIN, LED_OFF);
            break;
        default:
            break;
        }
    }
}