#include <Arduino.h>
#include "pins.hpp"
#include "kart.hpp"
#include "ld06.hpp"
#include "dma.hpp"
#include "logger.hpp"
#include "pure_pursuit.hpp"
#include "f1tenth_gap_follow.hpp"
#include "naive_gap_follow.hpp"

// Robot control
TinyKart *tinyKart;

// LiDAR
LD06 ld06{};

// Scan processor
ScanBuilder scan_builder{180, 360, ScanPoint{0.1524, 0}};

/// Starts/stops the kart
void estop() {
    logger.printf("Toggle Pause\n");

    tinyKart->toggle_pause();
    digitalToggle(LED_YELLOW);
}

void setup() {
    // LEDs
    pinMode(LED_GREEN, OUTPUT);
    pinMode(LED_RED, OUTPUT);
    pinMode(LED_YELLOW, OUTPUT);

    digitalWrite(LED_RED, HIGH);
    digitalWrite(LED_YELLOW, HIGH);

    // Setup blue user button on the board to stop the kart
    pinMode(USER_BTN, INPUT);
    attachInterrupt(digitalPinToInterrupt(USER_BTN), estop, FALLING);

    // Init PWM
    analogWriteResolution(PWM_BITS); // Range of 0-4096
    analogWriteFrequency(PWM_FREQ);

    // Prepare kart for motion
    ESC esc{THROTTLE_PIN, PWM_MAX_DUTY, PWM_FREQ};
    tinyKart = new TinyKart{STEERING_PIN, esc, 0.3, 4.5};

    // Init DMA and UART for LiDAR
    dmaSerialRx5.begin(230'400, [&](volatile LD06Buffer buffer) {
        // On each packet received, copy over to driver.
        ld06.add_buffer(buffer, 47);
    });

    digitalWrite(LED_RED, LOW);
    digitalWrite(LED_GREEN, LOW);
}

int status = 0;
auto target_pt;

void loop() {
    noInterrupts();
    auto res = ld06.get_scan();
    interrupts();

    // Check if we have a scan frame
    if (res) {
        auto scan_res = *res;

        // Check if frame erred
        if (scan_res) {
            auto maybe_scan = scan_builder.add_frame(scan_res.scan);

            // Check if we have a 180 degree scan built
            if (maybe_scan) {
                auto scan = *maybe_scan;

                //Scan is OK, begin driving logic

                auto front_obj_dist = scan[scan.size() / 2].dist(ScanPoint::zero());

                switch (status)
                {
                case 0: //STANDBY
                    if(front_obj_dist != 0.0 && front_obj_dist > .60)
                        {
                            tinyKart->set_steering(0);
                            digitalWrite(LED_RED, LOW);
                            tinyKart->set_forward(.20);
                            status = 1;
                            digitalWrite(LED_GREEN, HIGH);
                        }    
                    break;
                case 1://NO OBSTACLE; DRIVING STRAIGHT
                    if(front_obj_dist != 0.0 && front_obj_dist < 0.45 + 0.1524) //STOP at dist .45
                    {
                        logger.printf("Stopping because of object: %himm in front! \n", (int16_t) (front_obj_dist * 1000));
                        tinyKart->set_reverse(.17);
                        delay(100);
                        tinyKart->set_neutral();
                        status = 0;
                        digitalWrite(LED_RED, HIGH);
                        digitalWrite(LED_GREEN, LOW)
                    }
                    if(front_obj_dist != 0.0 && front_obj_dist < 1.0 && status == 1) //STEER AWAY at dist < 1
                    {
                        //find gap (scan to use, minimum gap size, minimum distance)
                        target_pt = find_gap_naive(scan,10,2);
                        status = 2;

                        //Old PoC steering. TODO: remove once new steering works
                        // tinyKart->set_steering(6/(front_obj_dist*front_obj_dist));
                    }
                    break;
                case 2: //OBSTACLE DETECTED; GAP FOUND; STEERING TOWARDS GAP
                    {
                        //TODO: Actually put shit here.
                        //Probably gonna just steal from the docs. No need to reinvent the wheel more than we already have
                        //In simple terms, set throttle and steering according to gap.
                        //ALSO need logic on when to find another gap and create another segment of steering.
                        //prbably gonna be a huge pain in the ass, but nothing lots of trial and error cant accomplish
                    }
                    break;
                default:
                    break;
                }
            }

        //Something has gone wrong with the scan
        } else {
            switch (scan_res.error) {
                case ScanResult::Error::CRCFail:
                    logger.printf("CRC error!\n");
                    break;

                case ScanResult::Error::HeaderByteWrong:
                    logger.printf("Header byte wrong!\n");
                    break;
            }
        }
    }
}