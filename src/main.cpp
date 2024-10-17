// Code as of May 2024
//boating with the baileys

#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <SPI.h>
#include "sensesp/signalk/signalk_output.h"
#include "sensesp/transforms/analogvoltage.h"
#include "sensesp/transforms/lambda_transform.h"
#include "sensesp/transforms/moving_average.h"
#include "sensesp/transforms/linear.h"
#include "sensesp_app_builder.h"

#define SDA 21
#define SCL 22

using namespace sensesp;

reactesp::ReactESP app;

Adafruit_ADS1115 ads;

const float FACTOR = 30; //30A/1V from the CT

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println(F("AC Monitor test"));

  SensESPAppBuilder builder;
  sensesp_app = (&builder)
                    ->set_hostname("sensesp-AC_monitor")
                    ->get_app();


  Wire.begin(SDA, SCL);

  ads.setGain(GAIN_FOUR);      // +/- 1.024V 1bit = 0.5mV
  ads.begin();  

  auto* analog_input_ads_01 = new RepeatSensor<float>(
    5, [](){ return ads.computeVolts(ads.readADC_Differential_0_1()) * FACTOR; });

  analog_input_ads_01
  ->connect_to(new LambdaTransform<float, float>([](float v){ return sq(v); }))
  ->connect_to(new MovingAverage(100))
  ->connect_to(new LambdaTransform<float, float>([](float v){ return sqrt(v); }))
  ->connect_to(
    new SKOutput<float>(
      "ac.current.amps",
      "/sensors/ADS1151/amps",
      new SKMetadata("A", "AC Amps")
    )
  )
  ->connect_to(new Linear(240.0, 0.0))
  ->connect_to(
    new SKOutput<float>(
      "ac.power.watts",
      "/sensors/ADS1151/watts",
      new SKMetadata("W", "AC Watts")
    )
  );

  sensesp_app->start();

}

void loop() {
  app.tick();
}
