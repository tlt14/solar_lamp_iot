#pragma once

// #ifndef ARDUINO_OTA_H
#define ARDUINO_OTA_H

inline void arduinoOtaSetup()
{
    ArduinoOTA.setHostname("solar_light");

  // No authentication by default
   ArduinoOTA.setPassword("admin");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");
    ArduinoOTA.onStart([]()
                    {
                        String type;
                        if (ArduinoOTA.getCommand() == U_FLASH)
                        {
                            type = "sketch";
                        }
                        else
                        { // U_SPIFFS
                            type = "filesystem";
                        }

                        // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
                        Serial.println("Start updating " + type);
                    });

    ArduinoOTA.onEnd([]()
                        { Serial.println("\nEnd of update"); });

    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total)
                            { Serial.printf("Progress: %u%%\r", (progress / (total / 100))); });

    ArduinoOTA.onError([](ota_error_t error)
                        {
                            Serial.printf("Error[%u]: ", error);
                            if (error == OTA_AUTH_ERROR)
                            {
                                Serial.println("Auth Failed");
                            }
                            else if (error == OTA_BEGIN_ERROR)
                            {
                                Serial.println("Begin Failed");
                            }
                            else if (error == OTA_CONNECT_ERROR)
                            {
                                Serial.println("Connect Failed");
                            }
                            else if (error == OTA_RECEIVE_ERROR)
                            {
                                Serial.println("Receive Failed");
                            }
                            else if (error == OTA_END_ERROR)
                            {
                                Serial.println("End Failed");
                            }
                        });

    ArduinoOTA.begin();
}