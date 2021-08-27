# Health Care - **Continuous Glucose Monitoring**  #

## Overview ##

This project shows an example of **Continuous Glucose Monitoring**  using the **Silicon Labs demo board.**

The demo board captured ADC data and use Bluetooth Continuous Glucose Monitoring Profile to transmit all the data.

## Simplicity Studio and Gecko SDK Suite version ##

Simplicity Studio V5.2.1.1 and GSDK v3.2.1

## Hardware Required ##

- EFR32BG22C112F352GM32 Silicon Labs demo board

## Software Required

LightBlue app/EFR connect app


## How It Works ##

### setup

1. get a silicon labs demo board, connect it with WSTK
2. git clone  https://github.com/mattji01/silabs-CGM
3. flash bootloader.s37 and bluetooth_cgm_iadc.s37 in folder SimplicityStudio
4. if the phone bonded with the device before, 
    please remove the device from the bluetooth setting in the phone

### Test

open EFR connect app, filter silabs-CGM, connect it.

there are two major service, Glucose and the IADC service, the IADC service UUID is d29b8a73-cb31-4521-96a3-002037cdf2c3

<img src=".\images\service.jpg" style="zoom:67%;" />

##### Get IADC data

tap More Info under the Unknown Service(IADC service), there are 7 characteristics in this service

1. internal ADC result(mV), UUID 94ec4ab6-0e78-45c7-ba7b-e4d85475b66e

   ![](.\images\internal_adc.jpg)

2. external ADC result(mV), UUID b1410159-92e4-47d7-9cb0-52ccabb1b8d8

   ![](.\images\external_adc.jpg)

3. DAC adjust, set DAC output voltage level, in range of 0-1.25v, UUID b29b0be9-b008-4ec5-953b-3cd6208654a2, 

   write 0.625v to DAC

   <img src=".\images\set_dac.jpg" style="zoom: 80%;" />

4. Die Temperature, UUID d211bc91-da0d-4198-8665-8967929574ba, read chipset die temperature

   ![](.\images\temperature.jpg)

5. offset of the internal ADC(mV), UUID 8cd026a3-143a-425c-ad6d-f16ff09d9eff, 

   ![](.\images\offset.jpg)

6. Gain error of the internal ADC, UUID 1f61ad1f-742a-4790-adca-8f7a7cd9e42c

   ![](.\images\gain_error.jpg)

##### Get Glucose data

1. the device have no record inside by default, 
2. open lightblue app, connect silabs-CGM, and accept the pairing request
3. find service Continuous Glucose Monitoring
4. set Notify of 0x2AA7 characteristic(**CGM Measurement**)
5. set indicate of 0x2AAC characteristic(**CGM Specific Ops Control Point**),
6. write "1A" to 0x2AAC, it means start session, you can find this test case in CGMS.TS.p6: CGMS/SEN/CGMCP/BV-22-C [CGM Specific Ops – ‘Start Session’

![](.\images\start_session.jpg)

6. there will be 1 indication in 0x2AAC to show that start session success.

<img src=".\images\succeed.jpg" style="zoom:67%;" />





5. the sensor(Silicon lab demo board) will continuous send notifications to 0x2AA7 characteristic until write 0x1B to 0x2AAC(stop session)

<img src=".\images\notifications.jpg" style="zoom:67%;" />



# Create an example application #

1. create a soc ecmpty project and rename it to bluetooth_cgm_iadc
2. add iostream usart(vcom) component
3. add log component
4. copy and overwrite gatt_configuration.btconf to projetc-location\config\btconf
6. comment sl_device_init_lfxo() this line in sl_event_handler.c in autogen folder
7. copy and overwrite sl_bluetooth.c to project-location\auto-gen
8. copy all cgm* files to project-location\ 

## .sls Projects Used ##

bluetooth_cgm_iadc.sls - This is the project. 

Also precompiled binaries in S-Record format (.s37) are included for the projects above test the applications instantly. The files can be programmed using for example _Simplicity Studio Flash Programmer_ tool or _Simplicity Commander_ application. Remember to flash also the bootloader at least once.

# More information #

## PTS test ##

You can download CGM spec and test case in https://www.bluetooth.com/specifications/specs/, or in doc folder in this repository.

## Pending test case ##

CGMS/SEN/CGMCP/BV-06-C [CGM Specific Ops – ‘Get Glucose Calibration Value’ Type 1]

