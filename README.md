# PlantingSetup  
3/30/21: Attempt reading 0-5V out of Hetronic with analogRead on ArduinoUno.  
4/5/21: Attempt reading 0-5V out with [INA219](https://www.amazon.com/dp/B08GZ7TVDD/ref=cm_sw_em_r_mt_dp_APBJY67PNXEXGWCZ6ACT?_encoding=UTF8&psc=1)  
Both of these attempts yield random results. It will read 10-12V for a few secounds after setting speed to 25% than no voltage reading. Hetronic output seems to be working just fine though. testing voltage on wire #12 with regular voltmeter yields expectted results, not sure what device I need to read it with my microcontroller. The hetronic doc shows #12 is just a regular PWM output, so I'm not sure why I'm having difficulty reading this with my device.  
[BoomCont2IGXMotor_Hetronic.ino](https://github.com/lsfarm/PlantingSetup/blob/main/BoomCont2IGXMotor_Hetronic.ino) shows how I have been controlling a [single SPI Digital POT](https://github.com/lsfarm/PlantingSetup/blob/main/microchipdigpot.pdf)  
