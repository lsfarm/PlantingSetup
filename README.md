# PlantingSetup  
3/30/21: Attempt reading 0-5V out of Hetronic with analogRead on ArduinoUno.  
4/5/21: Attempt reading 0-5V out with [INA219](https://www.amazon.com/dp/B08GZ7TVDD/ref=cm_sw_em_r_mt_dp_APBJY67PNXEXGWCZ6ACT?_encoding=UTF8&psc=1)  
Both of these attempts yield random results. It will read 10-12V for a few secounds after setting speed to 25% than no voltage reading. Hetronic output seems to be working just fine though. testing voltage on wire #12 yields expectted results, not sure what device I need to read it with my microcontroller.  
[BoomCont2IGXMotor_Hetronic.ino](https://github.com/lsfarm/PlantingSetup/blob/main/BoomCont2IGXMotor_Hetronic.ino) shows how I have been controlling 
