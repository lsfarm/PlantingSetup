# PlantingSetup  
3/30/21: Attempt reading 0-5V out of Hetronic with analogRead on ArduinoUno.  
4/5/21: Attempt reading 0-5V out with [INA219](https://www.amazon.com/dp/B08GZ7TVDD/ref=cm_sw_em_r_mt_dp_APBJY67PNXEXGWCZ6ACT?_encoding=UTF8&psc=1)  
Both of these attempts yield random results. It will read 10-12V for a few secounds after setting speed to 25% than no voltage reading. Hetronic output seems to be working just fine though. testing voltage on wire #12 with regular voltmeter yields expectted results, not sure what device I need to read it with my microcontroller. The hetronic doc shows #12 is just a regular PWM output, so I'm not sure why I'm having difficulty reading this with my device.  
[BoomCont2IGXMotor_Hetronic.ino](https://github.com/lsfarm/PlantingSetup/blob/main/BoomCont2IGXMotor_Hetronic.ino) shows how I have been controlling a [single SPI Digital POT](https://github.com/lsfarm/PlantingSetup/blob/main/microchipdigpot.pdf) and [wireup](https://github.com/lsfarm/PlantingSetup/blob/main/wireup.jpg) shows the diagram I've been using to connect this up. With the exception that pins 5,6&7 on the potentimeter are tied to the honda motor only.  
Also in this repo are the [TechManual](https://github.com/lsfarm/PlantingSetup/blob/main/igx390_tech_manual.pdf) and the [HondaECM diagram](https://github.com/lsfarm/PlantingSetup/blob/main/hondaecm_z8ra%20circuit.pdf)  
4/14/21: RPM INPUT ISSUE SOLVED -- I was able to read the pulses coming from wire #12 after installing a voltage divider. Hetronic output is 12V - my uC has an input limit of 5V. -- pulseIn 0 = 0% pulseIn 1000 = 25% pulseIn 2000 = 50% pulseIn 3000 = 75% pulseIn 4000 = 100%rpm  


4-15-21 the RPM input device from Design Tech is not reliable. If we could get this device working all the problems listed here would be solved. But I'm running out of time to mess with this device. I need to get the hetronic device setup where it acts like the rpm input device doesn't even exist. Anything above an idle cause Comm Error on hetronic TX. I'd like to leave the rpm input connected, but the comm error may get so annoying that I end up disconnecting rpm input altogether.  

Hetronic Re-programming  
- Starting motor: since we can't rely on rpm feedback for bump start, only hold starter on for 1 sec instead of 5.  
- When RPM feedback drops out motor goes back to idle and sometimes shuts off. Set this where RPM setpoint is allways whatever TX commands -- no logic to go back to idle -- Steps: Start motor, rpm shows 1,800, command 25%RPM on TX, RPM shows 2,300, command 50%RPM on TX, rpm begins to drop out shows 0-2,500, when RPM feedback stays on 0, TX automaticly commands 0%rpm and sometimes even shuts off the motor. I'll try making a video of this issue.  
- Allow boom valve to be opened and closed any time TX commands it -- right now its only possible to open it when motor is running -- it needs to stay open after motor shuts off to drain the boom.  
