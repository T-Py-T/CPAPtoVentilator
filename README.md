Last updated 2020/04/19

# Low-Cost Open-Source Ventilator-ish Device

NOTE: This is currently an independent project not affiliated with any commercial institution.

* In the event that COVID-19 hospitalizations exhaust the availability of FDA approved ventilators.  This project documents the process of converting a low-cost CPAP (Continuous Positive Airway Pressure) blower into a rudimentary non-invasive pressure support ventilator that could help with breathing during respiratory distress.  It's an evolving project, but in it's current form, it most aligned with the definition of a non-invasive pressure support BiPAP ventilator.

* You may be able to save many more lives by building a PAPR that protects a caregiver than a mediocre ventilator for one patient. Without participating in the detailed engineering and medical discussions regarding mitigating the risks of high pressure ventilation, do not attempt this project. 

### WARNING/DISCLAIMER: Whenever possible, please seek professional medical care with proper equipment setup by trained individuals. Do not use random information you found on the internet. I am not a medical professional, just a random person putting information on the internet. There are significant risks associated with using a high pressure BiPAP as a DIY ventilator without medical supervision.  

### What are the primary risks of a DIY ventilator?
* Being viewed as a viable alternative to available professional care or delaying seeking professional care.  This should be viewed as an educational resource or a last resort option only.  Community driven engineering efforts like this run the risk of making it "too easy to be harmful" which would be a bad outcome.

* Using non-invasive interfaces (such as masks and mouth pieces) can create aerosolized virus infecting many others in clean environments.  Infecing yourself or other healthy individuals multiplies the problem.  Usage should be restricted to places where aerosolized viruses are already abundant (lots of unmasked symptomatic individuals), COVID wards, or in isolation. <strong>NOTE: Self-quarantine in an apartment or multi-unit residence is NOT proper isolation due to potential shared ventilation and sewage.</strong>

* Giving too high a pressure (such as 20cmh2o and up) without observing lung compliance can cause rupture of infected lung tissue.  This design currently lacks several features that provide enough feedback to a caregiver to monitor that risk.

# External Images
## TOP VIEW
![Front Image](Images\TopView.png) 
## BACK VIEW
![Back Image](Images\BackView.png) 
### Item Description
1. System MODE button
2. Rotary dial & button for user input
3. System MENU button
4. Fault Indicator Light
5. System Display
6. Ventilation Hose Port – connect to patient breathing circuit
7. Flow sensing port – connect to patient breathing circuit
8. Power supply connection (120V)


# User Interface
![Front Image](Images\UserInterface.png) 
## Description
1. Current System Mode indicator
2. Peak inspiratory pressure setpoint indicator
3. Average tidal pressure indicator
4. PEEP setpoint indicator
5. Breathing rate setpoint indicator
6. I:E ratio setpoint indicator
7. Realtime Pressure plot
8. Realtime Pressure indicator
9. Realtime Flow plot
10. Realtime Flow indicator
11. Realtime Volume plot
12. Realtime Volume indicator

# Ventilation settings menu
![Front Image](Images\VentilationSettings.png) 
## Description
1. Mode indicator
2. Menu selection indicator
3. Menu selection item description
4. Menu selection current parameter value
5. Menu selection parameter unit(s), if applicable

### Device setting - Units - Description
Ppeak - cmH2O - Peak inspiratory pressure
PEEP - cmH2O - Positive end expiratory pressure
f total - b/min - Breathing rate
I:E	- ratio - Ratio of inspiratory time to expiratory time
Interface - N/A - Patient interface selection – MASK or ET
Mode Select - N/A - Set machine mode to Pressure Assist or Spontaneous
Inhale Trigger - l/min - Inhale Flow sensitivity – Spontaneous Mode

## Ventilator Fault display
![Front Image](Images\AlarmExample.png) 
 
 
