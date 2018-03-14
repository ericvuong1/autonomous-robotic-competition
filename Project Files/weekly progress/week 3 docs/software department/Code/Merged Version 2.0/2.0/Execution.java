import lejos.nxt.*;

public class Execution {
	public static void main(String[] args)
	{
		Odometer odometer = new Odometer();
		Navigator navigator = new Navigator(odometer);
		Robot robot = new Robot(odometer);
        Shoot shoot = new Shoot();
		OdometryDisplay odometryDisplay = new OdometryDisplay(odometer);
		UltrasonicSensor us = new UltrasonicSensor(SensorPort.S1);
		ColorSensor ls = new ColorSensor(SensorPort.S3);
		DifferentialFiltering df = new DifferentialFiltering(ls);
		OdometryCorrection odometryCorrection = new OdometryCorrection(odometer,df);
		USLocalizer usl = new USLocalizer(odometer, us, robot);
		LightLocalizer lsl = new LightLocalizer(odometer, ls, df, robot);
		int buttonChoice;
		do {
			// clear the display
			LCD.clear();

			// ask the user whether the motors should drive in a square or float
			LCD.drawString("< Left | Right >", 0, 0);
			LCD.drawString("       |        ", 0, 1);
			LCD.drawString(" Run   |  Run   ", 0, 2);
			LCD.drawString(" One   |  Two   ", 0, 3);
			
			buttonChoice = Button.waitForAnyPress();
		}
		while(buttonChoice !=Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);
		
		// options
		
		if(buttonChoice == Button.ID_LEFT)
		{
				//selects run 1 
			odometer.start();
			odometryDisplay.start();
			navigator.start();
			navigator.turnTo(-Math.PI);
			navigator.turnTo(Math.PI/4);
			navigator.turnTo(Math.PI/2);
			}
			
		else if(buttonChoice == Button.ID_RIGHT)
				{
			
			//selects run 2 
			odometer.start();
			odometryDisplay.start();
			usl.doLocalization();
			navigator.setPath(
					0, 60.96,
					60.96, 0);
			navigator.start();
                    //LAUNCHER PART
                    /*
                     * navigator.travelTo(9.2, 10.6); navigator.turnTo(53.13); shoot.launch();
                     * navigator.turnTo(12.09); shoot.launch();
                     */
				}
		while(Button.waitForAnyPress() != Button.ID_ESCAPE)
			;
		System.exit(0);
			
		}
}


