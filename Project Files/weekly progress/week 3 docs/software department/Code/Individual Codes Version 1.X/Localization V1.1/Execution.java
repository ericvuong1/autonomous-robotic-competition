import lejos.nxt.*;

public class Execution {
	public static void main(String[] args)
	{
		Odometer odometer = new Odometer();
		
		//OdometryCorrection OdometryCorrection = new OdometryCorrection(odometer);
		OdometryDisplay odometryDisplay = new OdometryDisplay(odometer);
		UltrasonicSensor us = new UltrasonicSensor(SensorPort.S1);
		ColorSensor ls = new ColorSensor(SensorPort.S3);
		
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
			USLocalizer usl = new USLocalizer(odometer, us);
			usl.doLocalization();
			Button.waitForAnyPress();
			
			/*odometer.getNavigator().setPath(
					60.96, 30.48,
					30.48, 30.48,
					30.48, 60.96,
					60.96, 0
					);
			odometer.getNavigator().start();*/

		}
			
		else if(buttonChoice == Button.ID_RIGHT)
				{
			
			//selects run 2 
			odometer.getNavigator().setPath(
					0, 60.96,
					60.96, 0);
			odometer.getNavigator().start();
			odometer.start();
			odometryDisplay.start();
				}
		while(Button.waitForAnyPress() != Button.ID_ESCAPE)
			;
		System.exit(0);
			
		}
}


