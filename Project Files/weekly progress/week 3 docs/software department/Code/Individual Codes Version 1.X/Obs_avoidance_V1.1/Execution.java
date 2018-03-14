import lejos.nxt.*;

public class Execution {
	public static void main(String[] args)
	{
		Navigator navigator = new Navigator();
		Odometer odometer = new Odometer();
		OdometryDisplay odometryDisplay = new OdometryDisplay(odometer);
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
			navigator.setPath(
					60.96, 30.48,
					30.48, 30.48,
					30.48, 60.96,
					60.96, 0
					);
			navigator.start();
			odometer.start();
			odometryDisplay.start();
		}
			
		else if(buttonChoice == Button.ID_RIGHT)
				{
			
			//selects run 2 
			navigator.setPath(
					0, 60.96,
					60.96, 0);
			navigator.start();
			odometer.start();
			odometryDisplay.start();
				}
		while(Button.waitForAnyPress() != Button.ID_ESCAPE)
			;
		System.exit(0);
			
		}
}


