import lejos.nxt.*;

public class Execution {
	public static void main(String[] args) {
		Odometer odometer = new Odometer();
		//Navigator navigator = new Navigator();
		OdometryDisplay odometryDisplay = new OdometryDisplay(odometer);
		UltrasonicSensor us = new UltrasonicSensor(SensorPort.S1);
		ColorSensor ls = new ColorSensor(SensorPort.S3);
		DifferentialFiltering df = new DifferentialFiltering(ls);
		USLocalizer usl = new USLocalizer(odometer, us);
		LightLocalizer lsl = new LightLocalizer(odometer, ls, df);
		// OdometryCorrection OdometryCorrection = new
		// OdometryCorrection(odometer,df);

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
		} while (buttonChoice != Button.ID_LEFT
				&& buttonChoice != Button.ID_RIGHT);

		// options

		if (buttonChoice == Button.ID_LEFT) {
			// selects run 1
			odometer.start();
			odometryDisplay.start();

			/*
			 * usl.doLocalization(); Button.waitForAnyPress();
			 */

			lsl.doLocalization();

			/*
			 * navigator.setPath( 60.96, 30.48, 30.48, 30.48, 30.48, 60.96,
			 * 60.96, 0 ); navigator.start();
			 */

		}

		else if (buttonChoice == Button.ID_RIGHT) {
			odometer.start();
			odometryDisplay.start();

			// selects run 2
			odometer.getNavigator().setPath(0, 60.96, 60.96, 0);
			odometer.getNavigator().start();
		}
		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		System.exit(0);

	}
}
