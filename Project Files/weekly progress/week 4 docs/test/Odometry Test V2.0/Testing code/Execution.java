import lejos.nxt.*;

public class Execution {
	public static void main(String[] args) {
		Odometer odometer = new Odometer();
		OdometryDisplay odometryDisplay = new OdometryDisplay(odometer);
		UltrasonicSensor frontUS = new UltrasonicSensor(SensorPort.S1);
		UltrasonicSensor sideUS = new UltrasonicSensor(SensorPort.S2);
		ColorSensor ls = new ColorSensor(SensorPort.S3);
		DifferentialFiltering df = new DifferentialFiltering(ls);
		Navigator navigator = new Navigator(odometer, frontUS, sideUS);
		Robot robot = new Robot(odometer, frontUS, sideUS);
		OdometryCorrection odometryCorrection = new OdometryCorrection(
				odometer, df, ls);
		// USLocalizer usl = new USLocalizer(odometer, us, robot);
		// LightLocalizer lsl = new LightLocalizer(odometer, ls, df, robot);
		Shoot shoot = new Shoot();
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
			navigator.start();
			navigator.turnTo(-Math.PI);
			navigator.turnTo(Math.PI / 4);
			navigator.turnTo(Math.PI / 2);
		} else if (buttonChoice == Button.ID_RIGHT) {
			// selects run 2
			odometer.start();
			odometryDisplay.start();
			odometryCorrection.start();
			// navigator.setPath(
			// 0, 60.96,
			// 60.96, 0);
			// navigator.start();
			robot.start();
			//robot.travelTo(0, 60);
			robot.travelTo(60, 30);
			robot.travelTo(75, 75);
			// Button.waitForAnyPress();
			robot.sleep(1000);
			shoot.launch();
			Button.waitForAnyPress();
		}
		while (Button.waitForAnyPress() != Button.ID_ESCAPE)
			;
		System.exit(0);
	}
}
