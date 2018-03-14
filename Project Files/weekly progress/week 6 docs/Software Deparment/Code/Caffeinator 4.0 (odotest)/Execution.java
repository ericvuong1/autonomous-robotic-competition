import lejos.nxt.*;

public class Execution {
	public static void main(String[] args) {
		Odometer odometer = new Odometer();
		OdometryDisplay odometryDisplay = new OdometryDisplay(odometer);
		UltrasonicSensor frontUS = new UltrasonicSensor(SensorPort.S1);
		UltrasonicSensor leftUS = new UltrasonicSensor(SensorPort.S2);
		UltrasonicSensor rightUS = new UltrasonicSensor(SensorPort.S4);
		ColorSensor ls = new ColorSensor(SensorPort.S3);
		DifferentialFiltering df = new DifferentialFiltering(ls);
		Navigator navigator = new Navigator(odometer, frontUS, leftUS, rightUS);
		Robot robot = new Robot(odometer, frontUS, leftUS, rightUS);
		USLocalizer usl = new USLocalizer(odometer, frontUS, robot);
		LightLocalizer lsl = new LightLocalizer(odometer, df, robot);
		Shoot shoot = new Shoot();
		OdometryCorrection odoCorrect = new OdometryCorrection(odometer, df, ls);
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
			while (buttonChoice == Button.waitForAnyPress()) {
				shoot.launch();
			}
		}

		else if (buttonChoice == Button.ID_RIGHT) {

			// selects run 2
			odometer.start();
			odometryDisplay.start();
			odoCorrect.start();
			robot.start();
			robot.travelTo(5*30.48, 9*30.48, true);
			
			
//			usl.doLocalization();
//			lsl.doLocalization();
//			robot.travelTo(0, 0, false);
//			lsl.doLocalization();
//			robot.travelTo(0, 0, false);
//			robot.turnTo(Math.PI / 2);
//			Sound.beep();
//			robot.sleep(1000);
//			robot.travelTo(6 * 30.48, 6 * 30.48, true);
//			robot.turnTo(Math.toRadians(-50));
//			robot.sleep(1000);
//			usl.doLocalization();
//			lsl.doLocalization();
//			robot.travelTo(0, 0, false);
//			lsl.doLocalization();
//			robot.turnTo(Math.toRadians(225));
//			robot.driveDist(-31.5);
//			robot.turn(-5);
//
//			shoot.launch();
//			robot.sleep(1000);
//			shoot.launch();
//			robot.sleep(1000);
//			shoot.launch();

		}
		while (Button.waitForAnyPress() != Button.ID_ESCAPE)
			;
		System.exit(0);

	}
}
