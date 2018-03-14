import lejos.nxt.*;

public class Lab {

	public static void main(String[] args) {
		// setup the odometer, display, and ultrasonic and light sensors
		TwoWheeledRobot patBot = new TwoWheeledRobot(Motor.A, Motor.B);
		Odometer odo = new Odometer(patBot, true);
		OdometryCorrection odometryCorrection = new OdometryCorrection(odo);
		LCDInfo lcd = new LCDInfo(odo);
		UltrasonicSensor us = new UltrasonicSensor(SensorPort.S2);
		ColorSensor ls = new ColorSensor(SensorPort.S1);
		Button.waitForAnyPress();
		
		// perform the ultrasonic localization
		USLocalizer usl = new USLocalizer(odo, us, USLocalizer.LocalizationType.RISING_EDGE);
		usl.doLocalization();
		
		Button.waitForAnyPress();
		
		// perform the light sensor localization
		LightLocalizer lsl = new LightLocalizer(odo, ls);
		lsl.doLocalization();
		

		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		System.exit(0);
	}

}
