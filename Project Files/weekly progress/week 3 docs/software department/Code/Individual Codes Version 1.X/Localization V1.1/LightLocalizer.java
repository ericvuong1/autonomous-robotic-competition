import lejos.nxt.*;

public class LightLocalizer {
	private Odometer odo;
	private ColorSensor ls;
	// setup array to store each line with count from 0
	private int count;
	private double lineAngle[];

	private int dToCenter = 12;

	public LightLocalizer(Odometer odo, ColorSensor ls) {
		this.odo = odo;
		this.ls = ls;
		lineAngle = new double[4];

		// turn on the light
		ls.setFloodlight(true);
	}

	public void doLocalization() {
		// drive to location listed in tutorial
		odo.getNavigator().travelTo(10, 10);

		
		// start rotating and clock all 4 grid lines
		odo.getNavigator().setRotationSpeed(-40);
		while (count < 4) {
			if (ls.getNormalizedLightValue() < 360) {
				lineAngle[count] = odo.getTheta();
				count++;
				Sound.beep();
				try {
					Thread.sleep(200);
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
			}
		}
		Motor.A.stop();
		Motor.B.stop();

		// do trig to compute (0,0) and 0 degrees
		double thetaY = lineAngle[0] - lineAngle[2];
		double thetaX = lineAngle[1] - lineAngle[3];
		double x = -dToCenter * Math.cos(Math.toRadians(thetaY / 2));
		double y = -dToCenter * Math.cos(Math.toRadians(thetaX / 2));
		double dTheta = 180 - (lineAngle[1] + lineAngle[3]) / 2;
		double theta = odo.getTheta() + dTheta;
		odo.setPosition(new double[] { x, y, theta },new boolean[] { true, true, true });

		// when done, travel to (0,0) and turn to 0 degrees
		Button.waitForAnyPress();

		odo.getNavigator().travelTo(0, 0);
		odo.getNavigator().turn(-odo.getTheta());
	}
}

