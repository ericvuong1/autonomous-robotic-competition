import lejos.nxt.*;

public class LightLocalizer {
	private Odometer odo;
	private ColorSensor ls;
	private DifferentialFiltering df;
	// setup array to store each line with count from 0
	private double lineAngle[];
	private Robot robot;

	private int dToCenter = 12;

	public LightLocalizer(Odometer odo, ColorSensor ls, DifferentialFiltering df, Robot robot) {
		this.odo = odo;
		this.ls = ls;
		this.df = df;
		this.robot = robot;
		lineAngle = new double[4];

		// turn on the light
		ls.setFloodlight(true);
	}

	public void doLocalization() {
		// drive to location listed in tutorial
		int count = 0;
		// start rotating and clock all 4 grid lines
		robot.setRotationSpeed(-40);
		while (count < 4) {
			if (df.lineDetection()) {
				lineAngle[count] = odo.getAngle();
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
		Motor.C.stop();

		// do trig to compute (0,0) and 0 degrees
		double thetaY = lineAngle[0] - lineAngle[2];
		double thetaX = lineAngle[3] - lineAngle[1];
		double x = -dToCenter * Math.cos(Math.toRadians(thetaY / 2));
		double y = -dToCenter * Math.cos(Math.toRadians(thetaX / 2));
		double dTheta = 270 - (lineAngle[1] + lineAngle[3]) / 2;
		double theta = odo.getAngle() + dTheta;
		odo.setPosition(new double[] { x, y, (theta)*Math.PI/180 },new boolean[] { true, true, true });

		// when done, travel to (0,0) and turn to 0 degrees
		Button.waitForAnyPress();

		robot.travelTo(0, 0);
		robot.turn(90 - odo.getAngle());

	}
}

