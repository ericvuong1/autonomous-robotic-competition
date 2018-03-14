/*
 OdometryCorrection.java
 */
import lejos.nxt.*;
import lejos.robotics.Color;

public class OdometryCorrection extends Thread {
	private static final long CORRECTION_PERIOD = 10;
	private Odometer odometer;
	private DifferentialFiltering df;

	// distance between center of rotation and light sensor
	double lightToCenterDist = 12;

	// first grid line starting at 15;
	public int StartingGridLine = 0;

	// constructor
	public OdometryCorrection(Odometer odometer, DifferentialFiltering df) {
		this.odometer = odometer;
		this.df = df;
	}

	// run method (required for Thread)
	public void run() {
		long correctionStart, correctionEnd;

		while (!LightLocalizer.isLocalizing && !USLocalizer.isLocalizing) {
			correctionStart = System.currentTimeMillis();

			double AdjustedX;
			double AdjustedY;

			if (df.lineDetection() == true) {
				double X = odometer.getX();
				double Y = odometer.getY();
				double theta = odometer.getAngle();

				// calculate light sensor position based on robot center
				double sensorPositionX = X - lightToCenterDist
						* Math.cos(theta * Math.PI / 180);
				double sensorPositionY = Y - lightToCenterDist
						* Math.sin(theta * Math.PI / 180);

				// find closest grid line based on position of light sensor
				long iX = Math.round((sensorPositionX - StartingGridLine) / 30);
				long iY = Math.round((sensorPositionY - StartingGridLine) / 30);
				double nearestGridLineX = 30 * iX + StartingGridLine;
				double nearestGridLineY = 30 * iY + StartingGridLine;

				double ErrorX = sensorPositionX - nearestGridLineX;
				double ErrorY = sensorPositionY - nearestGridLineY;

				// to choose which coordinate to correct
					if (Math.abs(ErrorX) < Math.abs(ErrorY)) {
						Sound.beep();
						sensorPositionX = nearestGridLineX;
						AdjustedX = sensorPositionX + lightToCenterDist
								* Math.cos(theta * Math.PI / 180);
						odometer.setX(AdjustedX);
					} else if (Math.abs(ErrorX) > Math.abs(ErrorY)) {
						Sound.beep();
						sensorPositionY = nearestGridLineY;
						AdjustedY = sensorPositionY + lightToCenterDist
								* Math.sin(theta * Math.PI / 180);
						odometer.setY(AdjustedY);
					}
				
				// to prevent light sensor from reading a line too many
				// times
				try {
					Thread.sleep(400);
				} catch (InterruptedException e) {
					e.printStackTrace();
				}

			}

			// this ensure the odometry correction occurs only once every
			// period
			correctionEnd = System.currentTimeMillis();
			if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
				try {
					Thread.sleep(CORRECTION_PERIOD
							- (correctionEnd - correctionStart));
				} catch (InterruptedException e) {
					// there is nothing to be done here because it is not
					// expected that the odometry correction will be
					// interrupted by another thread
				}
			}
		}
	}
}
