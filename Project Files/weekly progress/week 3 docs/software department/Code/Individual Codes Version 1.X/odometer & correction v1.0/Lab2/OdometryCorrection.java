package Lab2;
/* 

 * OdometryCorrection.java
 */
import lejos.nxt.ColorSensor.Color;
import lejos.nxt.*;

import java.util.*;

import Lab3.Odometer;

public class OdometryCorrection extends Thread {
	private static final long CORRECTION_PERIOD = 10;
	private Odometer odometer;

	// constructor
	public OdometryCorrection(Odometer odometer) {
		this.odometer = odometer;
	}

	// run method (required for Thread)
	public void run() {
		long correctionStart, correctionEnd;
		
		//enable the light sensor
				ColorSensor LS = new ColorSensor(SensorPort.S2);
				LS.setFloodlight(Color.RED);
				
		while (true) {
			correctionStart = System.currentTimeMillis();

			// put your correction code here
						double xls;
						double yls;
						//distance from the sensor to the center of the robot
						double sensordistance = 12;
						double xError;
						double yError;
						//detected value from light sensor
						int Light = LS.getLightValue();
						//threshold value to detect the grid
						int GridIndicator = 40;
						//x & y coordinate of the sensor
			xls = odometer.getX() - sensordistance * Math.sin(odometer.getTheta()*Math.PI/180);
			yls = odometer.getY() - sensordistance * Math.cos(odometer.getTheta()*Math.PI/180);
			
			if(Light < GridIndicator){
				//beep whenever the sensor detects the grid
				Sound.setVolume(100);
				Sound.beep();
				synchronized (odometer.lock) {
				
				xError = (xls %30) - 15;
				yError = (yls %30) - 15;
				
				//adjust the x or y coordinate according to the smaller error
				if((Math.abs(xError) > Math.abs(yError)) && (Math.abs(yError)<=1)){
					odometer.setY(odometer.getY() - yError);
				}else if((Math.abs(yError) > Math.abs(xError)) && (Math.abs(xError)<=1)){
					odometer.setX(odometer.getX() - xError);
				}
				}
				
				
			}
			// this ensure the odometry correction occurs only once every period
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