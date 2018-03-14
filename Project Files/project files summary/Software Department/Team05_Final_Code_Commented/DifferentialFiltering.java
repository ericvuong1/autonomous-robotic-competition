import lejos.nxt.*;

public class DifferentialFiltering extends Thread {
	private ColorSensor ls;
	// store Light Sensor readings
	private int previous_reading = 0;
	private int current_reading = 0;
	// Threshold for difference in light sensor reading
	private int lightDifferenceLimit = 40;

	// constructor
	public DifferentialFiltering(ColorSensor ls) {
		this.ls = ls;

		// turn on the light (red)
		ls.setFloodlight(true);
	}

	//Method for detecting line
	public boolean lineDetection() {
		//register first light sensor reading
		previous_reading = ls.getNormalizedLightValue();
		
		//main loop for reading
		while (true) {
			
			//sleep sensor
			try {
				Thread.sleep(50);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
			current_reading = ls.getNormalizedLightValue();

			//line detected
			if (Math.abs(current_reading - previous_reading) > lightDifferenceLimit) {
				return true;
			}
			//no line detected
			previous_reading = current_reading;

			return false;
		}
	}
}
