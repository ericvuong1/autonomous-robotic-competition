import lejos.nxt.*;

public class DifferentialFiltering extends Thread {
	private ColorSensor ls;
	private int previous_reading = 0;
	private int current_reading = 0;

	public DifferentialFiltering(ColorSensor ls) {
		this.ls = ls;
	}

	public boolean lineDetection() {	
		previous_reading = ls.getNormalizedLightValue();
		while (true) {

			try {
				Thread.sleep(50);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			current_reading = ls.getNormalizedLightValue();

			if (current_reading - previous_reading > 50) {
				return true;
			}
			
			previous_reading = current_reading;

			return false;
		}
	}
}
