import lejos.nxt.*;

public class UltrasonicPoller extends Thread {
	private UltrasonicSensor frontSensor = new UltrasonicSensor(SensorPort.S1);
	private UltrasonicSensor sideSensor = new UltrasonicSensor(SensorPort.S2);
	public static int frontData, sideData;
	
	//collect ultrasonic sensor readings and store under variable "data"
	public void run()
	{
		while(true)
		{
			frontData = frontSensor.getDistance();
			sideData = sideSensor.getDistance();
			try{
				Thread.sleep(10);
			}
			catch(Exception e)
			{
				
			}
		}
	}
}