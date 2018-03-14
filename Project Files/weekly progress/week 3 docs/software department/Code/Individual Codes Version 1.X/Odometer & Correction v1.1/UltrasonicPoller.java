package odotest;
import lejos.nxt.*;

public class UltrasonicPoller extends Thread {
	private UltrasonicSensor us = new UltrasonicSensor(SensorPort.S1);
	public static int data;
	
	//collect ultrasonic sensor readings and store under variable "data"
	public void run()
	{
		while(true)
		{
			data = us.getDistance();
			try{
				Thread.sleep(10);
			}
			catch(Exception e)
			{
				
			}
		}
	}
}