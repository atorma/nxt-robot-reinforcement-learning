package org.atorma.robot.communications;

import lejos.nxt.Button;
import lejos.nxt.SensorPort;
import lejos.nxt.UltrasonicSensor;

import org.atorma.robot.State;

public class NxtCommTests {
	
	
	private NxtToPcCommunications comm = new NxtToPcCommunications();


	public void transmit_arrays() {
		while (Button.ENTER.isUp()) {
			TestState state = new TestState(new double[] {7, 0});
		
			int actionId = comm.receiveActionIdForCurrentState(state);
		}
		comm.disconnect();
	}
	
	
	public void run_sensor_test() {
		
		final UltrasonicSensor ultrasonicSensor = new UltrasonicSensor(SensorPort.S4);
		
		while (Button.ENTER.isUp()) {
			int actionId = comm.receiveActionIdForCurrentState(new TestState(new double[] {ultrasonicSensor.getDistance()}));
			System.out.println("Action id " + actionId);
		}
		comm.disconnect();
		
	}
	
	// Testing
	public static void main(String[] args) {
		
		new NxtCommTests().run_sensor_test();
		
	}
	
	private class TestState implements State {
		
		private double[] values;

		private TestState(double[] values) {
			this.values = values;
		}

		@Override
		public double[] getValues() {
			return values;
		}
		
	}
	
}
