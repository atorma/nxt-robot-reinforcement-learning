package org.atorma.robot.communications;

import java.util.Random;

import lejos.nxt.Button;
import lejos.nxt.SensorPort;
import lejos.nxt.UltrasonicSensor;

import org.atorma.robot.NxtAction;
import org.atorma.robot.State;
import org.atorma.robot.util.Ranmar;

public class NxtCommTests {
	
	
	private NxtToPcCommunications comm = new NxtToPcCommunications();


	public void transmit_arrays() {
		while (Button.ENTER.isUp()) {
			TestState state = new TestState(new double[] {7, 0});
			TestAction action = new TestAction(new double[] {3});
		
			comm.pushStateAndAction(state, action);
		}
		comm.disconnect();
	}
	
	
	public void run_sensor_test() {
		
		final UltrasonicSensor ultrasonicSensor = new UltrasonicSensor(SensorPort.S4);
		NxtAction action = new RandomAction();
		
		while (Button.ENTER.isUp()) {
			comm.pushStateAndAction(new TestState(new double[] {ultrasonicSensor.getDistance()}), new TestAction(new double[] {3}));
		}
		comm.disconnect();
		
	}
	
	public void test_and_transmit_random_integers() {
		Ranmar random = new Ranmar();

		while (Button.ENTER.isUp()) {
			TestState state = new TestState(new double[] {0});
			int randomInt = random.nextInt(4);
			System.out.println(randomInt);
			TestAction action = new TestAction(new double[] {randomInt});
		
			comm.pushStateAndAction(state, action);
			
			try {
				Thread.sleep(1000);
			} catch (InterruptedException e) {}
		}
		comm.disconnect();
	}
	

	// Testing
	public static void main(String[] args) {
		
		new NxtCommTests().test_and_transmit_random_integers();
		
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
	
	private class TestAction implements NxtAction {
		
		private double[] values;

		private TestAction(double[] values) {
			this.values = values;
		}

		@Override
		public double[] getValues() {
			return this.values;
		}

		@Override
		public void perform() {
			
		}	
	}
	
	
	private class RandomAction implements NxtAction {
		
		private Random random = new Random();

		@Override
		public double[] getValues() {
			return new double[] {random.nextInt(10), random.nextInt(10)};
		}

		@Override
		public void perform() {
			
		}
		
	}
}
