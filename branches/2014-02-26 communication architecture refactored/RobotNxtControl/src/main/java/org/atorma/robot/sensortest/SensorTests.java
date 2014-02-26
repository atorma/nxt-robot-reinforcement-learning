package org.atorma.robot.sensortest;

import lejos.nxt.Button;
import lejos.nxt.LightSensor;
import lejos.nxt.SensorPort;
import lejos.nxt.TouchSensor;
import lejos.nxt.UltrasonicSensor;

import org.atorma.robot.State;
import org.atorma.robot.communications.NxtToPcCommunications;

public class SensorTests {
	
	
	private NxtToPcCommunications comms = new NxtToPcCommunications();

	
	public void transmitSensorValuesReceiveActionId() {
		
		UltrasonicSensor ultrasonicSensor = new UltrasonicSensor(SensorPort.S3);
		TouchSensor touchSensor = new TouchSensor(SensorPort.S4);
		LightSensor lightSensor = new LightSensor(SensorPort.S1, false);
		
		while (Button.ENTER.isUp()) {
			int actionId = comms.receiveActionIdForCurrentState(new TestState(new double[] {
					ultrasonicSensor.getDistance(),
					touchSensor.isPressed() ? 1 : 0,
					lightSensor.getLightValue()		
			}));
			System.out.println("Action id " + actionId);
		}
		comms.disconnect();
		
	}
	
	public static void main(String[] args) {
		
		new SensorTests().transmitSensorValuesReceiveActionId();
		
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
