package org.atorma.robot.simplebumper;

import lejos.nxt.LightSensor;
import lejos.nxt.Motor;
import lejos.nxt.SensorPort;
import lejos.nxt.TouchSensor;
import lejos.nxt.UltrasonicSensor;
import lejos.robotics.navigation.DifferentialPilot;

import org.atorma.robot.NxtAction;
import org.atorma.robot.NxtRobot;
import org.atorma.robot.State;

public class NxtBumper extends NxtRobot {
	
	public static final double WHEEL_DIAMETER_CM = 5.6;
	
	private UltrasonicSensor ultrasonicSensor = new UltrasonicSensor(SensorPort.S3);
	private LightSensor lightSensor = new LightSensor(SensorPort.S1, false);
	private TouchSensor touchSensor = new TouchSensor(SensorPort.S4);
	private DifferentialPilot pilot = new DifferentialPilot(WHEEL_DIAMETER_CM, 18 - WHEEL_DIAMETER_CM, Motor.A, Motor.C, false);
	
	private NxtAction[] actions = new NxtAction[] {new DriveForward(), new DriveBackward(), new TurnLeft(), new TurnRight()};

	
	@Override
	public State getCurrentState() {
		BumperState state = new BumperState(ultrasonicSensor.getDistance(), touchSensor.isPressed(), lightSensor.readNormalizedValue());
		return state;
	}
	
	@Override
	public NxtAction getAction(int actionId) {
		return actions[actionId];
	}

	private class DriveForward implements NxtAction {
		
		@Override
		public void perform() {
			pilot.travel(WHEEL_DIAMETER_CM * 0.5);
		}

		@Override
		public int getId() {
			return BumperAction.FORWARD.getId();  
		}

	}
	
	private class DriveBackward implements NxtAction {

		@Override
		public void perform() {
			pilot.travel(-1 * WHEEL_DIAMETER_CM * 0.5);
		}

		@Override
		public int getId() {
			return BumperAction.BACKWARD.getId();  
		}

	}
	
	private class TurnLeft implements NxtAction {

		@Override
		public void perform() {
			pilot.rotate(15); // degrees
		}

		@Override
		public int getId() {
			return BumperAction.LEFT.getId();  
		}
		
	}
	
	private class TurnRight implements NxtAction {

		@Override
		public void perform() {
			pilot.rotate(-15);
		}

		@Override
		public int getId() {
			return BumperAction.RIGHT.getId();
		}

	}
	

	public static void main(String[] args) {
		NxtRobot robot = new NxtBumper();
		robot.run();
	}

	

	
}
