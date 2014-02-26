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
	public static final double WHEEL_WIDTH_CM = 2.6;
	public static final double TRACK_WIDTH_EDGE_TO_EDGE_CM = 19.3;
	
	private UltrasonicSensor ultrasonicSensor = new UltrasonicSensor(SensorPort.S3);
	private LightSensor lightSensor = new LightSensor(SensorPort.S1, false);
	private TouchSensor touchSensor = new TouchSensor(SensorPort.S4);
	private DifferentialPilot pilot = new DifferentialPilot(WHEEL_DIAMETER_CM, TRACK_WIDTH_EDGE_TO_EDGE_CM - WHEEL_WIDTH_CM , Motor.A, Motor.C, false);
	
	private NxtAction driveForward = new DriveForward();
	private NxtAction driveBackward = new DriveBackward();
	private NxtAction turnLeft = new TurnLeft();
	private NxtAction turnRight = new TurnRight();

	
	@Override
	public State getCurrentState() {
		BumperPercept state = new BumperPercept(ultrasonicSensor.getDistance(), touchSensor.isPressed(), lightSensor.readNormalizedValue());
		return state;
	}
	
	@Override
	public NxtAction getAction(int actionId) {
		BumperAction bumperAction = BumperAction.getAction(actionId);
		switch (bumperAction) {
		case FORWARD: return driveForward;
		case BACKWARD: return driveBackward;
		case LEFT: return turnLeft;
		case RIGHT: return turnRight;
		default: throw new IllegalArgumentException();
		}
	}

	private class DriveForward implements NxtAction {
		
		@Override
		public void perform() {
			pilot.travel(BumperAction.DRIVE_DISTANCE_CM); 
		}

	}
	
	private class DriveBackward implements NxtAction {

		@Override
		public void perform() {
			pilot.travel(-BumperAction.DRIVE_DISTANCE_CM);
		}

	}
	
	private class TurnLeft implements NxtAction {

		@Override
		public void perform() {
			pilot.rotate(BumperAction.TURN_DEGREES); 
		}
		
	}
	
	private class TurnRight implements NxtAction {

		@Override
		public void perform() {
			pilot.rotate(-BumperAction.TURN_DEGREES);
		}

	}
	

	public static void main(String[] args) {
		NxtRobot robot = new NxtBumper();
		robot.run();
	}

	

	
}
