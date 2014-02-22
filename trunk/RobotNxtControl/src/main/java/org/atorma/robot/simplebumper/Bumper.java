package org.atorma.robot.simplebumper;

import java.util.HashMap;
import java.util.Map;

import lejos.nxt.Motor;
import lejos.nxt.SensorPort;
import lejos.nxt.UltrasonicSensor;
import lejos.robotics.navigation.DifferentialPilot;

import org.atorma.robot.EpsilonGreedyPolicy;
import org.atorma.robot.NxtAction;
import org.atorma.robot.NxtRobot;
import org.atorma.robot.PolicyIdMap;
import org.atorma.robot.State;
import org.atorma.robot.discretization.IdFunction;

public class Bumper extends NxtRobot {
	
	public static final double WHEEL_DIAMETER_CM = 5.6;
	
	private UltrasonicSensor ultrasonicSensor = new UltrasonicSensor(SensorPort.S4);
	private DifferentialPilot pilot = new DifferentialPilot(WHEEL_DIAMETER_CM, 18 - WHEEL_DIAMETER_CM, Motor.C, Motor.A, false);
	private IdFunction stateIdMap = new BumperStateIdMap();
	
	private NxtAction[] actions = new NxtAction[] {new DriveForward(), new DriveBackward(), new TurnLeft(), new TurnRight()};
	private IdFunction actionIdMap = new BumperActionIdMap();
	private EpsilonGreedyPolicy epsilonGreedyPolicy = new EpsilonGreedyPolicy(actions, actionIdMap, 0.1);
	private Map<Integer, NxtAction> nxtActionMap;
	

	public Bumper() {
		nxtActionMap = new HashMap<Integer, NxtAction>(actions.length);
		for (NxtAction nxtAction : actions) {
			nxtActionMap.put(actionIdMap.getId(nxtAction.getValues()), nxtAction);
		}
	}
	
	@Override
	public State getCurrentState() {
		BumperState state = new BumperState(ultrasonicSensor.getDistance());
		return state;
	}
	
	@Override
	public NxtAction getAction(State state) {
		int stateId = stateIdMap.getId(state.getValues());
		int actionId = epsilonGreedyPolicy.getActionId(stateId);
		return nxtActionMap.get(actionId);
	}


	@Override
	public void updatePolicy(PolicyIdMap policy) {
		epsilonGreedyPolicy.setDeterministicPolicy(policy);
	}
	
	
	private class DriveForward implements NxtAction {
		
		@Override
		public void perform() {
			pilot.travel(WHEEL_DIAMETER_CM * 0.5);
		}

		@Override
		public double[] getValues() {
			return BumperAction.FORWARD.getValues();  
		}

	}
	
	private class DriveBackward implements NxtAction {

		@Override
		public void perform() {
			pilot.travel(-1 * WHEEL_DIAMETER_CM * 0.5);
		}

		@Override
		public double[] getValues() {
			return BumperAction.BACKWARD.getValues();  
		}

	}
	
	private class TurnLeft implements NxtAction {

		@Override
		public void perform() {
			pilot.rotate(15); // degrees
		}

		@Override
		public double[] getValues() {
			return BumperAction.LEFT.getValues();  
		}
		
	}
	
	private class TurnRight implements NxtAction {

		@Override
		public void perform() {
			pilot.rotate(-15);
		}

		@Override
		public double[] getValues() {
			return BumperAction.RIGHT.getValues();
		}

	}
	

	public static void main(String[] args) {
		NxtRobot robot = new Bumper();
		robot.run();
	}

	

	
}
