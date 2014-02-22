package org.atorma.robot.simplebumper;

import java.util.HashMap;
import java.util.Map;

import javax.vecmath.Vector3d;

import org.atorma.robot.EpsilonGreedyPolicy;
import org.atorma.robot.PolicyIdMap;
import org.atorma.robot.SimbadAction;
import org.atorma.robot.SimbadRobot;
import org.atorma.robot.State;
import org.atorma.robot.discretization.IdFunction;

import simbad.sim.RangeSensorBelt;
import simbad.sim.RobotFactory;

public class Bumper extends SimbadRobot {
	
	private RangeSensorBelt ultrasonicSensor;
	
	private IdFunction stateIdMap = new BumperStateIdMap();
	
	private SimbadAction[] actions = new SimbadAction[] {new DriveForward(), new DriveBackward(), new TurnLeft(), new TurnRight()};
	private IdFunction actionIdMap = new BumperActionIdMap();
	private EpsilonGreedyPolicy epsilonGreedyPolicy = new EpsilonGreedyPolicy(actions, actionIdMap, 0.1);
	private Map<Integer, SimbadAction> actionMap;

	public Bumper(Vector3d startingPosition, String name) {
		super(startingPosition, name);
		
		ultrasonicSensor = RobotFactory.addSonarBeltSensor(this);
		
		actionMap = new HashMap<Integer, SimbadAction>(actions.length);
		for (SimbadAction nxtAction : actions) {
			actionMap.put(actionIdMap.getId(nxtAction.getValues()), nxtAction);
		}
	}

	@Override
	public State getCurrentState() {
		BumperState state = new BumperState((int) (ultrasonicSensor.getMeasurement(0)*100));
		return state;
	}
	
	@Override
	public SimbadAction getAction(State state) {
		int stateId = stateIdMap.getId(state.getValues());
		int actionId = epsilonGreedyPolicy.getActionId(stateId);
		return actionMap.get(actionId);
	}


	@Override
	public void updatePolicy(PolicyIdMap policy) {
		epsilonGreedyPolicy.setDeterministicPolicy(policy);
	}
	
	
	private class DriveForward implements SimbadAction {
		
		@Override
		public void perform() {
			Bumper.this.setTranslationalVelocity(0.5);
		}

		@Override
		public double[] getValues() {
			return BumperAction.FORWARD.getValues();  
		}

	}
	
	private class DriveBackward implements SimbadAction {

		@Override
		public void perform() {
			Bumper.this.setTranslationalVelocity(-0.5);
		}

		@Override
		public double[] getValues() {
			return BumperAction.BACKWARD.getValues();  
		}

	}
	
	private class TurnLeft implements SimbadAction {

		@Override
		public void perform() {
			Bumper.this.rotateY(15);
		}

		@Override
		public double[] getValues() {
			return BumperAction.LEFT.getValues();  
		}
		
	}
	
	private class TurnRight implements SimbadAction {

		@Override
		public void perform() {
			Bumper.this.rotateY(15);
		}

		@Override
		public double[] getValues() {
			return BumperAction.RIGHT.getValues();
		}

	}

}
