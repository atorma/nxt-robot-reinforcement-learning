package org.atorma.robot.objecttrackingbumper.prioritizedsweeping;

import static org.junit.Assert.*;

import org.atorma.robot.discretization.StateDiscretizer;
import org.atorma.robot.learning.ArrayQTable;
import org.atorma.robot.learning.prioritizedsweeping.PrioritizedSweeping;
import org.atorma.robot.mdp.StateAction;
import org.atorma.robot.mdp.TransitionReward;
import org.atorma.robot.objecttracking.TrackedObject;
import org.atorma.robot.objecttrackingbumper.*;
import org.atorma.robot.objecttrackingbumper.BumperRewardFunction;
import org.atorma.robot.simplebumper.*;
import org.junit.Before;
import org.junit.Test;

public class PrioritizedSweepingTests {
	
	private PrioritizedSweeping prioritizedSweeping;
	private double discountFactor = 0.9;
	
	private StateDiscretizer bumperStateDiscretizer = new BumperStateDiscretizer();
	private StateDiscretizer collisionStateDiscretizer = new SingleSectorCollisionStateDiscretizer(90);
	private BumperRewardFunction rewardFunction = new BumperRewardFunction();
	private BumperModel model;
	
	
	
	@Before
	public void setUp() {
		model = new BumperModel(rewardFunction, collisionStateDiscretizer);
		setPriorCollisionProbabilities();
		
		prioritizedSweeping = new PrioritizedSweeping();
		prioritizedSweeping.setDiscountFactor(discountFactor);
		prioritizedSweeping.setStateDiscretizer(bumperStateDiscretizer);
		prioritizedSweeping.setModel(model);
		prioritizedSweeping.setQValueChangeThreshold(1E-4);
		prioritizedSweeping.setQTable(new ArrayQTable(bumperStateDiscretizer.getNumberOfStates(), BumperAction.values().length));
	}
	
	private void setPriorCollisionProbabilities() {
		ModeledBumperState fromState, toState;
		BumperAction action;
		
		// Add collisions when an obstacle is close in front, not collision yet, and the agent drives forward
		fromState = new ModeledBumperState();
		fromState.addObservation(TrackedObject.inPolarDegreeCoordinates(BumperAction.DRIVE_DISTANCE_CM, 0));
		fromState.setCollided(false);
		action = BumperAction.FORWARD;
		toState = fromState.afterAction(action);
		toState.setCollided(true);
		for (int i = 0; i < 80; i++) {
			TransitionReward transition = new TransitionReward(fromState, action, toState, -100); // the reward doesn't matter in this implementation
			model.update(transition);
		}

		// Same when already collided before starting the action
		fromState.setCollided(true);
		toState = fromState;
		for (int i = 0; i < 180; i++) {
			TransitionReward transition = new TransitionReward(fromState, action, toState, -100); 
			model.update(transition);
		}
		
	}

	@Test
	public void test_agent_bypasses_obstacle() {
		ModeledBumperState currentState;
		BumperAction action;
		
		currentState = new ModeledBumperState();
		currentState.addObservation(TrackedObject.inPolarDegreeCoordinates(7, 0));
		currentState.addObservation(TrackedObject.inPolarDegreeCoordinates(5.5, 161));
		currentState.addObservation(TrackedObject.inPolarDegreeCoordinates(10.7, 182));
		currentState.addObservation(TrackedObject.inPolarDegreeCoordinates(22.6, 199));
		currentState.addObservation(TrackedObject.inPolarDegreeCoordinates(2.8, 218));
		currentState.setCollided(true);
		
		prioritizedSweeping.setSweepStartStateAction(new StateAction(currentState, BumperAction.FORWARD));
		prioritizedSweeping.performIterations(5000);
		
		action = getBestActionInState(currentState);
		System.out.println("State " + currentState);
		System.out.println("Best action " + action);
		assertFalse(action == BumperAction.FORWARD);
		
		currentState = currentState.afterAction(action);
		
		action = getBestActionInState(currentState);
		System.out.println("State " + currentState);
		System.out.println("Best action " + action);
		assertFalse(action == BumperAction.FORWARD);
		
		currentState = currentState.afterAction(action);
		
		action = getBestActionInState(currentState);
		System.out.println("State " + currentState);
		System.out.println("Best action " + action);
		assertFalse(action == BumperAction.FORWARD);
		
		currentState = currentState.afterAction(action);
		
		action = getBestActionInState(currentState);
		System.out.println("State " + currentState);
		System.out.println("Best action " + action);
		assertTrue(action == BumperAction.FORWARD);
		
		currentState = currentState.afterAction(action);
		
		action = getBestActionInState(currentState);
		System.out.println("State " + currentState);
		System.out.println("Best action " + action);
		assertTrue(action == BumperAction.FORWARD);
	}

	private BumperAction getBestActionInState(ModeledBumperState state) {
		int stateId = bumperStateDiscretizer.getId(state);
		Integer actionId = prioritizedSweeping.getActionId(stateId);
		BumperAction action = BumperAction.getAction(actionId);
		return action;
	}
}
