package org.atorma.robot.objecttrackingbumper.prioritizedsweeping;

import static org.junit.Assert.*;

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
	
	private BumperStateDiscretizer stateDiscretizer = new BumperStateDiscretizer();
	private BumperRewardFunction rewardFunction = new BumperRewardFunction();
	private BumperModel model;
	
	
	
	@Before
	public void setUp() {
		model = new BumperModel(rewardFunction, new ObstacleDistanceDiscretizer());
		
		prioritizedSweeping = new PrioritizedSweeping();
		prioritizedSweeping.setDiscountFactor(discountFactor);
		prioritizedSweeping.setStateDiscretizer(stateDiscretizer);
		prioritizedSweeping.setModel(model);
		prioritizedSweeping.setQValueChangeThreshold(1E-4);
		prioritizedSweeping.setQTable(new ArrayQTable(stateDiscretizer.getNumberOfStates(), BumperAction.values().length));
	}
	
	@Test
	public void test_best_action_given_prior_collision_probabilities() {
		ModeledBumperState currentState, previousState;
		BumperAction action;
		BumperPercept percept;
		
		currentState = new ModeledBumperState();
		currentState.addObservation(TrackedObject.inPolarDegreeCoordinates(7, 0));
		currentState.addObservation(TrackedObject.inPolarDegreeCoordinates(5.5, 161));
		currentState.addObservation(TrackedObject.inPolarDegreeCoordinates(10.7, 182));
		currentState.addObservation(TrackedObject.inPolarDegreeCoordinates(22.6, 199));
		currentState.addObservation(TrackedObject.inPolarDegreeCoordinates(2.8, 218));
		currentState.setCollided(true);
		
		prioritizedSweeping.setSweepStartStateAction(new StateAction(currentState, BumperAction.FORWARD));
		prioritizedSweeping.performIterations(1000);
		
		action = getBestActionInState(currentState);
		System.out.println("Best action " + action);
		assertFalse(action == BumperAction.FORWARD);
		
		previousState = currentState;
		percept = new BumperPercept(255, false);
		currentState = previousState.afterActionAndObservation(action, percept);
		prioritizedSweeping.updateModel(new TransitionReward(previousState, action, currentState, -1));
		prioritizedSweeping.setSweepStartStateAction(new StateAction(currentState, action));
		prioritizedSweeping.performIterations(1000);
		
		action = getBestActionInState(currentState);
		System.out.println("Best action " + action);
		assertFalse(action == BumperAction.FORWARD);
		
		previousState = currentState;
		percept = new BumperPercept(255, false);
		currentState = previousState.afterActionAndObservation(action, percept);
		prioritizedSweeping.updateModel(new TransitionReward(previousState, action, currentState, -1));
		prioritizedSweeping.setSweepStartStateAction(new StateAction(currentState, action));
		prioritizedSweeping.performIterations(1000);
		
		action = getBestActionInState(currentState);
		System.out.println("Best action " + action);
		assertFalse(action == BumperAction.FORWARD);
		
		previousState = currentState;
		percept = new BumperPercept(255, false);
		currentState = previousState.afterActionAndObservation(action, percept);
		prioritizedSweeping.updateModel(new TransitionReward(previousState, action, currentState, -1));
		prioritizedSweeping.setSweepStartStateAction(new StateAction(currentState, action));
		prioritizedSweeping.performIterations(1000);
		
		action = getBestActionInState(currentState);
		System.out.println("Best action " + action);
		//assertTrue(action == BumperAction.FORWARD);
		
		previousState = currentState;
		percept = new BumperPercept(255, false);
		currentState = previousState.afterActionAndObservation(action, percept);
		prioritizedSweeping.updateModel(new TransitionReward(previousState, action, currentState, -1));
		prioritizedSweeping.setSweepStartStateAction(new StateAction(currentState, action));
		prioritizedSweeping.performIterations(1000);
		
		action = getBestActionInState(currentState);
		System.out.println("Best action " + action);
		assertTrue(action == BumperAction.FORWARD);
	}

	private BumperAction getBestActionInState(ModeledBumperState state) {
		int stateId = stateDiscretizer.getId(state);
		Integer actionId = prioritizedSweeping.getActionId(stateId);
		BumperAction action = BumperAction.getAction(actionId);
		return action;
	}
}
