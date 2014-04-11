package org.atorma.robot.objecttrackingbumper.prioritizedsweeping;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.Arrays;
import java.util.List;

import org.atorma.robot.learning.ArrayQTable;
import org.atorma.robot.learning.prioritizedsweeping.PrioritizedSweeping;
import org.atorma.robot.mdp.StateAction;
import org.atorma.robot.objecttracking.CircleSector;
import org.atorma.robot.objecttracking.TrackedObject;
import org.atorma.robot.objecttrackingbumper.*;
import org.atorma.robot.simplebumper.BumperAction;
import org.junit.Before;
import org.junit.Test;

public class PrioritizedSweepingTests {
	
	private PrioritizedSweeping prioritizedSweeping;
	private double discountFactor = 0.9;
	
	private BumperStateDiscretizer bumperStateDiscretizer;
	private BumperRewardFunction rewardFunction = new BumperRewardFunction();
	private BumperModel model;
	private List<CircleSector> obstacleSectors;
	
	
	
	@Before
	public void setUp() {
		obstacleSectors = Arrays.asList(
				new CircleSector(-45, -15),
				new CircleSector(-15, 15),
				new CircleSector(15, 45));
		bumperStateDiscretizer = new BumperStateDiscretizer(obstacleSectors);
		model = new BumperModel(rewardFunction, bumperStateDiscretizer);
		BumperModelUtils.setPriorCollisionProbabilities(model, bumperStateDiscretizer, 0.8, 0.99);
		
		prioritizedSweeping = new PrioritizedSweeping();
		prioritizedSweeping.setDiscountFactor(discountFactor);
		prioritizedSweeping.setStateDiscretizer(bumperStateDiscretizer);
		prioritizedSweeping.setModel(model);
		prioritizedSweeping.setQValueChangeThreshold(1E-4);
		prioritizedSweeping.setQTable(new ArrayQTable(bumperStateDiscretizer.getNumberOfStates(), BumperAction.values().length));
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
		prioritizedSweeping.performIterations(3000);
		
		action = getBestActionInState(currentState);
		System.out.println("State " + currentState);
		System.out.println("Best action " + action);
		assertFalse(action == BumperAction.FORWARD);
		
		currentState = currentState.afterAction(action);
		
		prioritizedSweeping.setSweepStartStateAction(new StateAction(currentState, BumperAction.FORWARD));
		prioritizedSweeping.performIterations(3000);
		
		action = getBestActionInState(currentState);
		System.out.println("State " + currentState);
		System.out.println("Best action " + action);
		assertFalse(action == BumperAction.FORWARD);
		
		currentState = currentState.afterAction(action);
		
		prioritizedSweeping.setSweepStartStateAction(new StateAction(currentState, BumperAction.FORWARD));
		prioritizedSweeping.performIterations(3000);
		
		action = getBestActionInState(currentState);
		System.out.println("State " + currentState);
		System.out.println("Best action " + action);
		assertFalse(action == BumperAction.FORWARD);
		
		currentState = currentState.afterAction(action);
		
		prioritizedSweeping.setSweepStartStateAction(new StateAction(currentState, BumperAction.FORWARD));
		prioritizedSweeping.performIterations(3000);
		
		action = getBestActionInState(currentState);
		System.out.println("State " + currentState);
		System.out.println("Best action " + action);
		assertFalse(action == BumperAction.FORWARD);
		
		currentState = currentState.afterAction(action);
		
		prioritizedSweeping.setSweepStartStateAction(new StateAction(currentState, BumperAction.FORWARD));
		prioritizedSweeping.performIterations(3000);
		
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
