package org.atorma.robot.learning;

import static org.junit.Assert.*;
import static org.mockito.Mockito.*;

import java.util.*;

import org.atorma.robot.discretization.VectorDiscretizer;
import org.atorma.robot.learning.PrioritizedSweeping.PrioritzedStateAction;
import org.atorma.robot.mdp.*;
import org.junit.Before;
import org.junit.Test;
import org.mockito.Mock;
import org.mockito.MockitoAnnotations;

import com.google.common.collect.Sets;

public class PrioritizedSweepingTests {

	private PrioritizedSweeping sweeping;
	private double discountFactor = 0.9;
	private double qValueChangeThreshold = 0.05;
	
	@Mock private DiscreteQFunction qFunction;
	@Mock private MarkovModel model;
	@Mock private VectorDiscretizer stateDiscretizer;
	
	@Before
	public void setUp() {
		MockitoAnnotations.initMocks(this);
		
		sweeping = new PrioritizedSweeping();
		sweeping.setDiscountFactor(discountFactor);
		sweeping.setQFunction(qFunction);
		sweeping.setModel(model);
		sweeping.setStateDiscretizer(stateDiscretizer);
		sweeping.setQValueChangeThreshold(qValueChangeThreshold);
	}
	
	@Test
	public void set_current_state_action() {
		StateAction stateAction = mockStateAction(1, 1);
		
		sweeping.setCurrentStateAction(stateAction);
		
		assertEquals(1, sweeping.getPriorityQueue().size());
		PrioritzedStateAction prioritizedStateAction = sweeping.getPriorityQueue().peek();
		assertEquals(stateAction, prioritizedStateAction.getStateAction());
		assertEquals(Double.MIN_VALUE, prioritizedStateAction.getPriority(), 0);
	}
	
	@Test
	public void perform_iterations() {
		// Starting state and action, starting state value
		State startState = mockState(1);
		DiscreteAction startAction = mockAction(1);
		DiscretizedStateAction startStateActionId = new DiscretizedStateAction(1, 1);
		StateAction startStateAction = new StateAction(startState, startAction);
		
		double startStateValue = -100.0; // max Q-value over actions in start state
		when(qFunction.getMaxValueForState(1)).thenReturn(startStateValue); // Low q-value means update causes sweep of predecessor state-action pairs
		
		// Two possible transitions with expected rewards and transition probabilities
		State endState1 = mockState(2);
		double reward1 = 5;
		double prob1 = 0.3;
		StochasticTransitionReward tr1 = new StochasticTransitionReward(startStateAction.getState(), startStateAction.getAction(), endState1, reward1, prob1);
		double endState1Value = 2; // max Q-value in end state 1
		when(qFunction.getMaxValueForState(2)).thenReturn(endState1Value);
		
		State endState2 = mockState(3);
		double reward2 = -1;
		double prob2 = 0.7;
		StochasticTransitionReward tr2 = new StochasticTransitionReward(startStateAction.getState(), startStateAction.getAction(), endState2, reward2, prob2);
		double endState2Value = -5;
		when(qFunction.getMaxValueForState(3)).thenReturn(endState2Value);
		
		when(model.getTransitions(startStateAction)).thenReturn(Sets.newHashSet(tr1, tr2));
		
		// Predecessor states and actions of the starting state
		StateAction predecessor1 = mockStateAction(4, 4);
		double prob4 = 0.4;
		when(model.getTransitionProbability(new Transition(predecessor1.getState(), predecessor1.getAction(), startState))).thenReturn(prob4);
		
		StateAction predecessor2 = mockStateAction(5, 5);
		double prob5 = 0.6; // P(s=1 | s=5, a=5) has the higher probability so should get higher priority after the sweep
		when(model.getTransitionProbability(new Transition(predecessor2.getState(), predecessor2.getAction(), startState))).thenReturn(prob5);
		
		when(model.getPredecessors(startStateAction.getState())).thenReturn(Sets.newHashSet(predecessor1, predecessor2));

		
		// Perform one iteration from the starting state		
		sweeping.setCurrentStateAction(startStateAction);
		sweeping.performIterations(1);
		
		// Q-value updated 
		double expectedNewQ = prob1 * (reward1 + discountFactor*endState1Value) + prob2 * (reward2 + discountFactor*endState2Value);
		verify(qFunction).setValue(startStateActionId, expectedNewQ);
		
		// New Q-value becomes the max Q-value of startState and the change is large enough to cause repriorisation of 
		// predecessor state-action pairs
		assertTrue(expectedNewQ > startStateValue);
		double expectedQValueChange = Math.abs(expectedNewQ - startStateValue);
		assertTrue(expectedQValueChange > qValueChangeThreshold);
		assertEquals(2, sweeping.getPriorityQueue().size());
		List<PrioritzedStateAction> priorityList = new ArrayList<>(sweeping.getPriorityQueue());
		assertEquals(predecessor2, priorityList.get(0).getStateAction()); 
		assertEquals(predecessor1, priorityList.get(1).getStateAction()); 
		
		// Next iteration starts with predecessor2 
		
		double precessor2StateValue = 100.0;
		when(qFunction.getMaxValueForState(5)).thenReturn(precessor2StateValue); // High q-value means update does not cause sweeping of predecessor2:s predecessors
		
	}
	
	private StateAction mockStateAction(int stateId, int actionId) {
		State state = mockState(stateId);
		DiscreteAction action = mockAction(actionId);
		return new StateAction(state, action);
	}
	
	private State mockState(int stateId) {
		State state = mock(State.class);
		double[] values = new double[] {Math.random()};
		when(state.getValues()).thenReturn(values);
		when(stateDiscretizer.getId(values)).thenReturn(stateId);
		return state;
	}
	
	private DiscreteAction mockAction(int actionId) {
		DiscreteAction action = mock(DiscreteAction.class);
		when(action.getId()).thenReturn(actionId);
		return action;
	}
	
}
