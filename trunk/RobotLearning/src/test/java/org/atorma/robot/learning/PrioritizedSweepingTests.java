package org.atorma.robot.learning;

import static org.junit.Assert.*;
import static org.mockito.Mockito.*;

import java.util.Set;

import org.atorma.robot.discretization.VectorDiscretizer;
import org.atorma.robot.learning.PrioritizedSweeping.PrioritzedStateAction;
import org.atorma.robot.mdp.*;
import org.junit.Before;
import org.junit.Test;
import org.mockito.Mock;
import org.mockito.MockitoAnnotations;

import com.google.common.collect.Sets;

public class PrioritizedSweepingTests {

	private PrioritizedSweeping<State, DiscreteAction> sweeping;
	private final double discountFactor = 0.9;
	
	@Mock private DiscreteQFunction qFunction;
	@Mock private MarkovModel<State, DiscreteAction> model;
	@Mock private VectorDiscretizer stateDiscretizer;
	
	@Before
	public void setUp() {
		MockitoAnnotations.initMocks(this);
		
		sweeping = new PrioritizedSweeping<>();
		sweeping.setDiscountFactor(discountFactor);
		sweeping.setQFunction(qFunction);
		sweeping.setModel(model);
		sweeping.setStateDiscretizer(stateDiscretizer);
	}
	
	@Test
	public void set_current_state_action() {
		StateAction<State, DiscreteAction> stateAction = mock(StateAction.class);
		
		sweeping.setCurrentStateAction(stateAction);
		
		assertEquals(1, sweeping.getPriorityQueue().size());
		PrioritzedStateAction prioritizedStateAction = sweeping.getPriorityQueue().peek();
		assertEquals(stateAction, prioritizedStateAction.getStateAction());
		assertEquals(Double.MAX_VALUE, prioritizedStateAction.getPriority(), 0);
	}
	
	@Test
	public void perform_one_iteration() {
		// Starting state
		DiscretizedStateAction startStateActionId = new DiscretizedStateAction(1, 1);
		StateAction<State, DiscreteAction> startStateAction = mockStateAction(startStateActionId);
		
		// Two possible transitions with expected rewards and transition probabilities
		
		State endState1 = mockState(2);
		double reward1 = 5;
		double prob1 = 0.3;
		StochasticTransitionWithReward<State, DiscreteAction> tr1 = new StochasticTransitionWithReward<>(startStateAction.getState(), startStateAction.getAction(), endState1, reward1, prob1);
		double maxQValue1 = 2;
		when(qFunction.getMaxValueForState(2)).thenReturn(maxQValue1);
		
		State endState2 = mockState(3);
		double reward2 = -1;
		double prob2 = 0.7;
		StochasticTransitionWithReward<State, DiscreteAction> tr2 = new StochasticTransitionWithReward<>(startStateAction.getState(), startStateAction.getAction(), endState2, reward2, prob2);
		double maxQValue2 = -5;
		when(qFunction.getMaxValueForState(3)).thenReturn(maxQValue2);
		
		when(model.getTransitions(startStateAction)).thenReturn(Sets.newHashSet(tr1, tr2));

		
		// Perform one iteration from known starting state		
		sweeping.setCurrentStateAction(startStateAction);
		sweeping.performIterations(1);
		
		// Q-value updated 
		double expectedNewQ = prob1 * (reward1 + discountFactor * maxQValue1) + prob2 * (reward2 + discountFactor * maxQValue2);
		verify(qFunction).setValue(startStateActionId, expectedNewQ);
	}
	
	private StateAction<State, DiscreteAction> mockStateAction(DiscretizedStateAction discretizedStateAction) {
		State state = mockState(discretizedStateAction.getStateId());
		
		DiscreteAction action = mock(DiscreteAction.class);
		when(action.getId()).thenReturn(discretizedStateAction.getActionId());
		
		return new StateAction<State, DiscreteAction>(state, action);
	}
	
	private State mockState(int stateId) {
		State state = mock(State.class);
		double[] values = new double[] {Math.random()};
		when(state.getValues()).thenReturn(values);
		when(stateDiscretizer.getId(values)).thenReturn(stateId);
		return state;
	}
	
}
