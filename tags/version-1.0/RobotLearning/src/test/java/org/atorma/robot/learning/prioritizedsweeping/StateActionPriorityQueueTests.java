package org.atorma.robot.learning.prioritizedsweeping;

import static org.junit.Assert.*;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

import org.atorma.robot.discretization.StateDiscretizer;
import org.atorma.robot.mdp.*;
import org.junit.Before;
import org.junit.Test;
import org.mockito.Mock;
import org.mockito.MockitoAnnotations;

public class StateActionPriorityQueueTests {

	private DiscretizingStateActionPriorityQueue priorityQueue;
	@Mock private StateDiscretizer stateDiscretizer;
	
	@Before
	public void setUp() {
		MockitoAnnotations.initMocks(this);
		
		priorityQueue = new DiscretizingStateActionPriorityQueue(stateDiscretizer);
	}
	
	@Test
	public void add_or_update_priority_when_StateActions_have_different_discretizations() {
		StateAction stateAction1 = mockStateAction(1, 1);
		StateAction stateAction2 = mockStateAction(12, 2);
		
		priorityQueue.addOrDecreasePriority(stateAction1, 1);
		priorityQueue.addOrDecreasePriority(stateAction2, 2);
		assertEquals(2, priorityQueue.size());
		assertEquals(stateAction1, priorityQueue.peekMin());
		
		priorityQueue.addOrDecreasePriority(stateAction2, -1);
		assertEquals(2, priorityQueue.size());
		assertEquals(stateAction2, priorityQueue.peekMin());
	}
	
	@Test
	public void when_StateAction_discretizations_same_then_priority_updated() {
		StateAction stateAction1 = mockStateAction(1, 1);
		StateAction stateAction2 = mockStateAction(1, 1);
		assertNotEquals(stateAction1, stateAction2);
		
		priorityQueue.addOrDecreasePriority(stateAction1, 1);
		priorityQueue.addOrDecreasePriority(stateAction2, 2);
		assertEquals(1, priorityQueue.size());
		assertEquals(stateAction1, priorityQueue.peekMin());
		
		priorityQueue.addOrDecreasePriority(stateAction2, -1);
		assertEquals(1, priorityQueue.size());
		assertEquals(stateAction1, priorityQueue.peekMin());
	}
	
	private StateAction mockStateAction(int stateId, int actionId) {
		State state = mock(State.class);
		DiscreteAction action = mock(DiscreteAction.class);
		StateAction stateAction = mock(StateAction.class);
		when(stateAction.getState()).thenReturn(state);
		when(stateAction.getAction()).thenReturn(action);
		when(stateDiscretizer.getId(state)).thenReturn(stateId);
		when(action.getId()).thenReturn(actionId);
		return stateAction;
	}
}
