package org.atorma.robot.learning.prioritizedsweeping;

import static org.junit.Assert.*;
import static org.mockito.Mockito.mock;

import org.atorma.robot.mdp.StateAction;
import org.junit.Before;
import org.junit.Test;

public class StateActionPriorityQueueTests {

	private StateActionPriorityQueue priorityQueue;
	
	@Before
	public void setUp() {
		priorityQueue = new StateActionPriorityQueue();
	}
	
	@Test
	public void add_and_update_priority() {
		StateAction stateAction1 = mock(StateAction.class);
		StateAction stateAction2 = mock(StateAction.class);
		
		priorityQueue.addOrUpdate(stateAction1, 1);
		priorityQueue.addOrUpdate(stateAction2, 2);
		assertEquals(stateAction1, priorityQueue.peek());
		
		priorityQueue.addOrUpdate(stateAction2, -1);
		assertEquals(stateAction2, priorityQueue.peek());
	}
}
