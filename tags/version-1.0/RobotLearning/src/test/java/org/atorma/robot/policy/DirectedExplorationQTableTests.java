package org.atorma.robot.policy;

import static org.junit.Assert.*;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;
import static java.lang.Math.*;

import org.atorma.robot.learning.QTable;
import org.atorma.robot.mdp.DiscretizedStateAction;
import org.junit.Before;
import org.junit.Test;
import org.mockito.Mock;
import org.mockito.MockitoAnnotations;

public class DirectedExplorationQTableTests {

	private DirectedExploration directedExploration;
	private double bonusMultiplier = 0.02; 
	private double defaultCount = 0.1;
	private int[] actionIds = new int[] {0, 1, 2};
	
	@Mock private QTable underlyingQTable;
	@Mock private DiscretizedStateAction stateAction;
	
	@Before
	public void setUp() {
		MockitoAnnotations.initMocks(this);
		
		directedExploration = new DirectedExploration(underlyingQTable, bonusMultiplier, defaultCount, actionIds); 
	}
	
	@Test
	public void when_state_action_never_tried_at_all_and_no_time_steps_recorder() {
		when(underlyingQTable.getValue(stateAction)).thenReturn(1.0);
		double expectedQ = 1.0;
		assertEquals(expectedQ, directedExploration.getValue(stateAction), 0);
	}
	
	@Test
	public void when_state_action_never_tried_at_all_but_some_time_steps_recorder() {
		when(underlyingQTable.getValue(stateAction)).thenReturn(1.0);
		for (int i = 0; i < 10; i++) {
			directedExploration.recordStateAction(mock(DiscretizedStateAction.class));
		}
		
		double expectedQ = 1.0 + bonusMultiplier * sqrt(10)/defaultCount;
		assertEquals(expectedQ, directedExploration.getValue(stateAction), 0);
	}
	
	@Test
	public void when_state_action_tried_before() {
		when(underlyingQTable.getValue(stateAction)).thenReturn(1.0);
		
		directedExploration.recordStateAction(stateAction);
		for (int i = 0; i < 10; i++) {
			directedExploration.recordStateAction(mock(DiscretizedStateAction.class));
		}
		directedExploration.recordStateAction(stateAction);
		for (int i = 0; i < 10; i++) {
			directedExploration.recordStateAction(mock(DiscretizedStateAction.class));
		}
		
		double expectedQ = 1.0 + bonusMultiplier * sqrt(10)/2; 
		assertEquals(expectedQ, directedExploration.getValue(stateAction), 0);
	}
	
	@Test
	public void get_best_action_in_state_changes_when_an_action_not_tried_for_a_long_time() {
		DiscretizedStateAction stateAction00 = new DiscretizedStateAction(0, 0);
		when(underlyingQTable.getValue(stateAction00)).thenReturn(1.0);
		DiscretizedStateAction stateAction01 = new DiscretizedStateAction(0, 1);
		when(underlyingQTable.getValue(stateAction01)).thenReturn(0.5);
		
		assertEquals(stateAction00, directedExploration.getBestActionInState(0));
		
		for (int i = 0; i < 100; i++) {
			directedExploration.recordStateAction(stateAction00);
		}
		
		assertEquals(stateAction01, directedExploration.getBestActionInState(0));
		assertTrue(directedExploration.getValue(stateAction01) > 1.0);
		assertEquals(Integer.valueOf(1), directedExploration.getActionId(0));
	}
	
}
