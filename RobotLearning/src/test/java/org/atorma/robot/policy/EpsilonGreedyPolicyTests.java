package org.atorma.robot.policy;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.HashSet;
import java.util.Set;

import org.junit.Before;
import org.junit.Test;

public class EpsilonGreedyPolicyTests {

	private EpsilonGreedyPolicy egp;
	private int[] actionIds = {0, 1, 2, 3};
	private Set<Integer> actionIdSet;
	
	@Before
	public void setUp() {
		egp = new EpsilonGreedyPolicy(0.1, actionIds, null);
		
		actionIdSet = new HashSet<>();
		for (int actionId : actionIds) {
			actionIdSet.add(actionId);
		}
	}
	
	@Test
	public void when_no_deterministic_policy_then_returns_random_action() {
		egp.setDeterministicPolicy(null);
		int actionId = egp.getActionId(0);
		assertTrue(actionIdSet.contains(actionId));
		
		egp.setDeterministicPolicy(new StateIdToActionIdMap()); // empty tabular policy
		actionId = egp.getActionId(0);
		assertTrue(actionIdSet.contains(actionId));
	}
	
	@Test
	public void when_has_deterministic_policy_and_epsilon_zero_then_returns_deterministic_action() {
		StateIdToActionIdMap tp = new StateIdToActionIdMap();
		tp.put(0, 1);
		
		egp.setDeterministicPolicy(tp);
		egp.setEpsilon(0);
		
		assertEquals(1, egp.getActionId(0).intValue());
	}
}
