package org.atorma.robot.policy;

import static org.junit.Assert.*;
import static org.mockito.Mockito.*;

import org.apache.commons.math3.distribution.EnumeratedIntegerDistribution;
import org.apache.commons.math3.stat.Frequency;
import org.atorma.robot.learning.QTable;
import org.atorma.robot.mdp.DiscretizedStateAction;
import org.junit.Before;
import org.junit.Test;
import org.mockito.Mock;
import org.mockito.MockitoAnnotations;

public class BoltzmannActionSelectionTests {

	private BoltzmannActionSelection boltzmann;
	private double temperature = 2.0;
	private int[] actionIds = new int[] {0, 1, 2};
	
	@Mock private QTable qTable;
	
	@Before
	public void setUp() {
		MockitoAnnotations.initMocks(this);
		boltzmann = new BoltzmannActionSelection(qTable, temperature, actionIds);
	}
	
	@Test
	public void test_distribution_computation_and_action_selection() {
		DiscretizedStateAction stateAction00 = new DiscretizedStateAction(0, 0);
		when(qTable.getValue(stateAction00)).thenReturn(0.0);
		DiscretizedStateAction stateAction01 = new DiscretizedStateAction(0, 1);
		when(qTable.getValue(stateAction01)).thenReturn(-1.0);
		DiscretizedStateAction stateAction02 = new DiscretizedStateAction(0, 2);
		when(qTable.getValue(stateAction02)).thenReturn(-2.0);
		
		// Different state than the others!
		DiscretizedStateAction stateAction10 = new DiscretizedStateAction(1, 0);
		when(qTable.getValue(stateAction10)).thenReturn(100.0);
		
		EnumeratedIntegerDistribution distribution = boltzmann.getActionDistribution(0);
		assertTrue(distribution.probability(0) > distribution.probability(1));
		assertTrue(distribution.probability(1) > distribution.probability(2));
		assertEquals(0.50648, distribution.probability(0), 0.00001);
		
		Frequency freq = new Frequency();
		for (int i = 0; i < 1000; i++) {
			int actionId = boltzmann.getActionId(0);
			freq.addValue(actionId);
		}
		assertEquals(0.51, freq.getPct(0), 0.2);
	}
}
