package org.atorma.robot.learning;

import org.atorma.robot.mdp.DiscreteAction;
import org.atorma.robot.mdp.State;

public class PrioritizedSweepingTests {

	private PrioritizedSweeping<State, DiscreteAction> sweeping;
	
	
	
	
	private static class TestState implements State {

		@Override
		public double[] getValues() {
			// TODO Auto-generated method stub
			return null;
		}
		
	}
	
	private static class TestAction implements DiscreteAction {

		@Override
		public int getId() {
			// TODO Auto-generated method stub
			return 0;
		}
		
	}
}
