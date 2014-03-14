package org.atorma.robot.learning.cliffworld;

import java.util.Arrays;

import org.atorma.robot.discretization.StateDiscretizer;
import org.atorma.robot.mdp.State;

public class CliffWorldStateDiscretizer implements StateDiscretizer {

	@Override
	public int getId(State state) {
		return Arrays.hashCode(state.getValues());
	}

	@Override
	public int getNumberOfStates() {
		return (CliffWorldState.X_MAX + 1) * (CliffWorldState.Y_MAX + 1);
	}

}
