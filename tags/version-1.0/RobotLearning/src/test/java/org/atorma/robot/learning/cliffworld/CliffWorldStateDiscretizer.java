package org.atorma.robot.learning.cliffworld;

import org.atorma.robot.discretization.StateDiscretizer;
import org.atorma.robot.mdp.State;

public class CliffWorldStateDiscretizer implements StateDiscretizer {

	@Override
	public int getId(State state) {
		CliffWorldState s = (CliffWorldState) state; 
		int id = s.getX() + s.getY()*(CliffWorldState.X_MAX + 1);
		return id;
	}

	@Override
	public int getNumberOfStates() {
		return (CliffWorldState.X_MAX + 1) * (CliffWorldState.Y_MAX + 1);
	}

}
