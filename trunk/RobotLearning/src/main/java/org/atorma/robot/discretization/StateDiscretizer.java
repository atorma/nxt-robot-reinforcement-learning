package org.atorma.robot.discretization;

import org.atorma.robot.mdp.State;

public interface StateDiscretizer {

	int getId(State state);
}
