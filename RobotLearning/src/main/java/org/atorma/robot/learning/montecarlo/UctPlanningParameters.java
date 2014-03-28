package org.atorma.robot.learning.montecarlo;

import org.atorma.robot.discretization.StateDiscretizer;
import org.atorma.robot.learning.QTable;

public class UctPlanningParameters {
	
	public ForwardModel model;
	public StateDiscretizer stateDiscretizer;
	public QTable qTable;
	public int horizon;

	
}