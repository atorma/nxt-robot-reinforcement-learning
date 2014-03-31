package org.atorma.robot.learning.montecarlo;

import org.atorma.robot.discretization.StateDiscretizer;
import org.atorma.robot.learning.QTable;

public class UctPlanningParameters {
	
	public ForwardModel model;
	public StateDiscretizer stateDiscretizer;
	public QTable longTermQValues;
	public int planningHorizon;
	public double uctConstant;
	public double discountFactor;
}