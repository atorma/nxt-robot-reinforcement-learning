package org.atorma.robot.learning.montecarlo;

import org.atorma.robot.learning.EligibilityTraces;
import org.atorma.robot.mdp.DiscreteAction;

public class QLearningUctPlanningParameters extends UctPlanningParameters {

	public double learningRate;
	public EligibilityTraces eligibilityTraces;
	public DiscreteAction[] allActions;
}
