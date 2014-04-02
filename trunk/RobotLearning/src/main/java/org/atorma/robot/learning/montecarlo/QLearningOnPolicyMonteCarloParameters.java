package org.atorma.robot.learning.montecarlo;

import org.atorma.robot.discretization.StateDiscretizer;
import org.atorma.robot.learning.EligibilityTraces;
import org.atorma.robot.mdp.DiscreteAction;
import org.atorma.robot.policy.DiscretePolicy;

public class QLearningOnPolicyMonteCarloParameters {
	public ForwardModel model;
	public DiscreteAction[] allActions;
	public StateDiscretizer stateDiscretizer;
	public DiscretePolicy policy;
	public int horizon;
	public double learningRate;
	public EligibilityTraces traces;
	public double defaultQValue;

}