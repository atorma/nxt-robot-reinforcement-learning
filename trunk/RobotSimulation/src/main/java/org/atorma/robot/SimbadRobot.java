package org.atorma.robot;

import javax.vecmath.Vector3d;

import simbad.gui.Simbad;
import simbad.sim.Agent;

public abstract class SimbadRobot extends Agent {
	
	private final DiscreteActionPolicy policy;
	
	private SimbadAction currentAction;

	public SimbadRobot(DiscreteActionPolicy actionIdProvider, Vector3d startingPosition, String name) {
		super(startingPosition, name);
		this.policy = actionIdProvider;
	}
	
	public abstract State getCurrentState();
	
	public abstract SimbadAction getAction(int actionId);
	
	
	/** This method is called by the simulator engine on reset. */
	@Override
    public void initBehavior() {
        // nothing particular in this case
    }

    /** This method is called cyclically (20 times per second)  by the simulator engine. */
	@Override
    public void performBehavior() {

		if (currentAction == null || currentAction.isCompleted()) {
			State currentState = getCurrentState();
			int actionId = policy.getActionId(currentState.getValues());
			currentAction = getAction(actionId);
		}

		currentAction.perform();

    }
	
	public void startSimulationGUI() {
		// request antialising
        System.setProperty("j3d.implicitAntialiasing", "true");
        // create Simbad instance with given environment
        Simbad frame = new Simbad(new RobotTestEnvironment(this), false);
	}
	
	public void startSimulationInBackground() {
		 Simbad frame = new Simbad(new RobotTestEnvironment(this), true);
	}
}
