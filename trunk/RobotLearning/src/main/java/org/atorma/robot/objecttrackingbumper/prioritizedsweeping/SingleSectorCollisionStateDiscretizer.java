package org.atorma.robot.objecttrackingbumper.prioritizedsweeping;

import org.atorma.robot.discretization.*;
import org.atorma.robot.mdp.State;
import org.atorma.robot.objecttracking.TrackedObject;
import org.atorma.robot.objecttrackingbumper.ModeledBumperState;
import org.atorma.robot.simplebumper.CollisionDiscretizer;
import org.atorma.robot.simplebumper.ObstacleDistanceDiscretizer;

/**
 * Discretizes {@link ModeledBumperState}s by only considering obstacles 
 * in a sector in the front of the agent. Obstacle distance within that sector 
 * is discretized using  {@link ObstacleDistanceDiscretizer}. States where the agent is not yet
 * collided versus is already collided are also considered different. Obstacles
 * elsewhere around the agent are ignored.  
 */
public class SingleSectorCollisionStateDiscretizer implements StateDiscretizer {
	
	private double sectorLeft;
	private double sectorRight;
	private VectorDiscretizer vectorDiscretizer;

	public SingleSectorCollisionStateDiscretizer(double sectorWidth) {
		this.sectorLeft = -sectorWidth/2;
		this.sectorRight = sectorWidth/2;
		
		vectorDiscretizer = new VectorDiscretizerImpl(new ObstacleDistanceDiscretizer(), new CollisionDiscretizer());
	}

	@Override
	public int getId(State state) {
		ModeledBumperState bumperState = (ModeledBumperState) state;
		TrackedObject frontObstacle = bumperState.getNearestInSectorDegrees(sectorLeft, sectorRight);
		double[] stateValues = new double[] { 
				frontObstacle != null ? frontObstacle.getDistance() : Double.MAX_VALUE, 
				bumperState.isCollided() ? 1 : 0};
		return vectorDiscretizer.getId(stateValues);
	}

	@Override
	public int getNumberOfStates() {
		return vectorDiscretizer.getNumberOfValues();
	}

}
