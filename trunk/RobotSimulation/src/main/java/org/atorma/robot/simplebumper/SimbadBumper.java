package org.atorma.robot.simplebumper;

import javax.vecmath.Vector3d;

import org.atorma.robot.*;
import org.atorma.robot.mdp.State;

import simbad.sim.RangeSensorBelt;
import simbad.sim.RobotFactory;

public class SimbadBumper extends SimbadRobot {
	
	private RangeSensorBelt ultrasonicSensor;

	public SimbadBumper(DiscreteRobotController controller) {
		super(controller, new Vector3d(0, 0, 0), "Toveri");
		
		this.height = 6f/100; // 6 cm height
		this.radius = 9f/100; // 9 cm radius
		
		ultrasonicSensor = RobotFactory.addSonarBeltSensor(this, 1);
		ultrasonicSensor.setUpdatePerSecond(60);
	}

	@Override
	public State getCurrentState() {
		int ultrasonicDistance = (int) (ultrasonicSensor.getMeasurement(0)*100); 
		ultrasonicDistance = Math.max(BumperPercept.MIN_ULTRASONIC_DIST, ultrasonicDistance);
		ultrasonicDistance = Math.min(BumperPercept.MAX_ULTRASONIC_DIST, ultrasonicDistance);
		
		BumperPercept state = new BumperPercept(ultrasonicDistance, this.collisionDetected());
		return state;
	}
	
	@Override
	public SimbadAction getAction(int actionId) {
		BumperAction action = BumperAction.getAction(actionId);
		if (action == BumperAction.FORWARD || action == BumperAction.BACKWARD) {
			return new Drive(action);
		} else if (action == BumperAction.LEFT || action == BumperAction.RIGHT) {
			return new Turn(action);
		} else {
			throw new IllegalArgumentException();
		}
	}
	
	private class Drive implements SimbadAction {
		
		private double velocityCmPerSec;
		private Double distanceTraveledCm = null;
		
		public Drive(BumperAction action) {
			
			// Assuming behavior is called 20 times per second in simulation time,
			// this velocity turns the agent the desired degrees in one call
			// after the speed is set.
			velocityCmPerSec = BumperAction.DRIVE_DISTANCE_CM * ACTION_CALL_FREQUENCY_HZ;
			if (action == BumperAction.BACKWARD) {
				velocityCmPerSec = -velocityCmPerSec;
			}
		}
		
		@Override
		public void perform() {
			if (distanceTraveledCm == null) {
				SimbadBumper.this.setRotationalVelocity(0);
				SimbadBumper.this.setTranslationalVelocity(velocityCmPerSec/100);
				distanceTraveledCm = 0.0;
			}
			distanceTraveledCm += velocityCmPerSec / ACTION_CALL_FREQUENCY_HZ;
		}

		@Override
		public boolean isCompleted() {
			return Math.abs(distanceTraveledCm)/BumperAction.DRIVE_DISTANCE_CM >= 0.99  || SimbadBumper.this.collisionDetected();
		}

	}

	private class Turn implements SimbadAction {
		
		private double rotationalVelocityDegPerSec;
		private Double angleTurnedDeg;
		
		public Turn(BumperAction action) {
			// Assuming behavior is called 20 times per second in simulation time,
			// this velocity turns the agent the desired degrees in one call
			// after the speed is set.
			rotationalVelocityDegPerSec = BumperAction.TURN_DEGREES * ACTION_CALL_FREQUENCY_HZ;
			if (action == BumperAction.RIGHT) {
				rotationalVelocityDegPerSec = -rotationalVelocityDegPerSec;
			}
		}

		@Override
		public void perform() {
			if (angleTurnedDeg == null) {
				SimbadBumper.this.setTranslationalVelocity(0);
				SimbadBumper.this.setRotationalVelocity(Math.toRadians(rotationalVelocityDegPerSec)); 
				angleTurnedDeg = 0.0;
			} 
			angleTurnedDeg += rotationalVelocityDegPerSec / ACTION_CALL_FREQUENCY_HZ;
		}

		@Override
		public boolean isCompleted() {
			return Math.abs(angleTurnedDeg)/BumperAction.TURN_DEGREES >= 0.99;
		}

	}
}
