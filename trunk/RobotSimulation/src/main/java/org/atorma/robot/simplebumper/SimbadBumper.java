package org.atorma.robot.simplebumper;

import javax.vecmath.Vector3d;

import org.atorma.robot.*;

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
		private double lifetimeWhenStarted;
		private double distanceTraveledCm = -1;
		
		public Drive(BumperAction action) {
			
			// Assuming behavior is called 20 times per second in simulation time,
			// this velocity turns the agent the desired degrees in one call
			// after the speed is set.
			velocityCmPerSec = BumperAction.DRIVE_DISTANCE_CM/0.05;
			if (action == BumperAction.BACKWARD) {
				velocityCmPerSec = -velocityCmPerSec;
			}
			
			lifetimeWhenStarted = SimbadBumper.this.getLifeTime();
		}
		
		@Override
		public void perform() {
			if (distanceTraveledCm < 0) {
				SimbadBumper.this.setRotationalVelocity(0);
				SimbadBumper.this.setTranslationalVelocity(velocityCmPerSec/100);
				distanceTraveledCm = 0;
			} else {
				distanceTraveledCm += (SimbadBumper.this.getLifeTime() - lifetimeWhenStarted) * Math.abs(velocityCmPerSec);
			}
		}

		@Override
		public boolean isCompleted() {
			distanceTraveledCm += (SimbadBumper.this.getLifeTime() - lifetimeWhenStarted) * Math.abs(velocityCmPerSec);
			//System.out.println("Distance traveled in action "+ distanceTraveledCm);
			return distanceTraveledCm/BumperAction.DRIVE_DISTANCE_CM >= 0.99  || SimbadBumper.this.collisionDetected();
		}

	}

	private class Turn implements SimbadAction {
		
		private double rotationalVelocityDegPerSec;
		private double lifetimeWhenStarted;
		private double angleTurnedDeg = -1;
		
		public Turn(BumperAction action) {
			// Assuming behavior is called 20 times per second in simulation time,
			// this velocity turns the agent the desired degrees in one call
			// after the speed is set.
			rotationalVelocityDegPerSec = BumperAction.TURN_DEGREES/0.05;
			if (action == BumperAction.RIGHT) {
				rotationalVelocityDegPerSec = -rotationalVelocityDegPerSec;
			}
			
			lifetimeWhenStarted = SimbadBumper.this.getLifeTime();
		}

		@Override
		public void perform() {
			if (angleTurnedDeg < 0) {
				SimbadBumper.this.setTranslationalVelocity(0);
				SimbadBumper.this.setRotationalVelocity(Math.toRadians(rotationalVelocityDegPerSec)); 
				angleTurnedDeg = 0;
			}
		}

		@Override
		public boolean isCompleted() {
			angleTurnedDeg += (SimbadBumper.this.getLifeTime() - lifetimeWhenStarted) * Math.abs(rotationalVelocityDegPerSec);
			//System.out.println("Degrees turned in action "+ angleTurnedDeg);
			return angleTurnedDeg/BumperAction.TURN_DEGREES >= 0.99;
		}

	}
}
