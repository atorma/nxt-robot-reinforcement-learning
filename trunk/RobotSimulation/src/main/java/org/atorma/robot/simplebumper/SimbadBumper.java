package org.atorma.robot.simplebumper;

import java.util.HashMap;
import java.util.Map;

import javax.vecmath.Vector3d;

import org.atorma.robot.DiscreteActionPolicy;
import org.atorma.robot.SimbadAction;
import org.atorma.robot.SimbadRobot;
import org.atorma.robot.State;

import simbad.sim.RangeSensorBelt;

public class SimbadBumper extends SimbadRobot {
	
	private RangeSensorBelt ultrasonicSensor;
	private Map<BumperAction, SimbadAction> actionMap = new HashMap<>();
	

	public SimbadBumper(DiscreteActionPolicy actionIdProvider) {
		super(actionIdProvider, new Vector3d(0, 0, 0), "Toveri");
		
		this.height = 6f/100; // 6 cm height
		this.radius = 9f/100; // 9 cm radius

		ultrasonicSensor = new RangeSensorBelt(this.radius, 7f/100, 255.0f/100, 1, RangeSensorBelt.TYPE_SONAR, RangeSensorBelt.FLAG_SHOW_FULL_SENSOR_RAY);
		ultrasonicSensor.setUpdatePerSecond(60);
		this.addSensorDevice(ultrasonicSensor, new Vector3d(0, 0, 0), 0);
		//ultrasonicSensor = RobotFactory.addSonarBeltSensor(this);
		
		actionMap.put(BumperAction.FORWARD, new DriveForward());
		actionMap.put(BumperAction.BACKWARD, new DriveBackward());
		actionMap.put(BumperAction.LEFT, new TurnLeft());
		actionMap.put(BumperAction.RIGHT, new TurnRight());
	}

	@Override
	public State getCurrentState() {
		BumperPercept state = new BumperPercept((int) (ultrasonicSensor.getMeasurement(0)*100), this.collisionDetected());
		return state;
	}
	
	@Override
	public SimbadAction getAction(int actionId) {
		BumperAction bumperAction = BumperAction.getAction(actionId);
		return actionMap.get(bumperAction);
	}


	private class DriveForward implements SimbadAction {
		
		@Override
		public void perform() {
			SimbadBumper.this.setRotationalVelocity(0);
			SimbadBumper.this.setTranslationalVelocity(0.5);
		}

	}
	
	private class DriveBackward implements SimbadAction {

		@Override
		public void perform() {
			SimbadBumper.this.setRotationalVelocity(0);
			SimbadBumper.this.setTranslationalVelocity(-0.5);
		}

	}
	
	private class TurnLeft implements SimbadAction {

		@Override
		public void perform() {
			SimbadBumper.this.setTranslationalVelocity(0);
			SimbadBumper.this.rotateY(0.262); // rad, 15 deg
		}

	}
	
	private class TurnRight implements SimbadAction {

		@Override
		public void perform() {
			SimbadBumper.this.setTranslationalVelocity(0);
			SimbadBumper.this.rotateY(-0.263); // rad, 15 deg
		}

	}

}
