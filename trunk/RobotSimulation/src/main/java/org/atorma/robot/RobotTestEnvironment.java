package org.atorma.robot;

import javax.vecmath.Vector3d;
import javax.vecmath.Vector3f;

import simbad.sim.Box;
import simbad.sim.EnvironmentDescription;
import simbad.sim.Wall;

public class RobotTestEnvironment extends EnvironmentDescription {

	public RobotTestEnvironment(SimbadRobot robot) {
		add(robot);
		
		setWorldSize(4);
		
		usePhysics = true;
		
		light1IsOn = true;
        light2IsOn = false;
        Wall w1 = new Wall(new Vector3d(2, 0, 0), 4, 0.1f, this);
        w1.rotate90(1);
        add(w1);
        Wall w2 = new Wall(new Vector3d(-2, 0, 0), 4, 0.1f, this);
        w2.rotate90(1);
        add(w2);
        Wall w3 = new Wall(new Vector3d(0, 0, 2), 4, 0.1f, this);
        add(w3);
        Wall w4 = new Wall(new Vector3d(0, 0, -2), 4, 0.1f, this);
        add(w4);
        Box b1 = new Box(new Vector3d(-1, 0, -1), new Vector3f(0.3f, 0.2f, 0.5f), this);
        add(b1);
        Box b2 = new Box(new Vector3d(0.8, 0, -0.4), new Vector3f(0.5f, 0.2f, 0.3f), this);
        add(b2);
        Box b3 = new Box(new Vector3d(1.2, 0, -1.2), new Vector3f(0.3f, 0.2f, 0.2f), this);
        add(b3);
        Box b4 = new Box(new Vector3d(1, 0, 1), new Vector3f(0.3f, 0.2f, 0.4f), this);
        add(b4);
        Box b5 = new Box(new Vector3d(-1, 0, 0.7), new Vector3f(0.3f, 0.2f, 0.4f), this);
        add(b5);
	}
	
	
}
