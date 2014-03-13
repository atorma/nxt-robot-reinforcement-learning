package org.atorma.robot.objecttrackingbumper.prioritizedsweeping;

import static org.junit.Assert.*;

import org.atorma.robot.objecttracking.TrackedObject;
import org.atorma.robot.objecttrackingbumper.ModeledBumperState;
import org.atorma.robot.simplebumper.BumperAction;
import org.atorma.robot.simplebumper.BumperPercept;
import org.junit.Before;
import org.junit.Test;

public class ModeledBumperStateTests {

	private ModeledBumperState state;
	
	@Before
	public void setUp() {
		state = new ModeledBumperState();
	}
	
	@Test
	public void copy_preserves_collision_state() {
		ModeledBumperState copy;
		
		state.setCollided(false);
		copy = (ModeledBumperState) state.copy();
		assertFalse(copy.isCollided());
		
		state.setCollided(true);
		copy = (ModeledBumperState) state.copy();
		assertTrue(copy.isCollided());
		
		state.setCollided(false);
		copy = (ModeledBumperState) state.copyAndChangeNumberOfSectors(1);
		assertFalse(copy.isCollided());
		
		state.setCollided(true);
		copy = (ModeledBumperState) state.copyAndChangeNumberOfSectors(1);
		assertTrue(copy.isCollided());
	}
	
	@Test
	public void when_not_collided_and_action_leads_to_collision_then_estimates_do_change() {
		state.addObservation(TrackedObject.inPolarDegreeCoordinates(10, 90));
		state.setCollided(false);
		BumperPercept percept = new BumperPercept(5, true);
		ModeledBumperState nextState = state.afterActionAndObservation(BumperAction.FORWARD, percept);
		
		assertTrue(nextState.isCollided());
		assertNull(nextState.getObjectInSectorDegree(90));
		assertEquals(5, nextState.getObjectInSectorDegree(0).getDistance(), 0);
	}
	
	@Test
	public void when_already_collided_and_action_leads_to_collision_then_estimates_do_not_change() {
		state.addObservation(TrackedObject.inPolarDegreeCoordinates(10, 90));
		state.setCollided(true);
		BumperPercept percept = new BumperPercept(5, true);
		ModeledBumperState nextState = state.afterActionAndObservation(BumperAction.FORWARD, percept);
		
		assertTrue(nextState.isCollided());
		assertEquals(10, nextState.getObjectInSectorDegree(90).getDistance(), 0);
		assertEquals(5, nextState.getObjectInSectorDegree(0).getDistance(), 0);
	}
}
