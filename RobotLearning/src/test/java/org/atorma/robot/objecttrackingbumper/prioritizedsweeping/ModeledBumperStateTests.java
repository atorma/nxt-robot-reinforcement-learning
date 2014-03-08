package org.atorma.robot.objecttrackingbumper.prioritizedsweeping;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.atorma.robot.objecttrackingbumper.ModeledBumperState;
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
}
