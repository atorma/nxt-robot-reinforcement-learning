package org.atorma.robot.communications;

import static org.atorma.robot.communications.MessageConstants.*;

import org.atorma.robot.mdp.State;

import lejos.nxt.Button;
import lejos.nxt.comm.BTConnection;
import lejos.nxt.comm.Bluetooth;

/**
 * The NXT side of PC <-> NXT Bluetooth communications. Waits for PC to connect, then initiates all communication.
 */
public class NxtToPcCommunications {
		
	private DataStreamCommunications comms;
	private BTConnection btc = null;

	public NxtToPcCommunications() {
        try {
            System.out.println("Waiting for client.");
            
            btc = Bluetooth.waitForConnection();            
            comms = new DataStreamCommunications(btc.openDataInputStream(), btc.openDataOutputStream());
            
            System.out.println("Connected.");
            
        } catch(Exception e) {
            System.out.println("Error in Bluetooth connection.");      
            Button.ENTER.waitForPressAndRelease();
        }
    }

	public void disconnect() {
		try {
			comms.flushInt(DISCONNECT);
			comms.closeDataStreams();
			btc.close();
		} catch (Exception e) {}
		System.out.println("Disconnected.");
	}

	public int receiveActionIdForCurrentState(State currentState) {
		comms.flushInt(SEND_PERCEPT_RECEIVE_ACTION);
		comms.transmitDoubleArray(currentState.getValues());
		return comms.readInt();
	}


	
}
