package org.atorma.robot.communications;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;

public class DataStreamCommunications {
	
	public final static int DISCONNECT = -1;
	public final static int STATE_AND_ACTION = 10;
	public final static int POLICY_VALUES = 20;
	
	protected DataOutputStream dos;
	protected DataInputStream dis;
	
	public DataStreamCommunications(DataInputStream dis, DataOutputStream dos) {
		this.dis = dis;
		this.dos = dos;
	}
	
	

	public void flushInt(int data) {
        try {
            dos.writeInt(data);
            dos.flush();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }
	
	public void flushDouble(double data) {
        try {
            dos.writeDouble(data);
            dos.flush();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    public int[] receiveIntArray() {
        int[] buffer;
		try {
            int numValues = dis.readInt();
            buffer =  new int[numValues];
            for(int i = 0; i < numValues; i++) {
                buffer[i] = dis.readInt();
            }
        } catch (IOException e) {
        	throw new RuntimeException(e);
        }
        return buffer;
    }

    public void transmitIntArray(int[] data) {
        flushInt(data.length);
        for (int i = 0; i < data.length; i++) {
        	flushInt(data[i]);
        }
    }
    
    public double[] receiveDoubleArray() {
        double[] buffer;
		try {
            int numValues = dis.readInt();
            buffer =  new double[numValues];
            for(int i = 0; i < numValues; i++) {
                buffer[i] = dis.readDouble();
            }
        } catch (IOException e) {
        	throw new RuntimeException(e);
        }
        return buffer;
    }

    public void transmitDoubleArray(double[] data) {
        flushInt(data.length);
        for (int i = 0; i < data.length; i++) {
        	flushDouble(data[i]);
        }
    }
    
    public void closeDataStreams() throws IOException {
    	dis.close();
    	dos.close();
    }

	public final boolean readBoolean() throws IOException {
		return dis.readBoolean();
	}

	public final char readChar() throws IOException {
		return dis.readChar();
	}

	public final double readDouble() throws IOException {
		return dis.readDouble();
	}

	public final int readInt() throws IOException {
		return dis.readInt();
	}

	public final String readUTF() throws IOException {
		return dis.readUTF();
	}

	public final void writeBoolean(boolean v) throws IOException {
		dos.writeBoolean(v);
	}

	public final void writeChar(int v) throws IOException {
		dos.writeChar(v);
	}

	public final void writeDouble(double v) throws IOException {
		dos.writeDouble(v);
	}

	public final void writeInt(int v) throws IOException {
		dos.writeInt(v);
	}

	public final void writeUTF(String s) throws IOException {
		dos.writeUTF(s);
	}

}
