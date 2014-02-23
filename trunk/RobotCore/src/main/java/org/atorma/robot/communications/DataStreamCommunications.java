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
            throw new CommunicationException(e);
        }
    }
	
	public void flushDouble(double data) {
        try {
            dos.writeDouble(data);
            dos.flush();
        } catch (IOException e) {
            throw new CommunicationException(e);
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
        	throw new CommunicationException(e);
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
        	throw new CommunicationException(e);
        }
        return buffer;
    }

    public void transmitDoubleArray(double[] data) {
        flushInt(data.length);
        for (int i = 0; i < data.length; i++) {
        	flushDouble(data[i]);
        }
    }
    
    public void closeDataStreams() {
    	try {
    		dis.close();
    		dos.close();
    	} catch (IOException e) {
    		throw new CommunicationException(e);
    	}
    }

	public final boolean readBoolean() {
		try {
			return dis.readBoolean();
		} catch (IOException e) {
			throw new CommunicationException(e);
		}
	}

	public final char readChar() {
		try {
			return dis.readChar();
		} catch (IOException e) {
			throw new CommunicationException(e);
		}
	}

	public final double readDouble() {
		try {
			return dis.readDouble();
		} catch (IOException e) {
			throw new CommunicationException(e);
		}
	}

	public final int readInt() {
		try {
			return dis.readInt();
		} catch (IOException e) {
			throw new CommunicationException(e);
		}
	}

	public final String readUTF()  {
		try {
			return dis.readUTF();
		} catch (IOException e) {
			throw new CommunicationException(e);
		}
	}

	public final void writeBoolean(boolean v) {
		try {
			dos.writeBoolean(v);
		} catch (IOException e) {
			throw new CommunicationException(e);
		}
	}

	public final void writeChar(int v) {
		try {
			dos.writeChar(v);
		} catch (IOException e) {
			throw new CommunicationException(e);
		}
	}

	public final void writeDouble(double v) {
		try {
			dos.writeDouble(v);
		} catch (IOException e) {
			throw new CommunicationException(e);
		}
	}

	public final void writeInt(int v) {
		try {
			dos.writeInt(v);
		} catch (IOException e) {
			throw new CommunicationException(e);
		}
	}

	public final void writeUTF(String s) {
		try {
			dos.writeUTF(s);
		} catch (IOException e) {
			throw new CommunicationException(e);
		}
	}

}
