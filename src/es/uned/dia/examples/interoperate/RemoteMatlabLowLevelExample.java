package es.uned.dia.examples.interoperate;

import es.uned.dia.interoperate.ExternalApp;
import es.uned.dia.interoperate.matlab.jimc.RMatlabExternalApp;
/**
 * Example of use of the low level protocol
 * @author <a href="mailto:gfarias@bec.uned.es">Gonzalo Farias</a>
 */
public class RemoteMatlabLowLevelExample {
  public static void main (String[] args) {
    // Declare local variables 
    double time=0, frequency=1, value=0;
    // Create the ExternalApplication
    ExternalApp externalApp = new RMatlabExternalApp("<matlab:localhost:2005>");
    // Start the connection
    if (!externalApp.connect()) {                 
      System.err.println ("ERROR: Could not connect!");
      return;
    }
    // Set the frequency
    externalApp.setValue("f",frequency);
    // Perform the simulation
    do {                                          
      externalApp.setValue("t",time);
      externalApp.eval("y=sin(2*pi*f*t)*cos(t)");
      value=externalApp.getDouble("y");
      System.out.println("time:"+time+" value:"+value);
      time=time+0.1;
    } while (time<=10);
    // Finish the connection 
    externalApp.disconnect();                           
  } 
}
