/**
 * 
 */
package es.uned.dia.examples.interoperate;

import es.uned.dia.interoperate.ExternalApp;
import es.uned.dia.interoperate.matlab.jimc.RMatlabExternalApp;

/**
 * Example of use of the high level protocol
 * @author <a href="mailto:gfarias@bec.uned.es">Gonzalo Farias</a>
 *
 */
public class RemoteMatlabHighLevelExampleAsynchronous {
  // Declare local variables
  public double time=0, frequency=1, value=0;     

  public static void main (String[] args) {
    new RemoteMatlabHighLevelExampleAsynchronous();
  }
  
  public RemoteMatlabHighLevelExampleAsynchronous() {
    // Create the external application
    ExternalApp externalApp = new RMatlabExternalApp("<matlabas:localhost:2005>");
    // Set the client application  
    externalApp.setClient(this);
    // Link variables with the external app's
    externalApp.linkVariables("time", "t");     
    externalApp.linkVariables("frequency", "f");
    externalApp.linkVariables("value", "y");
    // Configure the external application 
    externalApp.setCommand("y=sin(2*pi*f*t)*cos(t),t=t+0.1");
    // Start the connection
    if (!externalApp.connect()) {                 
      System.err.println ("ERROR: Could not connect!");
      return;
    }       
    // Perform the simulation
    do {                                          
      externalApp.step(1); // step once      
      if (time<5.10 && time>4.9) {frequency=0;externalApp.synchronize();} 
      System.out.println("time:"+time+" value:"+value);
    } while (time<=10);
    // Finish the connection  
    externalApp.disconnect();                              
  } 
}
