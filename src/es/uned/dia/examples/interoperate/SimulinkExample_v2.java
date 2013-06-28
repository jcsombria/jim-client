/**
 * 
 */
package es.uned.dia.examples.interoperate;

import es.uned.dia.interoperate.ExternalApp;
import es.uned.dia.interoperate.matlab.jimc.SimulinkExternalApp;

/**
 * Example of use of the SimulinkExternalApp using only methods of the ExternalApp Interface
 * @author <a href="mailto:gfarias@bec.uned.es">Gonzalo Farias</a>
 *
 */
public class SimulinkExample_v2 {
  // Declare local variables
  public double time=0, frequency=1, value=0;     

  public static void main (String[] args) {
    new SimulinkExample_v2();
  }

  public SimulinkExample_v2() {
    //Createa a Simulink connection
    ExternalApp externalApp=new SimulinkExternalApp("fsmk.mdl");
    
    // Set the client application  
    externalApp.setClient(this);
    
    //Link Java and Simulink variables
    externalApp.linkVariables("frequency","fsmk/product@in@1");
    externalApp.linkVariables("time","fsmk@param@time");
    externalApp.linkVariables("value","fsmk/function@out@1");
    
    // Start the connection
    if (!externalApp.connect()) {                 
      System.err.println ("ERROR: Could not connect!");
      return;
    }
        
    //Perform the simulation
    do{       
        externalApp.step(1);
        System.out.println("time:"+time+" value:"+value);     
    } while (time<10);
    
    //Finish the connection
    externalApp.disconnect();                          
  } 
}
