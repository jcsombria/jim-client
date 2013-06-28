/**
 * 
 */
package es.uned.dia.examples.interoperate;

import es.uned.dia.interoperate.matlab.jimc.RSimulinkExternalApp;

/**
 * Example of use of the RSimulinkExternalApp in mode synchronous
 * @author <a href="mailto:gfarias@bec.uned.es">Gonzalo Farias</a>
 *
 * Start the Jim server and copy the fsmk.mdl Simulink model into the work directory.
 */
public class RemoteSimulinkExample_v1 {
  // Declare local variables
  public double time=0, frequency=1, value=0;     

  public static void main (String[] args) {
    new RemoteSimulinkExample_v1();
  }

  public RemoteSimulinkExample_v1() {
    //Createa a Simulink connection
    RSimulinkExternalApp externalApp=new RSimulinkExternalApp("<matlab:localhost:2005>fsmk.mdl");

    // Set the client application  
    externalApp.setClient(this);

    //Link Java and Simulink variables
    externalApp.linkVariables("frequency","fsmk/product","in","1");
    externalApp.linkVariables("time","fsmk","param","time");
    externalApp.linkVariables("value","fsmk/function","out","1");
    //Delete Scope block
    externalApp.deleteBlock("fsmk/Scope");

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
