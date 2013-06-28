/**
 * 
 */
package es.uned.dia.examples.interoperate;

import es.uned.dia.interoperate.matlab.jimc.SimulinkExternalApp;

/**
 * Example of use of the SimulinkExternalApp with the Bouncing Ball model
 * @author <a href="mailto:gfarias@bec.uned.es">Gonzalo Farias</a>
 *
 */
public class SimulinkBouncingBallExample {
  // Declare local variables
  public double time=0, position=10, velocity=15;    

  public static void main (String[] args) {
    new SimulinkBouncingBallExample();
  }

  public SimulinkBouncingBallExample() {
    //Createa a Simulink connection
    SimulinkExternalApp externalApp=new SimulinkExternalApp("bounce.mdl");

    // Set the client application  
    externalApp.setClient(this);

    //Link Java and Simulink variables
    externalApp.linkVariables("position","bounce/Position","out","1");
    externalApp.linkVariables("velocity","bounce/Velocity","out","1");
    externalApp.linkVariables("time","bounce","param","time");

    // Start the connection
    if (!externalApp.connect()) {                 
      System.err.println ("ERROR: Could not connect!");
      return;
    }

    boolean firstReset=true;
    //Perform the simulation
    do{
      //Step the model
      externalApp.step(1);
      //reset at time=10
      if (time>=10 && firstReset){
        position=10;velocity=0;
        externalApp.synchronize();
        firstReset=false;
      }
      System.out.println("time:"+time+" position:"+position+" velocity:"+velocity);
    }while (time<13);     

    //Finish the connection
    externalApp.disconnect();                          
  } 
}
