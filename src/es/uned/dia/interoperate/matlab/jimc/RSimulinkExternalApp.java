package es.uned.dia.interoperate.matlab.jimc;

import java.io.*;
import java.util.Enumeration;
import java.util.jar.JarEntry;
import java.util.jar.JarFile;

/**
 * Class to implement the communication protocol with remote Simulink
 * @author <a href="mailto:gfarias@bec.uned.es">Gonzalo Farias</a>
 * @author <a href="mailto:fem@um.es">Francisco Esquembre</a> 
 */
public class RSimulinkExternalApp extends RMatlabExternalApp {

  protected String model; // The complete name of the model
  protected String theModel; // The name of the model without directory or the '.mdl' extension

  private boolean resetIC = false; // Whether to reset initial conditions of integrator blocks
  private boolean resetParam=false; // Whether to reset parameters of a block  
  private String theMFile = null; // The mFile as a command for Matlab workspace
  private double EJS_TIME; // The time of the Simulink model
  private boolean resetSystem=true; //Whether the model has been reset
  private String originalModel=""; //The name of the original model
  private double fixedStep=0;   //The size of the fixed step mode
  private boolean fixedUpdateStep=false; //Whether the model is on fixed step mode
  private final String EJS_PREFIX="Ejs_"; //A prefix for the modified blocks on the Simulink model

  /*********************
     Low level protocol
   **********************/

  //--------------------------------------------------------------
  //Connection and configuration
  //--------------------------------------------------------------

  //--------------------------------------------------------------
  //Setting and getting values
  //--------------------------------------------------------------  

  /**
   * Sets the value of the given variable of the application
   * @param variable String the variable name
   * @param value String the desired value
   */
  public void setValue(String variable, String value) {
    try{    
      if (!asynchronousSimulation){
        bufferOutputTCP.writeInt(EjsId);
        bufferOutputTCP.writeUTF("setValueString");
        bufferOutputTCP.writeUTF(variable);
        bufferOutputTCP.writeUTF(value);
        bufferOutputTCP.flush();
      }else if (resetIC || resetParam) setValue(variable, value, false);
    }catch(Exception e){
      System.out.println("Error setValue Remote Matlab "+e);
    } 
  }

  /**
   * Gets the value of a String variable of the application
   * @param variable String the variable name
   * @return String the value
   */
  public String getString(String variable) {    
    if (asynchronousSimulation) return getStringAS();
    String charArray = null;
    try{ 
      bufferOutputTCP.writeInt(EjsId);
      bufferOutputTCP.writeUTF("getString");
      bufferOutputTCP.writeUTF(variable);
      bufferOutputTCP.flush();
      charArray = (String) bufferInputTCP.readUTF();
    }catch(Exception e){
      System.out.println("Error getString Remote Matlab "+e);
    }       
    return (charArray);
  }

  /**
   * Sets the value of the given variable of the application
   * @param variable String the variable name
   * @param value double the desired value
   */ 
  public void setValue(String variable, double value) {
    try{
      if (!asynchronousSimulation){
        bufferOutputTCP.writeInt(EjsId);
        bufferOutputTCP.writeUTF("setValueDouble");
        bufferOutputTCP.writeUTF(variable);
        bufferOutputTCP.writeDouble(value);
        bufferOutputTCP.flush();
      }else if (resetIC || resetParam) setValue(variable, value, false);
    }catch(Exception e){
      System.out.println("Error setValue Remote Matlab "+e);
    } 
  }  

  /**
   * Gets the value of a double variable of the application
   * @param variable String the variable name
   * @return double the value
   */
  public double getDouble(String variable) {
    if (asynchronousSimulation) {
      if (variable.equalsIgnoreCase("ejs_time")) {
        EJS_TIME= getDoubleAS();
        return EJS_TIME;
      }      
      return getDoubleAS();
    }
    double valueDouble = 0.0;
    try{
      bufferOutputTCP.writeInt(EjsId);
      bufferOutputTCP.writeUTF("getDouble");
      bufferOutputTCP.writeUTF(variable);
      bufferOutputTCP.flush();
      valueDouble = bufferInputTCP.readDouble();
    }catch (Exception e){}        
    return (valueDouble);
  }

  /**
   * Sets the value of the given variable of the application
   * @param variable String the variable name
   * @param value double[] the desired value
   */
  public void setValue(String variable, double[] value) {
    try{      
      if (!asynchronousSimulation){
        bufferOutputTCP.writeInt(EjsId);
        bufferOutputTCP.writeUTF("setValueD[]");
        bufferOutputTCP.writeUTF(variable);
        bufferOutputTCP.writeInt(value.length);
        for (int i = 0; i < value.length; i++) {
          bufferOutputTCP.writeDouble(value[i]);
        }
        bufferOutputTCP.flush();
      }else if (resetIC || resetParam) setValue(variable, value, false);
    }catch(Exception e){
      System.out.println("Error setValue Remote Matlab "+e);
    } 
  }

  /**
   * Gets the value of a double[] variable of the application
   * @param variable String the variable name
   * @return double the value
   */
  public double[] getDoubleArray (String variable) {

    if (asynchronousSimulation) return getDoubleArrayAS();

    int _dim1;
    double[] vectorDouble=null;
    try{
      bufferOutputTCP.writeInt(EjsId);
      bufferOutputTCP.writeUTF("getDoubleArray");
      bufferOutputTCP.writeUTF(variable);
      bufferOutputTCP.flush();
      _dim1 = bufferInputTCP.readInt();
      vectorDouble = new double[_dim1];
      for (int i = 0; i < _dim1; i++) {
        vectorDouble[i] = bufferInputTCP.readDouble();
      }
    }catch (Exception e){}
    return(vectorDouble);
  }

  /**
   * Sets the value of the given variable of the application
   * @param variable String the variable name
   * @param value double[][] the desired value
   */
  public void setValue(String variable, double[][] value) {    
    try{
      if (!asynchronousSimulation){
        bufferOutputTCP.writeInt(EjsId);
        bufferOutputTCP.writeUTF("setValueD[][]");
        bufferOutputTCP.writeUTF(variable);

        bufferOutputTCP.writeInt(value.length);
        bufferOutputTCP.writeInt(value[0].length);
        for (int i = 0; i < value.length; i++) {
          for (int j = 0; j < value[0].length; j++) {
            bufferOutputTCP.writeDouble(value[i][j]);
          }
        }
        bufferOutputTCP.flush();
      }else if (resetIC || resetParam) setValue(variable, value, false);
    }catch(Exception e){
      System.out.println("Error setValue Remote Matlab "+e);
    }          
  }

  /**
   * Gets the value of a double[][] variable of the application
   * @param variable String the variable name
   * @return double the value
   */
  public double[][] getDoubleArray2D (String variable)  {
    if (asynchronousSimulation) return getDoubleArray2DAS();
    int _dim1,_dim2;
    double[][] arrayDouble=null;
    try{
      bufferOutputTCP.writeInt(EjsId);
      bufferOutputTCP.writeUTF("getDoubleArray2D");
      bufferOutputTCP.writeUTF(variable);
      bufferOutputTCP.flush();
      _dim1 = bufferInputTCP.readInt();
      _dim2 = bufferInputTCP.readInt();
      arrayDouble = new double[_dim1][_dim2];
      for (int i = 0; i < _dim1; i++) {
        for (int j = 0; j < _dim2; j++) {
          arrayDouble[i][j] = bufferInputTCP.readDouble();
        }
      }
    }catch (Exception e){}
    return(arrayDouble);
  }

  //--------------------------------------------------------------
  //Control commands
  //--------------------------------------------------------------

  /**
   * Resets the application
   */
  public void reset() {
    resetSystem=true;
    stop();
    if (model != null) eval("bdclose ('all')");
    eval("clear all");
    openModel();
    if (theMFile != null) eval(theMFile);
  }

  //--------------------------------------------------------------
  //Other low level methods
  //-------------------------------------------------------------- 

  /*********************
    High level protocol
   **********************/

  //--------------------------------------------------------------
  //Linking variables
  //--------------------------------------------------------------

  /**
   * Links a client variable with a variable of the External Application 
   * @param clientVariable String the client variable
   * @param externalVariable String the external variable (use separator '@' for Simulink models)
   * @return true if the client variable exists
   */
  public boolean linkVariables(String clientVariable, String externalVariable){
    String[] pathTypePort=externalVariable.split("@");
    if (pathTypePort.length==1) return super.linkVariables(clientVariable,externalVariable);
    else if (pathTypePort.length==3) return linkVariables(clientVariable, pathTypePort[0], pathTypePort[1], pathTypePort[2]);
    else return false;
  } 

  //--------------------------------------------------------------
  //Control commands
  //--------------------------------------------------------------

  /**
   * Synchronizes client and external applications
   */
  public void synchronize(){
    if (modeAS){
      removeBuffer=true;
      asynchronousSimulation=false;
    }else resetIC(); 
  }

  //--------------------------------------------------------------
  //Remote methods
  //-------------------------------------------------------------- 

  //--------------------------------------------------------------
  //Other high level methods
  //--------------------------------------------------------------

  //--------------------------------------------------------------
  //Other public methods
  //-------------------------------------------------------------- 

  /**
   * sets the size of the fixed step simulation
   * @param fixedStepSize a double to set the size of the step
   */
  public void fixedStep(double fixedStepSize){
    fixedUpdateStep=true;
    fixedStep=fixedStepSize;
  }

  /**
   * sets all external variables with client values
   */
  public void setValues(){    
    int k=0;
    String[] var;
    String evar;    
    //Set Values   
    for (int i=0; i<linkVector.size(); i++){
      var= (String[]) linkVector.elementAt(i);                                  
      if (var.length==2) evar=var[1]; //Matlab Variables
      else { 
        evar=EJS_PREFIX+var[0];    //Simulink Variables                         
      }           
      try {       
        varContextField= varContextFields[linkIndex[k]];      
        switch (linkType[k++]){
          case DOUBLE: setValue(evar,varContextField.getDouble(varContextObject)); break;
          case ARRAYDOUBLE: setValue(evar,(double[])varContextField.get(varContextObject)); break;
          case ARRAYDOUBLE2D: setValue(evar,(double[][])varContextField.get(varContextObject)); break;
          case STRING: setValue(evar,(String)varContextField.get(varContextObject)); break; 
        }         
      } catch (java.lang.IllegalAccessException e) {
        System.out.println("Error Step: setting a value " + e);
      }           
    }
  }

  /**
   * gets all external values
   */  
  public void getValues(){
    //Get Values   
    int k=0;
    String[] var;
    String evar;    
    for (int i=0; i<linkVector.size(); i++){
      var= (String[]) linkVector.elementAt(i);        
      if (var.length==2) evar=var[1]; //Matlab Variables
      else {
        evar=EJS_PREFIX+var[0];    //Simulink Variables 
        if (!var[2].equalsIgnoreCase("out")) {
          k++;
          continue; 
        }                               
      }
      try {       
        varContextField= varContextFields[linkIndex[k]];      
        switch (linkType[k++]){
          case DOUBLE: varContextField.setDouble(varContextObject,getDouble(evar)); break;
          case ARRAYDOUBLE: varContextField.set(varContextObject,getDoubleArray(evar));  break;
          case ARRAYDOUBLE2D: varContextField.set(varContextObject,getDoubleArray2D(evar)); break;
          case STRING: varContextField.set(varContextObject,getString(evar)); break;        
        }          
      } catch (java.lang.IllegalAccessException e) {
        System.out.println("Error Step: getting a value " + e);
      }           
    }                
  }

  /**
   * Package-internal constructor with configuration String
   * @param remoteConfiguration a String to set the remote server
   */
  public RSimulinkExternalApp(String remoteConfiguration) {
    EjsId = 2005;
    remoteConfiguration=remoteConfiguration.trim().replace("<","").replace(">",":");
    String[] params=remoteConfiguration.split(":");

    if (params.length!=4) {
      System.out.println("Error of Input Arguments");
      System.out.println("------------------------");
      System.out.println("Arguments for synchronous mode = \n <matlab:ipAddress:ipPort>aSimulinkModel");
      System.out.println("Arguments for asynchronous mode = \n <matlabas:ipAddress:ipPort>aSimulinkModel");
      return;
    }
    if (params[0].equals("matlabas")) modeAS=true; 
    String sAdress=params[1];
    String sPort=params[2];
    SERVICE_IP = sAdress;
    try {
      SERVICE_PORT = Integer.parseInt(sPort);
    }
    catch (NumberFormatException nfe) {
      System.out.println("Error in Port:" + nfe);
      return;
    }
    model = params[3];
    if (!model.toLowerCase().endsWith(".mdl")) model += ".mdl";
    originalModel=model; 
    theModel = model.substring(0,model.lastIndexOf('.'));

    //Gets the model from the jar base    
    try{ 
      File checkfile = new File(new java.net.URI(getBaseDirectory()+"/"+model));                      
      if (!checkfile.exists()){                                  
        File modelFile = new File(new java.net.URI(getBaseDirectory()+"/"+model));              
        //lookfor jimc.jar  
        JarFile jarbase = new JarFile(new File (this.getClass().getProtectionDomain().getCodeSource().getLocation().toURI()));
        JarEntry entry1;
        for (Enumeration<JarEntry> e = jarbase.entries(); e.hasMoreElements(); ) {
          entry1 = (JarEntry) e.nextElement();
          if (entry1.getName().endsWith(model)) {
            //get jimc.jar
            modelFile.createNewFile();
            InputStream in = this.getClass().getResourceAsStream("/"+entry1.getName());         
            FileOutputStream out = new FileOutputStream(modelFile);
            byte[] buffer = new byte[1048];
            while(in.available() > 0) {
              int read = in.read(buffer);
              out.write(buffer, 0, read);
            }
            out.close();
            modelFile.deleteOnExit();
            break;                          
          }                        
        }
      }                        
    }
    catch (java.io.IOException e){System.out.println("Error:"+e);} 
    catch (Exception e) { e.printStackTrace(); }
  }

  /**
   * Package-internal constructor with configuration String (with fixed step mode)
   * @param remoteConfiguration a String to set the remote server
   * @param fixedStepSize a double to set the size of the step
   */
  public RSimulinkExternalApp (String remoteConfiguration, double fixedStepSize) {    
    EjsId = 2005;
    remoteConfiguration=remoteConfiguration.trim().replace("<","").replace(">",":");
    String[] params=remoteConfiguration.split(":");    
    if (params.length!=4) {
      System.out.println("Error of Input Arguments");
      System.out.println("------------------------");
      System.out.println("Arguments for synchronous mode = \n <matlab:ipAddress:ipPort>aSimulinkModel");
      System.out.println("Arguments for asynchronous mode = \n <matlabas:ipAddress:ipPort>aSimulinkModel");
      return;
    }
    if (params[0].equals("matlabas")) modeAS=true;
    String sAdress=params[1];
    String sPort=params[2];
    SERVICE_IP = sAdress;
    try {
      SERVICE_PORT = Integer.parseInt(sPort);
    }
    catch (NumberFormatException nfe) {
      System.out.println("Error in Port:" + nfe);
      return;
    }
    model = params[3];
    if (!model.toLowerCase().endsWith(".mdl")) model += ".mdl";
    originalModel=model; 
    theModel = model.substring(0,model.lastIndexOf('.'));                   
    fixedStep=fixedStepSize;
    if (fixedStep<=0) fixedStep=1;
    fixedUpdateStep=true;    
    //Gets the model from the jar base    
    try{ 
      File checkfile = new File(new java.net.URI(getBaseDirectory()+"/"+model));                      
      if (!checkfile.exists()){                                  
        File modelFile = new File(new java.net.URI(getBaseDirectory()+"/"+model));                               
        //lookfor jimc.jar  
        JarFile jarbase = new JarFile(new File (this.getClass().getProtectionDomain().getCodeSource().getLocation().toURI()));
        JarEntry entry1;
        for (Enumeration<JarEntry> e = jarbase.entries(); e.hasMoreElements(); ) {
          entry1 = (JarEntry) e.nextElement();
          if (entry1.getName().endsWith(model)) {
            //get jimc.jar
            modelFile.createNewFile();
            InputStream in = this.getClass().getResourceAsStream("/"+entry1.getName());         
            FileOutputStream out = new FileOutputStream(modelFile);
            byte[] buffer = new byte[1048];
            while(in.available() > 0) {
              int read = in.read(buffer);
              out.write(buffer, 0, read);
            }
            out.close();
            modelFile.deleteOnExit();
            break;                          
          }                        
        }
      }                        
    }
    catch (java.io.IOException e){System.out.println("Error:"+e);} 
    catch (Exception e) { e.printStackTrace(); }
  }

  /**
   * Starts an authorized connection with the remote external application
   * @param user The user
   * @param pwd The password
   * @return boolean true if the the connection was successful
   */    
  public boolean connect (String user, String pwd){
    model=originalModel;
    int result=-1;

    if (jimTCP!=null){
      if (!jimTCP.isClosed()) {resultOfLastAction=CONNECTION_OK;return true;} //user ok
    }
    try {
      jimTCP = new java.net.Socket(SERVICE_IP, SERVICE_PORT);
      bufferInputTCP = new DataInputStream(new BufferedInputStream(jimTCP.getInputStream()));
      bufferOutputTCP = new DataOutputStream(new BufferedOutputStream(jimTCP.getOutputStream()));
      jimTCP.setSoTimeout(connectionTimeOut);
      jimTCP.setTcpNoDelay(true);

      //Configure buffer sizes
      int buffin=bufferInputTCP.readInt();
      int buffout=bufferInputTCP.readInt();
      jimTCP.setSendBufferSize(buffout);
      jimTCP.setReceiveBufferSize(buffin);

      //Is there any authorization requried?
      Boolean aut=bufferInputTCP.readBoolean();
      long rtime=-1;
      //Authentication       
      if (!aut){
        result=2;
      }else{
        bufferOutputTCP.writeUTF(user);
        bufferOutputTCP.writeUTF(pwd);
        bufferOutputTCP.flush();
        result=bufferInputTCP.readInt();
        rtime=bufferInputTCP.readLong();
      }      
      if (result==2) {
        System.out.println("Successful Connection");
        System.out.println("Time available:"+(rtime/(1000*60))+" minutes");
        connected = true;
        resultOfLastAction=CONNECTION_OK;        
      }
      else {
        System.out.println("Authentication Error");        
        disconnect();
        if (result==0) {resultOfLastAction=connectionAuthenticationFail; return false;}
        else {resultOfLastAction=connectionSlotFail;return true;}        
      }
    }
    catch (IOException ioe) {
      System.out.println("Error " + ioe);
      connected=false;
      resultOfLastAction=connectionNoServer;
      return false;
    }
    catch (Exception e2) {
      System.out.println("Error " + e2);
      connected=false;
      resultOfLastAction=connectionNoServer;
      return false;
    }

    //Gets the model from local disk or checks if it exists in remote server
    try{ 
      File modelFile = new File(new java.net.URI(getBaseDirectory()+"/"+model));                      
      if (!modelFile.exists()){                            
        if (!remoteFileExist(model)) {
          System.out.println("Error Model " + model +
          " doesn't exist in Local or Remote Place");
          model = null;
          connected=false;
          quit();
          resultOfLastAction=connectionNoModel;
          return false;
        }
      } else{
        createModel(getBaseDirectory()+"/",modelFile.length());
      }        
    }catch (Exception e) { e.printStackTrace(); }

    openModel();
    resetIC();
    resultOfLastAction=CONNECTION_OK;
    return true;   
  }

  /**
   * Links a client variable with a variable of the External Application 
   * @param ivar the name of the client variable
   * @param epath the path of the block
   * @param etype the string 'in', 'out' or 'param' for an input, output or parameter of the block respectively.  
   * @param eport the port if the type is an input or output, or the name of the parameter if the type is parameter.
   * @return true if the client variable exists
   */
  public boolean linkVariables(String ivar, String epath, String etype, String eport){
    if (varContextObject==null) return (false);               
    int type;
    //Search if the ivar exists
    for (int i=0; i < varContextFields.length; i++) {

      if (ivar.equals((varContextFields[i]).getName())) {
        //Detect type
        if (varContextFields[i].getType().getName().equals("double")) type=DOUBLE;
        else if (varContextFields[i].getType().getName().equals("[D")) type=ARRAYDOUBLE;
        else if (varContextFields[i].getType().getName().equals("[[D")) type=ARRAYDOUBLE2D;
        else if (varContextFields[i].getType().getName().equals("java.lang.String")) type=STRING;
        else return (false);

        if (linkVector==null) {
          linkVector=new java.util.Vector<String[]>();
          linkIndex= new int[1];
          linkIndex[0]=i;
          linkType = new int[1]; 
          linkType[0]=type;               
        }else{
          int[] _linkIndex=new int[linkIndex.length+1];
          System.arraycopy(linkIndex,0,_linkIndex,0,linkIndex.length);
          _linkIndex[linkIndex.length]=i; 
          linkIndex=_linkIndex;
          int[] _linkType=new int[linkType.length+1];
          System.arraycopy(linkType,0,_linkType,0,linkType.length);
          _linkType[linkType.length]=type;  
          linkType=_linkType;         
        }       

        if (epath.equals(theModel) && etype.equalsIgnoreCase("param") && eport.equalsIgnoreCase("time")) {

          String[] _element={ivar,EJS_PREFIX+"time"};               
          linkVector.addElement(_element); 
          return (true);  
        }
        //add to string initCommand 
        String evarInitValue=EJS_PREFIX+ivar+"=";
        try {    
          switch (type){
            case DOUBLE: evarInitValue=evarInitValue+varContextFields[i].getDouble(varContextObject);break;
            case ARRAYDOUBLE:
              double[] aux_value=(double[])varContextFields[i].get(varContextObject);
              String aux_valueString="[";
              for (int w=0;w<aux_value.length;w++){             
                aux_valueString=aux_valueString+aux_value[w]+",";
              }
              aux_valueString=aux_valueString+"]";
              evarInitValue=evarInitValue+aux_valueString;
              break;                                                                            
            case ARRAYDOUBLE2D:
              double[][] aux_value2D=(double[][])varContextFields[i].get(varContextObject);
              String aux_valueString2D="[";
              for (int w=0;w<aux_value2D.length;w++){
                for (int y=0;y<aux_value2D[0].length;y++){                          
                  aux_valueString2D=aux_valueString2D+aux_value2D[w][y]+",";                  
                }
                aux_valueString2D=aux_valueString2D+";";                 
              }
              aux_valueString2D=aux_valueString2D+"]";
              evarInitValue=evarInitValue+aux_valueString2D;
              break;                                    
            case STRING:
              evarInitValue=evarInitValue+"'"+(String)varContextFields[i].get(varContextObject)+"'"; break; 
          }  
          if (initCommand==null) initCommand="";
          if (etype.equalsIgnoreCase("param")) etype="Param";
          if (etype.equalsIgnoreCase("in")) etype="In";
          if (etype.equalsIgnoreCase("out")) etype="Out";

          initCommand=initCommand+evarInitValue+";"
          +"variables.path{end+1,1}='"+epath+"';"
          +"variables.name{end+1,1}='"+EJS_PREFIX+ivar+"';"
          +"variables.fromto{end+1,1}='"+etype+"';"
          +"variables.port{end+1,1}='"+eport+"';"; 

          String[] _element={ivar,epath,etype,eport};                
          linkVector.addElement(_element);                   
          return (true);
        }catch (java.lang.IllegalAccessException e) {
          System.out.println("Error Getting Initial Values " + e);
          return (false);       
        }
      }
    }       
    return(false);
  }

  /**
   * Deletes a block
   * @param epath the path of the Simulink block
   */
  public void deleteBlock(String epath){                                  
    initCommand=initCommand
    +"variables.path{end+1,1}='"+epath+"';"
    +"variables.name{end+1,1}='';"
    +"variables.fromto{end+1,1}='delete';"
    +"variables.port{end+1,1}='';"; 
  }


  //--------------------------------------------------------------
  //Protected method
  //-------------------------------------------------------------- 

  /**
   * Initializes a Simulink session. 
   */
  protected void initialize () {
    if (initCommand!=null) {
      eval("clear all;variables.path={};variables.name={};variables.fromto={};variables.port={};",false);
      eval("Ejs_random_model_name='"+theModel+"'");
      eval( initCommand,false);     
      eval("Ejs__ResetIC = 0",false);
      eval("sistema=gcs",false);
      eval(conversionCommand,true);
    }
  }

  /**
   * Steps the model synchronously
   */
  protected void synchronousStep (double steps) {
    setValues();
    asynchronousSimulation=false;
    if (theModel==null) return;
    asynchronousSimulation=false;
    try{
      bufferOutputTCP.writeInt (EjsId);
      bufferOutputTCP.writeUTF ("step");
      bufferOutputTCP.writeUTF (theModel);
      bufferOutputTCP.writeBoolean (resetIC);
      resetIC=false;
      bufferOutputTCP.writeBoolean (resetParam);
      resetParam=false;
      bufferOutputTCP.writeInt ((int)steps);
    }
    catch (Exception e) {
      System.out.println(" stepModel Remote Exception:"+e);
    }
    getValues();
  }


  /**
   * step the model asynchronously
   */
  protected void asynchronousStep (double dt) {
    int steps=(int)dt;
    if (!asynchronousSimulation || (resetIC || resetParam )){
      haltStepAS(removeBuffer);      
      setValues();              
      if (!resetSystem){
        try {
          setValue("Ejs_time",EJS_TIME,false);
        }
        catch (Exception e){};
      }
      resetSystem=false;
      eval("Ejs_xFinal=[]",false);
      eval ("set_param ('"+theModel+"', 'LoadInitialState','on')",false);
      eval ("set_param ('"+theModel+"', 'InitialState','Ejs_xFinal')",false);
      eval ("set_param ('"+theModel+"', 'SaveFinalState','on')",false);
      eval ("set_param ('"+theModel+"', 'FinalStateName','Ejs_xFinal')",false);
      try{
        bufferOutputTCP.writeInt (EjsId);
        bufferOutputTCP.writeUTF ("stepAS");
        bufferOutputTCP.writeUTF (theModel);
        bufferOutputTCP.writeBoolean (resetIC);
        resetIC=false;
        bufferOutputTCP.writeBoolean (resetParam);
        resetParam=false;
        bufferOutputTCP.writeInt (steps);
        bufferOutputTCP.writeInt (packageSize);
        bufferOutputTCP.flush();
      }catch (Exception e) {
        System.out.println("Remote Step Asynchronous Exception:"+e);
      }
      asynchronousSimulation=true;
    }
    getValues();
  }

  /**
   * step the model asynchronously (including the size of the package)
   */
  protected void asynchronousStep (double dt, int _package) {
    int steps=(int)dt;
    if (!asynchronousSimulation || (resetIC || resetParam )){
      try {
        setValue("Ejs_time",EJS_TIME,false); }
      catch (Exception e){};
      eval("Ejs_xFinal=[]",false);
      eval ("set_param ('"+theModel+"', 'LoadInitialState','on')",false);
      eval ("set_param ('"+theModel+"', 'InitialState','Ejs_xFinal')",false);
      eval ("set_param ('"+theModel+"', 'SaveFinalState','on')",false);
      eval ("set_param ('"+theModel+"', 'FinalStateName','Ejs_xFinal')",false);
      try{
        bufferOutputTCP.writeInt (EjsId);
        bufferOutputTCP.writeUTF ("stepAS");
        bufferOutputTCP.writeUTF (theModel);
        bufferOutputTCP.writeInt (_package);
        bufferOutputTCP.writeBoolean (resetIC);
        resetIC=false;
        bufferOutputTCP.writeBoolean (resetParam);
        resetParam=false;
        bufferOutputTCP.writeInt (steps);
        bufferOutputTCP.flush();
      }catch (Exception e) {
        System.out.println("Remote Step Asynchronous Exception:"+e);
      }
      asynchronousSimulation=true;
    }
  }

  /**
   * Stops the execution of the remote Simulink model
   * @param _remove boolean true to delete all data in the buffer
   */
  protected void haltStepAS(boolean _remove){
    byte outFirma1=(byte)((int)(Math.random()*255)),outFirma2=(byte)((int)(Math.random()*255)),outFirma3=(byte)((int)(Math.random()*255));
    byte[] inFirma=new byte[3];
    try{
      bufferOutputTCP.writeInt (EjsId);
      bufferOutputTCP.writeUTF ("haltStepAS");
      bufferOutputTCP.writeBoolean (_remove);
      bufferOutputTCP.flush();
    }
    catch (Exception e) {
      System.out.println(" Halt Remote Step Asynchronous Exception:"+e);
    }
    asynchronousSimulation=false; 
    if (_remove){
      try{
        int buffsizein=bufferInputTCP.available();
        while (buffsizein>0){
          bufferInputTCP.skip(buffsizein);
          buffsizein= bufferInputTCP.available();
        }
        bufferOutputTCP.writeByte(outFirma1);
        bufferOutputTCP.writeByte(outFirma2);
        bufferOutputTCP.writeByte(outFirma3);
        bufferOutputTCP.flush();
        inFirma[0]= bufferInputTCP.readByte();
        inFirma[1]= bufferInputTCP.readByte();
        inFirma[2]= bufferInputTCP.readByte();
        boolean synOK=false;
        while (!synOK){
          if ((inFirma[0]==outFirma1) && (inFirma[1]==outFirma2) && (inFirma[2]==outFirma3))
            synOK=true;
          else {
            inFirma[0]=inFirma[1];
            inFirma[1]=inFirma[2];
            inFirma[2]=bufferInputTCP.readByte();
          }
        }
      }catch (Exception e) {
        System.out.println("Halt Remote Step Asynchronous Exception:"+e);
      }
    }
  }
  //--------------------------------------------------------------
  //private methods
  //-------------------------------------------------------------- 

  /**
   * sets a string controlling the flush
   */
  private void setValue(String _name, String _value, boolean _flushNow) {
    try{                                
      bufferOutputTCP.writeInt(EjsId);
      bufferOutputTCP.writeUTF("setValueString");
      bufferOutputTCP.writeUTF(_name);
      bufferOutputTCP.writeUTF(_value);
      if (_flushNow)
        bufferOutputTCP.flush();
    }catch(Exception e){
      System.out.println("Error setValue Remote Matlab "+e);
    } 
  }

  /**
   * sets a double controlling the flush
   */
  private void setValue(String _variable, double _value, boolean _flushNow) {
    try{
      bufferOutputTCP.writeInt(EjsId);
      bufferOutputTCP.writeUTF("setValueDouble");
      bufferOutputTCP.writeUTF(_variable);
      bufferOutputTCP.writeDouble(_value);
      if (_flushNow)
        bufferOutputTCP.flush();
    }catch(Exception e){
      System.out.println("Error setValue Remote Matlab "+e);
    } 
  }

  /**
   * sets a double array controlling the flush
   */
  private void setValue(String _name, double[] _value, boolean _flushNow) {
    try {                                 
      bufferOutputTCP.writeInt(EjsId);
      bufferOutputTCP.writeUTF("setValueD[]");
      bufferOutputTCP.writeUTF(_name);
      bufferOutputTCP.writeInt(_value.length);
      for (int i = 0; i < _value.length; i++) {
        bufferOutputTCP.writeDouble(_value[i]);
      }
      if (_flushNow)
        bufferOutputTCP.flush();
    }catch(Exception e){
      System.out.println("Error setValue Remote Matlab "+e);
    } 
  }

  /**
   * sets a double matrix controlling the flush
   */
  private void setValue(String _name, double[][] _value, boolean _flushNow) {
    try{                                
      bufferOutputTCP.writeInt(EjsId);
      bufferOutputTCP.writeUTF("setValueD[][]");
      bufferOutputTCP.writeUTF(_name);

      bufferOutputTCP.writeInt(_value.length);
      bufferOutputTCP.writeInt(_value[0].length);
      for (int i = 0; i < _value.length; i++) {
        for (int j = 0; j < _value[0].length; j++) {
          bufferOutputTCP.writeDouble(_value[i][j]);
        }
      }
      if (_flushNow)
        bufferOutputTCP.flush();
    }catch(Exception e){
      System.out.println("Error setValue Remote Matlab "+e);
    } 
  }

  /**
   * Evals a command controlling the flush
   */
  private  void eval(String _command, boolean _flushNow) {
    try {
      bufferOutputTCP.writeInt(EjsId);
      bufferOutputTCP.writeUTF("eval");
      bufferOutputTCP.writeUTF(_command);
      if (_flushNow)
        bufferOutputTCP.flush();
    }
    catch (Exception e) {
      System.out.println(" eval Remote Exception:" + e);
    }
  }

  /**
   * Gets the base directory
   */
  private String getBaseDirectory(){
    String pathOfJimc2=this.getClass().getProtectionDomain().getCodeSource().getLocation().toString();
    pathOfJimc2=pathOfJimc2.substring(0,pathOfJimc2.lastIndexOf("/"));  
    return pathOfJimc2;
  } 

  /**
   * Creates the Simulink model in the remote server
   */
  private void createModel(String homeDir,long longitudArchivo){     
    byte abModel[];
    try{                          
      FileInputStream fstream = new FileInputStream(new File(new java.net.URI(homeDir+model)));
      DataInputStream in = new DataInputStream(fstream);
      abModel= new byte[(int)longitudArchivo];      
      in.readFully(abModel);
      in.close();
    }catch (Exception e){System.err.println("Model reading error:"+e);  model=null;return;};
    int index = model.lastIndexOf('/');
    if (index>=0) model=model.substring(index + 1, model.length());
    model=importModel(model,abModel);  //Create model in Remote Matlab
    theModel = model.substring(0, model.length() - 4);            
  }


  /**
   * Checks if the model exists 
   */
  private  boolean remoteFileExist(String modelPath){
    boolean result=false;
    try {
      bufferOutputTCP.writeInt(EjsId);
      bufferOutputTCP.writeUTF("remoteFileExist");
      bufferOutputTCP.writeUTF(modelPath);
      bufferOutputTCP.flush();
      result=bufferInputTCP.readBoolean();
    }
    catch (Exception e) {
      System.out.println(" fileExist Remote Exception:" + e);
    }
    return result;
  }

  /**
   * Imports the Simulink model from the local to the remote location
   */
  private String importModel(String _modelFile,byte[] miNuevoArray){
    String remoteModel=null;
    try {
      bufferOutputTCP.writeInt(EjsId);
      bufferOutputTCP.writeUTF("importModel");
      bufferOutputTCP.writeUTF(_modelFile);
      bufferOutputTCP.writeInt(miNuevoArray.length);
      bufferOutputTCP.write(miNuevoArray);
      bufferOutputTCP.flush();
      remoteModel=bufferInputTCP.readUTF();
    }
    catch (Exception e) {
      System.out.println("importModel Remote Exception:" + e);
    }
    return remoteModel;
  }

  /**
   * Resets the Simulink model
   */
  private void resetIC () {
    if (isConnected()){
      resetIC = true;
      eval("set_param ('" + theModel + "', 'SimulationCommand','Start')", true);
    }
  }


  /**
   * Opens the Simulink model
   */
  private void openModel () {
    if (model==null) return;
    eval("load_system ('" + theModel +"')",false);
    eval ("set_param ('"+theModel+"', 'SimulationCommand','Stop')",false);
    eval ("set_param ('"+theModel+"', 'StopTime','inf')",false);
    createEjsSubsystem ();
    initialize();
    //Clear buffer
    haltStepAS(true);
  }

  /**
   * Stops the Simulink model
   */
  private void stop () {
    startRequired = true;
    if (theModel!=null)
      eval("set_param ('"+theModel+"', 'SimulationCommand', 'Stop')"); // This makes Simulink beep
  }

  /**
   * Quits of Remote Matlab
   */
  private void quit()  {
    try {
      bufferOutputTCP.writeInt(EjsId);
      bufferOutputTCP.writeUTF("exit");
      bufferOutputTCP.flush();
      bufferOutputTCP.close();
      bufferOutputTCP.close();
      jimTCP.close();
    }catch(Exception e){
      System.out.println("Error closing Remote Matlab "+e);
    }
  }

  /**
   * Creates the subsystem in the Simulink model
   */
  private void createEjsSubsystem () {
    // Adds the Ejs sub block for the add-ons to the model
    eval(
        "Ejs_sub_name=['"+theModel+"','/','Ejs_sub_','"+theModel+"']; \n"
        //  + "%Get a Correct Block Name \n"
        //    + "number=1; \n"
        //    + "root=Ejs_sub_name; \n"
        //    + "while not(isempty(find_system(parent,'SearchDepth',1,'name',Ejs_sub_name))) \n"
        //    + "  Ejs_sub_name=[root,num2str(number)]; \n"
        //    + "  number=number+1; \n"
        //    + "end; \n"
        + "add_block('built-in/subsystem',Ejs_sub_name); \n"
        + "XY=get_param('"+theModel+"','location'); \n"
        + "height=XY(4)-XY(2); \n"
        + "width=XY(3)-XY(1); \n"
        + "sXY=[width/2-16,height-48,width/2+16,height-16]; \n"
        //## + "set_param(Ejs_sub_name,'position',sXY,'MaskDisplay','image(imread(''"+userDir+"EjsIcon.jpg'',''jpg''))'); \n"
        // 'Ejs_Done = 1' block
        //    + "add_block('built-in/Constant',[Ejs_sub_name,'/SetDone'],'value','1','position',[30,30,70,50]);\n"
        //    + "add_block('built-in/toworkspace',[Ejs_sub_name,'/DoneToWS'],'VariableName','Ejs_Done','Buffer','1','position',[150,30,200,50]);\n"
        //    + "add_line(Ejs_sub_name,'SetDone/1','DoneToWS/1');"
        ,true);
    // Set the stop time to infinity
    eval("set_param('"+theModel+"','StartTime','0','StopTime','inf');",false);  // Paco cambió de 1 a 0
    // Add a 'time to workspace' block
    eval(
        "add_block('built-in/clock',[Ejs_sub_name,'/Clock']); \n"
        + "set_param([Ejs_sub_name,'/Clock'],'DisplayTime','on','Position', [30, 75, 70, 95]); \n"
        + "add_block('built-in/toworkspace',[Ejs_sub_name,'/timeToWS']); \n"
        + "set_param([Ejs_sub_name,'/timeToWS'],'VariableName','Ejs_time','Position',[150, 75, 200, 95],'Buffer','1'); \n"
        + "add_line(Ejs_sub_name,'Clock/1','timeToWS/1');",false);

    if (fixedUpdateStep){
      // Add a fixedStep block
      eval(
          "add_block('built-in/digital clock',[Ejs_sub_name,'/fixedStep']);\n"
          + "set_param([Ejs_sub_name,'/fixedStep'],'Position', [30, 135, 70, 155],'sampletime','"+fixedStep+"'); \n"
          + "add_block('built-in/matlabfcn',[Ejs_sub_name,'/Pause Simulink']); \n"
          + "comando=['set_param(''"+theModel+"'',','''','SimulationCommand','''',',','''','Pause','''',')']; \n"
          + "set_param([Ejs_sub_name,'/Pause Simulink'],'MATLABFcn',comando,'OutputWidth','0','Position',[150, 125, 200, 165]); \n"
          + "add_line(Ejs_sub_name,'fixedStep/1','Pause Simulink/1'); \n",false);
    }else{
      // Add a pause block
      eval(
          "add_block('built-in/ground',[Ejs_sub_name,'/Gr1']); \n"
          + "set_param([Ejs_sub_name,'/Gr1'],'Position', [30, 135, 70, 155]); \n"
          + "add_block('built-in/matlabfcn',[Ejs_sub_name,'/Pause Simulink']); \n"
          + "comando=['set_param(''"+theModel+"'',','''','SimulationCommand','''',',','''','Pause','''',')']; \n"
          + "set_param([Ejs_sub_name,'/Pause Simulink'],'MATLABFcn',comando,'OutputWidth','0','Position',[150, 125, 200, 165]); \n"
          + "add_line(Ejs_sub_name,'Gr1/1','Pause Simulink/1'); \n",false);
    }
  }

  /**
   * Sets the command to convert a Simulink model in order to be used by the client application
   */
  static private final String conversionCommand =
    //      "%Filtrar por sistema a modificar \n"
    "var=variables.path; \n"
    + "index=strmatch(sistema,var); \n"
    + "index_F=[]; \n"
    + "flag_ok=0; \n"

    + "for i=1:size(index,1) varaux=char(var{index(i)});  \n"
    + "  flag_ok=0; \n"
    + "  flag_s=length(sistema)==length(varaux); \n"
    + "  if flag_s \n"
    + "    flag_ok=1; \n"
    + "  else \n"
    + "    if length(varaux)>length(sistema) \n"
    + "      if strcmp(varaux(length(sistema)+1),'/'); \n"
    + "        flag_ok=1; \n"
    + "      end; \n"
    + "    end; \n"
    + "  end; \n"
    + "  if flag_ok \n"
    + "    index_F=[index_F;index(i)]; \n"
    + "  end; \n"
    + "end; \n"

    + "variablesF.path={}; \n"
    + "variablesF.name={}; \n"
    + "variablesF.fromto={}; \n"
    + "variablesF.port={}; \n"

    + "variablesF.path=variables.path(index_F); \n"
    + "variablesF.name=variables.name(index_F); \n"
    + "variablesF.fromto=variables.fromto(index_F); \n"
    + "variablesF.port=variables.port(index_F); \n"
    + "[vax iax]=sortrows(variablesF.fromto); \n"
    + "comando='';\n"
    + "for k=1:size(iax,1) i=iax(k); \n"
    + "  fromto=variablesF.fromto{i}; \n"
    + "  switch fromto \n"
    + "    case 'In' \n"
    + "      path=variablesF.path{i}; \n"
    + "      port=variablesF.port{i}; \n"
    + "      variable=variablesF.name{i}; \n"
    + "      parent=get_param(path,'parent'); \n"
    + "      orientation=get_param(path,'orientation'); \n"
    + "      name=strrep(get_param(path,'name'),'/','//'); \n"
    + "      name_sub_in=['ejs_in_',variable]; \n"
    //    + "      %Get a Correct Block Name \n"
    + "      number=1; \n"
    + "      root=name_sub_in; \n"
    + "      while not(isempty(find_system(parent,'SearchDepth',1,'name',name_sub_in))) \n"
    + "        name_sub_in=[root,num2str(number)]; \n"
    + "        number=number+1; \n"
    + "      end; \n"
    + "      add_block('built-in/subsystem',[parent,'/',name_sub_in]); \n"
    + "      posIPs=get_param(path,'InputPorts'); \n"
    + "      posIP=posIPs(str2num(port),:); \n"
    + "      add_block('built-in/outport',[parent,'/',name_sub_in,'/OUT'],'position',[470,93,485,107]); \n"
    + "      add_block('built-in/matlabfcn',[parent,'/',name_sub_in,'/FromWS'],'MATLABFcn',variable,'position',[185,87,310,113]); \n"
    + "      add_block('built-in/ground',[parent,'/',name_sub_in,'/G'],'position',[15,90,35,110]); \n"
    + "      switch orientation \n"
    + "        case 'left' \n"
    + "          position=[posIP(1)+5,posIP(2)-5,posIP(1)+15,posIP(2)+5]; \n"
    + "        case 'right' \n"
    + "    position=[posIP(1)-15,posIP(2)-5,posIP(1)-5,posIP(2)+5]; \n"
    + "        case 'down' \n"
    + "      position=[posIP(1)-5,posIP(2)-15,posIP(1)+5,posIP(2)-5]; \n"
    + "        case 'up' \n"
    + "    position=[posIP(1)-5,posIP(2)+5,posIP(1)+5,posIP(2)+15]; \n"
    + "      end; \n"
    //    + "      %Delete Actual Connect \n"
    + "      delete_line(parent,posIP); \n"
    //    + "      %Connect \n"
    + "      autoline([parent,'/',name_sub_in],'G/1','FromWS/1'); \n"
    + "      autoline([parent,'/',name_sub_in],'FromWS/1','OUT/1'); \n"
    + "      set_param([parent,'/',name_sub_in],'Position',position,'MaskDisplay','patch([0 0 1 1], [0 1 1 0], [1 0 0])'); \n"
    + "      set_param([parent,'/',name_sub_in],'MaskIconFrame','off','ShowName','off','orientation',orientation); \n"
    + "      add_line(parent,[name_sub_in,'/1'],[name,'/',port]); \n"



    + "       case 'Out'  \n"
    + "          %Problemas con puertos State. \n"
    + "           \n"
    + "          path=variablesF.path{i}; \n"
    + "          port=variablesF.port{i}; \n"
    + "          name=variablesF.name{i}; \n"
    + "           \n"
    + "          orientation=get_param(path,'orientation'); \n"
    + "           \n"
    + "       %Get Input of Block from WorkSpace  \n"
    + "          blocktype=get_param(path,'BlockType'); \n"
    + "          flag_discrete=strcmp(deblank(blocktype),'DiscreteIntegrator'); \n"
    + "          flag_continuos=strcmp(deblank(blocktype),'Integrator'); \n"
    + "           \n"
    + "          if or(flag_discrete,flag_continuos) \n"
    + "              \n"
    + "             %----------------Inicio Integrador------------------ \n"
    + "             %Problemas \n"
    + "             %que pasa si la salida (puerto) es saturacion o estado? \n"
    + "  \n"
    + "         %Codigo Puertos \n"
    + "         %Normal= 1:inf \n"
    + "         %Estado= -1000 \n"
    + "         %Enable= -2000 \n"
    + "         %Trigger=-3000 \n"
    + "         %No Connection=-1 \n"
    + "  \n"
    + "         %Obtiene Puertos \n"
    + "         handle=get_param(path,'handle'); \n"
    + "         parent=get_param(path,'parent'); \n"
    + "             nameintegrator=strrep(get_param(path,'name'),'/','//');             \n"
    + "             fullname=[path,'_I_EJS']; \n"
    + "              \n"
    + "         ports=get_param(path,'PortHandles'); \n"
    + "         inports=ports.Inport; \n"
    + "         outports=ports.Outport; \n"
    + "         stateport=ports.State; \n"
    + "  \n"
    + "         %Inicializa Entradas/Salidas \n"
    + "         info_in=[]; \n"
    + "             info_out=[]; \n"
    + "             for i=1:length(inports), line=get_param(inports(i),'line'); \n"
    + "                if ne(line,-1) \n"
    + "                   blocksource=get_param(line,'SrcBlockHandle'); \n"
    + "                   if ne(blocksource,-1) \n"
    + "                      portsource=get_param(line,'SrcPortHandle'); \n"
    + "                      blocksn=strrep(get_param(blocksource,'name'),'/','//'); \n"
    + "                      portblocks=get_param(blocksource,'PortHandles'); \n"
    + "                      portblocks=portblocks.Outport; \n"
    + "                      portblocks=find(portblocks==portsource); \n"
    + "                      if isempty(portblocks) \n"
    + "                         portblocks=get_param(blocksource,'PortHandles'); \n"
    + "                         portblocks=portblocks.State; \n"
    + "                         if portblocks==portsource \n"
    + "                            portblocks=-1000; \n"
    + "                         else \n"
    + "                            info_in=[info_in;handle,i,-1,-1]; \n"
    + "                         end; \n"
    + "                      end; \n"
    + "                      info_in=[info_in;handle,i,blocksource,portblocks]; \n"
    + "                   else \n"
    + "                      info_in=[info_in;handle,i,-1,-1]; \n"
    + "                   end; \n"
    + "                else \n"
    + "                   info_in=[info_in;handle,i,-1,-1]; \n"
    + "                end; \n"
    + "             end; \n"
    + "              \n"
    + "             flag_state=not(isempty(stateport)); \n"
    + "             if flag_state \n"
    + "                outports(end+1)=stateport; \n"
    + "             end; \n"
    + "              \n"
    + "             for i=1:length(outports), flag_state_now=and(i==length(outports),flag_state); \n"
    + "                line=get_param(outports(i),'line'); \n"
    + "                if ne(line,-1) \n"
    + "                   blockdest=get_param(line,'DstBlockHandle'); \n"
    + "                   for j=1:length(blockdest), if ne(blockdest(j),-1) \n"
    + "                         if ne(blockdest(j),handle) \n"
    + "                            portdest=get_param(line,'DstPortHandle'); \n"
    + "                            portblockd=get_param(blockdest(j),'PortHandles'); \n"
    + "                            blockdn=strrep(get_param(blockdest(j),'name'),'/','//'); \n"
    + "                            portblockd_i=portblockd.Inport; \n"
    + "                            portblockd_e=portblockd.Enable; \n"
    + "                            portblockd_t=portblockd.Trigger; \n"
    + "                            portblockd=0; \n"
    + "                            if not(isempty(portblockd_i)) \n"
    + "                               portblockd=find(portblockd_i==portdest(j)); \n"
    + "                               if isempty(portblockd) \n"
    + "                                  portblockd=0; \n"
    + "                               end; \n"
    + "                            end; \n"
    + "                            if and(not(isempty(portblockd_e)),not(portblockd)) \n"
    + "                               flag_pbd=eq(portblockd_e,portdest(j)); \n"
    + "                               portblockd=flag_pbd*-2000; \n"
    + "                            end; \n"
    + "                            if and(not(isempty(portblockd_t)),not(portblockd)) \n"
    + "                               flag_pbd=eq(portblockd_t,portdest(j)); \n"
    + "                               portblockd=flag_pbd*-3000; \n"
    + "                            end; \n"
    + "                             \n"
    + "                            if ne(portblockd,0)             \n"
    + "                               info_out=[info_out;handle,i+(-i-1000)*flag_state_now,blockdest(j),portblockd]; \n"
    + "                            else \n"
    + "                               info_out=[info_out;handle,i+(-i-1000)*flag_state_now,-1,-1]; \n"
    + "                            end;               \n"
    + "                         end;    \n"
    + "                      else \n"
    + "                         info_out=[info_out;handle,i+(-i-1000)*flag_state_now,-1,-1]; \n"
    + "                      end; \n"
    + "                   end;  %for bloque \n"
    + "                else \n"
    + "                   info_out=[info_out;handle,i,-1,-1]; \n"
    + "                end; \n"
    + "             end; %for linea    \n"
    + "              \n"
    + "             %Guarda parámetros del Integrador                  \n"
    + "             flag_int_er=get_param(path,'ExternalReset'); \n"
    + "             flag_int_ic=get_param(path,'InitialCondition'); \n"
    + "             flag_int_is=get_param(path,'InitialConditionSource'); \n"
    + "             flag_int_lo=get_param(path,'LimitOutput'); \n"
    + "             flag_int_lu=get_param(path,'UpperSaturationLimit'); \n"
    + "             flag_int_ll=get_param(path,'LowerSaturationLimit'); \n"
    + "             flag_int_st=get_param(path,'ShowStatePort'); \n"
    + "             flag_int_ss=get_param(path,'ShowSaturationPort'); \n"
    + "              \n"
    + "             %Guarda Metodo Integracion Integrador de Tiempo Discreto \n"
    + "             if flag_discrete \n"
    + "                metodo_td=get_param(path,'IntegratorMethod'); \n"
    + "                int_sample=get_param(path,'SampleTime'); \n"
    + "                int_str='dpoly([0 1],[1 -1],''z'')'; \n"
    + "                int_type='DiscreteIntegrator'; \n"
    + "             else \n"
    + "                flag_int_at=get_param(path,'AbsoluteTolerance'); \n"
    + "                int_str='dpoly(1,[1 0])'; \n"
    + "                int_type='integrator'; \n"
    + "             end;     \n"
    + "              \n"
    + "             %construir subsistema \n"
    + "             add_block('built-in/subsystem',fullname); \n"
    + "             pos_sub=get_param(fullname,'position'); \n"
    + "             pos_int=get_param(path,'position'); \n"
    + "             set_param(path,'position',pos_sub); \n"
    + "             set_param(fullname,'position',pos_int); \n"
    + "             set_param(fullname,'orientation',orientation); \n"
    + "             set_param(fullname,'MaskDisplay',int_str); \n"
    + "             set_param(fullname,'Backgroundcolor','yellow');      \n"
    + "                          \n"
    + "             %Salidas \n"
    + "             for i=1:length(outports), add_block('built-in/outport',[fullname,'/Out',num2str(i)],'Position',[785, 80+95*(i-1), 805, 100+95*(i-1)]); \n"
    + "             end; \n"
    + "              \n"
    + "             %Entradas y sus Conexiones \n"
    + "             for i=1:length(inports), flag_igual=0; \n"
    + "            flag_state_igual=0; \n"
    + "                add_block('built-in/inport',[fullname,'/In',num2str(i)],'Position',[20, 75+40*(i-1), 40, 95+40*(i-1)]); \n"
    + "                blocknames=info_in(i,3); \n"
    + "                if blocknames==handle \n"
    + "                   blocknames=[nameintegrator,'_I_EJS']; \n"
    + "                   blocknamesdel=nameintegrator; \n"
    + "                   flag_igual=1; \n"
    + "                else \n"
    + "                   blocknames=strrep(get_param(info_in(i,3),'name'),'/','//'); \n"
    + "                   blocknamesdel=blocknames; \n"
    + "                end; \n"
    + "                blockports=info_in(i,4); \n"
    + "                if blockports==-1000 \n"
    + "                   blockports='State'; \n"
    + "                   if flag_igual \n"
    + "                      blockports=num2str(length(outports)); \n"
    + "                      flag_state_igual=1; \n"
    + "                   end; \n"
    + "                else \n"
    + "                   blockports=num2str(blockports); \n"
    + "                end; \n"
    + "                 \n"
    + "                if flag_state_igual \n"
    + "             delete_line(parent,[blocknamesdel,'/','state'],[nameintegrator,'/',num2str(info_in(i,2))]); \n"
    + "           else \n"
    + "               delete_line(parent,[blocknamesdel,'/',blockports],[nameintegrator,'/',num2str(info_in(i,2))]); \n"
    + "           end; \n"
    + "                 \n"
    + "                try  \n"
    + "                   autoline(parent,[blocknames,'/',blockports],[nameintegrator,'_I_EJS','/',num2str(info_in(i,2))]); \n"
    + "                catch \n"
    + "                   add_line(parent,[blocknames,'/',blockports],[nameintegrator,'_I_EJS','/',num2str(info_in(i,2))]); \n"
    + "                end; \n"
    + "             end; \n"
    + "              \n"
    + "             %Conexiones de Salida \n"
    + "             for i=1:size(info_out,1), blocknamed=strrep(get_param(info_out(i,3),'name'),'/','//');      \n"
    + "                blockportd=info_out(i,4); \n"
    + "                switch blockportd \n"
    + "                case -1000 \n"
    + "                   blockportd='State'; \n"
    + "                case -2000 \n"
    + "                   blockportd='Enable'; \n"
    + "                case -3000 \n"
    + "                   blockportd='Trigger'; \n"
    + "                otherwise \n"
    + "                   blockportd=num2str(blockportd);    \n"
    + "                end; \n"
    + "                 \n"
    + "                blockportsd=info_out(i,2); \n"
    + "                if blockportsd==-1000 \n"
    + "                   blockportsd=num2str(length(outports)); \n"
    + "                   blockportdel='State'; \n"
    + "                else \n"
    + "                   blockportsd=num2str(blockportsd); \n"
    + "                   blockportdel=blockportsd; \n"
    + "                end; \n"
    + "                 \n"
    + "                delete_line(parent,[nameintegrator,'/',blockportdel],[blocknamed,'/',blockportd]); \n"
    + "                try \n"
    + "                   autoline(parent,[nameintegrator,'_I_EJS','/',blockportsd],[blocknamed,'/',blockportd]); \n"
    + "                catch \n"
    + "                   add_line(parent,[nameintegrator,'_I_EJS','/',blockportsd],[blocknamed,'/',blockportd]);    \n"
    + "                end; \n"
    + "                 \n"
    + "             end; \n"
    + "              \n"
    + "             %Borrar integrador \n"
    + "             delete_block(handle); \n"
    + "                          \n"
    + "             %Modificar Subsistema Integrador \n"
    + "             %Agregar Integrador \n"
    + "             add_block(['built-in/',int_type],[fullname,'/I'],'Position',[585,80,655,120]); \n"
    + "             set_param([fullname,'/I'],'ExternalReset','rising'); \n"
    + "             set_param([fullname,'/I'],'InitialConditionSource','external');  \n"
    + "             if flag_discrete \n"
    + "                set_param([fullname,'/I'],'IntegratorMethod',metodo_td);   \n"
    + "                set_param([fullname,'/I'],'SampleTime',int_sample); \n"
    + "             else    \n"
    + "                set_param([fullname,'/I'],'AbsoluteTolerance',flag_int_at); \n"
    + "             end;  \n"
    + "             set_param([fullname,'/I'],'LimitOutput',flag_int_lo);            \n"
    + "             set_param([fullname,'/I'],'UpperSaturationLimit',flag_int_lu); \n"
    + "             set_param([fullname,'/I'],'LowerSaturationLimit',flag_int_ll); \n"
    + "             set_param([fullname,'/I'],'ShowSaturationPort',flag_int_ss); \n"
    + "             set_param([fullname,'/I'],'ShowStatePort',flag_int_st); \n"
    + "              \n"
    + "             %Agrega ToWorkSpace \n"
    + "             add_block('built-in/toworkspace',[fullname,'/toWS'],'Position',[705,19,780,41]); \n"
    + "             %Establece el nombre de la variable en el espacio de trabajo \n"
    + "             set_param([fullname,'/toWS'],'VariableName',name,'Buffer','1'); \n"
    + "              \n"
    + "             %Agrega IC \n"
    + "             add_block('built-in/sum',[fullname,'/IC'],'Position',[510,214,530,256]); \n"
    + "             set_param([fullname,'/IC'],'iconshape','rectangular','inputs','++'); \n"
    + "              \n"
    + "             %Agrega IC_smk_enabled \n"
    + "             add_block('built-in/product',[fullname,'/IC_smk_enabled'],'Position',[430,89,445,281]); \n"
    + "             set_param([fullname,'/IC_smk_enabled'],'inputs','3'); \n"
    + "              \n"
    + "             %Agrega IC_ejs_enabled \n"
    + "             add_block('built-in/product',[fullname,'/IC_ejs_enabled'],'Position',[430,308,445,372]); \n"
    + "             set_param([fullname,'/IC_ejs_enabled'],'inputs','2'); \n"
    + "              \n"
    + "             %Agrega reset? \n"
    + "             add_block('built-in/logic',[fullname,'/reset?'],'Position',[425,426,455,459]); \n"
    + "             set_param([fullname,'/reset?'],'inputs','2','Operator','OR'); \n"
    + "              \n"
    + "             %Agrega ejs_priority \n"
    + "             add_block('built-in/logic',[fullname,'/ejs_priority'],'Position',[360,234,390,266]); \n"
    + "             set_param([fullname,'/ejs_priority'],'Operator','NOT'); \n"
    + "                          \n"
    + "             %Agrega Reset Inicial EJS (De esta forma siempre al comienzo toma valores de EJS) \n"
    + "         add_block('built-in/InitialCondition',[fullname,'/resetinicial_ejs'],'Position',[310,310,340,340]); \n"
    + "         set_param([fullname,'/resetinicial_ejs'],'value','1'); \n"
    + "                                             \n"
    + "             %Agrega reset_ejs? \n"
    + "             add_block('built-in/RelationalOperator',[fullname,'/reset_ejs?'],'Position',[270,297,290,353]); \n"
    + "             set_param([fullname,'/reset_ejs?'],'Operator','=='); \n"
    + "              \n"
    + "             %Agrega reset_smk \n"
    + "             if not(strcmp(flag_int_er,'none')) \n"
    + "                 \n"
    + "                %Agrega reset_smk? \n"
    + "               add_block('built-in/RelationalOperator',[fullname,'/reset_smk?'],'Position',[270,187,290,243]); \n"
    + "              set_param([fullname,'/reset_smk?'],'Operator','=='); \n"
    + "                add_block('built-in/subsystem',[fullname,'/reset_smk'],'Position',[180,189,235,211]); \n"
    + "                add_block('built-in/inport',[fullname,'/reset_smk/in'],'Position',[25,128,55,142]); \n"
    + "                add_block('built-in/outport',[fullname,'/reset_smk/out'],'Position',[450,128,480,142]); \n"
    + "                add_line([fullname,'/reset_smk'],'in/1','out/1'); \n"
    + "                add_block('built-in/triggerport',[fullname,'/reset_smk/trigger'],'Position',[210,20,230,40]); \n"
    + "                set_param([fullname,'/reset_smk/trigger'],'TriggerType',flag_int_er); \n"
    + "             else \n"
    + "                add_block('built-in/constant',[fullname,'/noreset'],'position',[180,189,235,211],'value','0'); \n"
    + "             end; \n"
    + "              \n"
    + "             %Agrega reset_ejs \n"
    + "             add_block('built-in/subsystem',[fullname,'/reset_ejs'],'Position',[170,330,230,350]); \n"
    + "             add_block('built-in/inport',[fullname,'/reset_ejs/in'],'Position',[25,128,55,142]); \n"
    + "             add_block('built-in/outport',[fullname,'/reset_ejs/out'],'Position',[450,128,480,142]); \n"
    + "             add_line([fullname,'/reset_ejs'],'in/1','out/1'); \n"
    + "             add_block('built-in/triggerport',[fullname,'/reset_ejs/trigger'],'Position',[210,20,230,40]); \n"
    + "             set_param([fullname,'/reset_ejs/trigger'],'TriggerType','either'); \n"
    + "              \n"
    + "             %Agrega RS_ejs \n"
    + "             add_block('built-in/matlabfcn',[fullname,'/RS_ejs'],'MATLABFcn',['Ejs__ResetIC']); \n"
    + "             set_param([fullname,'/RS_ejs'],'Position',[60,250,120,280]); \n"
    + "             add_block('built-in/ground',[fullname,'/','GRS'],'Position',[20,255,40,275]); \n"
    + "              \n"
    + "             %Agrega IC_ejs \n"
    // + "             add_block('built-in/matlabfcn',[fullname,'/IC_ejs'],'MATLABFcn',[name,'_IC']); \n" //Gonzalo 090515
    + "             add_block('built-in/matlabfcn',[fullname,'/IC_ejs'],'MATLABFcn',[name]); \n"
    + "             set_param([fullname,'/IC_ejs'],'Position',[60,390,120,420]); \n"
    + "             add_block('built-in/ground',[fullname,'/','GIC'],'Position',[20,395,40,415]); \n"
    + "              \n"
    + "             %Agrega Clock \n"
    + "             add_block('built-in/clock',[fullname,'/Clock'],'Position',[20,190,40,210]); \n"
    + "                          \n"
    + "             %Agrega Bloque para condicion inicial interna \n"
    + "             if strcmp(flag_int_is,'internal') \n"
    + "                add_block('built-in/constant',[fullname,'/icinternal'],'position',[360,110,380,130]); \n"
    + "                set_param([fullname,'/icinternal'],'value',flag_int_ic); \n"
    + "             end; \n"
    + "              \n"
    + "             %Conecta Bloques \n"
    + "             %--------------- \n"
    + "             %Entrada al Integrador \n"
    + "             add_line([fullname],'In1/1','I/1'); \n"
    + "              \n"
    + "             %Entrada Condicion inicial \n"
    + "             flag_reset=not(strcmp(flag_int_er,'none')); \n"
    + "             flag_icext=strcmp(flag_int_is,'external'); \n"
    + "                                      \n"
    + "             if and(flag_icext,flag_reset) \n"
    + "                add_line(fullname,'In3/1','IC_smk_enabled/1'); \n"
    + "                add_line(fullname,'In2/1','reset_smk/trigger'); \n"
    + "                add_line(fullname,'Clock/1','reset_smk/1'); \n"
    + "                add_line(fullname,'reset_smk/1','reset_smk?/1'); \n"
    + "               add_line(fullname,'Clock/1','reset_smk?/2'); \n"
    + "                add_line(fullname,'reset_smk?/1','IC_smk_enabled/2');   \n"
    + "               add_line(fullname,'reset_smk?/1','reset?/1'); \n"
    + "             elseif and(flag_icext,not(flag_reset)) \n"
    + "           add_line(fullname,'In2/1','IC_smk_enabled/1');                \n"
    + "                add_line(fullname,'noreset/1','IC_smk_enabled/2'); \n"
    + "                add_line(fullname,'noreset/1','reset?/1'); \n"
    + "             elseif and(not(flag_icext),flag_reset) \n"
    + "                add_line(fullname,'icinternal/1','IC_smk_enabled/1'); \n"
    + "                add_line(fullname,'In2/1','reset_smk/trigger'); \n"
    + "                add_line(fullname,'Clock/1','reset_smk/1'); \n"
    + "                add_line(fullname,'reset_smk/1','reset_smk?/1'); \n"
    + "                add_line(fullname,'Clock/1','reset_smk?/2'); \n"
    + "                add_line(fullname,'reset_smk?/1','IC_smk_enabled/2');   \n"
    + "               add_line(fullname,'reset_smk?/1','reset?/1');                \n"
    + "             else \n"
    + "                add_line(fullname,'icinternal/1','IC_smk_enabled/1'); \n"
    + "                add_line(fullname,'noreset/1','IC_smk_enabled/2'); \n"
    + "                add_line(fullname,'noreset/1','reset?/1');   \n"
    + "             end; \n"
    + "              \n"
    + "             %Clock \n"
    + "             add_line(fullname,'Clock/1','reset_ejs/1'); \n"
    + "             add_line(fullname,'Clock/1','reset_ejs?/1'); \n"
    + "              \n"
    + "             %Grounds \n"
    + "             add_line(fullname,'GRS/1','RS_ejs/1'); \n"
    + "             add_line(fullname,'GIC/1','IC_ejs/1'); \n"
    + "              \n"
    + "             %Matlab Functions \n"
    + "             add_line(fullname,'RS_ejs/1','reset_ejs/trigger'); \n"
    + "             add_line(fullname,'IC_ejs/1','IC_ejs_enabled/2'); \n"
    + "              \n"
    + "             %reset e IC \n"
    + "             add_line(fullname,'reset_ejs/1','reset_ejs?/2'); \n"
    + "             add_line(fullname,'reset_ejs?/1','resetinicial_ejs/1'); \n"
    + "             add_line(fullname,'resetinicial_ejs/1','reset?/2'); \n"
    + "             add_line(fullname,'resetinicial_ejs/1','IC_ejs_enabled/1'); \n"
    + "             add_line(fullname,'resetinicial_ejs/1','ejs_priority/1'); \n"
    + "             add_line(fullname,'ejs_priority/1','IC_smk_enabled/3'); \n"
    + "             add_line(fullname,'IC_smk_enabled/1','IC/1'); \n"
    + "             add_line(fullname,'IC_ejs_enabled/1','IC/2'); \n"
    + "             add_line(fullname,'IC/1','I/3'); \n"
    + "             add_line(fullname,'reset?/1','I/2'); \n"
    + "              \n"
    + "             %Integrador \n"
    + "             add_line(fullname,'I/1','toWS/1'); \n"
    + "             add_line(fullname,'I/1','Out1/1'); \n"
    + "              \n"
    + "             if strcmp(flag_int_ss,'on') \n"
    + "                add_line(fullname,'I/2','Out2/1'); \n"
    + "             end;    \n"
    + "             if strcmp(flag_int_st,'on') \n"
    + "                add_line(fullname,'I/state',['Out',num2str(length(outports)),'/1']); \n"
    + "             end; \n"
    + "           %------------------Fin Integrador------------------ \n"
    + "       else \n"
    + "                          \n"
    + "             parent=get_param(path,'parent'); \n"
    + "             blockname=strrep(get_param(path,'name'),'/','//'); \n"
    + "              \n"
    + "             name_sub_out=['ejs_out_',name]; \n"
    + "              \n"
    + "             %Get a Correct Block Name \n"
    + "            number=1; \n"
    + "           root=name_sub_out; \n"
    + "         while not(isempty(find_system(parent,'SearchDepth',1,'name',name_sub_out))) \n"
    + "             name_sub_out=[root,num2str(number)]; \n"
    + "             number=number+1; \n"
    + "         end;       \n"
    + "              \n"
    + "         add_block('built-in/subsystem',[parent,'/',name_sub_out]); \n"
    + "  \n"
    + "         posOPs=get_param(path,'OutputPorts'); \n"
    + "         posOP=posOPs(str2num(port),:); \n"
    + "              \n"
    + "           switch orientation \n"
    + "         case 'left' \n"
    + "           position=[posOP(1)-20,posOP(2)-5,posOP(1)-10,posOP(2)+5]; \n"
    + "         case 'right' \n"
    + "           position=[posOP(1)+10,posOP(2)-5,posOP(1)+20,posOP(2)+5]; \n"
    + "         case 'down' \n"
    + "           position=[posOP(1)-5,posOP(2)+10,posOP(1)+5,posOP(2)+20]; \n"
    + "         case 'up' \n"
    + "           position=[posOP(1)-5,posOP(2)-20,posOP(1)+5,posOP(2)-10]; \n"
    + "         end; \n"
    + "              \n"
    + "         add_block('built-in/inport',[parent,'/',name_sub_out,'/IN'],'position',[30,108,45,122]); \n"
    + "         add_block('built-in/toworkspace',[parent,'/',name_sub_out,'/ToWS'],'position',[285,98,485,132]); \n"
    + "  \n"
    + "         %Set WorkSpace Variable \n"
    + "         set_param([parent,'/',name_sub_out,'/ToWS'],'VariableName',name,'Buffer','1'); \n"
    + "  \n"
    + "         %Connect \n"
    + "         add_line([parent,'/',name_sub_out],'IN/1','ToWS/1'); \n"
    + "  \n"
    + "         set_param([parent,'/',name_sub_out],'position',position,'MaskDisplay','patch([0 0 1 1], [0 1 1 0], [0 0 1])'); \n"
    + "         set_param([parent,'/',name_sub_out],'MaskIconFrame','off','ShowName','off','orientation',orientation); \n"
    + "         add_line(parent,[blockname,'/',port],[name_sub_out,'/1']); \n"
    + "             \n"
    + "         end; \n"

    // Paco changed this    + "        add_block('built-in/matlabfcn',[varintegrador,'/Reset Signal'],'MATLABFcn',[name,'_RS'],'Position',[65, 120, 125, 150]); \n"
    //    + "        add_block('built-in/matlabfcn',[varintegrador,'/Reset Signal'],'MATLABFcn','Ejs__ResetIC','Position',[65, 120, 125, 150]); \n"

    + "    case 'Delete' \n"
    + "      bloque=get_param(variablesF.path(i),'Handle'); \n"
    + "      bloque=bloque{1}; \n"
    + "      inlines=get_param(bloque,'InputPorts'); \n"
    + "      parent=get_param(bloque,'Parent'); \n"
    //    + "      %Delete In Lines       \n"
    + "      for iLB=1:size(inlines,1) \n"
    + "        delete_line(parent,inlines(iLB,:)); \n"
    + "      end; \n"
    //    + "      %Delete Block         \n"
    + "      delete_block(bloque); \n"
    + "    case 'Param' \n"  
    + "      comando=[comando,'set_param(''',variables.path{i},''',''',variables.port{i},''',[''['',num2str(',variables.name{i},'),'']'',]);'];\n"    
    //+ "      set_param(variablesF.path{i},variablesF.port{i},eval(variablesF.name{i})); \n"   
    + "        if (strcmpi(class(eval(variablesF.name{i})),'char')),set_param(variablesF.path{i},variablesF.port{i},eval(variablesF.name{i})),else,set_param(variablesF.path{i},variablesF.port{i},variablesF.name{i}),end;\n"
    //+ "      set_param(variablesF.path{i},variablesF.port{i},variablesF.name{i});\n" 
    + "    end; \n"   
    + "end;     \n"
    + "Ejs_sub_name=[sistema,'/','Ejs_sub_',sistema]; \n"
    + "comando=[comando,'set_param(''',sistema,''',','''','SimulationCommand','''',',','''','Pause','''',');']; \n"
    + "comando=['eval(''',strrep(comando,'''',''''''),''');'];\n"
    + "set_param([Ejs_sub_name,'/Pause Simulink'],'MATLABFcn',comando); \n"
    //    + " %Avoid warnings \n"
    + " addterms(sistema); \n";

}
