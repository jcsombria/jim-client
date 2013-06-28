package es.uned.dia.interoperate.matlab.jimc;

import java.io.File;
import java.io.FileOutputStream;
import java.io.InputStream;
import java.util.Enumeration;
import java.util.jar.JarEntry;
import java.util.jar.JarFile;

/**
 * Class to implement the communication protocol with Simulink
 * @author <a href="mailto:gfarias@bec.uned.es">Gonzalo Farias</a>
 * @author <a href="mailto:fem@um.es">Francisco Esquembre</a> 
 */
public class SimulinkExternalApp extends MatlabExternalApp {

  protected String model;    // The complete name of the model
  protected String theModel; // The name of the model without directory or the '.mdl' extension
  private final String EJS_PREFIX="Ejs_"; //A prefix for external variables
  private boolean fixedUpdateStep=false;// A flag to indicate the fixed step mode
  private double fixedStep=0;  //Updating time of the fixed step mode
  private java.util.Vector<String[]> linkVector=null; // A Vector of Array of Strings to keep information of the connected variables
  boolean resetIC = false; // Whether to reset initial conditions of integrator blocks
  boolean resetParam=false;// Whether to reset parameters

  /*********************
     Low level protocol
   **********************/

  //--------------------------------------------------------------
  //Connection and configuration
  //--------------------------------------------------------------
  /**
   * Starts the connection with the external application  
   * @return boolean true if the connection was successful
   */    
  public boolean connect(){  
    if (model!=null & super.connect()){
      openModel();
      return true;
    }
    resultOfLastAction=CONNECTION_FAILED;      
    return false;    
  }

  /**
   * Finishes the connection with the external application     
   */         
  public void disconnect(){    
    if (matlabEng!=null){ 
      stop(); 
      matlabEng.engEvalString (id,"bdclose ('all')");
      matlabEng.engClose(id);
      matlabEng=null;
      id=-1;                                
    }
  }

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
   */
  public boolean linkVariables(String clientVariable, String externalVariable){
    String[] pathTypePort=externalVariable.split("@");
    if (pathTypePort.length==1) return super.linkVariables(clientVariable,externalVariable);
    else if (pathTypePort.length==3) return linkVariables(clientVariable, pathTypePort[0], pathTypePort[1], pathTypePort[2]);
    else return false;
  }

  /**
   * Links a client variable with a variable of the Simulink model (long format)
   * @param clientVariable String the client variable
   * @param epath String the path of the external variable
   * @param etype String the type of the external variable
   * @param eport String the port of the external variable
   */
  public boolean linkVariables(String clientVariable, String epath, String etype, String eport){
    if (varContextObject==null) return (false);               
    int type;
    //Search if the ivar exists
    for (int i=0; i < varContextFields.length; i++) {

      if (clientVariable.equals((varContextFields[i]).getName())) {
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
          String[] _element={clientVariable,EJS_PREFIX+"time"};               
          linkVector.addElement(_element); 
          return (true);  
        }

        //add to string initCommand 
        String evarInitValue=EJS_PREFIX+clientVariable+"=";

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

          initCommand=initCommand+evarInitValue+";"
          +"variables.path{end+1,1}='"+epath+"';"
          +"variables.name{end+1,1}='"+EJS_PREFIX+clientVariable+"';"
          +"variables.fromto{end+1,1}='"+etype.toLowerCase()+"';"
          +"variables.port{end+1,1}='"+eport+"';"; 


          String[] _element={clientVariable,epath,etype,eport};                
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


  //--------------------------------------------------------------
  //Control commands
  //--------------------------------------------------------------

  /**
   * Steps the Simulink model a number of times.
   * @param dt double indicates the number of times to step the Simulink model.
   */
  public void step (double dt) {
    setValues();
    if (theModel==null) return;
    if (resetIC) {
      matlabEng.engEvalString (id,"Ejs__ResetIC = 1 - Ejs__ResetIC");
      startRequired = true;
      resetIC = false;
    }

    if (resetParam){
      matlabEng.engEvalString (id,"set_param('"+theModel+"','SimulationCommand','update')");
      resetParam=false;
    }

    if (startRequired) {
      startRequired = false;
      matlabEng.engEvalString (id,"set_param ('"+theModel+"', 'SimulationCommand','Start')");
      int max = 10;
      do {
        matlabEng.engEvalString(id,
            "smlkstatus=strcmp(get_param('" + theModel +
        "','SimulationStatus'),'paused')");
        max--;                    
      }while (matlabEng.engGetScalar(id, "smlkstatus") == 0 && max>0);
    }
    for (int i=0,times=(int) dt; i<times; i++) {

      matlabEng.engEvalString (id,"set_param ('"+theModel+"', 'SimulationCommand','Continue')");
      matlabEng.engEvalString(id,"EjsSimulationStatus='unknown'"); 
      if (waitForEverFlag){  
        do {
          matlabEng.engEvalString(id,
              "smlkstatus=strcmp(get_param('" + theModel +
          "','SimulationStatus'),'paused')");
        }while (matlabEng.engGetScalar(id, "smlkstatus") == 0);
      }else{

        int max = 10;
        do {
          matlabEng.engEvalString(id,
              "smlkstatus=strcmp(get_param('" + theModel +
          "','SimulationStatus'),'paused')");
          max--;                    
        }while (matlabEng.engGetScalar(id, "smlkstatus") == 0 && max>0);                                                                 
      }
    }    
    getValues();
  } 

  /**
   * Synchronizes client and external applications
   */  
  public void synchronize(){
    resetIC(); 
  }

  //--------------------------------------------------------------
  //Other public methods
  //-------------------------------------------------------------- 

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
      else evar=EJS_PREFIX+var[0];    //Simulink Variables
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
      else evar=EJS_PREFIX+var[0];    //Simulink Variables                      
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
   * Package-internal constructor
   * @param mdlSmlkFile String to define the Simulink model
   */
  public SimulinkExternalApp (String mdlSmlkFile) {
	  super();            
	  mdlSmlkFile = mdlSmlkFile.trim().replace('\\','/');
	  if (mdlSmlkFile.startsWith("<matlab(")) { // remove any trailing <matlab> keyword
		  fixedUpdateStep=true;
		  String _fixedStepStr=mdlSmlkFile.substring(8, mdlSmlkFile.lastIndexOf(')'));
		  try {
			  fixedStep = Double.parseDouble(_fixedStepStr);
			  if (fixedStep<=0) fixedStep=1;
		  }
		  catch (NumberFormatException nfe) {
			  System.out.println("Error in fixed step number:" + nfe);
		  }
		  int index = mdlSmlkFile.indexOf('>');
		  if (index>0) mdlSmlkFile = mdlSmlkFile.substring(index+1).trim();
		  else mdlSmlkFile = "";
	  }else
		  if (mdlSmlkFile.startsWith("<matlab")) { // remove any trailing <matlab> keyword
			  int index = mdlSmlkFile.indexOf('>');
			  if (index>0) mdlSmlkFile = mdlSmlkFile.substring(index+1).trim();
			  else mdlSmlkFile = "";
		  }
	  model = mdlSmlkFile.trim();
	  if (!model.toLowerCase().endsWith(".mdl")) model += ".mdl";
	  try{ 
		  File checkfile = new File(new java.net.URI(getBaseDirectory()+"/"+model));                      
		  if (!checkfile.exists()){          
			  File modelFile = new File(new java.net.URI(temporalDirectory+"/"+model));                                         
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
//	  catch (java.io.IOException e){System.out.println("Error:"+e);} 
	  catch (Exception e) { e.printStackTrace(); }
	  theModel = model.substring(0,model.lastIndexOf('.'));     
  }

  /**
   * Creates a Simulink session
   * @param mdlSmlkFile The mdlFile with the Simulink model
   * @param fixedStepRequired the time to update the Simulink model
   */
  public SimulinkExternalApp (String mdlSmlkFile, double fixedStepRequired) {
    super();            
    mdlSmlkFile = mdlSmlkFile.trim().replace('\\','/');    
    fixedStep=fixedStepRequired;
    if (fixedStep<=0) fixedStep=1;
    fixedUpdateStep=true;
    model = mdlSmlkFile.trim();
    if (!model.toLowerCase().endsWith(".mdl")) model += ".mdl";

    try{ 
      File checkfile = new File(new java.net.URI(getBaseDirectory()+"/"+model));                      
      if (!checkfile.exists()){          
        File modelFile = new File(new java.net.URI(temporalDirectory+"/"+model));                   

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
    theModel = model.substring(0,model.lastIndexOf('.'));      
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
  //Protected methods
  //-------------------------------------------------------------- 

  /**
   * Initializes a Matlab/Simulink session. 
   */
  protected void initialize () {
    if (initCommand!=null) {
      matlabEng.engEvalString(id, "clear all;variables.path={};variables.name={};variables.fromto={};variables.port={};");
      matlabEng.engEvalString(id, initCommand.substring(0,initCommand.lastIndexOf(";")));  
      matlabEng.engEvalString(id, "Ejs__ResetIC = 0");
      matlabEng.engEvalString(id, "sistema='"+theModel+"'");
      matlabEng.engEvalString(id, conversionCommand);
    }
  }

  //--------------------------------------------------------------
  //Private methods
  //-------------------------------------------------------------- 


  /**
   * Resets integrator blocks
   */
  private void resetIC() {
    resetIC = true;
  }


  /**
   * Stops the Simulink model
   */
  private void stop () {
    startRequired = true;
    if (matlabEng!=null && theModel!=null)
      matlabEng.engEvalString (id,"set_param ('"+theModel+"', 'SimulationCommand', 'Stop')"); // This makes Simulink beep
  }

  /**
   * Opens the Simulink model 
   */
  private void openModel () {         
    matlabEng.engEvalString (id,"load_system ('" + theModel +"')");
    matlabEng.engEvalString (id,"set_param ('"+theModel+"', 'SimulationCommand','Stop')");
    matlabEng.engEvalString (id,"set_param ('"+theModel+"', 'StopTime','inf')");
    createEjsSubsystem ();
    initialize();
  }

  /**
   * Creates the EJS subsystem in the Simulink model
   */
  private void createEjsSubsystem () {
    // Adds the Ejs sub block for the add-ons to the model
    matlabEng.engEvalString(id,
        "Ejs_sub_name=['"+theModel+"','/','Ejs_sub_','"+theModel+"']; \n"
        + "add_block('built-in/subsystem',Ejs_sub_name); \n"
        + "XY=get_param('"+theModel+"','location'); \n"
        + "height=XY(4)-XY(2); \n"
        + "width=XY(3)-XY(1); \n"
        + "sXY=[width/2-16,height-48,width/2+16,height-16]; \n"       
        + "ico1=['image(ind2rgb(['];\n"
        + "ico2=['4,4,4,4,4,4,4,4,4,4,4,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,4,4,4,4,4,4;',...\n"
        + "'4,4,4,4,4,4,4,4,4,4,4,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,4,4,4,4,4,4;',...\n"
        + "'4,4,4,4,4,4,4,4,4,4,2,2,2,2,3,3,3,3,3,3,3,3,3,3,3,4,4,4,4,4,4,4;',...\n"
        + "'4,4,4,4,4,4,4,4,4,4,2,2,2,2,3,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4;',...\n"
        + "'4,4,4,4,4,4,4,4,4,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,4,4,4,4,4,4,4,4;',...\n"
        + "'4,4,4,4,4,4,4,4,4,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,4,4,4,4,4,4,4,4;',...\n"
        + "'4,4,4,4,4,4,4,4,4,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,4,4,4,4,4,4,4,4;',...\n"
        + "'4,4,4,4,4,4,4,4,4,2,2,2,2,3,3,3,3,3,3,3,3,3,3,3,4,4,2,2,2,2,3,4;',...\n"
        + "'4,4,4,4,4,4,4,4,4,2,2,2,2,3,4,4,4,4,4,4,4,4,4,4,4,4,2,2,2,2,3,4;',...\n"
        + "'4,4,4,4,4,4,4,4,2,2,2,2,3,4,4,4,4,4,4,4,4,4,4,4,4,4,2,2,2,2,3,4;',...\n"
        + "'4,4,4,4,4,4,4,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,4,4,4,2,2,2,2,3,4;',...\n"
        + "'4,4,4,4,4,4,4,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,4,4,2,2,2,2,2,4,4;',...\n"
        + "'4,4,4,4,4,4,4,4,2,2,2,2,2,2,2,3,3,3,3,3,3,3,3,4,4,2,2,2,2,3,4,4;',...\n"
        + "'4,4,4,4,4,4,2,2,2,2,2,2,2,2,3,4,4,4,4,4,4,4,4,4,2,2,2,2,2,4,4,4;',...\n"
        + "'4,4,4,4,4,2,2,2,2,2,2,2,2,2,2,3,4,4,4,4,4,4,4,4,1,1,1,1,3,4,4,4;',...\n"
        + "'4,4,4,4,2,2,2,2,2,2,2,2,2,2,2,2,4,4,4,4,4,4,4,4,1,1,1,1,3,4,4,4;',...\n"
        + "'4,4,4,2,2,2,2,2,3,3,3,2,2,2,2,2,3,4,4,4,4,4,4,1,1,1,1,1,3,4,4,4;',...\n"
        + "'4,4,4,2,2,2,2,3,4,4,4,4,4,2,2,2,2,4,4,4,4,4,4,1,1,1,1,1,3,4,4,4;',...\n"
        + "'4,4,4,2,2,2,2,3,4,4,4,4,4,3,3,3,3,4,4,4,4,4,4,1,1,1,1,1,3,4,4,4;',...\n"
        + "'4,4,4,2,2,2,2,2,4,4,4,4,4,2,2,2,2,3,4,4,4,4,4,2,2,2,2,3,4,4,4,4;',...\n"
        + "'4,4,4,4,2,2,2,2,2,2,4,4,4,2,2,2,2,3,3,4,4,4,2,2,2,2,2,3,4,4,4,4;',...\n"
        + "'4,4,4,4,4,1,1,1,1,1,1,1,4,2,2,2,2,2,2,2,2,2,2,2,2,2,3,4,4,4,4,4;',...\n"
        + "'4,4,4,4,4,4,1,1,1,1,1,1,1,1,2,2,2,2,2,2,2,2,2,2,2,2,3,4,4,4,4,4;',...\n"
        + "'4,4,4,4,4,4,4,4,1,1,1,1,1,1,1,2,2,2,2,2,2,2,2,2,2,3,4,4,4,4,4,4;',...\n"
        + "'4,4,4,4,4,4,4,4,4,4,2,2,2,2,2,3,3,3,3,3,3,3,3,3,3,4,4,4,4,4,4,4;',...\n"
        + "'4,4,4,4,4,4,4,4,4,4,4,2,2,2,2,3,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4;',...\n"
        + "'4,2,2,2,2,4,4,4,4,4,4,2,2,2,3,3,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4;',...\n"
        + "'4,2,2,2,2,4,4,4,4,4,4,2,2,2,3,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4;',...\n"
        + "'4,2,2,2,2,2,2,2,2,2,2,2,2,2,3,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4;',...\n"
        + "'4,4,2,2,2,2,2,2,2,2,2,2,2,3,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4;',...\n"
        + "'4,4,4,2,2,2,2,2,2,2,2,2,3,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4;',...\n"
        + "'4,4,4,4,3,3,3,3,3,3,3,3,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4;'];\n"
        + "ico3=['],[1 1 0;1 0 0;0 0 0;1 1 1]))'];\n"
        + "set_param(Ejs_sub_name,'position',sXY,'ShowName','off','MaskDisplay',[ico1,ico2,ico3],'MaskIconFrame','off');\n"
    );
    // Set the stop time to infinity
    matlabEng.engEvalString(id,"set_param('"+theModel+"','StartTime','0','StopTime','inf');");
    // Add a 'time to workspace' block
    matlabEng.engEvalString(id,
        "add_block('built-in/clock',[Ejs_sub_name,'/Clock']); \n"
        + "set_param([Ejs_sub_name,'/Clock'],'DisplayTime','on','Position', [30, 75, 70, 95]); \n"
        + "add_block('built-in/toworkspace',[Ejs_sub_name,'/timeToWS']); \n"
        + "set_param([Ejs_sub_name,'/timeToWS'],'VariableName','Ejs_time','Position',[150, 75, 200, 95],'Buffer','1'); \n"
        + "add_line(Ejs_sub_name,'Clock/1','timeToWS/1');");
    //fixed update
    if (fixedUpdateStep){
      matlabEng.engEvalString(id,"set_param('"+theModel+"','FixedStep','"+fixedStep+"');");
      // Add a fixedStep block
      matlabEng.engEvalString(id,
          "add_block('built-in/digital clock',[Ejs_sub_name,'/fixedStep']);\n"
          + "set_param([Ejs_sub_name,'/fixedStep'],'Position', [30, 135, 70, 155],'sampletime','"+fixedStep+"'); \n"
          + "add_block('built-in/matlabfcn',[Ejs_sub_name,'/Pause Simulink']); \n"
          + "comando=['set_param(''"+theModel+"'',','''','SimulationCommand','''',',','''','Pause','''',')']; \n"
          + "set_param([Ejs_sub_name,'/Pause Simulink'],'MATLABFcn',comando,'OutputWidth','0','Position',[150, 125, 200, 165]); \n"
          + "add_line(Ejs_sub_name,'fixedStep/1','Pause Simulink/1'); \n");
    }else{
      // Add a pause block
      matlabEng.engEvalString(id,
          "add_block('built-in/ground',[Ejs_sub_name,'/Gr1']); \n"
          + "set_param([Ejs_sub_name,'/Gr1'],'Position', [30, 135, 70, 155]); \n"
          + "add_block('built-in/matlabfcn',[Ejs_sub_name,'/Pause Simulink']); \n"
          + "comando=['set_param(''"+theModel+"'',','''','SimulationCommand','''',',','''','Pause','''',')']; \n"
          + "set_param([Ejs_sub_name,'/Pause Simulink'],'MATLABFcn',comando,'OutputWidth','0','Position',[150, 125, 200, 165]); \n"
          + "add_line(Ejs_sub_name,'Gr1/1','Pause Simulink/1'); \n");
    }
  }


  /**
   * Command to modify the simulink model
   */
  static private final String conversionCommand =
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
    + "    case 'in' \n"
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
    + "       case 'out'  \n"
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
    + "             %Guarda par�metros del Integrador                  \n"
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
    + "    case 'delete' \n"
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
    + "    case 'param' \n"    
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

