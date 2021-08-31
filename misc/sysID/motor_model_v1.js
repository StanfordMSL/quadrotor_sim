/* //////////////// Sweep characterization ////////////////
 The script will sweep between the input values "minVal" and "maxVal". The sweep will be made in discrete "stepsQty" steps. Each step will consist of a settling time "settlingTime" after which a new log entry will be recorded. To reduce noise, "samplesAvg" will be averaged and recorded.

 ^ Motor Input
 |                       ___ ___  maxVal                   4_
 |                   ___|       |___                       3_
 |               ___|               |___                   2_                
 |           ___|                       |___               1_ stepsQty = 4
 | minVal___|                               |___           0_
 |________________________________________________> Time

///////////// User defined variables //////////// */

var minVal = 1000;       // Min. input value [700us, 2300us] 
var maxVal = 2000;       // Max. input value [700us, 2300us]
var stepsQty = 10;        // Number of steps
var settlingTime = 3;    // Settling time before measurement input change [s]
var samplesAvg = 20;     // Number of samples to average
var stepsGoDown = true;  // If true, the test will step down, if false, the steps will only go up.
var repeat = 1;          // Number of times to repeat the same sequence
var filePrefix = "StepsTest";

///////////////// Beginning of the script //////////////////

//Starting new file
rcb.files.newLogFile({prefix: filePrefix});

//ESC initialization
rcb.console.print("Initializing ESC...");
rcb.output.set("esc",1000);
rcb.wait(startSteps, 4);

//Start steps
function startSteps(){
    rcb.console.print("Starting Steps Up...");
    rcb.console.setVerbose(true);
    rcb.output.steps("esc", minVal, maxVal, stepsQty, stepFct);
}

// The following function will be executed at each step.
var goingUp = true;
function stepFct(isLastStep, nextStepFct){
    rcb.console.setVerbose(false);
    
    //Settling time, then read sensors
    rcb.wait(function(){
        rcb.sensors.read(readDone, samplesAvg);
    }, settlingTime);

    //Function called when read completed
    function readDone(result){

        // Write the results and proceed to next step
        rcb.files.newLogEntry(result,function(){
            if(isLastStep){ // If it is the last step
                if (goingUp && stepsGoDown){
                    goingUp = false;
                    rcb.console.print("Starting Steps Down...");
                    rcb.console.setVerbose(true);
                    rcb.output.steps("esc", maxVal, minVal, stepsQty, stepFct);
                }else{
                    if(repeat>1){
                        repeat --;
                        rcb.console.print("Repeating script " + repeat + " more times.");
                        goingUp = true;
                        startSteps();
                    }else{
                        rcb.endScript(); // End script 
                    }
                }
            }else
                nextStepFct();
        }); 
    }    
}
