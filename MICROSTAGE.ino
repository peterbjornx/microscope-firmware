/**
 * Firmware for a 3 axis microscope stage
 *
 * Derived from Micromanager MM_vLStep_Z_Stepper.ino
 *
 * Most work (c) 2020 Peter Bosch
 *
 */

#include <AccelStepper.h>

#define PIN_X_DRV_STP (A0)
#define PIN_X_DRV_DIR (A1)
#define PIN_Y_DRV_STP (A2)
#define PIN_Y_DRV_DIR (A3)
#define PIN_Z_DRV_STP (6)
#define PIN_Z_DRV_DIR (5)

#define PIN_X_LMT_NEG (A5)
#define PIN_X_LMT_POS (A4)
#define PIN_Y_LMT_NEG (3)
#define PIN_Y_LMT_POS (4)
#define PIN_Z_LMT_NEG (7)
#define PIN_Z_LMT_POS (8)

#define AXIS_SIZE_X      (237501)
#define AXIS_SIZE_Y      (93780)

#define XY_MICRO_STEPS       (8L)

#define XY_STEPS_PER_REV     (200L)

#define Z_MICRO_STEPS        (1L)

#define Z_STEPS_PER_REV      (90112L)

#define Z_UM_PER_REV         (6400L)

#define XY_UM_PER_REV        (1000L)
// 100 divs / 22 teeth * 16 teeth 
#define DEBOUNCE_TIME (10)

#include "Debounce.h"
#include "LinearStage.h"


int             g_ReportDone;

class LStepStageController {
public:
	int             f_DimX = POS_UNIT_UM;
	int             f_DimY = POS_UNIT_UM;
	int             f_DimZ = POS_UNIT_UM;

	AccelStepper    stepper_x;
	AccelStepper    stepper_y;
	AccelStepper    stepper_z;

	DebouncedSwitch g_LimitSwXNeg;
	DebouncedSwitch g_LimitSwXPos;
	DebouncedSwitch g_LimitSwYNeg;
	DebouncedSwitch g_LimitSwYPos;
	DebouncedSwitch g_LimitSwZNeg;
	DebouncedSwitch g_LimitSwZPos;

	LinearStage     g_StageX;
	LinearStage     g_StageY;
	LinearStage     g_StageZ;
    
	LStepStageController() :
		stepper_x    ( AccelStepper::DRIVER   , PIN_X_DRV_STP, PIN_X_DRV_DIR),
		stepper_y    ( AccelStepper::DRIVER   , PIN_Y_DRV_STP, PIN_Y_DRV_DIR),
		stepper_z    ( AccelStepper::DRIVER   , PIN_Z_DRV_STP, PIN_Z_DRV_DIR),
		g_LimitSwXNeg( PIN_X_LMT_NEG ),
		g_LimitSwXPos( PIN_X_LMT_POS ),
		g_LimitSwYNeg( PIN_Y_LMT_NEG ),
		g_LimitSwYPos( PIN_Y_LMT_POS ),
		g_LimitSwZNeg( PIN_Z_LMT_NEG ),
		g_LimitSwZPos( PIN_Z_LMT_POS ),
		g_StageX     ( "X", stepper_x, g_LimitSwXNeg, g_LimitSwXPos ),
		g_StageY     ( "Y", stepper_y, g_LimitSwYNeg, g_LimitSwYPos ),
		g_StageZ     ( "Z", stepper_z, g_LimitSwZNeg, g_LimitSwZPos )
	{
        g_ReportDone = 0;
        g_StageZ.f_Microsteps = Z_MICRO_STEPS;
        g_StageZ.f_StepsPerRev = Z_STEPS_PER_REV;
        g_StageZ.f_MicronsPerRev = Z_UM_PER_REV;
        g_StageZ.f_MaxSpeed = 0.005;
        g_StageZ.f_Acceleration = 0.75e-3;
        g_StageZ.f_ReportDoneDelay = 50;
        g_StageZ.updateCalibration( );
	}
	
	void loadCalibration( ) {
		//TODO: Load calibration from EEPROM
		g_StageX.restoreCalibrationNoAbs( -59112, 59112 );
		g_StageY.restoreCalibrationNoAbs( -23190, 23190 );
	}
	
	void doCalibration( ) {
		//TODO: Load calibration from EEPROM
		g_StageX.restoreCalibration( -59112, 59112 );
		g_StageY.restoreCalibration( -23190, 23190 );
	}
	
	void setup ( ) {
	  reply ("Vers:LS");
	  loadCalibration( );
	  centreAxes( );
	}

	bool isAnyMoving() {
		return g_StageX.isMoving() || g_StageY.isMoving() || g_StageZ.isMoving();
	}

	bool isAnyOffTarget() {
		return g_StageX.isOffTarget() || g_StageY.isOffTarget() || g_StageZ.isOffTarget();
	}

	void runUntilStopped( ) {
		while ( isAnyMoving() ) {
			g_StageX.process();
			g_StageY.process();
            g_StageZ.process();
		}
	}

	void completeMovement( ) {
		while ( isAnyOffTarget() ) {
			g_StageX.process();
			g_StageY.process();
            g_StageZ.process();
		}
	}

	void selfTest() {
		g_StageX.performFullCalibration( );
		g_StageY.performFullCalibration( );
	}

	void homeAxes() {
		g_StageX.findHome( );
		g_StageY.findHome( );
	}

	void centreAxes() {
		g_StageX.moveCentre();
		g_StageY.moveCentre();
		completeMovement( );
	}

	void runSteppers() {
		g_StageX.process();
		g_StageY.process();
        g_StageZ.process();
	}

	void processLimitSwitches() {
	}


	void reply(String s) {
	  Serial.print(s);
	  Serial.print("\r");
	}
	
	bool f_CmdWait = false;
	unsigned long f_CmdTime;
	String f_OldCommand;
	
	bool handleDelayed() {
		if ( !f_CmdWait )
			return true;
		if ( millis() < f_CmdTime ) 
			return false;
		f_CmdWait = false;
		processCommand( f_OldCommand, true );
		return !f_CmdWait;
	}
	
	void scheduleCommand(String &s, long delay ) {
		f_CmdWait = true;
		f_CmdTime = millis() + delay;
		f_OldCommand = s;
	}

	void processCommand(String s, bool immed) {
	  if (s.startsWith("?ver")) {
		reply ("Vers:LS");
	  } else if (s.startsWith("!autostatus 0")) {
		delay(5);
	  } else if (s.startsWith("?det")) {
		reply ("60");
	  } else if (s.startsWith("?calcal")) {
		doCalibration( );
	  } else if (s.startsWith("?pitch x")) {
		Serial.println( g_StageX.getPitch() );
	  } else if (s.startsWith("?pitch y")) {
		Serial.println( g_StageY.getPitch() );
	  } else if (s.startsWith("?pitch z")) {
    Serial.println( g_StageZ.getPitch() );
    } else if (s.startsWith("?vel x")) {
		Serial.println( g_StageX.getMaxSpeed() );
	  } else if (s.startsWith("?vel y")) {
		Serial.println( g_StageY.getMaxSpeed() );
	  }  else if (s.startsWith("?vel z")) {
    Serial.println( g_StageZ.getMaxSpeed() );
    } else if (s.startsWith("?accel x")) {
		Serial.println( g_StageX.getAcceleration() );
	  }else if (s.startsWith("?accel y")) {
		Serial.println( g_StageY.getAcceleration() );
	  }else if (s.startsWith("?accel z")) {
    Serial.println( g_StageZ.getAcceleration() );
    } else if (s.startsWith("!dim x")) {
		f_DimX = s.substring(s.indexOf("x") + 1).toInt();
		delay(5);
	  } else if (s.startsWith("!dim y")) {
		f_DimY = s.substring(s.indexOf("y") + 1).toInt();
		delay(5);
	  } else if (s.startsWith("!dim ")) {
		String args = s.substring(s.indexOf(" ")+1);
		String args2 = args.substring(args.indexOf(" ")+1);
		String args1 = args.substring(0,args.indexOf(" "));
		f_DimX = args1.toInt();
		f_DimY = args2.toInt();
	  } else if (s.startsWith("?statusaxis")) {
		reply(isAnyOffTarget() ? "MM" : "@@");
	  } else if (s.startsWith("!vel x")) {
		g_StageX.setMaxSpeed(s.substring(s.indexOf("x") + 1).toFloat());
	  } else if (s.startsWith("!vel y")) {
		g_StageY.setMaxSpeed(s.substring(s.indexOf("y") + 1).toFloat());
	  }  else if (s.startsWith("!accel x")) {
		g_StageX.setAcceleration(s.substring(s.indexOf("x") + 1).toFloat());
	  } else if (s.startsWith("!accel y")) {
		g_StageY.setAcceleration(s.substring(s.indexOf("y") + 1).toFloat());
	  } else if (s.startsWith("?pos")) {
		if ( immed ) {
			reply( String( g_StageX.getPosition( f_DimX ) ) + " " + 
				String( g_StageY.getPosition( f_DimY ) )  + " " + 
				String( g_StageZ.getPosition( f_DimZ ) ) );
		} else {
			scheduleCommand( s, 200 );
		}
	  } else if (s.startsWith("?ipos")) {
      reply( String( g_StageX.getPosition( f_DimX ) ) + " " + 
        String( g_StageY.getPosition( f_DimY ) )  + " " + 
        String( g_StageZ.getPosition( f_DimZ ) ) );
    } else if (s.startsWith("?lim x")) {
		reply( String( g_StageX.getNegLimit( f_DimX ) ) + " " + 
		       String( g_StageX.getPosLimit( f_DimX ) ) );
	  } else if (s.startsWith("?lim y")) {
		reply( String( g_StageY.getNegLimit( f_DimY ) ) + " " + 
		       String( g_StageY.getPosLimit( f_DimY ) ) );
	  } else if (s.startsWith("?lim z")) {
   reply( String( g_StageZ.getNegLimit( f_DimZ ) ) + " " + 
           String( g_StageZ.getPosLimit( f_DimZ ) ) );
    } else if (s.startsWith("?lim ")) {
		reply( "-10000.00 10000.00" );
	  } else if (s.startsWith("!pos ")) {
		delay(5);
	  } else if (s.startsWith("?status")) {
		reply ("OK...");
	  } else if (s.startsWith("!dim z 0")) {
		delay(5);
	  } else if (s.startsWith("!speed ")) {
		delay(5);
	  } else if (s.startsWith("!mor x")) {
		String delta = s.substring(s.indexOf("x") + 1);
		g_StageX.move( f_DimX, delta.toInt() );
        if ( g_ReportDone == -1 )
            g_ReportDone = 1;
	  } else if (s.startsWith("!moa x")) {
		String apos = s.substring(s.indexOf("x") + 1);
		g_StageX.moveTo( f_DimX, apos.toInt() );
        if ( g_ReportDone == -1 )
            g_ReportDone = 1;
	  }  else if (s.startsWith("!mor y")) {
		String delta = s.substring(s.indexOf("y") + 1);
		g_StageY.move( f_DimY, delta.toInt() );
        if ( g_ReportDone == -1 )
            g_ReportDone = 1;
	  } else if (s.startsWith("!moa y")) {
		String apos = s.substring(s.indexOf("y") + 1);
		g_StageY.moveTo( f_DimY, apos.toInt() );
        if ( g_ReportDone == -1 )
            g_ReportDone = 1;
	  } else if (s.startsWith("!mor z")) {
		String delta = s.substring(s.indexOf("z") + 1);
		g_StageZ.move( f_DimZ, delta.toInt() );
        if ( g_ReportDone == -1 )
            g_ReportDone = 1;
	  } else if (s.startsWith("!moa z")) {
		String apos = s.substring(s.indexOf("z") + 1);
		g_StageZ.moveTo( f_DimZ, apos.toInt() );
        if ( g_ReportDone == -1 )
            g_ReportDone = 1;
	  } else if (s.startsWith("!mor ")) {
		String args = s.substring(s.indexOf(" ")+1);
		String args2 = args.substring(args.indexOf(" ")+1);
		String args1 = args.substring(0,args.indexOf(" "));
		g_StageX.move( f_DimX, args1.toInt() );
		g_StageY.move( f_DimY, args2.toInt() );
        if ( g_ReportDone == -1 )
            g_ReportDone = 1;
	  } else if (s.startsWith("!moa ")) {
		String args = s.substring(s.indexOf(" ")+1);
		String args2 = args.substring(args.indexOf(" ")+1);
		String args1 = args.substring(0,args.indexOf(" "));
		g_StageX.moveTo( f_DimX, args1.toInt() );
		g_StageY.moveTo( f_DimY, args2.toInt() );
        if ( g_ReportDone == -1 )
            g_ReportDone = 1;
	  } else if (s.startsWith("?err")) {
		reply ("0");
	  } else if (s.startsWith("!reqc")) {
        g_ReportDone = -1;
      }
	}

};

void *operator new( size_t size, void *ptr );/*{
  return ptr;
}*/
 
void operator delete( void *obj, void *alloc );/*{
  return;
}*/

//XXX: Arduino compiler is haunted.
char g_CursedHack[ sizeof(LStepStageController) ];

LStepStageController *g_TheController;

void setup() {
	Serial.begin( 115200 );
	g_TheController = new( ( void * ) g_CursedHack ) LStepStageController;
	g_TheController->setup();
}

char c = '*';
float z;
String cmd = "" ;
int ig = 0;

void loop()
{
	if ( g_TheController->handleDelayed() && ig == 0 ) {
		if (Serial.available() > 2) {
		  cmd = Serial.readStringUntil('\r');
		  g_TheController->processCommand(cmd, false);
		  cmd = "";
		}
	} else if ( g_ReportDone == 1 && !g_TheController->isAnyOffTarget() ) {
        g_TheController->reply("d");
        g_ReportDone = 0;
	}
    g_TheController->processLimitSwitches();
    g_TheController->runSteppers();
	ig = (ig + 1 ) % 1000;
}
