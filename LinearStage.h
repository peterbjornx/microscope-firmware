#define DIR_REVERSE (0)
#define DIR_FORWARD (1)

#define POS_UNIT_USTEP (0)
#define POS_UNIT_UM    (1)
#define POS_UNIT_MM    (2)

class LinearStage {
public:
	
	String           f_Name;
	
	/* Object references */
    AccelStepper    &f_Stepper;
    DebouncedSwitch &f_NegSwitch;
    DebouncedSwitch &f_PosSwitch;
	bool             f_VerboseLogging = true;
	
	/* State fields */
	float            f_Acceleration        = 1.0e-3;   /* m / s^2           */
	float            f_MaxSpeed            = 1.0;      /* rev / s           */
	bool             f_WasStopped          = false;
	unsigned long    f_ProjectedStopTime   = 0;
	bool             f_WasAtTarget         = false;
	unsigned long    f_ProjectedTargetTime = 0;
	/* Calibration fields */
	long             f_NegLimit            = 0;        /* microsteps        */
	long             f_PosLimit            = 0;        /* microsteps        */
	long             f_SoftLimitOffset_Neg = 256;      /* microsteps        */
	long             f_SoftLimitOffset_Pos = 256;      /* microsteps        */
	
	long             f_Microsteps          = 8;        /* microsteps / step */
	long             f_StepsPerRev         = 200;      /* steps / rev       */
	long             f_MicronsPerRev       = 1000;     /* um    / rev       */
	
	unsigned long    f_ReportDoneDelay     = 200;      /* ms */
	
	/* Derived calibration fields */
	long             f_AngstromPerRev;
	long             f_MicrostepsPerRev;
	long             f_StepsPerMicron;
	float            f_MicrostepsPerMetre;
	
	/* Configuration Fields */
    float            f_HomeSpeed_Coarse    =  10000.0; /* microsteps/s      */
    float            f_HomeSpeed_Rewind    =    300.0; /* microsteps/s      */
    float            f_HomeSpeed_Fine      =     50.0; /* microsteps/s      */
    float            f_HomeAccel           = 100000.0; /* microsteps/s^2    */
    
    LinearStage(
		String           name,
		AccelStepper    &stepper, 
		DebouncedSwitch &limitSwNeg, 
		DebouncedSwitch &limitSwPos ) :
		
		f_Name( name ),
		f_Stepper( stepper ),
		f_NegSwitch( limitSwNeg ),
		f_PosSwitch( limitSwPos ) {
			
		updateCalibration( );
		
    }

    bool isMoving() {
		bool l_IsMoving = f_Stepper.speed() != 0;
		if ( l_IsMoving ) {
			f_WasStopped = false;
			return true;
		}
		if ( !f_WasStopped ) {
			f_ProjectedStopTime = millis() + f_ReportDoneDelay;
			f_WasStopped = true;
			return true;
		} 
		if ( !f_ProjectedStopTime )
			return false;
		if ( f_ProjectedStopTime < millis() ) {
			f_ProjectedStopTime = 0;
			return false;
		}
		return true;
    }
    
    bool isOffTarget() {
		bool l_IsMoving = f_Stepper.distanceToGo() != 0;;
		if ( l_IsMoving ) {
			f_WasAtTarget = false;
			return true;
		}
		if ( !f_WasAtTarget ) {
			f_ProjectedTargetTime = millis() + f_ReportDoneDelay;
			f_WasAtTarget = true;
			return true;
		} 
		if ( !f_ProjectedTargetTime )
			return false;
		if ( f_ProjectedTargetTime < millis() ) {
			f_ProjectedTargetTime = 0;
			return false;
		}
		return true;
    }

    void runUntilStopped( ) {
      while ( isMoving() ) {
		  process( );
      }
    }

    void completeMovement( ) {
      while ( isOffTarget() ) {
		  process( );
      }
    }
	
	void updateCalibration( ) {
		f_MicrostepsPerRev   = f_Microsteps * f_StepsPerRev;
		f_MicrostepsPerMetre = f_MicrostepsPerRev * 1e6 / f_MicronsPerRev;
		f_AngstromPerRev     = f_MicronsPerRev * 10000;
        setAcceleration( f_Acceleration );
		setMaxSpeed    ( f_MaxSpeed     );
	}
 
  inline void process() {
    f_NegSwitch.read();
    f_PosSwitch.read();
        f_Stepper.run();
  }
    
private:
	
    const long sz   = 100000L;
	const long sz10 = sz / 10;
	  
    void switchSeek( DebouncedSwitch &sw, float spd, int pol, int dir ) {
      f_Stepper.setMaxSpeed(spd);
      while ( sw.read() != pol ) {
        int dtg = f_Stepper.distanceToGo();
        if ( dtg < 0 )
          dtg = -dtg;
        if ( dtg < sz10 ) {
          f_Stepper.move( dir ? sz : -sz);
		}
        f_Stepper.run();
      }    
      f_Stepper.setSpeed(0);
      f_Stepper.move(0);
      runUntilStopped( );
      f_Stepper.move(0);
    }

	/**
	 * @brief Accurately  determine  limit  switch  actuation  point
	 * @param dir    The direction for which the limit switch should
	 *               be found: DIR_FORWARD will look up the positive
	 *               switch,  and  DIR_REVERSE  will  look  up  the 
	 *               negative switch.
	 * @return The position of the limit switch, in microsteps.
	 */
    long findLimitSwitch( int dir ) {
		/* The limit switch to find is determined by the direction:
		 * If we move forward, we will never hit the negative limit
		 * switch, provided that they were correctly specified.
		 */
		DebouncedSwitch &l_LimitSwitch =
		    dir ? f_PosSwitch : f_NegSwitch;
		
		/* Set the acceleration to the homing acceleration value */
		f_Stepper.setAcceleration( f_HomeAccel );
		
		/* Ensure the stability of the limit switch output */
		l_LimitSwitch.blockingEnsure();
		
		switchSeek( l_LimitSwitch, f_HomeSpeed_Coarse, HIGH,  dir );
		switchSeek( l_LimitSwitch, f_HomeSpeed_Rewind, LOW , !dir );
		switchSeek( l_LimitSwitch, f_HomeSpeed_Fine  , HIGH,  dir );
		
		/* Reload operational motion parameters */
		setAcceleration( f_Acceleration );
		setMaxSpeed    ( f_MaxSpeed     );
		
		/* Return the current position of the stepper, which should be
		 * right at the switch actuation point */
		return f_Stepper.currentPosition();
    }
	
public:

	void performFullCalibration( ) {
		long l_PosLimitSw, l_NegLimitSw;
		long l_PosLimit  , l_NegLimit;
		long l_MaxTravel;
		long l_CentrePos;
		
		/* Determine limit switch positions */
		l_NegLimitSw = findLimitSwitch( DIR_REVERSE );
		l_PosLimitSw = findLimitSwitch( DIR_FORWARD );
		
		/* Apply limit offsets */
		l_NegLimit   = l_NegLimitSw + f_SoftLimitOffset_Neg;
		l_PosLimit   = l_PosLimitSw - f_SoftLimitOffset_Pos;
		
		/* Compute max travel and centre */
		l_MaxTravel = l_PosLimit - l_NegLimit;
		l_CentrePos = l_PosLimit - l_MaxTravel / 2;
		
		/* Move the stage to the positive limit */
		f_Stepper.moveTo( l_PosLimit );
		completeMovement( );
		
		/* Ensure that the limit switch is no longer tripped */
		if ( f_PosSwitch.read( ) ) {
			
			if ( f_VerboseLogging )
				Serial.println(f_Name + " failed to calibrate: "
                   "f_SoftLimitOffset_Pos is too small!");
				   
			//TODO: Other error handling?
			
		}
		
		/* Tell the stepper library that the midpoint 
		 * between the limits is to be the origin. */
		f_Stepper.setCurrentPosition( l_MaxTravel / 2 );

		/* Translate the limit positions into the new origin */
		f_PosLimit = l_PosLimit - l_CentrePos; /* this equates to l_MaxTravel / 2 */
		f_NegLimit = l_NegLimit - l_CentrePos;
		
		if ( f_VerboseLogging ) {
			Serial.println( f_Name + " calibrated");
			Serial.println( "    Negative limit: " + String(f_NegLimit) );
			Serial.println( "    Positive limit: " + String(f_PosLimit) );
		}
		//TODO: Store calibration data in NVRAM ?
		
	}
	
	void restoreCalibrationNoAbs( long NegLimit, long PosLimit ) {
		
		/* Load calibration constants */
		f_NegLimit   = NegLimit;
		f_PosLimit   = PosLimit;
		
	}
	
	void restoreCalibration( long NegLimit, long PosLimit ) {
		
		/* Load calibration constants */
		f_NegLimit   = NegLimit;
		f_PosLimit   = PosLimit;
		
		/* Find one of the limit switches to restore 
		 * absolute position calibration */
		findHome();
		
	}
	
	void findHome( ) {
		long l_NegLimitSw, l_NegLimit;
		
		/* Determine limit switch position */
		l_NegLimitSw = findLimitSwitch( DIR_REVERSE );
		
		/* Apply limit offset */
		l_NegLimit   = l_NegLimitSw + f_SoftLimitOffset_Neg;
		
		/* Move the stage to the negative limit */
		f_Stepper.moveTo( l_NegLimit );
		completeMovement( );
		
		/* Ensure that the limit switch is no longer tripped */
		if ( f_NegSwitch.read( ) ) {
			
			if ( f_VerboseLogging )
				Serial.println(f_Name + " failed to home: "
                   "f_SoftLimitOffset_Neg is too small!");
				   
			//TODO: Other error handling?
			
		}
		
		/* We should now be at the negative limit */
		
		/* We restore the calibrated coordinate system by 
		 * changing the position to the calibration neg limit
		 * without moving. */
		f_Stepper.setCurrentPosition( f_NegLimit );
		
	}
	
	void moveCentre( ) {
		f_Stepper.setAcceleration( 100 );
		f_Stepper.setMaxSpeed( 400 );
		f_Stepper.moveTo( 0 );
		setAcceleration( f_Acceleration );
		setMaxSpeed    ( f_MaxSpeed );
	}
	
	void moveToSteps( long x ) {
		f_Stepper.moveTo( x );
	}
	
	void moveSteps( long x ) {
		f_Stepper.move( x );
	}
	
	long micronsToMicrosteps( long v ) {
		long l_IntermediateA;
		
		l_IntermediateA = v * f_MicrostepsPerRev; /* um * ustep / rev */
		return l_IntermediateA / f_MicronsPerRev; /* usteps           */
	}
	
	long microstepsToAngstrom( long v ) {
		long l_IntermediateA;
		
		l_IntermediateA = v * f_AngstromPerRev;   /* ustep * Ang / rev */
		return l_IntermediateA / f_MicrostepsPerRev; /* Angstrom       */
	}
	
	long microstepsToMicron( long v ) {
		long l_IntermediateA;
		
		l_IntermediateA = v * f_MicronsPerRev;   /* ustep * Ang / rev */
		return l_IntermediateA / f_MicrostepsPerRev; /* Angstrom       */
	}
	
	/**
	 * @brief Computes the difference between the requested position
	 *        and the position attainable using the stepper resolution.
	 *
	 * @note  The return value is in Angstrom because the steps need not be
	 *        multiples of nanometres, and this guarantees an error of at most
	 *        10^-10 metres per conversion. This low error is needed because
	 *        it is integrated over relative moves, which could happen 1000s
	 *        of times over a single run; meaning that this error would cause
	 *        drift of the coordinate system.
	 *
	 * @param v     The position in micron
	 *
	 * @returns The difference in Angstrom.
	 */
	long micronsResidualAngstrom( long v ) {
		long l_RevRemainder;
		long l_Microsteps;
		long l_StepDist;
		long l_RevRemainderAngstrom;
		
		/* Trim off the full revolutions so we can prevent overflow */
		l_RevRemainder         = v - f_MicronsPerRev * ( v / f_MicronsPerRev );
		
		/* Compute the number of steps we need to take */
		l_Microsteps           = micronsToMicrosteps( l_RevRemainder ); 
		
		/* Compute the distance this would bring us, in Angstrom */
		l_StepDist             = microstepsToAngstrom( l_Microsteps );
		
		/* Match units */
		l_RevRemainderAngstrom = l_RevRemainder * 10000;
		
		/* Compute the difference between the requested position and
		 * the step selected for it */
		return l_RevRemainderAngstrom - l_StepDist;
		
	}
	
	void moveToMicrons( long pos ) {
		long l_MicrostepPos;
		
		l_MicrostepPos = micronsToMicrosteps( pos );
		
		//TODO: Handle residuals
		
		f_Stepper.moveTo( l_MicrostepPos );
	}
	
	void moveMicrons( long dist ) {
		long l_MicrostepDist;
		
		l_MicrostepDist = micronsToMicrosteps( dist );
		
		f_Stepper.move( l_MicrostepDist );
	}

	void setAcceleration( float value ) {
		f_Acceleration = value;
		f_Stepper.setAcceleration( f_Acceleration * f_MicrostepsPerMetre );
	}

	void  setMaxSpeed( float value ) {
		f_MaxSpeed = value;
		f_Stepper.setMaxSpeed( f_MaxSpeed * f_MicrostepsPerRev );
	}
	
	float  getAcceleration() {
		return f_Acceleration;
	}
	
	float  getMaxSpeed() {
		return f_MaxSpeed;
	}
	
	float  getPitch() {
		return f_MicronsPerRev;
	}
	
	long   getPositionSteps() {
		return f_Stepper.currentPosition();
	}
	
	long   getPositionMicrons() {
		return microstepsToMicron( f_Stepper.currentPosition() );
	}
	
	long convertToUnit( int dim, long steps ) {
		switch( dim ) {
			case POS_UNIT_USTEP:
				return steps;
			case POS_UNIT_UM:
				return microstepsToMicron( steps );
			case POS_UNIT_MM:
				return microstepsToMicron( steps ) / 1000;
		}
	}
	
	long getPosition( int dim ) {
		switch( dim ) {
			case POS_UNIT_USTEP:
				return getPositionSteps();
			case POS_UNIT_UM:
				return getPositionMicrons();
			case POS_UNIT_MM:
				return getPositionMicrons() / 1000;
		}
	}

	void moveTo( int dim, long pos ) {
		switch( dim ) {
			case POS_UNIT_USTEP:
				moveToSteps( pos );
				return;
			case POS_UNIT_UM:
				moveToMicrons( pos );
				return;
		}
	}

	void move( int dim, long pos ) {
		switch( dim ) {
			case POS_UNIT_USTEP:
				moveSteps( pos );
				return;
			case POS_UNIT_UM:
				moveMicrons( pos );
				return;
		}
	}

	long getNegLimit( int dim ) {
		return convertToUnit( dim, f_NegLimit );
	}

	long getPosLimit( int dim ) {
		return convertToUnit( dim, f_PosLimit );
	}
	
};
