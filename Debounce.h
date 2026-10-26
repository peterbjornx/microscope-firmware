class DebouncedSwitch {
	
private:
	int           f_Pin;
	unsigned long f_DebounceTime = DEBOUNCE_TIME;
	unsigned long f_LastRead;
	unsigned long f_LastChange;
	bool          f_LastValue;
	bool          f_StableValue;

	void setupPin( void ) {
		pinMode( f_Pin, INPUT );		
	}
	
	bool readRaw( void ) {
		return digitalRead( f_Pin );
	}
	
	void initFields( void ) {
		f_LastRead    = millis( );
		f_LastChange  = f_LastRead;
		f_LastValue   = readRaw( );
		f_StableValue = f_LastValue;
	}
	
public:

	DebouncedSwitch( int pin ) {
		f_Pin = pin;
		setupPin( );
		initFields( );
	}
	
	int read( void ) {
		bool          l_RawValue;
		unsigned long l_MeasureTime;
		unsigned long l_DeltaTime;
		
		l_RawValue    = readRaw();
		l_MeasureTime = millis();
		
		f_LastRead    = l_RawValue;
		
		l_DeltaTime   = l_MeasureTime - f_LastChange;
		
		if ( l_RawValue != f_LastValue ) {
			f_LastChange  = l_MeasureTime;
			f_LastValue   = l_RawValue;
		} else if ( l_DeltaTime >= f_DebounceTime ) {
			f_StableValue = l_RawValue;
		}
		
		return f_StableValue ? HIGH : LOW;
	}

	void blockingEnsure( void ) {
		
		unsigned long l_DeltaTime;
		
		l_DeltaTime = f_LastRead - millis();
		
		if ( l_DeltaTime < f_DebounceTime )
			return;
		
		for ( ; ; ) {
			
			read( );
			
			l_DeltaTime = millis() - f_LastChange;
			
			if ( l_DeltaTime > f_DebounceTime )
				return;
			
		}
		
	}

};
