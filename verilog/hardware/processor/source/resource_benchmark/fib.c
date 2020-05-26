int
main(void)
{
	// Variable for LED flash
	volatile unsigned int *		debugLEDs = (unsigned int *) 0x2000; 
	enum
	{
		kSpinDelay = 400000,
	};

	// Variables for computations
	const int count = 8;
	int first_term = 0, second_term = 1, next_term, i;
	int fib_list[count];
	int correct_fib_list[] = {0, 1, 1, 2, 3, 5, 8, 13};

	for ( i = 0 ; i < count ; i++ ) {
		// Flash LED
		*debugLEDs = 0xFF;
		for (int j = 0; j < kSpinDelay; j++);
		*debugLEDs = 0x00;
			if ( i <= 1 )
				next_term = i;
			else
			{
				next_term = first_term + second_term;
				first_term = second_term;
				second_term = next_term;
			}
			fib_list[i]=next_term;
				}
	if (fib_list == correct_fib_list)
	return 1;
	else
	return 0;
}
