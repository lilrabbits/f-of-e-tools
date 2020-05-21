int
main(void)
{
	// Vairable for LED flash
	volatile unsigned int *		debugLEDs = (unsigned int *)0x08004000;
	enum
	{
		kSpinDelay = 400000,
	};

	// Variables for computations
	const int data[] = {3, 8, 23, 32, 89, 34, 12, 34, 23, 6};
	const int data_length = sizeof(data)/sizeof(data[0]);

	const int correct_mean = 26;
	const int correct_varience = 580;

	int mean = correct_mean;
	int varience = correct_varience;

	// while (mean == correct_mean && varience == correct_varience) {
	for (int k = 0; k < 100; k++) {
		// Flash LED
		*debugLEDs = 0xFF;
		for (int j = 0; j < kSpinDelay; j++);
		*debugLEDs = 0x00;

		// Do computations
		int sum = 0;
		int sum_squares = 0;
		for (int i = 0; i < data_length; i++) {
			sum += data[i];
			sum_squares += (data[i]*data[i]);
		};

		mean = sum/data_length;
		varience = sum_squares/data_length - mean*mean;
	}
	return 0;
}
