#include <math.h>

int Stats_GetStats(unsigned long data[], unsigned int size, unsigned long* stddev, unsigned long* average, unsigned long* maxdev) {
	unsigned long sum = 0;
	unsigned long stddevsum = 0;
	unsigned long max = 0;
	unsigned long min = 0xFFFFFFFF;
	for(int iii = 0; iii < size; iii++) {
		sum += data[iii];
		if(data[iii] > max) {
			max = data[iii];
		}
		if(data[iii] < min) {
			min = data[iii];
		}
	}
	*average = sum / size;
	for(int iii = 0; iii < size; iii++) {
		stddevsum += pow((data[iii] - *average), 2);
	}
	*stddev = sqrt(stddevsum / size);
	*maxdev = max - min;
	return 0;
}
