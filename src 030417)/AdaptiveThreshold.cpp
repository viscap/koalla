#include <stdlib.h>

#include "AdaptiveThreshold.h"

void adaptiveThreshold(unsigned char* input, unsigned char* output, int width, int height)
{
	long sum = 0;
	unsigned long* integralImage;
	
	int i, j;
	int count = 0;
	int index;
	int x1, y1, x2, y2;
	int s = width / 8;

	integralImage = (unsigned long*)malloc(width * height * sizeof(unsigned long*));

	for (i = 0; i < width; i++)
	{
		sum = 0;

		for (j = 0; j < height; j++)
		{
			index = j * width + i;

			sum += input[index];
			if (i == 0)
				integralImage[index] = sum;
			else
				integralImage[index] = integralImage[index - 1] + sum;
		}
	}

	for (i = 0; i < width; i++)
	{
		for (j = 0; j < height; j++)
		{
			index = j * width + i;

			x1 = i - s; x2 = i + s;
			y1 = j - s; y2 = j + s;

			if (x1 < 0) x1 = 0;
			if (x2 >= width) x2 = width - 1;
			if (y1 < 0) y1 = 0;
			if (y2 >= height) y2 = height - 1;

			count = (x2 - x1)*(y2 - y1);

			sum = integralImage[y2 * width + x2] - 
				integralImage[y1 * width + x2] -
				integralImage[y2 * width + x1] +
				integralImage[y1 * width + x1];

			if ((long)(input[index] * count) < (long)(sum * (1.0 - T)))
				output[index] = 0;
			else
				output[index] = 255;
		}
	}

	free(integralImage);
}
