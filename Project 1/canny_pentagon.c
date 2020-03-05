#include <stdio.h>
#include <stdlib.h>
#include "dc_image.h"


#define CANNY_THRESH2 10
#define CANNY_BLUR2   2

#define MIN(a,b)  ( (a) < (b) ? (a) : (b) )
#define MAX(a,b)  ( (a) > (b) ? (a) : (b) )
#define ABS(x)    ( (x) <= 0 ? 0-(x) : (x) )

struct Ransac{
    int p_count;
    double line_m;
    double line_c;
    int p_x0,p_y0,p_x1,p_y1;
};

void bsort(struct Ransac list[20000], int s)
{
    int i, j;
    struct Ransac temp;
    
    for (i = 0; i < s - 1; i++)
    {
        for (j = 0; j < (s - 1-i); j++)
        {
            if (list[j].p_count < list[j + 1].p_count)
            {
                temp = list[j];
                list[j] = list[j + 1];
                list[j + 1] = temp;
            }
        }
    }
}

int main()
{
	int y,x;
	int rows2, cols2, chan2;

	//-----------------
	// Read the image    [y][x][c]   y number rows2   x cols2  c 3
	//-----------------
	byte ***img2 = LoadRgb("pentagon.png", &rows2, &cols2, &chan2);
	printf("img2 %p rows %d cols %d chan %d\n", img2, rows2, cols2, chan2);

	
	//SaveRgbPng(img2, "out/1_img2.png", rows2, cols2);
	
	//-----------------
	// Convert to gray2scale
	//-----------------
	byte **gray2 = malloc2d(rows2, cols2);
	for (y=0; y<rows2; y++){
		for (x=0; x<cols2; x++) {
			int r = img2[y][x][0];
			int g = img2[y][x][1];
			int b = img2[y][x][2];
			gray2[y][x] =  (r+g+b) / 3;
		}
	}
	
	//SavegrayPng(gray2, "out/2_gray2.png", rows2, cols2);

	//-----------------
	// Box Blur   ToDo: Gaussian Blur is better
	//-----------------
	
	// Box blur is separable, so separately blur x and y
	int k_x=CANNY_BLUR2, k_y=CANNY_BLUR2;
	
	// blur in the x dimension
	byte **blurx2 = (byte**)malloc2d(rows2, cols2);
	for (y=0; y<rows2; y++) {
		for (x=0; x<cols2; x++) {
			
			// Start and end to blur
			int minx = x-k_x/2;      // k_x/2 left of pixel
			int maxx = minx + k_x;   // k_x/2 right of pixel
			minx = MAX(minx, 0);     // keep in bounds
			maxx = MIN(maxx, cols2);
			
			// average blur it
			int x2;
			int total = 0;
			int count = 0;
			for (x2=minx; x2<maxx; x2++) {
				total += gray2[y][x2];    // use "gray2" as input
				count++;
			}
			blurx2[y][x] = total / count; // blurx2 is output
		}
	}
	
	//SavegrayPng(blurx2, "out/3_blur_just_x.png", rows2, cols2);
	
	// blur in the y dimension
	byte **blur2 = (byte**)malloc2d(rows2, cols2);
	for (y=0; y<rows2; y++) {
		for (x=0; x<cols2; x++) {
			
			// Start and end to blur
			int miny = y-k_y/2;      // k_x/2 left of pixel
			int maxy = miny + k_y;   // k_x/2 right of pixel
			miny = MAX(miny, 0);     // keep in bounds
			maxy = MIN(maxy, rows2);
			
			// average blur it
			int y2;
			int total = 0;
			int count = 0;
			for (y2=miny; y2<maxy; y2++) {
				total += blurx2[y2][x];    // use blurx2 as input
				count++;
			}
			blur2[y][x] = total / count;   // blur is output
		}
	}
	
	//SavegrayPng(blur2, "out/3_blur.png", rows2, cols2);
	
	
	//-----------------
	// Take the "sobel2" (magnitude of derivative)
	//  (Actually we'll make up something similar)
	//-----------------
	
	byte **sobel2 = (byte**)malloc2d(rows2, cols2);
	
	for (y=0; y<rows2; y++) {
		for (x=0; x<cols2; x++) {
			int mag=0;
			
			if (y>0)      mag += ABS(blur2[y-1][x] - blur2[y][x]);
			if (x>0)      mag += ABS(blur2[y][x-1] - blur2[y][x]);
			if (y<rows2-1) mag += ABS(blur2[y+1][x] - blur2[y][x]);
			if (x<cols2-1) mag += ABS(blur2[y][x+1] - blur2[y][x]);
			
			sobel2[y][x] = 3*mag;
		}
	}
	
	//SavegrayPng(sobel2, "out/4_sobel2.png", rows2, cols2);
	
	//-----------------
	// Non-max suppression
	//-----------------
	byte **nonmax2 = malloc2d(rows2, cols2);    // note: *this* initializes to zero!
	
	for (y=1; y<rows2-1; y++)
	{
		for (x=1; x<cols2-1; x++)
		{
			// Is it a local maximum
			int is_y_max = (sobel2[y][x] > sobel2[y-1][x] && sobel2[y][x]>=sobel2[y+1][x]);
			int is_x_max = (sobel2[y][x] > sobel2[y][x-1] && sobel2[y][x]>=sobel2[y][x+1]);
			if (is_y_max || is_x_max)
				nonmax2[y][x] = sobel2[y][x];
			else
				nonmax2[y][x] = 0;
		}
	}
	
	//SavegrayPng(nonmax2, "out/5_nonmax2.png", rows2, cols2);
	
	//-----------------
	// Final Threshold
	//-----------------
	byte **edges2 = malloc2d(rows2, cols2);    // note: *this* initializes to zero!
	
	for (y=0; y<rows2; y++) {
		for (x=0; x<cols2; x++) {
			if (nonmax2[y][x] > CANNY_THRESH2)
				edges2[y][x] = 255;
			else
				edges2[y][x] = 0;
		}
	}
	
	//SavegrayPng(edges2, "out/6_edges2.png", rows2, cols2);
    
    // chan2ge to red edges2
    
    byte ***colorimg2= malloc3d(rows2, cols2, chan2);
    
    for (y=0; y<rows2;y++){
        for(x=0; x<cols2; x++){
            colorimg2[y][x][0]=edges2[y][x];
            colorimg2[y][x][1]=0;
            colorimg2[y][x][2]=0;
        }
    }
    
    //SaveRgbPng(colorimg2, "out/7_rededges2.png", rows2, cols2);
    
    //RANSAC
    
    struct Ransac array2[20000];

    for(int iter=0;iter<20000;){
        
        int x0=rand()%cols2;//arr[a].xval;
        int y0=rand()%rows2;//arr[a].yval;
        int x1=rand()%cols2;//arr[b].xval;
        int y1=rand()%rows2;

        double dx=x1-x0;
        double dy=y1-y0;
       
        if(dx!=0 & dy!=0){
            int m=dx/dy;
            int c=x0-(m*y0);
            
            
            int pc=0;
            
            
            for (y=0; y<rows2; y++) {
                for (x=0; x<cols2; x++) {
                    int z=m*y+c;
                    if((x==z) && (colorimg2[y][x][0]==255))
                        pc++;
                }
            }
            
            array2[iter].p_count=pc;
            array2[iter].line_m=m;
            array2[iter].line_c=c;
            array2[iter].p_x0=x0;
            array2[iter].p_y0=y0;
            array2[iter].p_x1=x1;
            array2[iter].p_y1=y1;
            
            iter++;
        }
    }
    
    bsort(array2,20000);
    
//    for(int iter=0;iter<5;iter++){
//        printf("\n For iter=%d \n m=%d \t c=%d \t count=%d p0=%d,%d p1=%d,%d\n ",iter,array2[iter].line_m,array2[iter].line_c,array2[iter].p_count,array2[iter].p_x0,array2[iter].p_y0,array2[iter].p_x1,array2[iter].p_y1);
//    }
    
    for(int iter=0;iter<65;iter++){
        for(y=0;y<rows2;y++){
            x=(array2[iter].line_m*y)+array2[iter].line_c;
//            printf("\n %d \t %d \n",x,y);
            if(colorimg2[y][x][0]==255){
//               printf("\n %d \t %d \n",x,y);
                colorimg2[y][x][1]=255;
           }
        }
    }
    
    SaveRgbPng(colorimg2, "out/lines_pentagon.png", rows2, cols2);
    printf("Done!\n");
	
	return 0;
}


/*

	printf("load image\n");
	byte *data = stbi_load("puppy.jpg", &cols2, &rows2, &chan2, 4);

	printf("data = %p\n", data);
	int rt=stbi_write_png("output.png", cols2, rows2, 4, data, cols2*4);
*/
