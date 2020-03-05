#include <stdio.h>
#include <stdlib.h>
#include "dc_image.h"


#define CANNY_THRESH4 35
#define CANNY_blur4   7

#define MIN(a,b)  ( (a) < (b) ? (a) : (b) )
#define MAX(a,b)  ( (a) > (b) ? (a) : (b) )
#define ABS(x)    ( (x) <= 0 ? 0-(x) : (x) )

struct Ransac{
    int p_count;
    int line_m;
    int line_c;
    int p_x0,p_y0,p_x1,p_y1;
};

void bsort(struct Ransac list[5000], int s)
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
	int rows4, cols4, chan4;

	//-----------------
	// Read the image    [y][x][c]   y number rows4   x cols4  c 3
	//-----------------
	byte ***img4 = LoadRgb("sidewalk.png", &rows4, &cols4, &chan4);
	printf("img4 %p rows %d cols %d chan %d\n", img4, rows4, cols4, chan4);

	
	//SaveRgbPng(img4, "out/1_img4.png", rows4, cols4);
	
	//-----------------
	// Convert to gray4scale
	//-----------------
	byte **gray4 = malloc2d(rows4, cols4);
	for (y=0; y<rows4; y++){
		for (x=0; x<cols4; x++) {
			int r = img4[y][x][0];
			int g = img4[y][x][1];
			int b = img4[y][x][2];
			gray4[y][x] =  (r+g+b) / 3;
		}
	}
	
	//Savegray4Png(gray4, "out/2_gray4.png", rows4, cols4);

	//-----------------
	// Box blur4   ToDo: Gaussian blur4 is better
	//-----------------
	
	// Box blur4 is separable, so separately blur4 x and y
	int k_x=CANNY_blur4, k_y=CANNY_blur4;
	
	// blur4 in the x dimension
	byte **blurx4 = (byte**)malloc2d(rows4, cols4);
	for (y=0; y<rows4; y++) {
		for (x=0; x<cols4; x++) {
			
			// Start and end to blur4
			int minx = x-k_x/2;      // k_x/2 left of pixel
			int maxx = minx + k_x;   // k_x/2 right of pixel
			minx = MAX(minx, 0);     // keep in bounds
			maxx = MIN(maxx, cols4);
			
			// average blur4 it
			int x2;
			int total = 0;
			int count = 0;
			for (x2=minx; x2<maxx; x2++) {
				total += gray4[y][x2];    // use "gray4" as input
				count++;
			}
			blurx4[y][x] = total / count; // blur4x is output
		}
	}
	
	//Savegray4Png(blur4x, "out/3_blur4_just_x.png", rows4, cols4);
	
	// blur4 in the y dimension
	byte **blur4 = (byte**)malloc2d(rows4, cols4);
	for (y=0; y<rows4; y++) {
		for (x=0; x<cols4; x++) {
			
			// Start and end to blur4
			int miny = y-k_y/2;      // k_x/2 left of pixel
			int maxy = miny + k_y;   // k_x/2 right of pixel
			miny = MAX(miny, 0);     // keep in bounds
			maxy = MIN(maxy, rows4);
			
			// average blur4 it
			int y2;
			int total = 0;
			int count = 0;
			for (y2=miny; y2<maxy; y2++) {
				total += blurx4[y2][x];    // use blur4x as input
				count++;
			}
			blur4[y][x] = total / count;   // blur4 is output
		}
	}
	
	//Savegray4Png(blur4, "out/3_blur4.png", rows4, cols4);
	
	
	//-----------------
	// Take the "sobel4" (magnitude of derivative)
	//  (Actually we'll make up something similar)
	//-----------------
	
	byte **sobel4 = (byte**)malloc2d(rows4, cols4);
	
	for (y=0; y<rows4; y++) {
		for (x=0; x<cols4; x++) {
			int mag=0;
			
			if (y>0)      mag += ABS(blur4[y-1][x] - blur4[y][x]);
			if (x>0)      mag += ABS(blur4[y][x-1] - blur4[y][x]);
			if (y<rows4-1) mag += ABS(blur4[y+1][x] - blur4[y][x]);
			if (x<cols4-1) mag += ABS(blur4[y][x+1] - blur4[y][x]);
			
			sobel4[y][x] = 3*mag;
		}
	}
	
	//Savegray4Png(sobel4, "out/4_sobel4.png", rows4, cols4);
	
	//-----------------
	// Non-max suppression
	//-----------------
	byte **nonmax4 = malloc2d(rows4, cols4);    // note: *this* initializes to zero!
	
	for (y=1; y<rows4-1; y++)
	{
		for (x=1; x<cols4-1; x++)
		{
			// Is it a local maximum
			int is_y_max = (sobel4[y][x] > sobel4[y-1][x] && sobel4[y][x]>=sobel4[y+1][x]);
			int is_x_max = (sobel4[y][x] > sobel4[y][x-1] && sobel4[y][x]>=sobel4[y][x+1]);
			if (is_y_max || is_x_max)
				nonmax4[y][x] = sobel4[y][x];
			else
				nonmax4[y][x] = 0;
		}
	}
	
	//Savegray4Png(nonmax4, "out/5_nonmax4.png", rows4, cols4);
	
	//-----------------
	// Final Threshold
	//-----------------
	byte **edges4 = malloc2d(rows4, cols4);    // note: *this* initializes to zero!
	
	for (y=0; y<rows4; y++) {
		for (x=0; x<cols4; x++) {
			if (nonmax4[y][x] > CANNY_THRESH4)
				edges4[y][x] = 255;
			else
				edges4[y][x] = 0;
		}
	}
	
	//Savegray4Png(edges4, "out/6_edges4.png", rows4, cols4);
    
    // chan4ge to red edges4
    
    byte ***colorimg4= malloc3d(rows4, cols4, chan4);
    
    for (y=0; y<rows4;y++){
        for(x=0; x<cols4; x++){
            colorimg4[y][x][0]=edges4[y][x];
            colorimg4[y][x][1]=0;
            colorimg4[y][x][2]=0;
        }
    }
    
    //SaveRgbPng(colorimg4, "out/7_rededges4.png", rows4, cols4);
    
    //RANSAC
    
    struct Ransac array4[5000];

    for(int iter=0;iter<5000;){
        
        int x0=rand()%cols4;//arr[a].xval;
        int y0=rand()%rows4;//arr[a].yval;
        int x1=rand()%cols4;//arr[b].xval;
        int y1=rand()%rows4;

        int dx=x1-x0;
        int dy=y1-y0;
       
        if(dx!=0 & dy!=0){
            int m=dx/dy;
            int c=x0-(m*y0);
            
            
            int pc=0;
            
            
            for (y=0; y<rows4; y++) {
                for (x=0; x<cols4; x++) {
                    int z=m*y+c;
                    if((x==z) && (colorimg4[y][x][0]==255))
                        pc++;
                }
            }
            
            array4[iter].p_count=pc;
            array4[iter].line_m=m;
            array4[iter].line_c=c;
            array4[iter].p_x0=x0;
            array4[iter].p_y0=y0;
            array4[iter].p_x1=x1;
            array4[iter].p_y1=y1;
            
            iter++;
        }
    }
    
    bsort(array4,5000);
    
//    for(int iter=0;iter<5;iter++){
//        printf("\n For iter=%d \n m=%d \t c=%d \t count=%d p0=%d,%d p1=%d,%d\n ",iter,array4[iter].line_m,array4[iter].line_c,array4[iter].p_count,array4[iter].p_x0,array4[iter].p_y0,array4[iter].p_x1,array4[iter].p_y1);
//    }
    
    for(int iter=0;iter<100;iter++){
        for(y=0;y<rows4;y++){
            x=(array4[iter].line_m*y)+array4[iter].line_c;
//            printf("\n %d \t %d \n",x,y);
            if(x>0 && colorimg4[y][x][0]==255){
//               printf("\n %d \t %d \n",x,y);
                colorimg4[y][x][1]=255;
           }
        }
    }
    
    SaveRgbPng(colorimg4, "out/lines_sidewalk.png", rows4, cols4);
    printf("Done!\n");
	
	return 0;
}


/*

	printf("load image\n");
	byte *data = stbi_load("puppy.jpg", &cols4, &rows4, &chan4, 4);

	printf("data = %p\n", data);
	int rt=stbi_write_png("output.png", cols4, rows4, 4, data, cols4*4);
*/
